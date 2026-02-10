package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.CommandBase.CameraSettings;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class LocalVision implements VisionProcessor {

    public enum TargetColor {
        PURPLE,
        GREEN,
        BOTH  // New option for dual detection
    }

    // Dashboard toggles
    public static boolean SHOW_MASK = false;
    public static boolean DRAW_OVERLAY = true;

    // Downscale factor
    public static int DOWNSCALE = 2;

    // Detection params
    public static double MIN_RADIUS_PIXELS = 6.0;
    public static int MORPH_KERNEL_SIZE = 5;

    // HSV ranges for PURPLE
    public static int PURPLE_H_MIN = 125;
    public static int PURPLE_H_MAX = 155;
    public static int PURPLE_S_MIN = 50;
    public static int PURPLE_S_MAX = 255;
    public static int PURPLE_V_MIN = 50;
    public static int PURPLE_V_MAX = 255;

    // HSV ranges for GREEN
    public static int GREEN_H_MIN = 40;
    public static int GREEN_H_MAX = 92;
    public static int GREEN_S_MIN = 50;
    public static int GREEN_S_MAX = 255;
    public static int GREEN_V_MIN = 50;
    public static int GREEN_V_MAX = 255;

    // Outputs for the BEST target found (regardless of color)
    public volatile boolean hasTarget = false;
    public volatile TargetColor detectedColor = TargetColor.PURPLE;
    public volatile double distanceCm = 0.0;
    public volatile double hAngleDeg = 0.0;
    public volatile double vAngleDeg = 0.0;
    public volatile double angleToBallDeg = 0.0;
    public volatile double xPosCm = 0.0;
    public volatile double yPosCm = 0.0;
    public volatile double pixelsFromBottom = 0.0;
    public volatile double radiusPixels = 0.0;

    private final TargetColor targetColor;

    // Full image dims
    private int imageWidth = 0;
    private int imageHeight = 0;

    // Downscaled dims
    private int smallWidth = 0;
    private int smallHeight = 0;

    // Reused Mats
    private final Mat smallRgb = new Mat();
    private final Mat smallHsv = new Mat();
    private final Mat purpleMask = new Mat();
    private final Mat greenMask = new Mat();
    private final Mat combinedMask = new Mat();
    private final Mat hierarchy = new Mat();
    private Mat kernel = new Mat();
    private int currentKernelSize = -1;

    private final MatOfPoint2f contour2f = new MatOfPoint2f();

    // Snapshot for drawing
    private static class DetectionResult {
        TargetColor color;
        MatOfPoint contourSmall;
        double centerXFull;
        double centerYFull;
        double radiusFull;
        double forwardCm;
        double lateralCm;
        double hAngleDeg;
        double losAngleDeg;
        double vFromCenterDeg;
        double pixelsFromBottom;
        double scaleX;
        double scaleY;
    }

    private static class FrameAnalysis {
        boolean hasTarget;
        DetectionResult bestResult;
        double scaleX;
        double scaleY;
    }

    public LocalVision(TargetColor targetColor) {
        this.targetColor = targetColor;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        imageWidth = width;
        imageHeight = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int fullW = frame.width();
        int fullH = frame.height();

        FrameAnalysis analysis = new FrameAnalysis();

        // Decide downscaled size
        int ds = Math.max(1, DOWNSCALE);
        int targetW = fullW / ds;
        int targetH = fullH / ds;

        if (targetW != smallWidth || targetH != smallHeight) {
            smallWidth = targetW;
            smallHeight = targetH;
        }

        analysis.scaleX = (double) fullW / smallWidth;
        analysis.scaleY = (double) fullH / smallHeight;

        // Resize and convert to HSV
        Imgproc.resize(frame, smallRgb, new Size(smallWidth, smallHeight), 0, 0, Imgproc.INTER_AREA);
        Imgproc.cvtColor(smallRgb, smallHsv, Imgproc.COLOR_RGB2HSV);

        // Create masks based on target color mode
        List<DetectionResult> detections = new ArrayList<>();

        if (targetColor == TargetColor.PURPLE || targetColor == TargetColor.BOTH) {
            DetectionResult purpleResult = detectColor(
                    TargetColor.PURPLE,
                    new Scalar(PURPLE_H_MIN, PURPLE_S_MIN, PURPLE_V_MIN),
                    new Scalar(PURPLE_H_MAX, PURPLE_S_MAX, PURPLE_V_MAX),
                    fullW, fullH, analysis.scaleX, analysis.scaleY
            );
            if (purpleResult != null) {
                detections.add(purpleResult);
            }
        }

        if (targetColor == TargetColor.GREEN || targetColor == TargetColor.BOTH) {
            DetectionResult greenResult = detectColor(
                    TargetColor.GREEN,
                    new Scalar(GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN),
                    new Scalar(GREEN_H_MAX, GREEN_S_MAX, GREEN_V_MAX),
                    fullW, fullH, analysis.scaleX, analysis.scaleY
            );
            if (greenResult != null) {
                detections.add(greenResult);
            }
        }

        // Pick the closest target (smallest forward distance)
        if (detections.isEmpty()) {
            analysis.hasTarget = false;
            telemetryUpdateFromAnalysis(analysis);
            return analysis;
        }

        DetectionResult best = detections.get(0);
        for (DetectionResult d : detections) {
            if (d.forwardCm < best.forwardCm) {
                best = d;
            }
        }

        analysis.hasTarget = true;
        analysis.bestResult = best;

        telemetryUpdateFromAnalysis(analysis);
        return analysis;
    }

    private DetectionResult detectColor(TargetColor color, Scalar lower, Scalar upper,
                                        int fullW, int fullH, double scaleX, double scaleY) {
        Mat mask = (color == TargetColor.PURPLE) ? purpleMask : greenMask;

        // Threshold
        Core.inRange(smallHsv, lower, upper, mask);

        // Morphology
        int kSize = MORPH_KERNEL_SIZE;
        if (kSize < 1) kSize = 1;
        if (kSize != currentKernelSize) {
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kSize, kSize));
            currentKernelSize = kSize;
        }
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.isEmpty()) {
            return null;
        }

        // Find largest contour
        MatOfPoint largest = null;
        double maxArea = 0.0;
        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area > maxArea) {
                maxArea = area;
                largest = c;
            }
        }

        if (largest == null) {
            return null;
        }

        // Min enclosing circle
        contour2f.fromArray(largest.toArray());
        Point centerSmall = new Point();
        float[] radiusSmall = new float[1];
        Imgproc.minEnclosingCircle(contour2f, centerSmall, radiusSmall);

        if (radiusSmall[0] < MIN_RADIUS_PIXELS) {
            return null;
        }

        // Scale to full resolution
        double centerXFull = centerSmall.x * scaleX;
        double centerYFull = centerSmall.y * scaleY;
        double radiusFull = radiusSmall[0] * (scaleX + scaleY) * 0.5;
        double pixBottomFull = fullH - centerYFull;

        // Ground projection
        CameraSettings.GroundPosition gp = CameraSettings.projectPixelToGround(
                centerXFull, centerYFull, fullW, fullH
        );

        if (!gp.valid) {
            return null;
        }

        // Build result
        DetectionResult result = new DetectionResult();
        result.color = color;
        result.contourSmall = largest;
        result.centerXFull = centerXFull;
        result.centerYFull = centerYFull;
        result.radiusFull = radiusFull;
        result.pixelsFromBottom = pixBottomFull;
        result.forwardCm = gp.forwardCm;
        result.lateralCm = gp.lateralCm;
        result.hAngleDeg = gp.horizontalAngleDeg;
        result.losAngleDeg = gp.verticalAngleFromHorizontalDeg;
        result.vFromCenterDeg = gp.verticalFromCenterDeg;
        result.scaleX = scaleX;
        result.scaleY = scaleY;

        return result;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {

        FrameAnalysis a = (FrameAnalysis) userContext;
        if (a == null) return;

        float scale = scaleBmpPxToCanvasPx;
        Paint paint = new Paint();
        paint.setStrokeWidth(2 * scaleCanvasDensity);
        paint.setAntiAlias(false);

        if (SHOW_MASK) {
            if (a.bestResult != null) {
                paint.setStyle(Paint.Style.FILL);
                paint.setColor(Color.argb(120, 0, 255, 255));

                Path path = new Path();
                Point[] pts = a.bestResult.contourSmall.toArray();
                if (pts.length > 0) {
                    float sx = (float) (scale * a.scaleX);
                    float sy = (float) (scale * a.scaleY);

                    float x0 = (float) (pts[0].x * sx);
                    float y0 = (float) (pts[0].y * sy);
                    path.moveTo(x0, y0);
                    for (int i = 1; i < pts.length; i++) {
                        float x = (float) (pts[i].x * sx);
                        float y = (float) (pts[i].y * sy);
                        path.lineTo(x, y);
                    }
                    path.close();
                    canvas.drawPath(path, paint);
                }
            }

            if (a.hasTarget && a.bestResult != null) {
                paint.setStyle(Paint.Style.FILL);
                paint.setColor(Color.RED);
                float cx = (float) (a.bestResult.centerXFull * scale);
                float cy = (float) (a.bestResult.centerYFull * scale);
                canvas.drawCircle(cx, cy, 4 * scaleCanvasDensity, paint);
            }
            return;
        }

        if (!DRAW_OVERLAY) {
            return;
        }

        // Normal overlay
        if (a.hasTarget && a.bestResult != null) {
            paint.setStyle(Paint.Style.STROKE);

            // Color the circle based on detected color
            if (a.bestResult.color == TargetColor.PURPLE) {
                paint.setColor(Color.MAGENTA);
            } else {
                paint.setColor(Color.GREEN);
            }

            float cx = (float) (a.bestResult.centerXFull * scale);
            float cy = (float) (a.bestResult.centerYFull * scale);
            float r = (float) (a.bestResult.radiusFull * scale);
            canvas.drawCircle(cx, cy, r, paint);

            paint.setStyle(Paint.Style.FILL);
            paint.setColor(Color.RED);
            canvas.drawCircle(cx, cy, 4 * scaleCanvasDensity, paint);
        }

        // Crosshair at image center
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(Color.BLUE);
        float midX = (imageWidth / 2f) * scale;
        float midY = (imageHeight / 2f) * scale;
        canvas.drawLine(midX, 0, midX, onscreenHeight, paint);
        canvas.drawLine(0, midY, onscreenWidth, midY, paint);

        // Helper lines
        if (a.hasTarget && a.bestResult != null) {
            float cx = (float) (a.bestResult.centerXFull * scale);
            float cy = (float) (a.bestResult.centerYFull * scale);

            paint.setColor(Color.YELLOW);
            canvas.drawLine(cx, onscreenHeight, cx, cy, paint);

            paint.setColor(Color.CYAN);
            canvas.drawLine(midX, cy, cx, cy, paint);
        }
    }

    private void telemetryUpdateFromAnalysis(FrameAnalysis a) {
        hasTarget = a.hasTarget;

        if (!a.hasTarget || a.bestResult == null) {
            detectedColor = targetColor;
            distanceCm = 0;
            hAngleDeg = 0;
            vAngleDeg = 0;
            angleToBallDeg = 0;
            xPosCm = 0;
            yPosCm = 0;
            pixelsFromBottom = 0;
            radiusPixels = 0;
            return;
        }

        DetectionResult r = a.bestResult;
        detectedColor = r.color;
        distanceCm = r.forwardCm;
        hAngleDeg = r.hAngleDeg;
        vAngleDeg = r.vFromCenterDeg;
        angleToBallDeg = r.losAngleDeg;
        xPosCm = r.lateralCm;
        yPosCm = r.forwardCm;
        pixelsFromBottom = r.pixelsFromBottom;
        radiusPixels = r.radiusFull;
    }
}