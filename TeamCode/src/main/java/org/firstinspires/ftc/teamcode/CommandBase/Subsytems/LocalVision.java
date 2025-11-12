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
        GREEN
    }

    // Dashboard toggles
    public static boolean SHOW_MASK = false;    // when true: tint mask region on camera
    public static boolean DRAW_OVERLAY = true;  // when false: no overlays at all

    // Downscale factor for processing (1 = no downscale, 2 = half-res, etc.)
    public static int DOWNSCALE = 2;            // 2 → 640x480 processed as 320x240

    // Detection params (in downscaled pixels)
    public static double MIN_RADIUS_PIXELS = 6.0;
    public static int MORPH_KERNEL_SIZE = 5;

    // HSV ranges
    public static int PURPLE_H_MIN = 125;
    public static int PURPLE_H_MAX = 155;
    public static int PURPLE_S_MIN = 50;
    public static int PURPLE_S_MAX = 255;
    public static int PURPLE_V_MIN = 50;
    public static int PURPLE_V_MAX = 255;

    public static int GREEN_H_MIN = 40;
    public static int GREEN_H_MAX = 80;
    public static int GREEN_S_MIN = 50;
    public static int GREEN_S_MAX = 255;
    public static int GREEN_V_MIN = 50;
    public static int GREEN_V_MAX = 255;

    // Outputs
    public volatile boolean hasTarget = false;
    public volatile TargetColor detectedColor = TargetColor.PURPLE;

    public volatile double distanceCm = 0.0;
    public volatile double hAngleDeg = 0.0;
    public volatile double vAngleDeg = 0.0;
    public volatile double angleToBallDeg = 0.0;
    public volatile double xPosCm = 0.0;
    public volatile double yPosCm = 0.0;
    public volatile double pixelsFromBottom = 0.0;
    public volatile double radiusPixels = 0.0;  // in full-res pixels

    private final TargetColor targetColor;


    // Full image dims
    private int imageWidth = 0;
    private int imageHeight = 0;

    // Downscaled dims
    private int smallWidth = 0;
    private int smallHeight = 0;

    // Reused Mats (small)
    private final Mat smallRgb = new Mat();
    private final Mat smallHsv = new Mat();
    private final Mat smallMask = new Mat();
    private final Mat hierarchy = new Mat();
    private Mat kernel = new Mat();
    private int currentKernelSize = -1;

    private final MatOfPoint2f contour2f = new MatOfPoint2f();

    // Snapshot for drawing
    private static class FrameAnalysis {
        boolean hasTarget;

        MatOfPoint bestContourSmall;  // contour in downscaled coords

        double centerXFull;           // full-res coords
        double centerYFull;
        double radiusFull;

        double forwardCm;
        double lateralCm;
        double hAngleDeg;
        double losAngleDeg;
        double vFromCenterDeg;
        double pixelsFromBottom;

        TargetColor color;
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

        FrameAnalysis a = new FrameAnalysis();
        a.color = targetColor;

        // Decide downscaled size based on DOWNSCALE
        int ds = Math.max(1, DOWNSCALE);
        int targetW = fullW / ds;
        int targetH = fullH / ds;

        if (targetW != smallWidth || targetH != smallHeight) {
            smallWidth = targetW;
            smallHeight = targetH;
        }

        a.scaleX = (double) fullW / smallWidth;
        a.scaleY = (double) fullH / smallHeight;

        // Resize full frame -> smallRgb (uses interpolation suited for shrinking)
        Imgproc.resize(frame, smallRgb, new Size(smallWidth, smallHeight), 0, 0, Imgproc.INTER_AREA);

        // RGB -> HSV on downscaled
        Imgproc.cvtColor(smallRgb, smallHsv, Imgproc.COLOR_RGB2HSV);

        // HSV bounds
        Scalar lower, upper;
        if (targetColor == TargetColor.PURPLE) {
            lower = new Scalar(PURPLE_H_MIN, PURPLE_S_MIN, PURPLE_V_MIN);
            upper = new Scalar(PURPLE_H_MAX, PURPLE_S_MAX, PURPLE_V_MAX);
        } else {
            lower = new Scalar(GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN);
            upper = new Scalar(GREEN_H_MAX, GREEN_S_MAX, GREEN_V_MAX);
        }

        // Threshold on downscaled
        Core.inRange(smallHsv, lower, upper, smallMask);

        // Morphology with reused kernel (still on small image)
        int kSize = MORPH_KERNEL_SIZE;
        if (kSize < 1) kSize = 1;
        if (kSize != currentKernelSize) {
            kernel = Imgproc.getStructuringElement(
                    Imgproc.MORPH_RECT,
                    new Size(kSize, kSize)
            );
            currentKernelSize = kSize;
        }
        Imgproc.morphologyEx(smallMask, smallMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(smallMask, smallMask, Imgproc.MORPH_CLOSE, kernel);

        // Contours on smallMask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(smallMask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.isEmpty()) {
            hasTarget = false;
            telemetryUpdateFromAnalysis(a);
            return a;
        }

        // Largest contour
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
            hasTarget = false;
            telemetryUpdateFromAnalysis(a);
            return a;
        }

        // Min enclosing circle in downscaled coords
        contour2f.fromArray(largest.toArray());
        Point centerSmall = new Point();
        float[] radiusSmall = new float[1];
        Imgproc.minEnclosingCircle(contour2f, centerSmall, radiusSmall);

        if (radiusSmall[0] < MIN_RADIUS_PIXELS) {
            hasTarget = false;
            telemetryUpdateFromAnalysis(a);
            return a;
        }

        // Map center/radius back to full-res coordinates
        double centerXFull = centerSmall.x * a.scaleX;
        double centerYFull = centerSmall.y * a.scaleY;
        double radiusFull = radiusSmall[0] * (a.scaleX + a.scaleY) * 0.5; // average scale

        double pixBottomFull = fullH - centerYFull;

        a.centerXFull = centerXFull;
        a.centerYFull = centerYFull;
        a.radiusFull = radiusFull;
        a.pixelsFromBottom = pixBottomFull;
        a.bestContourSmall = largest;

        // Ground projection uses full-res center
        CameraSettings.GroundPosition gp =
                CameraSettings.projectPixelToGround(centerXFull, centerYFull, fullW, fullH);

        if (!gp.valid) {
            hasTarget = false;
            telemetryUpdateFromAnalysis(a);
            return a;
        }

        a.forwardCm = gp.forwardCm;
        a.lateralCm = gp.lateralCm;
        a.hAngleDeg = gp.horizontalAngleDeg;
        a.losAngleDeg = gp.verticalAngleFromHorizontalDeg;
        a.vFromCenterDeg = gp.verticalFromCenterDeg;
        a.hasTarget = true;

        telemetryUpdateFromAnalysis(a);
        return a;
    }

    @Override
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth,
                            int onscreenHeight,
                            float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity,
                            Object userContext) {

        FrameAnalysis a = (FrameAnalysis) userContext;
        if (a == null) return;

        float scale = scaleBmpPxToCanvasPx;

        Paint paint = new Paint();
        paint.setStrokeWidth(2 * scaleCanvasDensity);
        paint.setAntiAlias(false); // faster

        if (SHOW_MASK) {
            // Color mask view: we DO NOT clear the canvas.
            // We draw a translucent tint over all pixels that are in the thresholded region.
            if (a.bestContourSmall != null) {
                paint.setStyle(Paint.Style.FILL);
                // Semi-transparent cyan tint
                paint.setColor(Color.argb(120, 0, 255, 255));

                Path path = new Path();
                Point[] pts = a.bestContourSmall.toArray();
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

            // Optional: small red dot at center
            if (a.hasTarget) {
                paint.setStyle(Paint.Style.FILL);
                paint.setColor(Color.RED);
                float cx = (float) (a.centerXFull * scale);
                float cy = (float) (a.centerYFull * scale);
                canvas.drawCircle(cx, cy, 4 * scaleCanvasDensity, paint);
            }
            return;
        }

        if (!DRAW_OVERLAY) {
            // No overlay drawing at all
            return;
        }

        // NORMAL OVERLAY VIEW
        if (a.hasTarget) {
            paint.setStyle(Paint.Style.STROKE);
            paint.setColor(Color.GREEN);

            float cx = (float) (a.centerXFull * scale);
            float cy = (float) (a.centerYFull * scale);
            float r = (float) (a.radiusFull * scale);
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
        if (a.hasTarget) {
            float cx = (float) (a.centerXFull * scale);
            float cy = (float) (a.centerYFull * scale);

            paint.setColor(Color.YELLOW);
            canvas.drawLine(cx, onscreenHeight, cx, cy, paint);

            paint.setColor(Color.CYAN);
            canvas.drawLine(midX, cy, cx, cy, paint);
        }
    }

    private void telemetryUpdateFromAnalysis(FrameAnalysis a) {
        hasTarget = a.hasTarget;
        detectedColor = a.color;

        if (!a.hasTarget) {
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

        distanceCm = a.forwardCm;
        hAngleDeg = a.hAngleDeg;
        vAngleDeg = a.vFromCenterDeg;
        angleToBallDeg = a.losAngleDeg;
        xPosCm = a.lateralCm;
        yPosCm = a.forwardCm;
        pixelsFromBottom = a.pixelsFromBottom;
        radiusPixels = a.radiusFull;
    }
}
