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

    // Distance compensation power (higher = more weight to distance)
    // Score = area * (distance ^ DISTANCE_COMPENSATION_POWER)
    // 1.0 = linear compensation, 2.0 = quadratic (accounts for area scaling)
    public static double DISTANCE_COMPENSATION_POWER = 2.0;

    // Timeout settings (assuming 10fps = 100ms between frames)
    public static long FRAME_TIMEOUT_MS = 100;

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

    // Outputs for the BEST target found (compensated for distance)
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
    public volatile double compensatedScore = 0.0;  // New: the score used for selection

    private final TargetColor targetColor;

    // Frame timeout tracking
    private volatile long lastFrameTimeMs = 0;

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
        double pixelArea;  // New: pixel area of the blob
        double compensatedScore;  // New: distance-compensated score
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

    /**
     * Check if the current frame data is stale (camera disconnected or frozen)
     * @return true if data is older than FRAME_TIMEOUT_MS
     */
    public boolean isDataStale() {
        if (lastFrameTimeMs == 0) return true;  // No frames received yet
        long elapsedMs = System.currentTimeMillis() - lastFrameTimeMs;
        return elapsedMs > FRAME_TIMEOUT_MS;
    }

    /**
     * Call this before reading detection values to ensure data is fresh.
     * If data is stale, all values are cleared to defaults.
     * @return true if data is fresh, false if stale/cleared
     */
    public boolean ensureFreshData() {
        if (isDataStale()) {
            clearDetection();
            return false;
        }
        return true;
    }

    /**
     * Clear all detection values to defaults (used on timeout)
     */
    private void clearDetection() {
        hasTarget = false;
        detectedColor = targetColor;
        distanceCm = 0;
        hAngleDeg = 0;
        vAngleDeg = 0;
        angleToBallDeg = 0;
        xPosCm = 0;
        yPosCm = 0;
        pixelsFromBottom = 0;
        radiusPixels = 0;
        compensatedScore = 0;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        imageWidth = width;
        imageHeight = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Update frame timestamp
        lastFrameTimeMs = System.currentTimeMillis();

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

        // Collect all detections from all colors
        List<DetectionResult> detections = new ArrayList<>();

        if (targetColor == TargetColor.PURPLE || targetColor == TargetColor.BOTH) {
            List<DetectionResult> purpleResults = detectAllBlobsOfColor(
                    TargetColor.PURPLE,
                    new Scalar(PURPLE_H_MIN, PURPLE_S_MIN, PURPLE_V_MIN),
                    new Scalar(PURPLE_H_MAX, PURPLE_S_MAX, PURPLE_V_MAX),
                    fullW, fullH, analysis.scaleX, analysis.scaleY
            );
            detections.addAll(purpleResults);
        }

        if (targetColor == TargetColor.GREEN || targetColor == TargetColor.BOTH) {
            List<DetectionResult> greenResults = detectAllBlobsOfColor(
                    TargetColor.GREEN,
                    new Scalar(GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN),
                    new Scalar(GREEN_H_MAX, GREEN_S_MAX, GREEN_V_MAX),
                    fullW, fullH, analysis.scaleX, analysis.scaleY
            );
            detections.addAll(greenResults);
        }

        // Pick the blob with the highest compensated score
        if (detections.isEmpty()) {
            analysis.hasTarget = false;
            telemetryUpdateFromAnalysis(analysis);
            return analysis;
        }

        DetectionResult best = detections.get(0);
        for (DetectionResult d : detections) {
            if (d.compensatedScore > best.compensatedScore) {
                best = d;
            }
        }

        analysis.hasTarget = true;
        analysis.bestResult = best;

        telemetryUpdateFromAnalysis(analysis);
        return analysis;
    }

    private List<DetectionResult> detectAllBlobsOfColor(TargetColor color, Scalar lower, Scalar upper,
                                                        int fullW, int fullH, double scaleX, double scaleY) {
        List<DetectionResult> results = new ArrayList<>();
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
            return results;
        }

        // Process ALL contours (not just the largest)
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            // Min enclosing circle
            contour2f.fromArray(contour.toArray());
            Point centerSmall = new Point();
            float[] radiusSmall = new float[1];
            Imgproc.minEnclosingCircle(contour2f, centerSmall, radiusSmall);

            if (radiusSmall[0] < MIN_RADIUS_PIXELS) {
                continue;  // Skip too-small blobs
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
                continue;  // Skip invalid projections
            }

            // Calculate compensated score
            // Objects further away appear smaller due to perspective
            // We compensate by: score = area * (distance ^ power)
            // This makes distant clusters compete fairly with nearby ones
            double distanceM = gp.forwardCm / 100.0;  // Convert to meters
            if (distanceM < 0.01) distanceM = 0.01;  // Prevent division issues

            double compensatedScore = area * Math.pow(distanceM, DISTANCE_COMPENSATION_POWER);

            // Build result
            DetectionResult result = new DetectionResult();
            result.color = color;
            result.contourSmall = contour;
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
            result.pixelArea = area;
            result.compensatedScore = compensatedScore;

            results.add(result);
        }

        return results;
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
        // First check if data is stale due to camera disconnect/timeout
        if (isDataStale()) {
            clearDetection();
            return;
        }

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
            compensatedScore = 0;
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
        compensatedScore = r.compensatedScore;
    }
}