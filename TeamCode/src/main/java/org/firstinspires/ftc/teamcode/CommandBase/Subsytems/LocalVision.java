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
import org.opencv.core.Rect;
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
        BOTH
    }

    // ====================== Dashboard Toggles ======================
    public static boolean SHOW_MASK = false;
    public static boolean DRAW_OVERLAY = true;

    // Downscale factor
    public static int DOWNSCALE = 2;

    // Detection params
    public static double MIN_RADIUS_PIXELS = 6.0;
    public static int MORPH_KERNEL_SIZE = 5;

    // Distance compensation power
    public static double DISTANCE_COMPENSATION_POWER = 2.0;

    // Timeout settings
    public static long FRAME_TIMEOUT_MS = 100;

    // ====================== NEW FEATURES ======================
    // Manual Exposure (tune here, apply in your OpMode with ExposureControl)
    // ── Manual Glare/Overexposure Suppression ──────────────────────────────────
// Clamps the V channel: pixels BRIGHTER than this are cut to this value.
// 255 = disabled | lower values = aggressively remove glare/hotspots
// Typical useful range: 180–230
    public static int EXPOSURE_CLAMP_MAX_V = 255;

    // ── Cropping (fractions of the full frame, 0.0 – 1.0) ─────────────────────
    public  double CROP_X = 0.15;   // left edge fraction
    public  double CROP_Y = 0.6;   // top  edge fraction
    public  double CROP_W = 1.0;   // width  fraction  (must be > 0)
    public  double CROP_H = 1.0;   // height fraction  (must be > 0)

    // ── Pre-process Blur ───────────────────────────────────────────────────────
// Gaussian blur kernel (odd number, 1 = disabled)
    public static int BLUR_KERNEL_SIZE = 3;
    public boolean blueAlance = true;

    // ====================== HSV Ranges ======================
    // PURPLE
    public static int PURPLE_H_MIN = 290;
    public static int PURPLE_H_MAX = 300;
    public static int PURPLE_S_MIN = 20;
    public static int PURPLE_S_MAX = 55;
    public static int PURPLE_V_MIN = 30;
    public static int PURPLE_V_MAX = 50;

    // GREEN
    public static int GREEN_H_MIN = 150;
    public static int GREEN_H_MAX = 165;
    public static int GREEN_S_MIN = 35;
    public static int GREEN_S_MAX = 100;
    public static int GREEN_V_MIN = 34;
    public static int GREEN_V_MAX = 95;

    // ====================== Output Variables ======================
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
    public volatile double compensatedScore = 0.0;

    // ====================== Angular Velocity Prediction ======================
    private static final int MAX_ANGLE_BUFFER = 60;
    private final double[] angleBuf = new double[MAX_ANGLE_BUFFER];
    private final long[] timeBuf = new long[MAX_ANGLE_BUFFER];
    private int bufHead = 0;
    private int bufCount = 0;
    private final Object bufferLock = new Object();

    public volatile double angularVelocityDegPerSec = 0.0;

    // ====================== Internal Fields ======================
    private final TargetColor targetColor;
    private volatile long lastFrameTimeMs = 0;
    private int imageWidth = 0;
    private int imageHeight = 0;
    private int smallWidth = 0;
    private int smallHeight = 0;

    private final Mat smallRgb = new Mat();
    private final Mat smallHsv = new Mat();
    private final Mat blurred = new Mat();      // ← ADD THIS

    private final Mat purpleMask = new Mat();
    private final Mat greenMask = new Mat();
    private final Mat combinedMask = new Mat();
    private final Mat hierarchy = new Mat();
    private Mat kernel = new Mat();
    private int currentKernelSize = -1;
    private final MatOfPoint2f contour2f = new MatOfPoint2f();

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
        double pixelArea;
        double compensatedScore;
        double cropOffsetX;     // ← ADD THIS
        double cropOffsetY;     // ← ADD THIS
    }

    private static class FrameAnalysis {
        public int cropW;
        public int cropX;
        public int cropY;
        public int cropH;

        boolean hasTarget;
        DetectionResult bestResult;
        double scaleX;
        double scaleY;
    }

    public LocalVision(TargetColor targetColor) {
        this.targetColor = targetColor;
    }

    // ====================== Prediction API ======================
    public double predictAngleDeg(int frames, double lookAheadSec) {
        synchronized (bufferLock) {
            int n = Math.min(Math.max(frames, 2), bufCount);
            if (n < 2) {
                angularVelocityDegPerSec = 0.0;
                return Double.NaN;
            }
            double[] t = new double[n];
            double[] a = new double[n];
            for (int i = 0; i < n; i++) {
                int idx = ((bufHead - n + i) % MAX_ANGLE_BUFFER + MAX_ANGLE_BUFFER) % MAX_ANGLE_BUFFER;
                t[i] = timeBuf[idx] / 1000.0;
                a[i] = angleBuf[idx];
            }
            double sumT = 0, sumA = 0, sumTT = 0, sumTA = 0;
            for (int i = 0; i < n; i++) {
                sumT += t[i];
                sumA += a[i];
                sumTT += t[i] * t[i];
                sumTA += t[i] * a[i];
            }
            double denom = n * sumTT - sumT * sumT;
            if (Math.abs(denom) < 1e-9) {
                angularVelocityDegPerSec = 0.0;
                return a[n - 1];
            }
            double slope = (n * sumTA - sumT * sumA) / denom;
            double intercept = (sumA - slope * sumT) / n;
            angularVelocityDegPerSec = slope;
            double predictTimeSec = t[n - 1] + lookAheadSec;
            return slope * predictTimeSec + intercept;
        }
    }

    public double predictAngleDeg() {
        return predictAngleDeg(10, 1.0);
    }

    private void recordAngleSample(double angleDeg, long timeMs) {
        synchronized (bufferLock) {
            angleBuf[bufHead] = angleDeg;
            timeBuf[bufHead] = timeMs;
            bufHead = (bufHead + 1) % MAX_ANGLE_BUFFER;
            if (bufCount < MAX_ANGLE_BUFFER) bufCount++;
        }
    }

    private void clearAngleBuffer() {
        synchronized (bufferLock) {
            bufHead = 0;
            bufCount = 0;
        }
    }

    public boolean isDataStale() {
        if (lastFrameTimeMs == 0) return true;
        return (System.currentTimeMillis() - lastFrameTimeMs) > FRAME_TIMEOUT_MS;
    }

    public boolean ensureFreshData() {
        if (isDataStale()) {
            clearDetection();
            return false;
        }
        return true;
    }

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
        angularVelocityDegPerSec = 0;
        clearAngleBuffer();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        imageWidth = width;
        imageHeight = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        lastFrameTimeMs = System.currentTimeMillis();
        if (blueAlance){
            CROP_X = 0.15;
        }else{
            CROP_X = -0.15;

        }


        int fullW = frame.width();
        int fullH = frame.height();

        // ── 1. Compute crop rectangle in full-frame pixels ─────────────────
// Positive CROP_X = remove from left, Negative CROP_X = remove from right
// Positive CROP_Y = remove from top,  Negative CROP_Y = remove from bottom
        int cropX = CROP_X >= 0 ? (int)(CROP_X * fullW) : 0;
        int cropY = CROP_Y >= 0 ? (int)(CROP_Y * fullH) : 0;
        int cropR = CROP_X >= 0 ? (int)(CROP_W * fullW) : (int)((1.0 + CROP_X) * fullW);
        int cropB = CROP_Y >= 0 ? (int)(CROP_H * fullH) : (int)((1.0 + CROP_Y) * fullH);
        int cropW = Math.max(1, Math.min(cropR - cropX, fullW - cropX));
        int cropH = Math.max(1, Math.min(cropB - cropY, fullH - cropY));
        // ── 2. PHYSICALLY copy the crop region into smallRgb ──────────────
        // submat() is a view — using it directly means coordinates still
        // reference the original origin.  Resize forces a real pixel copy
        // into a fresh Mat so contour coords are relative to (0,0) of the crop.
        int ds = Math.max(1, DOWNSCALE);
        int targetW = Math.max(1, cropW / ds);
        int targetH = Math.max(1, cropH / ds);
        if (targetW != smallWidth || targetH != smallHeight) {
            smallWidth  = targetW;
            smallHeight = targetH;
        }

        // scaleX/Y: how many full-frame pixels each small-mat pixel covers
        double scaleX = (double) cropW / smallWidth;
        double scaleY = (double) cropH / smallHeight;

        // Resize the cropped region — this is a true copy, not a view
        Imgproc.resize(
                frame.submat(new Rect(cropX, cropY, cropW, cropH)),
                smallRgb,
                new Size(smallWidth, smallHeight),
                0, 0, Imgproc.INTER_AREA
        );

        // ── 3. Gaussian blur ───────────────────────────────────────────────
        int blurK = Math.max(1, BLUR_KERNEL_SIZE);
        if (blurK % 2 == 0) blurK++;
        if (blurK > 1) {
            Imgproc.GaussianBlur(smallRgb, blurred, new Size(blurK, blurK), 0);
        } else {
            smallRgb.copyTo(blurred);
        }

        // ── 4. Convert to HSV ──────────────────────────────────────────────
        Imgproc.cvtColor(blurred, smallHsv, Imgproc.COLOR_RGB2HSV);

        // ── 5. Glare suppression: clamp V channel from above ──────────────
        int clampV = Math.max(0, Math.min(EXPOSURE_CLAMP_MAX_V, 255));
        if (clampV < 255) {
            List<Mat> channels = new ArrayList<>();
            Core.split(smallHsv, channels);
            // Any pixel brighter than clampV is cut down to clampV
            Core.min(channels.get(2), new Scalar(clampV), channels.get(2));
            Core.merge(channels, smallHsv);
            for (Mat ch : channels) ch.release();
        }

        // ── 6. Detect blobs (pass crop offsets for coordinate remapping) ───
        List<DetectionResult> detections = new ArrayList<>();
        if (targetColor == TargetColor.PURPLE || targetColor == TargetColor.BOTH) {
            detections.addAll(detectAllBlobsOfColor(TargetColor.PURPLE, new Scalar(PURPLE_H_MIN, PURPLE_S_MIN, PURPLE_V_MIN), new Scalar(PURPLE_H_MAX, PURPLE_S_MAX, PURPLE_V_MAX), cropX, cropY, fullW, fullH, scaleX, scaleY));
        }
        if (targetColor == TargetColor.GREEN || targetColor == TargetColor.BOTH) {
            detections.addAll(detectAllBlobsOfColor(TargetColor.GREEN, new Scalar(GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN), new Scalar(GREEN_H_MAX, GREEN_S_MAX, GREEN_V_MAX), cropX, cropY, fullW, fullH, scaleX, scaleY));
        }

        FrameAnalysis analysis = new FrameAnalysis();
        analysis.scaleX = scaleX;
        analysis.scaleY = scaleY;
        analysis.cropX  = cropX;
        analysis.cropY  = cropY;
        analysis.cropW  = cropW;
        analysis.cropH  = cropH;

        if (!detections.isEmpty()) {
            DetectionResult best = detections.get(0);
            for (DetectionResult d : detections)
                if (d.compensatedScore > best.compensatedScore) best = d;
            analysis.hasTarget  = true;
            analysis.bestResult = best;
        }

        telemetryUpdateFromAnalysis(analysis);
        return analysis;
    }
    private List<DetectionResult> detectAllBlobsOfColor(
            TargetColor color, Scalar lower, Scalar upper,
            int cropX, int cropY,      // full-frame pixel origin of the crop
            int fullW, int fullH,      // true camera frame dimensions
            double scaleX, double scaleY) {

        List<DetectionResult> results = new ArrayList<>();
        Mat mask = (color == TargetColor.PURPLE) ? purpleMask : greenMask;

        Core.inRange(smallHsv, lower, upper, mask);

        int kSize = Math.max(1, MORPH_KERNEL_SIZE);
        if (kSize != currentKernelSize) {
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kSize, kSize));
            currentKernelSize = kSize;
        }
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN,  kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.isEmpty()) return results;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            contour2f.fromArray(contour.toArray());

            Point   centerSmall = new Point();
            float[] radiusSmall = new float[1];
            Imgproc.minEnclosingCircle(contour2f, centerSmall, radiusSmall);
            if (radiusSmall[0] < MIN_RADIUS_PIXELS) continue;

            // Remap from small-crop coords → full-frame coords by adding crop offset
            double centerXFull   = cropX + centerSmall.x * scaleX;
            double centerYFull   = cropY + centerSmall.y * scaleY;
            double radiusFull    = radiusSmall[0] * (scaleX + scaleY) * 0.5;
            double pixBottomFull = fullH - centerYFull;

            // projectPixelToGround uses full-frame coordinates and dimensions — correct
            CameraSettings.GroundPosition gp = CameraSettings.projectPixelToGround(
                    centerXFull, centerYFull, fullW, fullH);
            if (!gp.valid) continue;

            double distanceM        = Math.max(gp.forwardCm / 100.0, 0.01);
            double compensatedScore = area * Math.pow(distanceM, DISTANCE_COMPENSATION_POWER);

            DetectionResult result      = new DetectionResult();
            result.color                = color;
            result.contourSmall         = contour;
            result.centerXFull          = centerXFull;
            result.centerYFull          = centerYFull;
            result.radiusFull           = radiusFull;
            result.pixelsFromBottom     = pixBottomFull;
            result.forwardCm            = gp.forwardCm;
            result.lateralCm            = gp.lateralCm;
            result.hAngleDeg            = gp.horizontalAngleDeg;
            result.losAngleDeg          = gp.verticalAngleFromHorizontalDeg;
            result.vFromCenterDeg       = gp.verticalFromCenterDeg;
            result.scaleX               = scaleX;
            result.scaleY               = scaleY;
            result.pixelArea            = area;
            result.compensatedScore     = compensatedScore;
            result.cropOffsetX          = cropX;
            result.cropOffsetY          = cropY;
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
                        path.lineTo((float) (pts[i].x * sx), (float) (pts[i].y * sy));
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

        if (!DRAW_OVERLAY) return;

        if (a.hasTarget && a.bestResult != null) {
            paint.setStyle(Paint.Style.STROKE);
            paint.setColor(a.bestResult.color == TargetColor.PURPLE ? Color.MAGENTA : Color.GREEN);
            float cx = (float) (a.bestResult.centerXFull * scale);
            float cy = (float) (a.bestResult.centerYFull * scale);
            float r = (float) (a.bestResult.radiusFull * scale);
            canvas.drawCircle(cx, cy, r, paint);

            paint.setStyle(Paint.Style.FILL);
            paint.setColor(Color.RED);
            canvas.drawCircle(cx, cy, 4 * scaleCanvasDensity, paint);
        }

        // Crosshair
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(Color.BLUE);
        float midX = (imageWidth / 2f) * scale;
        float midY = (imageHeight / 2f) * scale;
        canvas.drawLine(midX, 0, midX, onscreenHeight, paint);
        canvas.drawLine(0, midY, onscreenWidth, midY, paint);

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

        recordAngleSample(r.hAngleDeg, lastFrameTimeMs);
    }
}