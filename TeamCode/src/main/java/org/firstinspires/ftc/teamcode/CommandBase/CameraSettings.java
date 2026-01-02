package org.firstinspires.ftc.teamcode.CommandBase;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CameraSettings {

    // Pose (tune on Dashboard)
    public static double CAMERA_HEIGHT_CM = 26.5;
    public static double TILT_DOWN_DEG   = 38.8;   // positive means pitched down

    // Hikvision DS-U12 FOV
    public static double H_FOV_DEG = 81.0;
    public static double V_FOV_DEG = 50.0;

    // Principal-point offsets (px)
    public static double X_OFFSET_CM = 5.0; // +right
    public static double Y_OFFSET_CM = -7.0; // +forward

    // Optional scaling
    public static double X_SCALE = 1.0;
    public static double Y_SCALE = 1.05;

    // Guard against near-horizon rays
    public static double MIN_PHI_DEG = 0.5;

    public static class GroundPosition {
        public Boolean valid;
        public double lateralCm;                   // + right
        public double forwardCm;                   // + away
        public double horizontalAngleDeg;          // yaw from centerline
        public double verticalAngleFromHorizontalDeg; // LOS vs horizontal
        public double verticalFromCenterDeg;       // image offset in deg
    }

    public static GroundPosition projectPixelToGround(double px, double py,
                                                      int width, int height) {
        GroundPosition res = new GroundPosition();

        final double cx = width  / 2.0;
        final double cy = height / 2.0;

        // Right positive, DOWN positive
        final double dx = px - cx;
        final double dy = py - cy;

        final double hAngleDeg      = dx * (H_FOV_DEG / width);
        final double vFromCenterDeg = dy * (V_FOV_DEG / height);

        // Camera pitched down by TILT_DOWN_DEG. Pixels lower in image look further down.
        final double phiDeg = TILT_DOWN_DEG + vFromCenterDeg;

        res.horizontalAngleDeg = hAngleDeg;
        res.verticalAngleFromHorizontalDeg = phiDeg;
        res.verticalFromCenterDeg = vFromCenterDeg;

        if (phiDeg <= MIN_PHI_DEG) {
            res.valid = false;
            return res;
        }

        final double phiRad   = Math.toRadians(phiDeg);
        final double forward  = CAMERA_HEIGHT_CM / Math.tan(phiRad);
        final double hAngleRad= Math.toRadians(hAngleDeg);
        final double lateral  = forward * Math.tan(hAngleRad);

        res.forwardCm = (forward * Y_SCALE) + Y_OFFSET_CM;
        res.lateralCm = (lateral * X_SCALE) + X_OFFSET_CM;
        res.valid = true;
        return res;
    }
}