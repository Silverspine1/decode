package org.firstinspires.ftc.teamcode.CommandBase;
import com.acmerobotics.dashboard.config.Config;

/**
 * Converts image pixel positions into ground-plane coordinates (cm).
 * Assumes:
 *  - camera at fixed height above floor
 *  - camera pitched downward by TILT_DOWN_DEG
 *  - balls lie on the floor (z = 0)
 */
@Config
public class CameraSettings {

    // Camera pose relative to floor (tune in Dashboard)
    public static double CAMERA_HEIGHT_CM = 40.0;
    public static double TILT_DOWN_DEG   = 15.0;

    // Hikvision DS-U12 FOV, tunable if needed
    public static double H_FOV_DEG = 81.0;
    public static double V_FOV_DEG = 50.0;

    // Pixel center offsets for misalignment
    public static double CENTER_X_OFFSET_PX = 0.0;
    public static double CENTER_Y_OFFSET_PX = 0.0;

    // Scale factors for final X/Y
    public static double X_SCALE = 1.0;
    public static double Y_SCALE = 1.0;

    public static class GroundPosition {
        public boolean valid;
        public double lateralCm;                  // + right
        public double forwardCm;                  // + away
        public double horizontalAngleDeg;
        public double verticalAngleFromHorizontalDeg;
        public double verticalFromCenterDeg;
    }

    public static GroundPosition projectPixelToGround(double px, double py,
                                                      int width, int height) {
        GroundPosition res = new GroundPosition();

        double cx = width  / 2.0 + CENTER_X_OFFSET_PX;
        double cy = height / 2.0 + CENTER_Y_OFFSET_PX;

        double dx = px - cx;     // right positive
        double dy = cy - py;     // up positive

        double hAngleDeg = dx * (H_FOV_DEG / width);
        double vFromCenterDeg = dy * (V_FOV_DEG / height);

        double phiDeg = TILT_DOWN_DEG - vFromCenterDeg;  // LOS angle to floor

        res.horizontalAngleDeg = hAngleDeg;
        res.verticalAngleFromHorizontalDeg = phiDeg;
        res.verticalFromCenterDeg = vFromCenterDeg;

        if (phiDeg <= 0.5) {
            res.valid = false;
            return res;
        }

        double phiRad = Math.toRadians(phiDeg);
        double forward = CAMERA_HEIGHT_CM / Math.tan(phiRad);

        double hAngleRad = Math.toRadians(hAngleDeg);
        double lateral = forward * Math.tan(hAngleRad);

        res.forwardCm = forward * Y_SCALE;
        res.lateralCm = lateral * X_SCALE;

        res.valid = true;
        return res;
    }
}
