package org.firstinspires.ftc.teamcode.CommandBase.Auto.tuner;

/**
 * Model-based PD tuner (replaces the black-box Twiddle search).
 *
 * The Nexus follower's PD gains map an ERROR straight to a motor POWER:
 *   translation gains: error in cm  -> power  (setX/YOnPathPD, setX/YLastAdjustmentPD)
 *   heading gains:     error in deg -> turn power (setFast/SlowHeadingPD)
 *
 * Near the target the velocity feed-forward is ~0, so each axis behaves like a
 * second-order position loop:
 *
 *     error'' + A*Kd*error' + A*Kp*error = 0
 *
 * where A = "acceleration produced per unit motor power" (cm/s^2 per power for
 * translation, deg/s^2 per power for heading). Matching to the standard
 * second-order form  s^2 + 2*zeta*wn*s + wn^2  gives closed-form gains:
 *
 *     Kp = wn^2 / A
 *     Kd = 2*zeta*wn / A
 *     wn = 4 / (zeta * ts)          (2% settling-time relation)
 *
 * A is not guessed - it is MEASURED on the robot with direct-power step runs
 * (see PathTuner CHARACTERISE phase). zeta = 1 gives a critically damped,
 * overshoot-free response (matches the loose, accuracy-first gate).
 *
 * Refs: pole-placement PD design for 2nd-order position control
 *   Caltech CDS "Control of Second-Order Systems"
 *   https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/pph02-ch13.pdf
 *   Astrom & Murray, "PID Control" (Feedback Systems, ch.10)
 *   https://www.cds.caltech.edu/~murray/books/AM08/pdf/am08-pid_02Dec08.pdf
 */
public class ModelTuner {

    // Measured accel-per-unit-power. Translation in cm/s^2, heading in deg/s^2.
    public double aFwd  = 0;   // forward  (robot Y) -> sets the Y gains
    public double aStr  = 0;   // strafe   (robot X) -> sets the X gains
    public double aTurn = 0;   // rotation          -> sets the heading gains

    // Fallbacks if a characterisation run is bad (use the robot's rated limits).
    public double aFwdFallback, aStrFallback, aTurnFallback;

    // Design knobs. zeta stays 1 (no overshoot). The per-axis settling times are
    // NOT set by hand - PathTuner's speed search drives them down until each axis
    // just meets its tolerance, so the user only ever specifies tolerances.
    public double zeta = 1.0;     // 1.0 = critically damped (no overshoot)
    public double tsX  = 0.70;    // s, X (strafe) settling target
    public double tsY  = 0.70;    // s, Y (forward) settling target
    public double tsH  = 0.60;    // s, heading settling target

    // On-path tracking is made stiffer than the endpoint hold by this ratio.
    private static final double PATH_RATIO = 0.75;

    // Safety clamps on the produced gains.
    private static final double KP_MIN = 1e-4, KP_MAX = 1.0;
    private static final double KD_MIN = 0.0,  KD_MAX = 0.30;

    private double naturalFreq(double ts) {
        return 4.0 / (zeta * ts);
    }

    /** {Kp, Kd} for one axis given its measured A and a settling target. */
    public double[] pd(double A, double fallback, double ts) {
        double a = (A > 1.0) ? A : fallback;   // reject junk measurements
        double wn = naturalFreq(ts);
        double kp = clamp(wn * wn / a, KP_MIN, KP_MAX);
        double kd = clamp(2 * zeta * wn / a, KD_MIN, KD_MAX);
        return new double[]{kp, kd};
    }

    /**
     * Full 12-gain vector in PathTuner's order:
     *  0,1 onPathX   2,3 onPathY   4,5 endX   6,7 endY   8,9 hdgFast   10,11 hdgSlow
     */
    public double[] gains() {
        double[] px = pd(aStr,  aStrFallback,  tsX * PATH_RATIO); // on-path X (strafe)
        double[] py = pd(aFwd,  aFwdFallback,  tsY * PATH_RATIO); // on-path Y (forward)
        double[] ex = pd(aStr,  aStrFallback,  tsX);             // end X
        double[] ey = pd(aFwd,  aFwdFallback,  tsY);             // end Y
        double[] hf = pd(aTurn, aTurnFallback, tsH);             // heading fast
        // "slow" heading profile: same damping, ~40% gentler proportional term
        double[] hs = new double[]{ clamp(hf[0] * 0.6, KP_MIN, KP_MAX), hf[1] };

        return new double[]{
                px[0], px[1], py[0], py[1],
                ex[0], ex[1], ey[0], ey[1],
                hf[0], hf[1], hs[0], hs[1]
        };
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
