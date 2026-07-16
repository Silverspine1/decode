package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Auto.tuner.ModelTuner;
import org.firstinspires.ftc.teamcode.CommandBase.Auto.tuner.TrialResult;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import dev.weaponboy.nexus_pathing.Follower.Follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.SectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.PathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

/**
 * Model-based PID path tuner (Phase A: tracking / heading gains).
 *
 * YOU ONLY SET THREE NUMBERS: TOL_X, TOL_Y, TOL_H (the stop tolerances). The
 * tuner keeps damping critical (zeta = 1, no overshoot) and finds the FASTEST
 * gains that still stop inside those tolerances - i.e. max speed while in range.
 *
 * Pipeline:
 *   0. MOTION LIMITS - full-power runs on each axis (forward, strafe, rotate)
 *      measure real max velocity (plateau) and max accel (ramp-up slope), used
 *      to set RobotConstants instead of guessed constants. GoBILDA 435 RPM /
 *      96mm wheel theoretical free-speed is used only as a sanity bound and a
 *      seed for the plateau detector - not as the final number.
 *   1. CHARACTERISE - constant-power step pushes on each axis (forward, strafe,
 *      rotate) measure A = accel per unit power:  A = 2d / (u*t^2).
 *   2. SPEED SEARCH - per axis, drop the settling target ts (raising the gains,
 *      going faster) until a probe run breaks the tolerance / overshoot budget,
 *      then lock the last passing ts. Gains come from pole placement:
 *          Kp = wn^2/A,  Kd = 2*zeta*wn/A,  wn = 4/(zeta*ts)   (see ModelTuner).
 *   3. VERIFY - run a battery of <=100 cm paths with the final gains and log
 *      how well they track (avgDev / maxDev / final error).
 *
 * Place the robot in the MIDDLE of the field (180,180) facing 0, then dpad_up.
 * Outputs to /sdcard/FIRST/datalogs/: tune_best.txt, tune_paths.csv.
 *
 * Controls:  dpad_up = start,  B = stop & save,  X = skip current step.
 */
@TeleOp(name = "PathTuner (model based)", group = "tuning")
public class PathTuner extends OpModeEX {

    // ===================== THE ONLY THINGS YOU SET =====================
    private static final double TOL_X = 10.0;   // cm - allowed stop error, strafe/X
    private static final double TOL_Y = 10.0;   // cm - allowed stop error, forward/Y
    private static final double TOL_H = 6.0;   // deg - allowed stop error, heading
    // ===================================================================

    // ===================== DRIVETRAIN / MOTOR PROFILE =====================
    // goBILDA 5203/5202-series motor, 435 RPM at the output (after gearbox),
    // 96mm (3.78in) wheel diameter, 4 drive motors. Used only to compute a
    // THEORETICAL free-speed as a seed/sanity-check for the measured plateau
    // (real max speed is always lower once you add traction, weight, voltage
    // sag, and drivetrain losses - measurement below overrides this).
    private static final double MOTOR_RPM        = 435.0;
    private static final double WHEEL_DIAM_MM    = 96.0;
    private static final double WHEEL_DIAM_CM    = WHEEL_DIAM_MM / 10.0;
    private static final double WHEEL_CIRC_CM    = Math.PI * WHEEL_DIAM_CM;
    // Theoretical max wheel-surface speed, cm/s (free-spin, no load, no losses).
    private static final double THEORETICAL_MAX_V = (MOTOR_RPM / 60.0) * WHEEL_CIRC_CM;
    // Practical ceiling used to sanity-clamp the measured plateau. Real robots
    // land at roughly 70-90% of free-spin speed once loaded; clamp generously
    // high (100%) so we only reject clearly-bad measurements (odometry glitch,
    // slip, etc), not genuinely fast robots.
    private static final double MEASURED_V_CLAMP  = THEORETICAL_MAX_V * 1.05;
    // ========================================================================

    // Damping is fixed - critical, so there is no overshoot to trade off.
    private static final double ZETA = 1.0;

    // Speed search bounds/steps (settling time, seconds). Smaller ts = faster.
    private static final double TS_START_T = 0.55;  // translation start - was 0.80, now starts fast
    private static final double TS_START_H = 0.45;  // heading start - was 0.65
    private static final double TS_FLOOR   = 0.10;  // fastest allowed - was 0.20
    private static final double TS_CEIL    = 1.40;  // slowest allowed
    private static final double TS_DOWN    = 0.80;  // shrink ts when a probe passes
    private static final double TS_UP      = 1.15;  // grow ts when a probe fails - gentler backoff
    // Overshoot budget as a fraction of tolerance. This used to be 0.5 and
    // compared against a single-sample PEAK overshoot (see old searchDecision),
    // which meant one noisy/twitchy sample failed an otherwise-good run even
    // when final position error was tiny. Two changes fix that together:
    //   1. overshoot is now measured as "how far past tolerance, sustained
    //      for several consecutive samples" instead of raw single-sample peak
    //      (see sampleMetrics / OVERSHOOT_DEBOUNCE_N below),
    //   2. budget raised to the full tolerance, since tolerance IS the box
    //      you actually care about - overshooting INTO the box is not a
    //      failure, only overshooting past tolerance and staying there is.
    private static final double OVER_FRAC  = 1.0;
    private static final int    MAX_ITER   = 10;    // probes per axis cap - was 7, more room to converge fast

    // Characterisation step (PD gain characterisation, not max V/A).
    private static final double CHAR_POWER = 0.5;
    private static final double CHAR_TIME  = 0.40;
    // After each push we cut power and coast to measure the UNPOWERED
    // deceleration - this is what decides the soonest point the follower can be
    // switched off without the robot drifting back out of the target zone.
    private static final double COAST_TIME = 0.18;   // s of free coast to sample
    private static final double DC_TRANS_FALLBACK = 150; // cm/s^2 if unmeasured
    private static final double DC_TURN_FALLBACK  = 200; // deg/s^2 if unmeasured

    // ===================== MAX V / MAX A CHARACTERISATION =====================
    // Full-power run per axis: drive at 1.0 power until velocity plateaus
    // (steady state, forces balanced) or we run out of safe room in the box.
    private static final double MAXVA_POWER        = 1.0;
    private static final double MAXVA_MAX_TIME     = 1.2;   // s, hard cap per run
    // Full-power runs build real momentum and stopDrive() only cuts power - it
    // does NOT brake - so the robot coasts a real distance after we decide to
    // stop. Cut power at this margin INSIDE BOX_MIN/BOX_MAX, not at the wall
    // itself, or the coast will carry it off the field before returnHome()
    // can catch it. Tune this up if you still see it sliding out.
    private static final double MAXVA_BOX_MARGIN    = 40;
    private static final double MAXVA_SAMPLE_DT_MIN = 0.02; // ignore samples closer than this (noise)
    private static final int    MAXVA_PLATEAU_N    = 5;     // consecutive samples to confirm plateau
    private static final double MAXVA_PLATEAU_TOL  = 0.04;  // fractional change considered "flat"
    private static final double MAXVA_MIN_RUN_TIME = 0.15;  // ignore accel spikes before this (noise/backlash)
    // =============================================================================

    private static final double SETTLE_TIMEOUT = 3.5;
    private static final double RETURN_TIMEOUT = 3.5;
    private static final double VEL_EPS        = 5.0;

    // Field / safety
    private static final double CX = 180, CY = 180;
    private static final double BOX_MIN = 55, BOX_MAX = 305;

    // Fallback speed/accel limits, only used if a MAX_V/A measurement is
    // unavailable (e.g. user skipped that step). Overwritten by measured
    // values in computeMaxVA() otherwise.
    private static final double FALLBACK_XV = 130, FALLBACK_YV = 181, FALLBACK_XA = 650, FALLBACK_YA = 700;
    private static final double FALLBACK_TURN_V = 250, FALLBACK_TURN_A = 300;

    // Measured robot constants (populated by MAXVA phase; start as fallbacks).
    private double measXV = FALLBACK_XV, measYV = FALLBACK_YV;
    private double measXA = FALLBACK_XA, measYA = FALLBACK_YA;
    private double measTurnV = FALLBACK_TURN_V, measTurnA = FALLBACK_TURN_A;
    private boolean maxVaDone = false;

    // Safe (known-good) gains used only for driving home between steps.
    private static final double[] SAFE_GAINS = {
            0.08, 0.004,  0.2, 0.004,
            0.02, 0.004,  0.02, 0.009,
            0.01, 0.0005, 0.012, 0.002
    };

    // ============================= PATHS =============================
    private static class Item {
        final double[][] rel;
        final double hdgDelta;
        Item(double[][] rel, double hdgDelta) { this.rel = rel; this.hdgDelta = hdgDelta; }
    }

    // One probe per search axis: 0 = X (strafe), 1 = Y (forward), 2 = heading.
    private final Item[] PROBE = {
            new Item(new double[][]{{100, 0}}, 0),
            new Item(new double[][]{{0, 100}}, 0),
            new Item(new double[][]{{0, 80}}, 45)
    };

    private final Item[] BATTERY = {
            new Item(new double[][]{{0, 100}}, 0),
            new Item(new double[][]{{0, -100}}, 0),
            new Item(new double[][]{{100, 0}}, 0),
            new Item(new double[][]{{-100, 0}}, 0),
            new Item(new double[][]{{70, 70}}, 0),
            new Item(new double[][]{{50, 50}, {100, 0}}, 0),
            new Item(new double[][]{{0, 80}}, 45)
    };

    // ============================= MAX V/A CHARACTERISATION =============================
    private enum MvaAxis { FWD, STR, TURN }
    private final MvaAxis[] MVA_SEQ = { MvaAxis.FWD, MvaAxis.STR, MvaAxis.TURN };
    private int mvaIdx = 0;
    private double mvaStartX, mvaStartY, mvaStartHeading;
    private double mvaLastV = 0;
    private double mvaLastSampleT = 0;
    private int mvaPlateauCount = 0;
    private double mvaPeakAccel = 0;
    private double mvaPrevV = 0;
    private double mvaPrevT = 0;
    private double mvaPlateauV = 0;
    // per-axis measured results, filled as each axis finishes
    private final double[] mvaMaxV = { Double.NaN, Double.NaN, Double.NaN }; // FWD, STR, TURN
    private final double[] mvaMaxA = { Double.NaN, Double.NaN, Double.NaN };
    private boolean[] mvaPlateauReached = { false, false, false };

    // ============================= CHARACTERISATION =============================
    private enum Axis { FWD, STR, TURN }
    private final Axis[] CHAR_SEQ = { Axis.FWD, Axis.FWD, Axis.STR, Axis.STR, Axis.TURN, Axis.TURN };
    private double sumFwd = 0, sumStr = 0, sumTurn = 0;
    private int    nFwd = 0,   nStr = 0,   nTurn = 0;
    private int charIdx = 0;
    private double charStartX, charStartY, charStartHeading;

    // coast (unpowered) deceleration measurement
    private double sumDcF = 0, sumDcS = 0, sumDcT = 0;
    private int    nDcF = 0,   nDcS = 0,   nDcT = 0;
    private double dcTrans = DC_TRANS_FALLBACK; // cm/s^2, used to predict coast
    private double dcTurn  = DC_TURN_FALLBACK;  // deg/s^2
    private double coastV0, coastW0;            // motion captured at power-cut

    // ============================= STATE =============================
    private enum State {
        IDLE,
        MAXVA_PUSH, MAXVA_RETURN,
        CHAR_PUSH, CHAR_COAST, CHAR_RETURN,
        SEARCH_PROBE, SEARCH_RETURN,
        VERIFY_RUN, VERIFY_RETURN,
        FINISHED
    }
    private State state = State.IDLE;

    // fixedGenConfig is rebuilt once MAX V/A is known (uses measured constants
    // for the safe/return follower too, clamped down for safety - see below).
    private RobotConfig fixedGenConfig;
    private PathsManager paths;
    private Follower follow;       // built with the current tuned gains
    private Follower safeFollow;   // seed gains, for returning home

    private final ModelTuner model = new ModelTuner();
    private double[] tunedGains = null;

    // speed-search state
    private int sIdx = 0;              // 0=X, 1=Y, 2=H
    private double curTs, bestTs;
    private boolean anyPass;
    private int sIter;
    private boolean searching = false;

    // first-cycle baseline vs locked result, per axis (X, Y, H) - for the improvement readout
    private final double[] firstSettle = { Double.NaN, Double.NaN, Double.NaN };
    private final double[] firstErr    = { Double.NaN, Double.NaN, Double.NaN };
    private final double[] bestSettleA = { Double.NaN, Double.NaN, Double.NaN };
    private final double[] bestErrA    = { Double.NaN, Double.NaN, Double.NaN };

    // active path state
    private int batteryIdx = 0;
    private double startX, startY, startHeading;
    private double targetX, targetY, dirX, dirY, targetHeading;
    private TrialResult tr = new TrialResult();
    private boolean settled = false;
    // Debounced overshoot: counts consecutive samples where the robot is past
    // TARGET+tolerance in the direction of travel. A single noisy/twitch
    // sample no longer fails a run - only overshoot that's actually held for
    // several samples in a row counts. This is what searchDecision checks,
    // NOT tr.overshoot (which stays as the raw peak, for logging).
    private static final int OVERSHOOT_DEBOUNCE_N = 4; // consecutive samples to count as "real"
    private int overshootStreak = 0;
    private boolean sustainedOvershoot = false;

    private final ElapsedTime timer = new ElapsedTime();
    private FileWriter pathCsv;

    // ============================= LIFECYCLE =============================
    @Override
    public void initEX() {
        odometry.startPosition((int) CX, (int) CY, 0);
        driveBase.tele = false;
        driveBase.speed = 1;

        // Build with fallback constants until MAX V/A phase overwrites them.
        fixedGenConfig = buildConfig(SAFE_GAINS, FALLBACK_XV, FALLBACK_YV, FALLBACK_XA, FALLBACK_YA);
        paths = new PathsManager(fixedGenConfig);
        safeFollow = new Follower(fixedGenConfig);

        model.zeta = ZETA;
        model.tsX = TS_START_T;
        model.tsY = TS_START_T;
        model.tsH = TS_START_H;
        model.aFwdFallback = FALLBACK_YA;
        model.aStrFallback = FALLBACK_XA;
        model.aTurnFallback = FALLBACK_TURN_A;

        telemetry.addLine("PathTuner (model based). Robot at CENTRE, facing 0.");
        telemetry.addData("tolerances", String.format("X=%.1f Y=%.1f cm  H=%.1f deg", TOL_X, TOL_Y, TOL_H));
        telemetry.addData("theoretical max V", String.format("%.0f cm/s (435rpm, 96mm wheel)", THEORETICAL_MAX_V));
        telemetry.addLine("dpad_up = start,  B = stop+save,  X = skip step");
    }

    @Override
    public void loopEX() {
        boolean start = currentGamepad1.dpad_up && !lastGamepad1.dpad_up;
        boolean stop  = currentGamepad1.b && !lastGamepad1.b;
        boolean skip  = currentGamepad1.x && !lastGamepad1.x;

        if (stop && state != State.IDLE && state != State.FINISHED) {
            if (tunedGains != null) saveBest();
            stopDrive();
            state = State.FINISHED;
        }

        switch (state) {
            case IDLE:
                stopDrive();
                if (start) {
                    openLog();
                    mvaIdx = 0;
                    beginMaxVA();
                    state = State.MAXVA_PUSH;
                }
                break;

            case MAXVA_PUSH:
                maxVaPush(skip);
                break;

            case MAXVA_RETURN:
                if (returnHome()) {
                    mvaIdx++;
                    if (mvaIdx >= MVA_SEQ.length) {
                        computeMaxVA();
                        charIdx = 0;
                        beginChar();
                        state = State.CHAR_PUSH;
                    } else {
                        beginMaxVA();
                        state = State.MAXVA_PUSH;
                    }
                }
                break;

            case CHAR_PUSH:
                charPush(skip);
                break;

            case CHAR_COAST:
                charCoast();
                break;

            case CHAR_RETURN:
                if (returnHome()) {
                    charIdx++;
                    if (charIdx >= CHAR_SEQ.length) {
                        computeModel();
                        sIdx = 0;
                        beginSearchAxis();
                    } else {
                        beginChar();
                        state = State.CHAR_PUSH;
                    }
                }
                break;

            case SEARCH_PROBE:
                runPath(skip);
                break;

            case SEARCH_RETURN:
                if (returnHome()) searchDecision();
                break;

            case VERIFY_RUN:
                runPath(skip);
                break;

            case VERIFY_RETURN:
                if (returnHome()) {
                    batteryIdx++;
                    if (batteryIdx >= BATTERY.length) {
                        state = State.FINISHED;
                    } else {
                        beginPath(BATTERY[batteryIdx]);
                        state = State.VERIFY_RUN;
                    }
                }
                break;

            case FINISHED:
                stopDrive();
                break;
        }

        drawTelemetry();
    }

    // ============================= MAX V / MAX A =============================
    private void beginMaxVA() {
        odometry.queueCommand(odometry.update);
        mvaStartX = odometry.X();
        mvaStartY = odometry.Y();
        mvaStartHeading = odometry.Heading();
        mvaLastV = 0;
        mvaPrevV = 0;
        mvaPrevT = 0;
        mvaLastSampleT = 0;
        mvaPlateauCount = 0;
        mvaPeakAccel = 0;
        mvaPlateauV = 0;
        timer.reset();
    }

    /**
     * Drive at full power on the current axis until velocity plateaus
     * (steady-state - accel decays to ~0) or we run out of safe room / time.
     * Records max accel (peak dV/dt during ramp-up) and max velocity
     * (plateau value, or best value seen if plateau was never reached).
     */
    private void maxVaPush(boolean skip) {
        odometry.queueCommand(odometry.update);

        MvaAxis axis = MVA_SEQ[mvaIdx];
        double t = timer.seconds();

        boolean boxHit = outsideMaxVaBox();
        boolean timeUp = t >= MAXVA_MAX_TIME;

        if (boxHit || timeUp || skip) {
            finishMaxVaRun(axis, boxHit);
            return;
        }

        switch (axis) {
            case FWD:  driveBase.queueCommand(driveBase.drivePowers(MAXVA_POWER, 0, 0)); break;
            case STR:  driveBase.queueCommand(driveBase.drivePowers(0, 0, MAXVA_POWER)); break;
            case TURN: driveBase.queueCommand(driveBase.drivePowers(0, MAXVA_POWER, 0)); break;
        }

        // sample velocity, throttled so odometry noise doesn't spam plateau checks
        if (t - mvaLastSampleT < MAXVA_SAMPLE_DT_MIN) return;

        double v;
        if (axis == MvaAxis.TURN) {
            v = Math.abs(degPerSec());
        } else {
            v = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        }

        // peak accel from ramp-up samples only (skip earliest noise/backlash window)
        if (mvaPrevT > 0 && t > MAXVA_MIN_RUN_TIME) {
            double dt = t - mvaPrevT;
            if (dt > 0) {
                double a = (v - mvaPrevV) / dt;
                if (a > mvaPeakAccel) mvaPeakAccel = a;
            }
        }

        // plateau detection: N consecutive samples within MAXVA_PLATEAU_TOL of each other
        if (mvaLastV > 1) {
            double frac = Math.abs(v - mvaLastV) / mvaLastV;
            if (frac < MAXVA_PLATEAU_TOL) {
                mvaPlateauCount++;
            } else {
                mvaPlateauCount = 0;
            }
        }
        if (mvaPlateauCount >= MAXVA_PLATEAU_N) {
            mvaPlateauV = v;
            finishMaxVaRun(axis, false);
            return;
        }

        mvaPrevV = v;
        mvaPrevT = t;
        mvaLastV = v;
        mvaLastSampleT = t;
    }

    private void finishMaxVaRun(MvaAxis axis, boolean boxHit) {
        if (boxHit) {
            // Hit the inner margin while still possibly accelerating - a plain
            // power cut isn't enough braking at these speeds. Pulse reverse
            // power briefly to kill velocity before handing off to returnHome().
            brakeAxis(axis);
        } else {
            stopDrive();
        }
        int i = axis.ordinal();
        boolean plateauReached = mvaPlateauCount >= MAXVA_PLATEAU_N;
        mvaPlateauReached[i] = plateauReached;
        // Use the plateau value if we got one; otherwise best (highest) speed seen -
        // this is a floor, not a true max, and gets flagged in telemetry/log.
        double vResult = plateauReached ? mvaPlateauV : Math.max(mvaLastV, mvaPrevV);

        double vClampBound = (axis == MvaAxis.TURN) ? Double.POSITIVE_INFINITY : MEASURED_V_CLAMP;
        if (vResult > vClampBound) vResult = vClampBound; // reject clearly-bad odometry spikes

        mvaMaxV[i] = vResult;
        mvaMaxA[i] = mvaPeakAccel;

        buildReturnPath();
        state = State.MAXVA_RETURN;
        timer.reset();
    }

    private void computeMaxVA() {
        // FWD -> Y axis, STR -> X axis, TURN -> heading. Falls back to the
        // theoretical/fallback constants for any axis whose run was skipped
        // or produced a clearly bad (NaN/zero) reading.
        double yV = safeVal(mvaMaxV[MvaAxis.FWD.ordinal()], FALLBACK_YV);
        double yA = safeVal(mvaMaxA[MvaAxis.FWD.ordinal()], FALLBACK_YA);
        double xV = safeVal(mvaMaxV[MvaAxis.STR.ordinal()], FALLBACK_XV);
        double xA = safeVal(mvaMaxA[MvaAxis.STR.ordinal()], FALLBACK_XA);
        double tV = safeVal(mvaMaxV[MvaAxis.TURN.ordinal()], FALLBACK_TURN_V);
        double tA = safeVal(mvaMaxA[MvaAxis.TURN.ordinal()], FALLBACK_TURN_A);

        measYV = yV; measYA = yA;
        measXV = xV; measXA = xA;
        measTurnV = tV; measTurnA = tA;

        // Feed the model's accel-per-unit-power directly from these full-power
        // measurements (divide by MAXVA_POWER to convert peak-accel-at-that-power
        // into the same "per unit power" units the CHAR phase / ModelTuner use).
        // This used to only populate model.aFwdFallback etc, while the REAL
        // model.aFwd/aStr/aTurn were left to the separate half-power CHAR phase
        // below - meaning the fast, full-power numbers measured here were
        // basically discarded and gains were computed off a slower, less
        // representative estimate. Setting them here means CHAR's redundant
        // measurement (if it disagrees) won't quietly override this with a
        // weaker number.
        model.aFwd  = yA / MAXVA_POWER;
        model.aStr  = xA / MAXVA_POWER;
        model.aTurn = tA / MAXVA_POWER;

        // Rebuild the shared gen config / safe follower now that real robot
        // constants are known, so return-home paths and the search phase both
        // use measured limits instead of the initial fallback guesses.
        fixedGenConfig = buildConfig(SAFE_GAINS, measXV, measYV, measXA, measYA);
        paths = new PathsManager(fixedGenConfig);
        safeFollow = new Follower(fixedGenConfig);

        model.aFwdFallback = measYA;
        model.aStrFallback = measXA;
        model.aTurnFallback = measTurnA;

        maxVaDone = true;
    }

    private static double safeVal(double measured, double fallback) {
        if (Double.isNaN(measured) || measured <= 1) return fallback;
        return measured;
    }

    // ============================= CHARACTERISE (PD gain accel) =============================
    private void beginChar() {
        odometry.queueCommand(odometry.update);
        charStartX = odometry.X();
        charStartY = odometry.Y();
        charStartHeading = odometry.Heading();
        timer.reset();
    }

    private void charPush(boolean skip) {
        odometry.queueCommand(odometry.update);

        if (outsideBox() || skip) {
            stopDrive();
            buildReturnPath();
            state = State.CHAR_RETURN;
            timer.reset();
            return;
        }

        Axis axis = CHAR_SEQ[charIdx];
        switch (axis) {
            case FWD:  driveBase.queueCommand(driveBase.drivePowers(CHAR_POWER, 0, 0)); break;
            case STR:  driveBase.queueCommand(driveBase.drivePowers(0, 0, CHAR_POWER)); break;
            case TURN: driveBase.queueCommand(driveBase.drivePowers(0, CHAR_POWER, 0)); break;
        }

        if (timer.seconds() >= CHAR_TIME) {
            double t = timer.seconds();
            double a;
            if (axis == Axis.TURN) {
                double dTheta = Math.abs(wrap180(odometry.Heading() - charStartHeading));
                a = 2 * dTheta / (CHAR_POWER * t * t);
            } else {
                double d = Math.hypot(odometry.X() - charStartX, odometry.Y() - charStartY);
                a = 2 * d / (CHAR_POWER * t * t);
            }
            if (a > 1.0) record(axis, a);

            // capture motion at the instant of power-cut, then coast to measure decel
            coastV0 = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
            coastW0 = Math.abs(degPerSec());
            stopDrive();
            state = State.CHAR_COAST;
            timer.reset();
        }
    }

    private void charCoast() {
        odometry.queueCommand(odometry.update);
        stopDrive();   // free coast, no power

        if (timer.seconds() >= COAST_TIME) {
            Axis axis = CHAR_SEQ[charIdx];
            double t = timer.seconds();
            if (axis == Axis.TURN) {
                double w1 = Math.abs(degPerSec());
                double dc = (coastW0 - w1) / t;
                if (coastW0 > 30 && dc > 1) { sumDcT += dc; nDcT++; }
            } else {
                double v1 = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
                double dc = (coastV0 - v1) / t;
                if (coastV0 > 15 && dc > 1) {
                    if (axis == Axis.FWD) { sumDcF += dc; nDcF++; }
                    else                  { sumDcS += dc; nDcS++; }
                }
            }
            buildReturnPath();
            state = State.CHAR_RETURN;
            timer.reset();
        }
    }

    private double degPerSec() {
        return odometry.getHVelocity() * 180.0 / Math.PI;
    }

    private void record(Axis axis, double a) {
        switch (axis) {
            case FWD:  sumFwd += a;  nFwd++;  break;
            case STR:  sumStr += a;  nStr++;  break;
            case TURN: sumTurn += a; nTurn++; break;
        }
    }

    private void computeModel() {
        // CHAR measures accel at CHAR_POWER (0.5) - weaker signal than the
        // full-power MAXVA measurement already seeded into model.aFwd/aStr/aTurn
        // in computeMaxVA(). Only use CHAR's numbers as a gap-filler if MAXVA's
        // measurement was missing/unusable (e.g. that axis was skipped), so a
        // real full-power measurement never gets quietly replaced by a weaker one.
        double charFwd  = nFwd  > 0 ? sumFwd  / nFwd  : 0;
        double charStr  = nStr  > 0 ? sumStr  / nStr  : 0;
        double charTurn = nTurn > 0 ? sumTurn / nTurn : 0;

        if (model.aFwd  <= 1) model.aFwd  = charFwd;
        if (model.aStr  <= 1) model.aStr  = charStr;
        if (model.aTurn <= 1) model.aTurn = charTurn;

        // Coast decel: use the SMALLER of forward/strafe (longer coast = safer,
        // never under-predicts drift) for the translation prediction.
        double avgF = nDcF > 0 ? sumDcF / nDcF : DC_TRANS_FALLBACK;
        double avgS = nDcS > 0 ? sumDcS / nDcS : DC_TRANS_FALLBACK;
        dcTrans = Math.max(1, Math.min(avgF, avgS));
        dcTurn  = nDcT > 0 ? Math.max(1, sumDcT / nDcT) : DC_TURN_FALLBACK;
    }

    // ============================= SPEED SEARCH =============================
    private void beginSearchAxis() {
        curTs = (sIdx == 2) ? TS_START_H : TS_START_T;
        bestTs = Double.NaN;
        anyPass = false;
        sIter = 0;
        applyTs(curTs);
        buildTunedFollower();
        searching = true;
        beginPath(PROBE[sIdx]);
        state = State.SEARCH_PROBE;
    }

    private void searchDecision() {
        double err = (sIdx == 0) ? tr.finalErrX : (sIdx == 1) ? tr.finalErrY : tr.finalHdgErr;
        double tol = axisTol(sIdx);
        // Pass = actually ended up within tolerance, and didn't spend several
        // consecutive samples meaningfully past it (real overshoot, not a
        // single noisy/twitchy sample - see sampleMetrics/OVERSHOOT_DEBOUNCE_N).
        // tr.overshoot (raw peak) is intentionally NOT used here anymore.
        boolean pass = !tr.timedOut && !tr.leftBox
                && err <= tol && !sustainedOvershoot;

        if (sIter == 0) {                       // first cycle for this axis = baseline
            firstSettle[sIdx] = tr.settleTime;
            firstErr[sIdx] = err;
        }
        if (pass) {                             // last passing probe = the locked result
            bestSettleA[sIdx] = tr.settleTime;
            bestErrA[sIdx] = err;
            bestTs = curTs; anyPass = true; curTs *= TS_DOWN;
        } else {
            curTs *= TS_UP;
        }
        sIter++;

        boolean done = (anyPass && !pass)          // just failed after a good one
                || sIter >= MAX_ITER
                || curTs < TS_FLOOR || curTs > TS_CEIL;

        if (done) {
            double locked = anyPass ? bestTs : clamp(curTs, TS_FLOOR, TS_CEIL);
            applyTs(locked);
            sIdx++;
            if (sIdx >= 3) {
                buildTunedFollower();
                saveBest();
                searching = false;
                batteryIdx = 0;
                beginPath(BATTERY[0]);
                state = State.VERIFY_RUN;
            } else {
                beginSearchAxis();
            }
        } else {
            applyTs(curTs);
            buildTunedFollower();
            beginPath(PROBE[sIdx]);
            state = State.SEARCH_PROBE;
        }
    }

    private void applyTs(double ts) {
        if (sIdx == 0) model.tsX = ts;
        else if (sIdx == 1) model.tsY = ts;
        else model.tsH = ts;
    }

    private double axisTol(int i) { return i == 0 ? TOL_X : i == 1 ? TOL_Y : TOL_H; }

    private void buildTunedFollower() {
        tunedGains = model.gains();
        follow = new Follower(buildConfig(tunedGains, measXV, measYV, measXA, measYA));
    }

    // ============================= PATH RUN (probe + verify) =============================
    private void beginPath(Item it) {
        odometry.queueCommand(odometry.update);
        double cx = odometry.X(), cy = odometry.Y();
        startX = cx; startY = cy;
        startHeading = wrap360(odometry.Heading());

        double[] last = it.rel[it.rel.length - 1];
        targetX = cx + last[0];
        targetY = cy + last[1];
        double len = Math.hypot(last[0], last[1]);
        dirX = last[0] / len;
        dirY = last[1] / len;
        targetHeading = wrap360(startHeading + it.hdgDelta);

        final Vector2D[] pts = new Vector2D[it.rel.length + 1];
        pts[0] = new Vector2D(cx, cy);
        for (int i = 0; i < it.rel.length; i++) {
            pts[i + 1] = new Vector2D(cx + it.rel[i][0], cy + it.rel[i][1]);
        }
        SectionBuilder[] section = new SectionBuilder[]{ () -> addPts(pts) };
        paths.addNewPath("outPath");
        paths.buildPath(section);

        follow.setPath(paths.returnPath("outPath"));
        follow.usePathHeadings(false);
        follow.holdPositionAtPathEnd(true);

        tr = new TrialResult();
        tr.settleTime = SETTLE_TIMEOUT;
        settled = false;
        overshootStreak = 0;
        sustainedOvershoot = false;
        timer.reset();
    }

    private void runPath(boolean skip) {
        if (outsideBox()) { tr.leftBox = true; finishPath(); return; }
        if (skip) { tr.timedOut = true; finishPath(); return; }

        drive(follow, targetHeading);
        sampleMetrics();

        // "In target" (soonest safe deactivation) requires BOTH:
        //   (a) the robot is ACTUALLY inside the zone right now  - so it can't
        //       claim a fast time while still far away and coasting in, and
        //   (b) the predicted coast rest is ALSO inside the zone - so it isn't so
        //       fast it will shoot straight through and exit the far side.
        // settleTime is therefore the honest time until it is in range and stays.
        double aX = Math.abs(odometry.X() - targetX);
        double aY = Math.abs(odometry.Y() - targetY);
        double aH = Math.abs(wrap180(targetHeading - odometry.Heading()));
        boolean actualInZone = aX <= TOL_X && aY <= TOL_Y && aH <= TOL_H;

        double xv = odometry.getXVelocity(), yv = odometry.getYVelocity();
        double speed = Math.hypot(xv, yv);
        double prX = odometry.X(), prY = odometry.Y();
        if (speed > 0.1) {
            double coast = speed * speed / (2 * dcTrans);
            prX += (xv / speed) * coast;
            prY += (yv / speed) * coast;
        }
        double w = degPerSec();
        double prH = odometry.Heading() + (w * Math.abs(w)) / (2 * dcTurn); // signed
        boolean restInZone = Math.abs(prX - targetX) <= TOL_X
                && Math.abs(prY - targetY) <= TOL_Y
                && Math.abs(wrap180(targetHeading - prH)) <= TOL_H;

        if (!settled && actualInZone && restInZone) {
            settled = true;
            tr.settleTime = timer.seconds();
            // record the ACTUAL error at the deactivation instant (the truth)
            tr.finalErrX = aX;
            tr.finalErrY = aY;
            tr.finalPosErr = Math.hypot(aX, aY);
            tr.finalHdgErr = aH;
            finishPath();
            return;
        }
        if (timer.seconds() > SETTLE_TIMEOUT) {
            tr.timedOut = true;
            tr.settleTime = SETTLE_TIMEOUT;
            captureFinal();
            finishPath();
        }
    }

    private void finishPath() {
        if (!searching) logPath();
        buildReturnPath();
        state = searching ? State.SEARCH_RETURN : State.VERIFY_RETURN;
        timer.reset();
    }

    private void buildReturnPath() {
        final Vector2D[] pts = new Vector2D[]{
                new Vector2D(odometry.X(), odometry.Y()),
                new Vector2D(CX, CY)
        };
        SectionBuilder[] section = new SectionBuilder[]{ () -> addPts(pts) };
        paths.addNewPath("backPath");
        paths.buildPath(section);
        safeFollow.setPath(paths.returnPath("backPath"));
        safeFollow.usePathHeadings(false);
        safeFollow.holdPositionAtPathEnd(true);
    }

    /** @return true when home (or timed out). */
    private boolean returnHome() {
        drive(safeFollow, 0);
        boolean stopped = (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())) < VEL_EPS;
        boolean home = safeFollow.isFinished(8, 8);
        if ((home && stopped) || timer.seconds() > RETURN_TIMEOUT) {
            stopDrive();
            return true;
        }
        return false;
    }

    // ============================= HELPERS =============================
    private void drive(Follower f, double heading) {
        odometry.queueCommand(odometry.update);
        RobotPower cp = f.followPathAuto(heading, odometry.Heading(),
                odometry.X(), odometry.Y(),
                odometry.getXVelocity(), odometry.getYVelocity());
        driveBase.queueCommand(driveBase.drivePowers(cp));
    }

    private void stopDrive() {
        driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
    }

    // Short reverse-power pulse to actively kill velocity when we cut a
    // full-power MAXVA push early (box margin hit, possibly still ramping).
    // A plain power-cut only relies on passive friction/coast, which at max
    // speed can carry the robot well past the margin and off the field.
    private static final double BRAKE_POWER    = 0.6;
    private static final double BRAKE_TIME_SEC = 0.12;

    private void brakeAxis(MvaAxis axis) {
        ElapsedTime bt = new ElapsedTime();
        while (bt.seconds() < BRAKE_TIME_SEC) {
            odometry.queueCommand(odometry.update);
            switch (axis) {
                case FWD:  driveBase.queueCommand(driveBase.drivePowers(-BRAKE_POWER, 0, 0)); break;
                case STR:  driveBase.queueCommand(driveBase.drivePowers(0, 0, -BRAKE_POWER)); break;
                case TURN: driveBase.queueCommand(driveBase.drivePowers(0, -BRAKE_POWER, 0)); break;
            }
        }
        stopDrive();
    }

    private void sampleMetrics() {
        double speed = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        if (speed > tr.peakSpeed) tr.peakSpeed = speed;

        double px = odometry.X() - targetX;
        double py = odometry.Y() - targetY;
        double beyond = px * dirX + py * dirY;
        if (beyond > tr.overshoot) tr.overshoot = beyond;

        // Debounced check: only count it as "real" overshoot if the robot is
        // past target+tolerance for several consecutive samples, not a single
        // noisy/twitchy one.
        double tolAlongAxis = (sIdx == 1) ? TOL_Y : TOL_X; // rough - dominant axis of the probe
        if (beyond > tolAlongAxis * OVER_FRAC) {
            overshootStreak++;
            if (overshootStreak >= OVERSHOOT_DEBOUNCE_N) sustainedOvershoot = true;
        } else {
            overshootStreak = 0;
        }

        double rx = odometry.X() - startX;
        double ry = odometry.Y() - startY;
        double along = rx * dirX + ry * dirY;
        double dev = Math.hypot(rx - along * dirX, ry - along * dirY);
        tr.sumDev += dev;
        tr.devCount++;
        if (dev > tr.maxDev) tr.maxDev = dev;
    }

    private void captureFinal() {
        tr.finalErrX = Math.abs(odometry.X() - targetX);
        tr.finalErrY = Math.abs(odometry.Y() - targetY);
        tr.finalPosErr = Math.hypot(tr.finalErrX, tr.finalErrY);
        tr.finalHdgErr = Math.abs(wrap180(targetHeading - odometry.Heading()));
    }

    private void addPts(Vector2D[] pts) {
        if (pts.length == 2) paths.addPoints(pts[0], pts[1]);
        else if (pts.length == 3) paths.addPoints(pts[0], pts[1], pts[2]);
        else paths.addPoints(pts[0], pts[1], pts[2], pts[3]);
    }

    private boolean outsideBox() {
        double x = odometry.X(), y = odometry.Y();
        return x < BOX_MIN || x > BOX_MAX || y < BOX_MIN || y > BOX_MAX;
    }

    /** Tighter box used only during full-power MAXVA pushes, so power is cut
     *  with enough margin left to coast to a stop before the real wall. */
    private boolean outsideMaxVaBox() {
        double x = odometry.X(), y = odometry.Y();
        return x < BOX_MIN + MAXVA_BOX_MARGIN || x > BOX_MAX - MAXVA_BOX_MARGIN
                || y < BOX_MIN + MAXVA_BOX_MARGIN || y > BOX_MAX - MAXVA_BOX_MARGIN;
    }

    private static RobotConfig buildConfig(double[] p, double maxXV, double maxYV, double maxXA, double maxYA) {
        return new RobotConfig()
                .setXOnPathPD(p[0], p[1])
                .setYOnPathPD(p[2], p[3])
                .setXLastAdjustmentPD(p[4], p[5])
                .setYLastAdjustmentPD(p[6], p[7])
                .setFastHeadingPD(p[8], p[9])
                .setSlowHeadingPD(p[10], p[11])
                .setRobotConstants(maxXV, maxYV, maxXA, maxYA);
    }

    private static double wrap360(double a) { a %= 360; if (a < 0) a += 360; return a; }
    private static double wrap180(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
    private static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

    // ============================= TELEMETRY / LOGGING =============================
    private void drawTelemetry() {
        telemetry.addData("state", state);

        if (!maxVaDone) {
            telemetry.addData("maxVA axis", (mvaIdx < MVA_SEQ.length ? MVA_SEQ[mvaIdx].name() : "-"));
            telemetry.addData("maxVA v (live)", String.format("%.0f", mvaLastV));
        } else {
            telemetry.addData("measured maxV X/Y/Turn", String.format("%.0f / %.0f / %.0f cm|deg per s",
                    measXV, measYV, measTurnV));
            telemetry.addData("measured maxA X/Y/Turn", String.format("%.0f / %.0f / %.0f",
                    measXA, measYA, measTurnA));
            telemetry.addData("plateau reached X/Y/Turn", String.format("%b / %b / %b",
                    mvaPlateauReached[MvaAxis.STR.ordinal()],
                    mvaPlateauReached[MvaAxis.FWD.ordinal()],
                    mvaPlateauReached[MvaAxis.TURN.ordinal()]));
        }

        telemetry.addData("A fwd/str/turn", String.format("%.0f / %.0f / %.0f",
                nFwd > 0 ? sumFwd / nFwd : 0, nStr > 0 ? sumStr / nStr : 0, nTurn > 0 ? sumTurn / nTurn : 0));
        telemetry.addData("coast dc trans/turn", String.format("%.0f / %.0f", dcTrans, dcTurn));
        if (state == State.SEARCH_PROBE || state == State.SEARCH_RETURN)
            telemetry.addData("search", axisName(sIdx) + "  ts=" + String.format("%.2f", curTs)
                    + "  iter=" + sIter);
        if (state == State.VERIFY_RUN || state == State.VERIFY_RETURN)
            telemetry.addData("verify path", (batteryIdx + 1) + "/" + BATTERY.length);
        telemetry.addData("locked ts X/Y/H", String.format("%.2f / %.2f / %.2f", model.tsX, model.tsY, model.tsH));
        telemetry.addData("pos", String.format("%.0f,%.0f  h%.0f", odometry.X(), odometry.Y(), odometry.Heading()));
        telemetry.addData("thisPath avgDev/maxDev", String.format("%.1f / %.1f cm", tr.avgDev(), tr.maxDev));
        if (tunedGains != null) telemetry.addLine("gains: " + fmt(tunedGains));

        // improvement from the first (slow) cycle to the locked result, per axis
        for (int i = 0; i < 3; i++) {
            if (Double.isNaN(firstSettle[i])) continue;
            telemetry.addData("improve " + axisName(i), improveStr(i));
        }
        if (state == State.FINISHED) telemetry.addLine(">>> DONE - saved to tune_best.txt <<<");
    }

    private static String axisName(int i) { return i == 0 ? "X" : i == 1 ? "Y" : "H"; }

    /** "1.42s->0.61s  57% faster, err 4.8->2.1" for one axis, or n/a if it never passed. */
    private String improveStr(int i) {
        if (Double.isNaN(bestSettleA[i])) {
            return String.format("baseline %.2fs (never met tol)", firstSettle[i]);
        }
        double pct = firstSettle[i] > 0 ? (firstSettle[i] - bestSettleA[i]) / firstSettle[i] * 100 : 0;
        return String.format("%.2fs->%.2fs  %.0f%% faster, err %.1f->%.1f",
                firstSettle[i], bestSettleA[i], pct, firstErr[i], bestErrA[i]);
    }

    private static String fmt(double[] p) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < p.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format("%.5f", p[i]));
        }
        return sb.toString();
    }

    private void openLog() {
        try {
            File dir = new File("/sdcard/FIRST/datalogs");
            if (!dir.exists()) dir.mkdirs();
            pathCsv = new FileWriter(new File(dir, "tune_paths.csv"), false);
            pathCsv.write("path," + TrialResult.csvHeader() + "\n");
            pathCsv.flush();
        } catch (IOException e) {
            telemetry.addData("LOG ERR", e.getMessage());
            pathCsv = null;
        }
    }

    private void logPath() {
        if (pathCsv == null) return;
        try {
            pathCsv.write(String.format("%d,%s\n", batteryIdx + 1, tr.csv()));
            pathCsv.flush();
        } catch (IOException ignored) {}
    }

    private void saveBest() {
        try {
            File dir = new File("/sdcard/FIRST/datalogs");
            if (!dir.exists()) dir.mkdirs();
            FileWriter w = new FileWriter(new File(dir, "tune_best.txt"), false);
            double[] b = tunedGains != null ? tunedGains : SAFE_GAINS;
            w.write("// PathTuner (model based) result\n");
            w.write(String.format("// tolerances  X=%.1f Y=%.1f cm  H=%.1f deg\n", TOL_X, TOL_Y, TOL_H));
            w.write(String.format("// motor profile  %.0f rpm, %.1fmm wheel, 4 motors -> theoretical maxV=%.0f cm/s\n",
                    MOTOR_RPM, WHEEL_DIAM_MM, THEORETICAL_MAX_V));
            w.write(String.format("// measured maxV  X(strafe)=%.0f  Y(fwd)=%.0f  Turn=%.0f  [plateau reached: %b/%b/%b]\n",
                    measXV, measYV, measTurnV,
                    mvaPlateauReached[MvaAxis.STR.ordinal()],
                    mvaPlateauReached[MvaAxis.FWD.ordinal()],
                    mvaPlateauReached[MvaAxis.TURN.ordinal()]));
            w.write(String.format("// measured maxA  X(strafe)=%.0f  Y(fwd)=%.0f  Turn=%.0f\n",
                    measXA, measYA, measTurnA));
            w.write(String.format("// measured A (PD-gain char, per unit power)  fwd=%.0f  strafe=%.0f  turn=%.0f\n",
                    model.aFwd, model.aStr, model.aTurn));
            w.write(String.format("// coast decel  trans=%.0f cm/s^2  turn=%.0f deg/s^2  (deactivation prediction)\n",
                    dcTrans, dcTurn));
            w.write(String.format("// locked ts  X=%.2f  Y=%.2f  H=%.2f  (zeta=%.2f)\n",
                    model.tsX, model.tsY, model.tsH, ZETA));
            w.write("// improvement (first slow cycle -> locked):\n");
            w.write("//   X: " + improveStr(0) + "\n");
            w.write("//   Y: " + improveStr(1) + "\n");
            w.write("//   H: " + improveStr(2) + "\n");
            w.write("new RobotConfig()\n");
            w.write(String.format("    .setXOnPathPD(%.5f, %.5f)\n", b[0], b[1]));
            w.write(String.format("    .setYOnPathPD(%.5f, %.5f)\n", b[2], b[3]));
            w.write(String.format("    .setXLastAdjustmentPD(%.5f, %.5f)\n", b[4], b[5]));
            w.write(String.format("    .setYLastAdjustmentPD(%.5f, %.5f)\n", b[6], b[7]));
            w.write(String.format("    .setFastHeadingPD(%.5f, %.5f)\n", b[8], b[9]));
            w.write(String.format("    .setSlowHeadingPD(%.5f, %.5f)\n", b[10], b[11]));
            w.write(String.format("    .setRobotConstants(%.0f, %.0f, %.0f, %.0f);\n",
                    measXV, measYV, measXA, measYA));
            w.close();
        } catch (IOException ignored) {}
    }
}