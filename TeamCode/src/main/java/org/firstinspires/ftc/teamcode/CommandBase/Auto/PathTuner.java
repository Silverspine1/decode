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
    private static final double TOL_X = 6.0;   // cm - allowed stop error, strafe/X
    private static final double TOL_Y = 6.0;   // cm - allowed stop error, forward/Y
    private static final double TOL_H = 5.0;   // deg - allowed stop error, heading
    // ===================================================================

    // Damping is fixed - critical, so there is no overshoot to trade off.
    private static final double ZETA = 1.0;

    // Speed search bounds/steps (settling time, seconds). Smaller ts = faster.
    private static final double TS_START_T = 0.80;  // translation start (safe/slow)
    private static final double TS_START_H = 0.65;  // heading start
    private static final double TS_FLOOR   = 0.20;  // fastest allowed
    private static final double TS_CEIL    = 1.40;  // slowest allowed
    private static final double TS_DOWN    = 0.82;  // shrink ts when a probe passes
    private static final double TS_UP      = 1.22;  // grow ts when a probe fails
    private static final double OVER_FRAC  = 0.5;   // overshoot budget = frac * tol
    private static final int    MAX_ITER   = 7;     // probes per axis cap

    // Characterisation step.
    private static final double CHAR_POWER = 0.5;
    private static final double CHAR_TIME  = 0.40;
    // After each push we cut power and coast to measure the UNPOWERED
    // deceleration - this is what decides the soonest point the follower can be
    // switched off without the robot drifting back out of the target zone.
    private static final double COAST_TIME = 0.18;   // s of free coast to sample
    private static final double DC_TRANS_FALLBACK = 150; // cm/s^2 if unmeasured
    private static final double DC_TURN_FALLBACK  = 200; // deg/s^2 if unmeasured

    private static final double SETTLE_TIMEOUT = 3.5;
    private static final double RETURN_TIMEOUT = 3.5;
    private static final double VEL_EPS        = 5.0;

    // Field / safety
    private static final double CX = 180, CY = 180;
    private static final double BOX_MIN = 55, BOX_MAX = 305;

    // Fixed speed limits for Phase A (seed values). Also A fallbacks.
    private static final double MAX_XV = 130, MAX_YV = 181, MAX_XA = 650, MAX_YA = 700;

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
    private enum State { IDLE, CHAR_PUSH, CHAR_COAST, CHAR_RETURN, SEARCH_PROBE, SEARCH_RETURN, VERIFY_RUN, VERIFY_RETURN, FINISHED }
    private State state = State.IDLE;

    private final RobotConfig fixedGenConfig = buildConfig(SAFE_GAINS);
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

    private final ElapsedTime timer = new ElapsedTime();
    private FileWriter pathCsv;

    // ============================= LIFECYCLE =============================
    @Override
    public void initEX() {
        odometry.startPosition((int) CX, (int) CY, 0);
        driveBase.tele = false;
        driveBase.speed = 1;

        paths = new PathsManager(fixedGenConfig);
        safeFollow = new Follower(fixedGenConfig);

        model.zeta = ZETA;
        model.tsX = TS_START_T;
        model.tsY = TS_START_T;
        model.tsH = TS_START_H;
        model.aFwdFallback = MAX_YA;
        model.aStrFallback = MAX_XA;
        model.aTurnFallback = 300;

        telemetry.addLine("PathTuner (model based). Robot at CENTRE, facing 0.");
        telemetry.addData("tolerances", String.format("X=%.1f Y=%.1f cm  H=%.1f deg", TOL_X, TOL_Y, TOL_H));
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
                    charIdx = 0;
                    beginChar();
                    state = State.CHAR_PUSH;
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

    // ============================= CHARACTERISE =============================
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
        model.aFwd  = nFwd  > 0 ? sumFwd  / nFwd  : 0;
        model.aStr  = nStr  > 0 ? sumStr  / nStr  : 0;
        model.aTurn = nTurn > 0 ? sumTurn / nTurn : 0;

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
        boolean pass = !tr.timedOut && !tr.leftBox
                && err <= tol && tr.overshoot <= OVER_FRAC * tol;

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
        follow = new Follower(buildConfig(tunedGains));
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

    private void sampleMetrics() {
        double speed = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        if (speed > tr.peakSpeed) tr.peakSpeed = speed;

        double px = odometry.X() - targetX;
        double py = odometry.Y() - targetY;
        double beyond = px * dirX + py * dirY;
        if (beyond > tr.overshoot) tr.overshoot = beyond;

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

    private static RobotConfig buildConfig(double[] p) {
        return new RobotConfig()
                .setXOnPathPD(p[0], p[1])
                .setYOnPathPD(p[2], p[3])
                .setXLastAdjustmentPD(p[4], p[5])
                .setYLastAdjustmentPD(p[6], p[7])
                .setFastHeadingPD(p[8], p[9])
                .setSlowHeadingPD(p[10], p[11])
                .setRobotConstants(MAX_XV, MAX_YV, MAX_XA, MAX_YA);
    }

    private static double wrap360(double a) { a %= 360; if (a < 0) a += 360; return a; }
    private static double wrap180(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
    private static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

    // ============================= TELEMETRY / LOGGING =============================
    private void drawTelemetry() {
        telemetry.addData("state", state);
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
            double[] b = tunedGains;
            w.write("// PathTuner (model based) result\n");
            w.write(String.format("// tolerances  X=%.1f Y=%.1f cm  H=%.1f deg\n", TOL_X, TOL_Y, TOL_H));
            w.write(String.format("// measured A  fwd=%.0f  strafe=%.0f  turn=%.0f  (per unit power)\n",
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
                    MAX_XV, MAX_YV, MAX_XA, MAX_YA));
            w.close();
        } catch (IOException ignored) {}
    }
}
