package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp
public class auto_tele extends OpModeEX {

    // ── Pathing & PIDs ───────────────────────────────────────────────────────
    pathsManager paths = new pathsManager(new RobotConfig(
            0.02, 0.004, 0.016, 0.005, 0.08, 0.004, 0.09, 0.004,
            0.01, 0.0005, 0.012, 0.002, 170, 193, 270, 920));
    follower follow = new follower(new RobotConfig(
            0.02, 0.004, 0.016, 0.005, 0.08, 0.004, 0.09, 0.004,
            0.01, 0.0005, 0.012, 0.002, 180, 193, 900, 1020));

    // headingPID  – heading hold / gate hold
    // xPID        – horizontal ball tracking (camera xPosCm centred on 0)
    // yPID        – forward/back X-position hold during vision collect
    PIDController headingPID = new PIDController(0.03,  0, 0.003);
    PIDController xPID       = new PIDController(0.06,  0, 0.003);
    PIDController yPID       = new PIDController(0.012, 0, 0.003);

    // ── Vision ───────────────────────────────────────────────────────────────
    private VisionPortal visionPortal;
    private LocalVision  processor;
    private FtcDashboard dashboard;

    // ═════════════════════════════════════════════════════════════════════════
    // Enums
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * What the robot should do AFTER the current shooting window closes.
     * Default: stopAutomaticControl (driver controls everything).
     *
     * Controls:
     *   Left  bumper  → backCycleBlue
     *   Right bumper  → backCycleRed
     *   D-pad up      → closeGateCycle      (extraGate style, nearby gate)
     *   D-pad down    → farGateCycle        (gateFromBack style, approaches via back waypoint)
     *   D-pad left    → frontToBackGateCycle (gate → back shoot zone)
     *   D-pad right   → stopAutomaticControl
     */
    enum NextStep {
        stopAutomaticControl,
        backCycleBlue,
        backCycleRed,
        closeGateCycle,
        farGateCycle,
        frontToBackGateCycle
    }

    enum CycleState {
        idle,
        backCollecting,
        drivingToShoot,
        waitingToShoot,
        drivingToGate,
        atGate,
        drivingToShootFromGate
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Field positions  (tune as needed)
    // ═════════════════════════════════════════════════════════════════════════

    // Back-side shoot zone (used by back cycles and far/frontToBack gate cycles)
    static final double SHOOT_BACK_X  = 120, SHOOT_BACK_Y  = 335;
    // Front-side shoot zone (used after closeGateCycle)
    static final double SHOOT_FRONT_X = 130, SHOOT_FRONT_Y = 170;
    // Back-sweep collect path: mid → end
    static final double COLLECT_MID_X = 140, COLLECT_MID_Y = 340;
    static final double COLLECT_END_X = 45,  COLLECT_END_Y = 340;

    // X held by yPID during vision collect
    static final double COLLECT_HOLD_X = 40;
    // Gate stop position
    static final double GATE_X = 30, GATE_Y = 218;
    static final double BACK_GATE_X = 31, BACK_GATE_Y = 212;

    // Far-gate approach waypoint (back area)
    static final double FAR_WP_X = 134, FAR_WP_Y = 325;
    // Heading the robot locks to while sitting at the gate
    static final double GATE_HEADING_BLUE = 296;

    // ═════════════════════════════════════════════════════════════════════════
    // State variables
    // ═════════════════════════════════════════════════════════════════════════

    NextStep   nextStep   = NextStep.stopAutomaticControl;
    CycleState cycleState = CycleState.idle;

    boolean autoCycleActive = false;
    boolean visionCollect   = false;
    boolean pathing         = false;
    boolean collectDone     = false;
    boolean ballsInIntake   = false;
    boolean ballShot        = false;
    boolean PIDAtGate       = false;
    boolean intakeOff       = false;

    double targetHeading = 270;
    double shootWait     = 700;
    double IntakeOffWait = 200;

    // ── Timers ───────────────────────────────────────────────────────────────
    ElapsedTime shootTime       = new ElapsedTime();
    ElapsedTime maxWait         = new ElapsedTime();
    ElapsedTime ballCollectWait = new ElapsedTime();
    ElapsedTime ballshot        = new ElapsedTime();
    ElapsedTime gateWait        = new ElapsedTime();
    ElapsedTime intakeoff       = new ElapsedTime();
    ElapsedTime telemetryTimer  = new ElapsedTime();

    // ═════════════════════════════════════════════════════════════════════════
    // Utility helpers
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Flip a blue-alliance heading target to red-alliance.
     * Red = (360 – blue) % 360, applied only when nextStep is backCycleRed.
     */
    private double h(double blueTarget) {
        return (nextStep == NextStep.backCycleRed)
                ? (360.0 - blueTarget) % 360.0
                : blueTarget;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * Build and activate a 2-point adaptive path starting from the robot's
     * current position.  Sets pathing = true.
     */
    private void setAdaptivePath(String name, double tx, double ty) {
        final double sx = odometry.X(), sy = odometry.Y();
        final sectionBuilder[] seg = {
                () -> paths.addPoints(new Vector2D(sx, sy), new Vector2D(tx, ty))
        };
        paths.addNewPath(name);
        paths.buildPath(seg);
        follow.setPath(paths.returnPath(name));
        pathing = true;
    }

    /**
     * Build and activate a 3-point adaptive path
     * (current position → waypoint → target).  Sets pathing = true.
     */
    private void setAdaptivePath(String name, double wx, double wy, double tx, double ty) {
        final double sx = odometry.X(), sy = odometry.Y();
        final sectionBuilder[] seg = {
                () -> paths.addPoints(new Vector2D(sx, sy),
                        new Vector2D(wx, wy),
                        new Vector2D(tx, ty))
        };
        paths.addNewPath(name);
        paths.buildPath(seg);
        follow.setPath(paths.returnPath(name));
        pathing = true;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Cycle management
    // ═════════════════════════════════════════════════════════════════════════

    /** Called when driver selects a new NextStep. Starts immediately if idle. */
    private void launchCycleIfIdle() {
        if (!autoCycleActive || cycleState == CycleState.idle) {
            launchCycle();
        }
        // If currently cycling, the new nextStep will take effect after the
        // current shooting window closes (in tickWaitingToShoot).
    }

    /** Begin the cycle currently specified by nextStep. */
    private void launchCycle() {
        if (nextStep == NextStep.stopAutomaticControl) {
            autoCycleActive   = false;
            cycleState        = CycleState.idle;
            pathing           = false;
            visionCollect     = false;
            PIDAtGate         = false;
            turret.stopTurret = false;
            return;
        }
        autoCycleActive = true;
        collectDone     = false;
        ballsInIntake   = false;
        ballShot        = false;

        switch (nextStep) {
            case backCycleBlue:
            case backCycleRed:
                beginBackCollect();
                break;
            case closeGateCycle:
            case farGateCycle:
            case frontToBackGateCycle:
                beginGateApproach();
                break;
        }
    }

    // ── Cycle entry points ────────────────────────────────────────────────────

    private void beginBackCollect() {
        cycleState    = CycleState.backCollecting;
        visionCollect = true;
        pathing       = false;   // visionCollect drive-output branch takes over
        collectDone   = false;
        ballsInIntake = false;
        targetHeading = h(270);
        intake.block  = true;
        intake.InTake = true;
        ballCollectWait.reset();
        maxWait.reset();

        // Build adaptive sweep path for follower internal progress tracking.
        // Drive output is PID-based, but followPathAuto must be called each loop
        // so the follower can advance its internal t-parameter.
        final double sx = odometry.X(), sy = odometry.Y();
        final sectionBuilder[] seg = {
                () -> paths.addPoints(new Vector2D(sx,          sy),
                        new Vector2D(COLLECT_MID_X, COLLECT_MID_Y),
                        new Vector2D(COLLECT_END_X, COLLECT_END_Y))
        };
        paths.addNewPath("autoCollect");
        paths.buildPath(seg);
        follow.setPath(paths.returnPath("autoCollect"));
    }

    private void beginGateApproach() {
        cycleState        = CycleState.drivingToGate;
        PIDAtGate         = false;
        turret.stopTurret = true;
        intake.block      = true;
        intake.InTake     = true;
        follow.usePathHeadings(false);

        switch (nextStep) {
            case closeGateCycle:
                // Direct path from current position straight to the gate
                targetHeading = h(297);
                setAdaptivePath("gateApproach", GATE_X, GATE_Y);
                break;
            case farGateCycle:
                // Swing via back-area waypoint first (mirrors gateFromBack in auto)
                targetHeading = h(308);
                setAdaptivePath("gateApproach", FAR_WP_X, FAR_WP_Y, BACK_GATE_X, BACK_GATE_Y);
                break;
            case frontToBackGateCycle:
                // Direct to gate from front; after gate it returns to back zone
                targetHeading = h(297);
                setAdaptivePath("gateApproach", GATE_X, GATE_Y);
                break;
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Cycle state machine
    // ═════════════════════════════════════════════════════════════════════════

    private void runCycleStateMachine() {
        switch (cycleState) {
            case backCollecting:         tickBackCollecting();          break;
            case drivingToShoot:         tickDrivingToShoot();          break;
            case waitingToShoot:         tickWaitingToShoot();          break;
            case drivingToGate:          tickDrivingToGate();           break;
            case atGate:                 tickAtGate();                  break;
            case drivingToShootFromGate: tickDrivingToShootFromGate(); break;
            case idle:                                                   break;
        }
    }

    // ── Back collect ─────────────────────────────────────────────────────────

    private void tickBackCollecting() {
        // Ball detection
        if (intake.ballCount > 2 && !ballsInIntake) {
            ballCollectWait.reset();
            ballsInIntake = true;
        } else if (ballsInIntake && intake.ballCount < 2) {
            ballsInIntake = false;
        }

        // Exit conditions (same guarded logic as the fixed auto)
        if (!collectDone) {
            boolean endOfSweep = maxWait.milliseconds() > 400 && odometry.X() < COLLECT_END_X ;
            boolean timeout    = maxWait.milliseconds() > 1500;
            boolean gotBalls   = ballsInIntake && ballCollectWait.milliseconds() > 250;
            boolean leftZone   = odometry.Y() < 245;

            if (endOfSweep || timeout || gotBalls || leftZone) {
                collectDone   = true;
                visionCollect = false;
            }
        }

        if (collectDone) {
            visionCollect = false;
            cycleState    = CycleState.drivingToShoot;
            targetHeading = h(270);
            intakeoff.reset();
            intakeOff = true;
            // Adaptive path from current position to back shoot zone
            setAdaptivePath("autoShoot", SHOOT_BACK_X, SHOOT_BACK_Y);
        }
    }

    // ── Drive to shoot ───────────────────────────────────────────────────────

    private void tickDrivingToShoot() {
        double vel = Math.abs(odometry.getXVelocity())
                + Math.abs(odometry.getYVelocity())
                + Math.abs(odometry.getHVelocity());
        if (follow.isFinished(15, 15) && vel < 5) {
            pathing       = false;
            cycleState    = CycleState.waitingToShoot;
            intake.block  = false;
            intake.InTake = true;
            ballShot      = false;
            shootTime.reset();
        }
    }

    // ── Waiting to shoot ─────────────────────────────────────────────────────

    private void tickWaitingToShoot() {
        intake.InTake = true;
        intake.block  = false;
        if (shootTime.milliseconds() > shootWait) {
            // Shoot done — start next cycle (or idle if nextStep == stop)
            launchCycle();
        }
    }

    // ── Drive to gate ────────────────────────────────────────────────────────

    private void tickDrivingToGate() {
        // Ease heading into gate angle as robot approaches
        if (odometry.X() < 82) {
            targetHeading = h(GATE_HEADING_BLUE);
        }
        if (follow.isFinished(6, 6)) {
            pathing    = false;
            PIDAtGate  = true;
            cycleState = CycleState.atGate;
            gateWait.reset();
        }
    }

    // ── At gate ─────────────────────────────────────────────────────────────

    private void tickAtGate() {
        // Heading hold is applied in handleDriveOutput() while PIDAtGate == true
        intake.block  = true;
        intake.InTake = true;

        if (gateWait.milliseconds() > 1100) {
            PIDAtGate         = false;
            turret.stopTurret = false;
            cycleState        = CycleState.drivingToShootFromGate;
            intakeoff.reset();
            intakeOff       = true;

            switch (nextStep) {
                case closeGateCycle:
                    // Close gate cycle shoots from the front zone
                    targetHeading = h(270);
                    setAdaptivePath("gateShoot", SHOOT_FRONT_X, SHOOT_FRONT_Y);
                    break;
                case farGateCycle:
                case frontToBackGateCycle:
                default:
                    // Far / front-to-back cycles return to back shoot zone
                    targetHeading = h(308);
                    setAdaptivePath("gateShoot", FAR_WP_X, FAR_WP_Y, SHOOT_BACK_X, SHOOT_BACK_Y);
                    break;
            }
        }
    }

    // ── Drive to shoot from gate ──────────────────────────────────────────────

    private void tickDrivingToShootFromGate() {
        double vel = Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                + Math.abs(odometry.getHVelocity() * 2);
        if (follow.isFinished(10, 10) && vel < 6) {
            pathing       = false;
            cycleState    = CycleState.waitingToShoot;
            intake.block  = false;
            intake.InTake = true;
            ballShot      = false;
            shootTime.reset();
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // OpMode lifecycle
    // ═════════════════════════════════════════════════════════════════════════

    @Override
    public void initEX() {
        follow.setHeadingOffset(90);

        turret.toggle            = false;
        turret.mapOfset          = 40;
        turret.turrofset         = -2.5;
        turret.TURRET_COMP_FACTOR = 0;

        Apriltag.limelight.pipelineSwitch(0);

        processor = new LocalVision(LocalVision.TargetColor.BOTH);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(processor);
        visionPortal = builder.build();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 4);
    }

    @Override
    public void start() {
        Apriltag.limelight.start();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Main loop
    // ═════════════════════════════════════════════════════════════════════════

    @Override
    public void loopEX() {

        // ── Subsystem feeds ──
        turret.robotX           = odometry.X();
        turret.robotY           = odometry.Y();
        turret.robotHeading     = odometry.normiliased();
        turret.robotXVelo       = odometry.getXVelocity();
        turret.robotYVelo       = odometry.getYVelocity();
        turret.robotHeadingVelo = odometry.getHVelocity();

        // ── Intake overflow guard ──
        if (!intake.InTake && intake.ballCount > 2) intake.reverse = true;

        // ── Delayed intake shutoff ──t
        if (intakeOff&& intakeoff.milliseconds() > IntakeOffWait) {
            intake.InTake = false;
            intakeOff     = false;
        }
        if (odometry.Y() >240){
            turret.mapOfset          = 80;
        }else {
            turret.mapOfset          = 40;
        }

        // ── Ball-shot edge detection ──
        if (intake.ballCount < 1 && !ballShot) {
            ballShot = true;
            ballshot.reset();
        }

        // ── Hood compensation ──
        turret.hoodCompensation = (intake.ballCount > 1) ? -2 : 0;


        // ── Intake during auto shooting windows (not collecting) ──
        if (autoCycleActive && cycleState == CycleState.waitingToShoot) {
            intake.block  = false;
            intake.InTake = true;
        }

        // ── Controls ──
        handleControls();

        // ── Auto cycle state machine ──
        if (autoCycleActive) {
            runCycleStateMachine();
        }

        // ── Drive output ──
        handleDriveOutput();

        // ── Telemetry ──
        if (telemetryTimer.milliseconds() > 100) {
            telemetryTimer.reset();
            telemetry.addData("nextStep",        nextStep);
            telemetry.addData("cycleState",      cycleState);
            telemetry.addData("autoCycleActive", autoCycleActive);
            telemetry.addData("X",               odometry.X());
            telemetry.addData("Y",               odometry.Y());
            telemetry.addData("heading",         odometry.Heading());
            telemetry.addData("balls",           intake.ballCount);
            telemetry.addData("visionAngle",     processor.hAngleDeg);
            telemetry.addData("xPosCm",          processor.xPosCm);
            telemetry.addData("pathing",         pathing);
            telemetry.addData("visionCollect",   visionCollect);
            telemetry.addData("PIDAtGate",       PIDAtGate);
            telemetry.addData("shootTime",       shootTime.milliseconds());
            telemetry.update();
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Controls
    // ═════════════════════════════════════════════════════════════════════════

    private void handleControls() {

        // ── Limelight position reset (hold left stick down) ──
        if (gamepad1.left_stick_y < -0.3) {
            Apriltag.enabled = true;
            if (Apriltag.getH() != 0 && Apriltag.getH() != 180) {
                gamepad1.rumble(200);
                odometry.odo.setPosX(-Apriltag.getX(), DistanceUnit.CM);
                odometry.odo.setPosY( Apriltag.getY(), DistanceUnit.CM);
                odometry.odo.setHeading(-Apriltag.getH(), AngleUnit.DEGREES);
                turret.toggle = true;
            }
        } else {
            Apriltag.enabled = false;
        }

        // ── NextStep selection (all edge-triggered) ──

        // Left bumper  → back cycle BLUE
        if (!lastGamepad1.left_bumper && currentGamepad1.left_bumper) {
            nextStep = NextStep.backCycleBlue;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // Right bumper → back cycle RED
        if (!lastGamepad1.right_bumper && currentGamepad1.right_bumper) {
            nextStep = NextStep.backCycleRed;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad up    → close gate cycle  (extraGate style)
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up) {
            nextStep = NextStep.closeGateCycle;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad down  → far gate cycle  (gateFromBack style)
        if (!lastGamepad1.dpad_down && currentGamepad1.dpad_down) {
            nextStep = NextStep.farGateCycle;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad left  → front-to-back gate cycle
        if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left) {
            nextStep = NextStep.frontToBackGateCycle;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad right → stop auto control
        if (!lastGamepad1.dpad_right && currentGamepad1.dpad_right) {
            nextStep          = NextStep.stopAutomaticControl;
            autoCycleActive   = false;
            cycleState        = CycleState.idle;
            pathing           = false;
            visionCollect     = false;
            PIDAtGate         = false;
            turret.stopTurret = false;
            gamepad1.rumble(400);
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Drive output
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * When the driver pushes the right stick past 0.3 magnitude:
     *   - Auto powers are clamped to ±0.5 so the driver can steer around obstacles.
     *   - Driver stick input is ADDED on top of the clamped auto power.
     * When the stick is at rest, pure auto powers are applied.
     * In idle (no auto) the robot is fully driver-controlled.
     */
    private void handleDriveOutput() {
        double stickMag      = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
        boolean driverSteer  = stickMag > 0.3;
        double driverFwd     = -gamepad1.right_stick_y;
        double driverStr     = -gamepad1.right_stick_x;
        double driverPivot   = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5;

        if (pathing) {
            // ── Path-following mode ──
            odometry.queueCommand(odometry.update);
            RobotPower auto = follow.followPathAuto(targetHeading, odometry.Heading(),
                    odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

            if (driverSteer) {
                driveBase.queueCommand(driveBase.drivePowers(
                        clamp(auto.getHorizontal(), -0.5, 0.5) + driverFwd,
                        auto.getPivot()                        + driverPivot,
                        clamp(auto.getVertical(),   -0.5, 0.5) + driverStr));
            } else {
                driveBase.queueCommand(driveBase.drivePowers(auto));
            }

        } else if (visionCollect) {
            // ── Vision-collect mode ──
            // followPathAuto is still called each loop to advance internal path progress
            // (required for follow.isFinished and heading tracking), but the returned
            // RobotPower is not used for motors — PID-based output drives the robot instead.
            odometry.queueCommand(odometry.update);
            follow.followPathAuto(targetHeading, odometry.Heading(),
                    odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

            // yPID holds the robot at COLLECT_HOLD_X (forward/back axis)
            // xPID strafes the robot to centre the detected ball
            double autoFwd   = -yPID.calculate(COLLECT_HOLD_X, odometry.X());
            double autoPivot =  headingPID.calculate(odometry.Heading() - targetHeading);
            double autoStr   = -xPID.calculate(processor.xPosCm);

            if (driverSteer) {
                driveBase.queueCommand(driveBase.drivePowers(
                        clamp(autoFwd,  -0.5, 0.5) + driverFwd,
                        autoPivot                  + driverPivot,
                        clamp(autoStr,  -0.5, 0.5) + driverStr));
            } else {
                driveBase.queueCommand(driveBase.drivePowers(autoFwd, autoPivot, autoStr));
            }

        } else if (PIDAtGate) {
            // ── Gate heading-hold mode ──
            double gateTarget = h(GATE_HEADING_BLUE);
            driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - gateTarget), 0);

        } else {
            // ── Pure driver control ──
            driveBase.drivePowers(
                    -gamepad1.right_stick_y,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * 0.7,
                    -gamepad1.right_stick_x);
        }
    }
}