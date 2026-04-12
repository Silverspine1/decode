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
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Intake;
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
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2,  0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 700));
    follower follow = new follower(new RobotConfig(
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2,  0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 700));

    PIDController headingPID = new PIDController(0.009, 0, 0.003);
    PIDController xPID       = new PIDController(0.06,  0, 0.003);
    PIDController yPID       = new PIDController(0.013, 0, 0.003);

    // ── Vision ───────────────────────────────────────────────────────────────
    private VisionPortal visionPortal;
    private LocalVision  processor;
    private FtcDashboard dashboard;

    // ═════════════════════════════════════════════════════════════════════════
    // Enums
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * What the robot should do after the current shooting window closes.
     *
     * Controls:
     *   Left  bumper  → backCycleBlue
     *   Right bumper  → backCycleRed
     *   D-pad up      → closeGateCycle
     *   D-pad down    → farGateCycle
     *   D-pad left    → frontToBackGateCycle
     *   D-pad right   → stopAutomaticControl  (also acts as manual override)
     *   Y button      → toggle manual override mid-cycle
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
        limelightReset,     // turn to 315, wait for reset or timeout
        drivingToGate,
        atGate,
        drivingToShootFromGate
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Field positions  (tune as needed)
    // ═════════════════════════════════════════════════════════════════════════
    static final double SHOOT_BACK_X    = 120,  SHOOT_BACK_Y    = 335;
    static final double SHOOT_FRONT_X   = 130,  SHOOT_FRONT_Y   = 170;
    static final double COLLECT_HOLD_X  = 40;
    static final double GATE_X          = 55,   GATE_Y          = 218;
    static final double BACK_GATE_X     = 55,   BACK_GATE_Y     = 212;
    static final double FAR_WP_X        = 134,  FAR_WP_Y        = 325;
    static final double GATE_HEADING_BLUE = 296;
    static final double LIMELIGHT_FACE_HEADING = 315; // global heading to face goal for reset

    // ═════════════════════════════════════════════════════════════════════════
    // State variables
    // ═════════════════════════════════════════════════════════════════════════
    NextStep   nextStep   = NextStep.stopAutomaticControl;
    CycleState cycleState = CycleState.idle;

    boolean autoCycleActive    = false;
    boolean manualOverride     = false;   // Y button toggles fully manual mid-cycle
    boolean visionCollect      = false;
    boolean pathing            = false;
    boolean collectDone        = false;
    boolean ballsInIntake      = false;
    boolean ballShot           = false;
    boolean PIDAtGate          = false;
    boolean intakeOff          = false;
    boolean intakePathSelected = false;
    boolean HoldHeadingWhileShooting = false;

    double targetHeading  = 270;
    double shootWait      = 700;
    double IntakeOffWait  = 200;
    double gateAngle      = 295;
    double gateTolX       = 10, gateTolY = 8;
    double gateTurnX      = 112;
    double gateTime       = 1200;

    // ── Timers ───────────────────────────────────────────────────────────────
    ElapsedTime shootTime         = new ElapsedTime();
    ElapsedTime maxWait           = new ElapsedTime();
    ElapsedTime ballCollectWait   = new ElapsedTime();
    ElapsedTime ballshot          = new ElapsedTime();
    ElapsedTime gateWait          = new ElapsedTime();
    ElapsedTime intakeoff         = new ElapsedTime();
    ElapsedTime telemetryTimer    = new ElapsedTime();
    ElapsedTime limelightAgeClock = new ElapsedTime(); // counts up since last good reset
    ElapsedTime resetWaitTimer    = new ElapsedTime(); // timeout inside limelightReset state

    // ═════════════════════════════════════════════════════════════════════════
    // Utility helpers
    // ═════════════════════════════════════════════════════════════════════════

    /** Mirror a blue heading to red when running backCycleRed. */
    private double h(double blueTarget) {
        return (nextStep == NextStep.backCycleRed)
                ? (360.0 - blueTarget) % 360.0
                : blueTarget;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Build a 2-point adaptive path from current position and activate it. */
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

    /** Build a 3-point adaptive path (current → waypoint → target). */
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

    private void launchCycleIfIdle() {
        if (!autoCycleActive || cycleState == CycleState.idle) {
            launchCycle();
        }
    }

    private void launchCycle() {
        if (nextStep == NextStep.stopAutomaticControl) {
            stopAll();
            return;
        }
        autoCycleActive    = true;
        manualOverride     = false;
        collectDone        = false;
        ballsInIntake      = false;
        ballShot           = false;
        intakePathSelected = false;

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

    private void stopAll() {
        autoCycleActive          = false;
        manualOverride           = false;
        cycleState               = CycleState.idle;
        pathing                  = false;
        visionCollect            = false;
        PIDAtGate                = false;
        HoldHeadingWhileShooting = false;
        turret.stopTurret        = false;
        intake.holdUp            = false;
        intake.poz               = Intake.intakePoz.normalPoz;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Back-collect entry  (mirrors back_solo visionCollect / backCollect logic)
    // ═════════════════════════════════════════════════════════════════════════

    private void beginBackCollect() {
        cycleState         = CycleState.backCollecting;
        visionCollect      = true;
        pathing            = false;
        collectDone        = false;
        ballsInIntake      = false;
        intakePathSelected = false;
        targetHeading      = h(270);
        intake.block       = true;
        intake.InTake      = true;
        ballCollectWait.reset();
        maxWait.reset();
        // Path will be built lazily once the vision angle is known (first loop tick)
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Gate-cycle entry
    // ═════════════════════════════════════════════════════════════════════════

    private void beginGateApproach() {
        cycleState        = CycleState.drivingToGate;
        PIDAtGate         = false;
        turret.stopTurret = true;
        intake.block      = true;
        intake.InTake     = true;
        follow.usePathHeadings(false);

        switch (nextStep) {
            case closeGateCycle:
                targetHeading = h(297);
                setAdaptivePath("gateApproach", GATE_X, GATE_Y);
                break;
            case farGateCycle:
                targetHeading = h(308);
                setAdaptivePath("gateApproach", FAR_WP_X, FAR_WP_Y, BACK_GATE_X, BACK_GATE_Y);
                break;
            case frontToBackGateCycle:
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
            case backCollecting:         tickBackCollecting();         break;
            case drivingToShoot:         tickDrivingToShoot();         break;
            case waitingToShoot:         tickWaitingToShoot();         break;
            case limelightReset:         tickLimelightReset();         break;
            case drivingToGate:          tickDrivingToGate();          break;
            case atGate:                 tickAtGate();                 break;
            case drivingToShootFromGate: tickDrivingToShootFromGate(); break;
            case idle:                                                  break;
        }
    }

    // ── Back collect ─────────────────────────────────────────────────────────
    // Mirrors back_solo visionCollect logic: angle-based path selection, same
    // exit conditions (isFinished or 1400ms timeout).

    private void tickBackCollecting() {
        intake.block  = true;
        intake.InTake = true;

        // ── Path selection (first tick, once vision angle is stable) ──
        if (!intakePathSelected) {
            if (processor.hAngleDeg > 16) {
                final double sx = odometry.X(), sy = odometry.Y();
                final sectionBuilder[] p = { () -> paths.addPoints(
                        new Vector2D(sx, sy), new Vector2D(50, 293)) };
                paths.addNewPath("bc_p3"); paths.buildPath(p);
                follow.setPath(paths.returnPath("bc_p3"));
                follow.setHeadingOffset(90); follow.usePathHeadings(true);
                follow.setHeadingLookAheadDistance(100);
                intake.block  = false;
                pathing       = true;
                maxWait.reset();
                intakePathSelected = true;

            } else if (processor.hAngleDeg < 1) {
                final double sx = odometry.X(), sy = odometry.Y();
                final sectionBuilder[] p = { () -> paths.addPoints(
                        new Vector2D(sx, sy),
                        new Vector2D(140, 340),
                        new Vector2D(50, 340)) };
                paths.addNewPath("bc_p1"); paths.buildPath(p);
                follow.setPath(paths.returnPath("bc_p1"));
                follow.setHeadingOffset(90); follow.usePathHeadings(true);
                follow.setHeadingLookAheadDistance(100);
                intake.block  = true;
                pathing       = true;
                maxWait.reset();
                intakePathSelected = true;

            } else { // 1 – 16 deg
                final double sx = odometry.X(), sy = odometry.Y();
                final sectionBuilder[] p = { () -> paths.addPoints(
                        new Vector2D(sx, sy), new Vector2D(50, 311)) };
                paths.addNewPath("bc_p2"); paths.buildPath(p);
                follow.setPath(paths.returnPath("bc_p2"));
                follow.setHeadingOffset(90); follow.usePathHeadings(true);
                follow.setHeadingLookAheadDistance(100);
                intake.block  = true;
                pathing       = true;
                maxWait.reset();
                intakePathSelected = true;
            }
        }

        // ── Ball tracking ──
        if (intake.ballCount > 2 && !ballsInIntake) {
            ballCollectWait.reset();
            ballsInIntake = true;
        } else if (ballsInIntake && intake.ballCount < 2) {
            ballsInIntake = false;
        }

        // ── Exit ──
        if (!collectDone) {
            boolean pathDone  = follow.isFinished(5, 10);
            boolean timeout   = maxWait.milliseconds() > 1400;
            if (pathDone || timeout) collectDone = true;
        }

        if (collectDone) {
            visionCollect      = false;
            pathing            = false;
            intakePathSelected = false;
            cycleState         = CycleState.drivingToShoot;
            targetHeading      = h(270);
            intakeoff.reset();
            intakeOff = true;
            follow.setHeadingOffset(-90);
            // Adaptive path from wherever the robot stopped
            setAdaptivePath("autoShoot", SHOOT_BACK_X, SHOOT_BACK_Y);
        }
    }

    // ── Drive to shoot ───────────────────────────────────────────────────────

    private void tickDrivingToShoot() {
        // Mirror back_solo: hold heading while robot is near destination
        if (follow.isFinished(17, 35)) {
            follow.usePathHeadings(false);
            targetHeading = h(278);
        }

        double vel = Math.abs(odometry.getXVelocity())
                + Math.abs(odometry.getYVelocity())
                + Math.abs(odometry.getHVelocity());

        if (follow.isFinished(22, 22) && vel < 32) {
            pathing                  = false;
            HoldHeadingWhileShooting = true;
            intake.block             = false;
            intake.InTake            = true;
            ballShot                 = false;
            shootTime.reset();
            cycleState = CycleState.waitingToShoot;
        }
    }

    // ── Waiting to shoot ─────────────────────────────────────────────────────

    private void tickWaitingToShoot() {
        intake.InTake = true;
        intake.block  = false;

        if (shootTime.milliseconds() > shootWait) {
            HoldHeadingWhileShooting = false;
            intake.poz               = Intake.intakePoz.normalPoz;
            intake.holdUp            = false;

            // ── Limelight staleness check ──
            // If it has been > 30 s since the last good limelight reset,
            // insert a limelightReset sub-state before continuing.
            if (limelightAgeClock.seconds() > 30) {
                cycleState = CycleState.limelightReset;
                targetHeading = LIMELIGHT_FACE_HEADING;  // face goal globally
                resetWaitTimer.reset();
                Apriltag.enabled = true;
            } else {
                launchCycle(); // straight into next cycle
            }
        }
    }

    // ── Limelight reset sub-state ─────────────────────────────────────────────
    // Turns to 315, waits until a good fix arrives OR 1500 ms pass, then continues.

    private void tickLimelightReset() {
        // Hold the heading toward the goal so the limelight can see the target
        driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - LIMELIGHT_FACE_HEADING), 0);

        boolean gotFix   = Apriltag.getH() != 0 && Apriltag.getH() != 180;
        boolean timedOut = resetWaitTimer.milliseconds() > 1500;

        if (gotFix) {
            // Accept and apply the fix
            odometry.odo.setPosX(-Apriltag.getX(), DistanceUnit.CM);
            odometry.odo.setPosY( Apriltag.getY(), DistanceUnit.CM);
            odometry.odo.setHeading(-Apriltag.getH(), AngleUnit.DEGREES);
            limelightAgeClock.reset(); // mark reset time
            gamepad1.rumble(200);
        }

        if (gotFix || timedOut) {
            Apriltag.enabled = false;
            launchCycle();   // continue with the next cycle now
        }
    }

    // ── Drive to gate ────────────────────────────────────────────────────────

    private void tickDrivingToGate() {
        if (odometry.X() < gateTurnX) {
            targetHeading = h(gateAngle);
        }
        if (follow.isFinished(gateTolX, gateTolY)) {
            pathing    = false;
            PIDAtGate  = true;
            cycleState = CycleState.atGate;
            gateWait.reset();
        }
    }

    // ── At gate ─────────────────────────────────────────────────────────────

    private void tickAtGate() {
        intake.block  = true;
        intake.InTake = true;
        intake.poz    = Intake.intakePoz.gatePoz;

        boolean ballsFull = ballsInIntake && ballCollectWait.milliseconds() > 180
                && gateWait.milliseconds() > 300;
        boolean timeout   = gateWait.milliseconds() > gateTime;

        if (ballsFull || timeout) {
            PIDAtGate         = false;
            turret.stopTurret = false;
            cycleState        = CycleState.drivingToShootFromGate;
            intakeoff.reset();
            intakeOff       = true;
            intake.poz      = Intake.intakePoz.normalPoz;

            switch (nextStep) {
                case closeGateCycle:
                    targetHeading = h(270);
                    setAdaptivePath("gateShoot", SHOOT_FRONT_X, SHOOT_FRONT_Y);
                    break;
                case farGateCycle:
                case frontToBackGateCycle:
                default:
                    targetHeading = h(308);
                    setAdaptivePath("gateShoot", FAR_WP_X, FAR_WP_Y, SHOOT_BACK_X, SHOOT_BACK_Y);
                    break;
            }
        }
    }

    // ── Drive to shoot from gate ──────────────────────────────────────────────

    private void tickDrivingToShootFromGate() {
        if (odometry.Y() > 280) targetHeading = h(278);

        // Raise intake before the shoot zone (mirrors back_solo logic)
        if (odometry.X() > 82 && shootTime.milliseconds() > 500) {
            intake.poz    = Intake.intakePoz.up;
            intake.InTake = false;
            intake.holdUp = true;
        }

        double vel = Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                + Math.abs(odometry.getHVelocity() * 2);

        if (follow.isFinished(15, 30) && vel < 32) {
            pathing                  = false;
            HoldHeadingWhileShooting = true;
            intake.block             = false;
            intake.InTake            = true;
            ballShot                 = false;
            shootTime.reset();
            cycleState = CycleState.waitingToShoot;
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // OpMode lifecycle
    // ═════════════════════════════════════════════════════════════════════════

    @Override
    public void initEX() {
        follow.setHeadingOffset(90);
        turret.toggle             = false;
        turret.mapOfset           = 40;
        turret.turrofset          = -2.5;
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

        limelightAgeClock.reset(); // assume fresh at match start
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

        // ── Map offset zone ──
        turret.mapOfset = (odometry.Y() > 240) ? 80 : 40;

        // ── Intake overflow guard ──
        if (!intake.InTake && intake.ballCount > 2) intake.reverse = true;

        // ── Delayed intake shutoff ──
        if (intakeOff && intakeoff.milliseconds() > 600) {
            intake.InTake = false;
            intakeOff     = false;
        } else if (intake.ballCount < 3 && !intakeOff) {
            intakeoff.reset();
        }

        // ── Ball-shot edge detection ──
        if (intake.ballCount < 1 && !ballShot) {
            ballShot = true;
            ballshot.reset();
        }

        // ── Global ball-in-intake tracking (used by gate state) ──
        if (intake.ballCount > 2 && !ballsInIntake) {
            ballCollectWait.reset();
            ballsInIntake = true;
        } else if (ballsInIntake && intake.ballCount < 2) {
            ballsInIntake = false;
        }

        // ── Hood compensation ──
        turret.hoodCompensation = (intake.ballCount > 1) ? -2 : 0;

        // ── Controls ──
        handleControls();

        // ── Limelight background poll (only when not in reset state) ──
        if (cycleState != CycleState.limelightReset && odometry.getHVelocity() < 2) {
            if (Apriltag.enabled) {
                boolean gotFix = Apriltag.getH() != 0 && Apriltag.getH() != 180;
                if (gotFix) {
                    odometry.odo.setPosX(-Apriltag.getX(), DistanceUnit.CM);
                    odometry.odo.setPosY( Apriltag.getY(), DistanceUnit.CM);
                    odometry.odo.setHeading(-Apriltag.getH(), AngleUnit.DEGREES);
                    limelightAgeClock.reset();
                    turret.toggle = true;
                    gamepad1.rumble(200);
                }
            }
        }

        // ── Waiting-to-shoot intake assist ──
        if (autoCycleActive && !manualOverride && cycleState == CycleState.waitingToShoot) {
            intake.block  = false;
            intake.InTake = true;
        }

        // ── Auto cycle state machine ──
        if (autoCycleActive && !manualOverride) {
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
            telemetry.addData("manualOverride",  manualOverride);
            telemetry.addData("X",               odometry.X());
            telemetry.addData("Y",               odometry.Y());
            telemetry.addData("heading",         odometry.Heading());
            telemetry.addData("balls",           intake.ballCount);
            telemetry.addData("visionAngle",     processor.hAngleDeg);
            telemetry.addData("pathing",         pathing);
            telemetry.addData("visionCollect",   visionCollect);
            telemetry.addData("PIDAtGate",       PIDAtGate);
            telemetry.addData("shootTime ms",    shootTime.milliseconds());
            telemetry.addData("limeStaleness s", limelightAgeClock.seconds());
            telemetry.update();
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Controls
    // ═════════════════════════════════════════════════════════════════════════

    private void handleControls() {

        // ── Y: toggle manual override (can be used mid-cycle to regain control) ──
        if (!lastGamepad1.y && currentGamepad1.y) {
            manualOverride = !manualOverride;
            if (manualOverride) {
                // Freeze auto motion but keep state so it can resume if toggled back
                pathing                  = false;
                visionCollect            = false;
                PIDAtGate                = false;
                HoldHeadingWhileShooting = false;
                gamepad1.rumble(600);
            } else {
                gamepad1.rumble(200);
            }
        }

        // ── Limelight position reset (hold left stick down) ──
        if (gamepad1.left_stick_y < -0.3) {
            Apriltag.enabled = true;
        } else {
            Apriltag.enabled = false;
        }

        // ── NextStep selection (edge-triggered; ignored during manual override) ──

        // Left bumper  → back cycle BLUE
        if (!lastGamepad1.left_bumper && currentGamepad1.left_bumper) {
            nextStep       = NextStep.backCycleBlue;
            manualOverride = false;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // Right bumper → back cycle RED
        if (!lastGamepad1.right_bumper && currentGamepad1.right_bumper) {
            nextStep       = NextStep.backCycleRed;
            manualOverride = false;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad up    → close gate cycle
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up) {
            nextStep       = NextStep.closeGateCycle;
            manualOverride = false;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad down  → far gate cycle
        if (!lastGamepad1.dpad_down && currentGamepad1.dpad_down) {
            nextStep       = NextStep.farGateCycle;
            manualOverride = false;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad left  → front-to-back gate cycle
        if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left) {
            nextStep       = NextStep.frontToBackGateCycle;
            manualOverride = false;
            gamepad1.rumble(150);
            launchCycleIfIdle();
        }
        // D-pad right → stop auto control entirely
        if (!lastGamepad1.dpad_right && currentGamepad1.dpad_right) {
            nextStep = NextStep.stopAutomaticControl;
            stopAll();
            gamepad1.rumble(500);
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Drive output
    // ═════════════════════════════════════════════════════════════════════════

    private void handleDriveOutput() {
        double stickMag    = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
        boolean driverSteer = stickMag > 0.3;
        double driverFwd   = -gamepad1.right_stick_y;
        double driverStr   = -gamepad1.right_stick_x;
        double driverPivot = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5;

        // limelightReset sub-state drives its own output inside tickLimelightReset();
        // return early to avoid overriding it.
        if (cycleState == CycleState.limelightReset) return;

        if (!manualOverride && pathing) {
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

        } else if (!manualOverride && visionCollect) {
            // ── Vision-collect mode ──
            odometry.queueCommand(odometry.update);
            follow.followPathAuto(targetHeading, odometry.Heading(),
                    odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

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

        } else if (!manualOverride && PIDAtGate) {
            // ── Gate heading-hold mode ──
            driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - h(gateAngle)), 0);

        } else if (!manualOverride && HoldHeadingWhileShooting) {
            // ── Heading hold at shoot zone ──
            driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - h(270)), 0);

        } else {
            // ── Pure driver control (manual override OR idle) ──
            driveBase.drivePowers(
                    -gamepad1.right_stick_y,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * 0.7,
                    -gamepad1.right_stick_x);
        }
    }
}