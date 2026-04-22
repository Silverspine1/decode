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
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp

public class red_Tele extends OpModeEX {
    private VisionPortal visionPortal;
    private LocalVision processor;
    private FtcDashboard dashboard;
    pathsManager paths = new pathsManager(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01,
            0.0005, 0.012, 0.002, 200, 273, 270, 320));

    follower follow = new follower(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01, 0.0005,
            0.012, 0.002, 200, 273, 270, 320));

    double heading;

    enum backState {
        backCollect,
        backShoot
    }

    backState backstate = backState.backCollect;
    boolean captureLastHeading = false;
    boolean brake = false;
    boolean autoCycles = false;
    double targetHeading = 270;
    boolean rest = false;
    double targetHood = 25;
    boolean blue = true;
    boolean togle = false;
    boolean visionCollect = false;
    boolean pathing = false;
    boolean built = false;
    boolean collectDone = false;

    // ── Back-cycle tracking ─────────────────────────────────────────────────
    boolean isBackCycling       = false;
    boolean backCycleShootReady = false;
    int     backCycles          = 0;
    // ────────────────────────────────────────────────────────────────────────

    // ── Vision-collect geometry & wall config (ported from auto) ────────────
    double frontOffset        =  8.0;   // intake face offset from robot centre along heading
    double sideOffsetDeg      =  0.0;   // sideways camera bias (+ = right)
    double robotHalfWidth     = 17.5;   // half robot width parallel to the wall
    double wallSafetyMargin   = 10.0;   // clearance beyond robotHalfWidth
    double wallBuffer         = 20.0;   // field units from safe limit where avoidance activates
    double maxWallAvoidPower  =  0.40;  // max Y power for wall avoidance
    double wallVelGain        =  0.018; // velocity feedforward gain
    double wallLookAheadSecs  =  0.12;  // seconds ahead to predict wall approach
    final  double WALL_Y      = 270.0;  // field Y of the wall
    double targetPos          = 44.0;   // field X the intake face should reach
    boolean ballsInIntake     = false;
    boolean driveBackToShoot  = false;
    ElapsedTime ballCollectWait = new ElapsedTime();
    ElapsedTime waitAfterCollected = new ElapsedTime();
    // ────────────────────────────────────────────────────────────────────────

    double lastBallCount = 0;
    double currentBallCount = 0;
    PIDController headingPID = new PIDController(0.013, 0, 0.0032);
    PIDController xVisionPID = new PIDController(0.03,  0, 0.003);
    PIDController correctiveXFinalAdjustment = new PIDController(0.03, 0.0, 0.004);
    PIDController correctiveYFinalAdjustment = new PIDController(0.03, 0.0, 0.004);
    ElapsedTime shooterOffWait = new ElapsedTime();
    ElapsedTime rumble = new ElapsedTime();
    ElapsedTime maxToGetToShoot = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime ejectTimer = new ElapsedTime();

    // Shoot-return paths (used after vision collect)
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(56, 340), new Vector2D(115, 320)),
    };
    private final sectionBuilder[] S2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(56, 320), new Vector2D(115, 320)),
    };
    private final sectionBuilder[] S3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(56, 284), new Vector2D(115, 320)),
    };

    // Variables for velocity and acceleration tracking
    private double lastXVelo = 0;
    private double lastYVelo = 0;
    private double lastHVelo = 0;
    double shootWait = 700;


    private ElapsedTime veloTimer = new ElapsedTime();
    ElapsedTime Timer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();

    ElapsedTime shootTime = new ElapsedTime();

    // Lists to store recent max values (averaging window)
    private List<Double> recentMaxVelos = new ArrayList<>();
    private List<Double> recentMaxAccels = new ArrayList<>();
    private final int AVERAGING_WINDOW = 10; // Average over 10 samples

    private double currentMaxVelo = 0;
    private double currentMaxAccel = 0;
    double baseOffset = 2;
    double baseMapOffset = 0;
    boolean shooting = false;
    PIDController XPID = new PIDController(0.03, 0.0, 0.004);
    PIDController YPID = new PIDController(0.03, 0.0, 0.004);

    // ── Clean cancel: zeroes every back-cycle flag in one place ────────────
    private void cancelBackCycle() {
        isBackCycling       = false;
        backCycleShootReady = false;
        visionCollect       = false;
        collectDone         = false;
        pathing             = false;
        driveBackToShoot    = false;
        ballsInIntake       = false;
        backstate           = backState.backCollect;
        intake.block        = false;
        driveBase.drivePowers(0, 0, 0);
    }
    // ────────────────────────────────────────────────────────────────────────

    @Override
    public void initEX() {


        turret.toggle = false;
        Apriltag.limelight.pipelineSwitch(0);

        processor = new LocalVision(LocalVision.TargetColor.BOTH);
        paths.addNewPath("S1");
        paths.buildPath(S1);
        paths.addNewPath("S2");
        paths.buildPath(S2);
        paths.addNewPath("S3");
        paths.buildPath(S3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(processor);

        visionPortal = builder.build();
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 4);

        veloTimer.reset();
        driveBase.tele = true;
        turret.targetX = 360;
        turret.StopSWM = true;


    }

    @Override
    public void stop() {
        if (visionPortal != null)
            visionPortal.close();
    }

    @Override
    public void start() {
        Apriltag.limelight.start();
        veloTimer.reset();
    }

    @Override
    public void loopEX() {
        lastBallCount = currentBallCount;
        currentBallCount = intake.ballCount;
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normiliased();

        turret.robotXVelo = odometry.getXVelocity();
        turret.robotYVelo = odometry.getYVelocity();
        turret.robotHeadingVelo = odometry.getHVelocity();

        driveBase.fieldVelY = odometry.getYVelocity();
        driveBase.fieldVelX = odometry.getXVelocity();


        // ── Track when 3 balls are loaded (mirrors auto) ─────────────────────
        if (intake.ballCount > 2 && !ballsInIntake) {
            ballCollectWait.reset();
            ballsInIntake = true;
        } else if (ballsInIntake && intake.ballCount < 2) {
            ballsInIntake = false;
        }
        // ────────────────────────────────────────────────────────────────────

        // Manual drive only when no automated back-cycle is running
        if (!isBackCycling ) {
            if (!rest && !shooting) {
                driveBase.drivePowers(-gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.7, -gamepad1.right_stick_x);
            } else if(!shooting){
                driveBase.driveFieldCentric(-gamepad1.right_stick_y, -gamepad1.right_stick_x, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.7, odometry.Heading() - 90);
            }
        }

        calculateVelocityAndAcceleration();

        if (intake.ballCount > 0) {
            shooterOffWait.reset();
        }

        if (intake.ballCount > 2 && rumble.milliseconds() > 1500) {
            gamepad1.rumble(300);
            rumble.reset();
        }
        if (gamepad2.y) {
            double currentHeading = odometry.Heading();

            double xDist = 84 - odometry.X();
            double yDist = 273 - odometry.Y();

            double robotRelativeXError = yDist * Math.sin(Math.toRadians(currentHeading))
                    + xDist * Math.cos(Math.toRadians(currentHeading));
            double robotRelativeYError = yDist * Math.cos(Math.toRadians(currentHeading))
                    - xDist * Math.sin(Math.toRadians(currentHeading));

            double xPower = XPID.calculate(robotRelativeXError);
            double yPower = YPID.calculate(robotRelativeYError);

            PathingPower correctivePower = new PathingPower();
            correctivePower.set(xPower, yPower);

            driveBase.drivePowers(
                    -correctivePower.getHorizontal(),
                    headingPID.calculate(currentHeading - 90),
                    -correctivePower.getVertical());
        }



        // ────────────────────────────────────────────────────────────────────
        if (odometry.Y() > 260) {
            turret.mapOfset = 50 + baseMapOffset;
            turret.turrofset = -4+ baseOffset;
        }else {
            turret.mapOfset = 30 + baseMapOffset;
            turret.turrofset = -4 + baseOffset;
        }
        if ( !shooting && gamepad2.right_bumper && turret.turretInRange && Math.abs( Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())) + Math.abs(odometry.getHVelocity() * 6) < 40){
            intake.InTake = true;
            intake.block = false;
            shooting = true;
            shootTime.reset();
            driveBase.drivePowers(0, 0, 0);


        } else if (shootTime.milliseconds() > 300) {
            shooting = false;
        }


        if (gamepad1.right_bumper && !shooting) {
            if (!turret.manuel) {
                intake.block = true;
            }
            intake.InTake = true;

        } else if (gamepad1.left_bumper && turret.diff < 270 && turret.turretInRange && !shooting) {
            intake.InTake = true;
            intake.block = false;

        } else if (turret.intakeTime && !shooting) {
            intake.InTake = true;
        } else if (!currentGamepad1.left_bumper && !currentGamepad1.right_bumper && !isBackCycling && !shooting) {
            intake.InTake = false;
        }

        if (!lastGamepad2.dpad_left && currentGamepad2.dpad_left) {
            baseOffset -= 1;
        }
        if (!lastGamepad2.dpad_right && currentGamepad2.dpad_right) {
            baseOffset += 1;
        }
        if (!lastGamepad2.dpad_up && currentGamepad2.dpad_up ){
            baseMapOffset += 10;
        }
        if (!lastGamepad2.dpad_down && currentGamepad2.dpad_down ){
            baseMapOffset -= 10;
        }

        if (gamepad1.left_stick_y < -0.3 || gamepad2.left_stick_y < -0.3 && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) + Math.abs(odometry.getHVelocity() ) < 5) {
            Apriltag.enabled = true;
            if (Apriltag.getH() != 0 && Apriltag.getH() != 180) {
                togle = true;
                gamepad1.rumble(200);
                rest = true;
                turret.reset = true;
                intake.reset = true;

                odometry.odo.setPosX(Apriltag.getX(), DistanceUnit.CM);
                odometry.odo.setPosY(-Apriltag.getY(), DistanceUnit.CM);
                odometry.odo.setHeading(-Apriltag.getH(), AngleUnit.DEGREES);
            }
        } else {
            Apriltag.enabled = false;
        }

        // ── Auto-style vision collect (direct drive, no pre-built paths) ────────
        if (visionCollect) {
            odometry.queueCommand(odometry.update);

            // Exit conditions — mirrors the auto exactly
            if (    (maxWait.milliseconds() > 400  && odometry.X() < targetPos - 5)
                    ||  maxWait.milliseconds() > 1100
                    ||  ballsInIntake
                    ||  odometry.Y() < 245) {
                if (!driveBackToShoot) {
                    driveBackToShoot = true;
                    collectDone      = true;
                    waitAfterCollected.reset();
                }
            }

            intake.block  = true;
            intake.InTake = true;

            // Front offset: shift X target so the intake face (not robot centre) reaches targetPos
            double headingRad     = Math.toRadians(odometry.normilised);
            double adjustedTarget = targetPos + frontOffset * Math.sin(headingRad);

            // Vision steer: heading PID drives the robot to centre the ball horizontally
            double visionSteer = headingPID.calculate(-processor.hAngleDeg - sideOffsetDeg);

            // Wall avoidance (field Y) — identical to auto
            double safeY       = WALL_Y - robotHalfWidth - wallSafetyMargin;
            double curY        = odometry.Y();
            double yVel        = odometry.getYVelocity();
            double predictedY  = curY + yVel * wallLookAheadSecs;
            double distToSafe  = safeY - predictedY;

            double wallAvoidPower = 0.0;
            if (distToSafe < wallBuffer) {
                double penetration = Math.max(0.0, wallBuffer - distToSafe);
                double posTerm     = Math.pow(penetration / wallBuffer, 2) * maxWallAvoidPower;
                double velTerm     = Math.max(0.0, yVel) * wallVelGain;
                wallAvoidPower     = -(posTerm + velTerm);
                wallAvoidPower     = Math.max(wallAvoidPower, -maxWallAvoidPower);
            }

            // X = approach/retreat to ball   Y-rot = vision steer   Z = wall avoid
            driveBase.queueCommand(driveBase.drivePowers(
                    -xVisionPID.calculate(targetPos, odometry.X()),
                    visionSteer,
                    wallAvoidPower));
        }
        // ────────────────────────────────────────────────────────────────────

        if (gamepad1.y) {
            driveBase.base2.setPosition(1);
            driveBase.base1.setPosition(0);
        } else if (gamepad1.a) {
            driveBase.base2.setPosition(0);
            driveBase.base1.setPosition(1);
        } else {
            driveBase.base2.setPosition(0.5);
            driveBase.base1.setPosition(0.5);
            liftTimer.reset();
        }
        if (liftTimer.milliseconds() > 600){
            turret.lift = true;
        }

        if (!lastGamepad2.a && currentGamepad2.a && turret.toggle || !lastGamepad1.dpad_up && currentGamepad1.dpad_up && turret.toggle) {
            turret.toggle = false;
            gamepad1.rumble(800);
        } else if (!lastGamepad2.a && currentGamepad2.a && !turret.toggle || !lastGamepad1.dpad_up && currentGamepad1.dpad_up && !turret.toggle) {
            turret.toggle = true;
            gamepad1.rumble(800);
        }

        if (!lastGamepad1.x && currentGamepad1.x && intake.poz == Intake.intakePoz.normalPoz) {
            intake.poz = Intake.intakePoz.gatePoz;
        } else if (!lastGamepad1.x && currentGamepad1.x && intake.poz == Intake.intakePoz.gatePoz) {
            intake.poz = Intake.intakePoz.normalPoz;
        }

        // ── Path following drive output ──────────────────────────────────────
        if (pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(),
                    odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }
        // ────────────────────────────────────────────────────────────────────

        if (Timer.milliseconds() > 100) {
            Timer.reset();
            telemetry.addData("Intake Rpm", intake.intakeRPM);
            telemetry.addData("in zone", turret.inZone);
            telemetry.addData("odometry x", odometry.X());
            telemetry.addData("odometry y", odometry.Y());
            telemetry.addData("Heading", odometry.Heading());
            telemetry.addData("distance", turret.distance);
            telemetry.addData("diff", turret.diff);
            telemetry.addData("hood", targetHood);
            telemetry.addData("rpm", turret.rpm);
            telemetry.addData("limeX", Apriltag.getX());
            telemetry.addData("limeY", Apriltag.getY());
            telemetry.addData("limeH", Apriltag.getH());
            telemetry.addData("ball x ", processor.xPosCm);
            telemetry.addData("block ", intake.block);
            telemetry.addData("ball", intake.ballCount);
            telemetry.addData("distance velo", turret.distanceVelocity);
            telemetry.addData("vision angle", processor.hAngleDeg);
            telemetry.addData("turretservang ", turret.turretAngle / turret.gearRatio + 180);
            telemetry.addData("Max Velocity", "%.1f", getAverageMaxVelo());
            telemetry.addData("Max Accel", "%.1f", getAverageMaxAccel());
            telemetry.addData("predicted angle", getAverageMaxAccel());

            // ── Back-cycle telemetry ─────────────────────────────────────────
            telemetry.addData("Back Cycles", backCycles);
            telemetry.addData("Back Cycling", isBackCycling ? (backstate == backState.backCollect ? "COLLECTING" : "SHOOTING") : "OFF");
            // ────────────────────────────────────────────────────────────────
            telemetry.update();
        }

    }

    private void calculateVelocityAndAcceleration() {
        double xVelo = odometry.getXVelocity();
        double yVelo = odometry.getYVelocity();

        double currentVelo = Math.sqrt(xVelo * xVelo + yVelo * yVelo);

        if (currentVelo > currentMaxVelo) {
            currentMaxVelo = currentVelo;

            recentMaxVelos.add(currentMaxVelo);
            if (recentMaxVelos.size() > AVERAGING_WINDOW) {
                recentMaxVelos.remove(0);
            }
        }

        double dt = veloTimer.seconds();
        veloTimer.reset();

        if (dt > 0.001) {
            double xAccel = (xVelo - lastXVelo) / dt;
            double yAccel = (yVelo - lastYVelo) / dt;

            double currentAccel = Math.sqrt(xAccel * xAccel + yAccel * yAccel);

            if (currentAccel > currentMaxAccel) {
                currentMaxAccel = currentAccel;

                recentMaxAccels.add(currentMaxAccel);
                if (recentMaxAccels.size() > AVERAGING_WINDOW) {
                    recentMaxAccels.remove(0);
                }
            }
        }

        lastXVelo = xVelo;
        lastYVelo = yVelo;
        lastHVelo = odometry.getHVelocity();
    }

    private double getAverageMaxVelo() {
        if (recentMaxVelos.isEmpty()) return 0;
        double sum = 0;
        for (double velo : recentMaxVelos) sum += velo;
        return sum / recentMaxVelos.size();
    }

    private double getAverageMaxAccel() {
        if (recentMaxAccels.isEmpty()) return 0;
        double sum = 0;
        for (double accel : recentMaxAccels) sum += accel;
        return sum / recentMaxAccels.size();
    }
}