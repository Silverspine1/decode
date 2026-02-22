package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.source.tree.IfTree;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Auto.back_In_A_Case;
import org.firstinspires.ftc.teamcode.CommandBase.Auto.back_In_A_Case_Hype_auto;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
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

public class blue_Tele extends OpModeEX {
    private VisionPortal visionPortal;
    private LocalVision processor;
    pathsManager paths = new pathsManager(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01,
            0.0005, 0.012, 0.002, 200, 273, 270, 320));

    follower follow = new follower(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01, 0.0005,
            0.012, 0.002, 200, 273, 270, 320));

    double heading;

    enum backState {
        backCollect,
        backShoot
    }

    enum shootPath {
        S1,
        S2,
        S3,
    }

    shootPath shootState = shootPath.S2;

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
    boolean intakeOff = false;

    boolean intakePathSelected = false;
    double lastBallCount = 0;
    double currentBallCount = 0;
    PIDController headingPID = new PIDController(0.013, 0, 0.0032);
    PIDController correctiveXFinalAdjustment = new PIDController(0.03, 0.0, 0.004);
    PIDController correctiveYFinalAdjustment = new PIDController(0.03, 0.0, 0.004);
    ElapsedTime shooterOffWait = new ElapsedTime();
    ElapsedTime rumble = new ElapsedTime();
    ElapsedTime maxToGetToShoot = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime ejectTimer = new ElapsedTime();

    private final sectionBuilder[] p1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(115, 317), new Vector2D(140, 340), new Vector2D(60, 340)),
    };
    private final sectionBuilder[] p2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(115, 317), new Vector2D(60, 317)),
    };
    private final sectionBuilder[] p3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(115, 317), new Vector2D(60, 284)),
    };
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(56, 340), new Vector2D(115, 320)),
    };
    private final sectionBuilder[] S2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(56, 320), new Vector2D(115, 320)),
    };
    private final sectionBuilder[] S3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(56, 284), new Vector2D(115, 320)),
    };
    private final sectionBuilder[] pEsh = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(115, 317), new Vector2D(65, 284)),
    };
    Vector2D park = new Vector2D(294, 267);

    // Variables for velocity and acceleration tracking
    private double lastXVelo = 0;
    private double lastYVelo = 0;
    private double lastHVelo = 0;
    double shootWait = 700;

    private ElapsedTime veloTimer = new ElapsedTime();
    ElapsedTime Timer = new ElapsedTime();
    ElapsedTime shootTime = new ElapsedTime();

    // Lists to store recent max values (averaging window)
    private List<Double> recentMaxVelos = new ArrayList<>();
    private List<Double> recentMaxAccels = new ArrayList<>();
    private final int AVERAGING_WINDOW = 10; // Average over 10 samples

    private double currentMaxVelo = 0;
    private double currentMaxAccel = 0;

    @Override
    public void initEX() {

        turret.toggle = false;
        Apriltag.limelight.pipelineSwitch(0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        processor = new LocalVision(LocalVision.TargetColor.BOTH);
        paths.addNewPath("p1");
        paths.buildPath(p1);
        paths.addNewPath("p2");
        paths.buildPath(p2);
        paths.addNewPath("p3");
        paths.buildPath(p3);
        paths.addNewPath("pEsh");
        paths.buildPath(pEsh);
        paths.addNewPath("S1");
        paths.buildPath(S1);
        paths.addNewPath("S2");
        paths.buildPath(S2);
        paths.addNewPath("S3");
        paths.buildPath(S3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Camera + settings BEFORE build()
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set lower resolution here
        builder.setCameraResolution(new Size(640, 480));

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Optional: disable RC live view to save CPU
        builder.enableLiveView(false);

        // Add BOTH processors
        builder.addProcessor(processor);

        // Now actually create the portal
        visionPortal = builder.build();

        // Dashboard camera stream
        dashboard.startCameraStream(visionPortal, 15);

        // Initialize velocity timer
        veloTimer.reset();
        turret.mapOfset = 40;
        turret.turrofset = -2.5;
        turret.TURRET_COMP_FACTOR = 0;
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
        turret.robotX = odometry.X(); // CM
        turret.robotY = odometry.Y(); // CM
        turret.robotHeading = odometry.normiliased(); // radians (wrapping)

        // If using TurretWithOdometryVelocity (recommended):
        turret.robotXVelo = odometry.getXVelocity(); // cm/s
        turret.robotYVelo = odometry.getYVelocity(); // cm/s
        turret.robotHeadingVelo = odometry.getHVelocity(); // rad/s

        // Calculate velocity and acceleration
        calculateVelocityAndAcceleration();

        // driveBase.drivePowers(-gamepad1.right_stick_y, -gamepad1.left_stick_x,
        // -gamepad1.right_stick_x);

        // if (turret.shootingLevel == Turret.LowMediumHigh.low
        // &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
        // turret.shootingLevel = Turret.LowMediumHigh.medium;
        // } else if (turret.shootingLevel == Turret.LowMediumHigh.medium
        // &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
        // turret.shootingLevel = Turret.LowMediumHigh.low;
        // }
        // targetHood = targetHood + gamepad1.right_stick_y/8;
        // if (intake.ballCount > 1) {
        // turret.setHoodDegrees(targetHood - 2);
        // }else {
        // turret.setHoodDegrees(targetHood);
        // }
        ////////
        ////////
        // turret.targetRPM = turret.targetRPM + gamepad1.left_stick_y*7;
        // if (!intake.InTake && intake.ballCount >2){
        // intake.reverse = true;
        // }
        //
        if (intake.ballCount > 1) {
            turret.hoodCompensation = -2;
        } else {
            turret.hoodCompensation = 0;
        }
        if (intake.ballCount > 0) {
            shooterOffWait.reset();

        }
        if (odometry.X() > 220) {
            turret.mapOfset = 120;
        } else {
            turret.mapOfset = 40;
        }

        if (intake.ballCount > 2) {
            if (togle) {
                turret.toggle = true;
            }
            shooterOffWait.reset();

        } else if (intake.ballCount < 1 && shooterOffWait.milliseconds() > 500 && !turret.manuel) {
            turret.toggle = false;
        }
        if (intake.ballCount > 2 && rumble.milliseconds() > 1500) {
            gamepad1.rumble(300);
        }

        if (gamepad1.right_bumper) {
            if (!turret.manuel) {
                intake.block = true;
            }
            intake.InTake = true;

        } else if (currentGamepad1.left_bumper && !intake.InTake && turret.diff < 170
                || turret.inZone && turret.diff < 170 && turret.toggle && turret.turretInRange
                        && odometry.getHVelocity() < 1 && (Math.abs(odometry.getXVelocity())
                                + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 17) {
            intake.InTake = true;
            intake.block = false;

        } else if (turret.intakeTime) {
            intake.InTake = true;
        } else if (!currentGamepad1.left_bumper && !currentGamepad1.right_bumper) {
            intake.InTake = false;
        }
        if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left) {
            turret.turrofset -= 1;
        }
        if (!lastGamepad1.dpad_right && currentGamepad1.dpad_right) {
            turret.turrofset += 1;
        }

        if (gamepad1.left_stick_y < -0.3 && Apriltag.getH() != 0 && Apriltag.getH() != 180) {
            togle = true;
            gamepad1.rumble(200);
            rest = true;

            odometry.odo.setPosX(-Apriltag.getX(), DistanceUnit.CM);
            odometry.odo.setPosY(Apriltag.getY(), DistanceUnit.CM);
            odometry.odo.setHeading(-Apriltag.getH(), AngleUnit.DEGREES);

        }
        if (visionCollect) {
            if (processor.hAngleDeg > 8 && !intakePathSelected) {
                follow.setPath(paths.returnPath("p3"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S3;
                targetHeading = 270;
                intake.block = false;
                intake.InTake = true;
                maxWait.reset();

            } else if (processor.hAngleDeg < -8 && !intakePathSelected) {
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S1;
                targetHeading = 270;
                intake.block = true;
                intake.InTake = true;
                maxWait.reset();

            } else if (!intakePathSelected) {
                follow.setPath(paths.returnPath("p2"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S2;
                targetHeading = 270;
                intake.block = true;
                intake.InTake = true;
                maxWait.reset();

            }
            if (follow.isFinished(5, 10) || maxWait.milliseconds() > 1700) {
                collectDone = true;
            }

            intake.block = true;
            intake.InTake = true;

        }
        if (gamepad1.y) {
            double currentHeading = odometry.Heading();

            // World-frame positional error to target (279, 267)
            double xDist = 276 - odometry.X();
            double yDist = 273 - odometry.Y();

            // Rotate world-frame error into robot-relative frame
            double robotRelativeXError = yDist * Math.sin(Math.toRadians(currentHeading))
                    + xDist * Math.cos(Math.toRadians(currentHeading));
            double robotRelativeYError = yDist * Math.cos(Math.toRadians(currentHeading))
                    - xDist * Math.sin(Math.toRadians(currentHeading));

            // Each axis gets its own independently-tunable PID
            double xPower = correctiveXFinalAdjustment.calculate(robotRelativeXError);
            double yPower = correctiveYFinalAdjustment.calculate(robotRelativeYError);

            PathingPower correctivePower = new PathingPower();
            correctivePower.set(xPower, yPower);

            driveBase.drivePowers(
                    -correctivePower.getHorizontal(),
                    headingPID.calculate(currentHeading - 90),
                    -correctivePower.getVertical());
        }
        if (!lastGamepad1.b && currentGamepad1.b && !turret.manuel) {
            turret.manuel = true;
            intake.block = false;
            turret.setHoodDegrees(36);

        } else if (!lastGamepad1.b && currentGamepad1.b && turret.manuel) {
            turret.manuel = false;

        }
        if (gamepad1.x) {
            ejectTimer.reset();
            turret.eject = true;
            intake.block = false;
            turret.targetRPM = 500;
            intake.InTake = true;
            turret.toggle = true;
        } else if (ejectTimer.milliseconds() < 600 && ejectTimer.milliseconds() > 150) {
            intake.block = true;
            turret.eject = false;
        }

        if (autoCycles) {
            switch (backstate) {
                case backShoot:

                    if (follow.isFinished(10, 10) && Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                            + Math.abs(odometry.getHVelocity() * 2) < 21) {
                        pathing = false;
                    }
                    if (!pathing && odometry.X() > 110 && !built
                            && Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                                    + Math.abs(odometry.getHVelocity() * 2) < 10) {
                        shootWait = 500;
                        shootTime.reset();
                        follow.usePathHeadings(false);
                        built = true;
                        pathing = false;
                        intake.block = false;
                        intake.InTake = true;

                    }

                    if (maxToGetToShoot.milliseconds() > 2400) {
                        follow.usePathHeadings(false);
                        built = true;
                        pathing = false;
                        intake.block = true;
                        intake.InTake = true;
                        backstate = backState.backCollect;
                        driveBase.speed = 1;
                        collectDone = false;
                        maxWait.reset();
                    }

                    if (built && shootTime.milliseconds() > shootWait) {
                        backstate = backState.backCollect;
                        driveBase.speed = 1;
                        collectDone = false;
                        maxWait.reset();

                    }
                    break;
                case backCollect:
                    if (built && !collectDone) {
                        visionCollect = true;
                    }
                    if (built && collectDone) {
                        visionCollect = false;
                    }

                    if (!built && follow.isFinished(5, 5)) {
                        pathing = false;
                        visionCollect = false;
                    }

                    if (built && collectDone) {
                        visionCollect = false;
                        backstate = backState.backCollect;
                        follow.setPath(paths.returnPath(shootState.name()));
                        turret.turrofset = 0;

                        intakePathSelected = false;
                        maxToGetToShoot.reset();

                        targetHeading = 270;

                        intakeoff.reset();
                        intakeOff = true;

                        pathing = true;
                        built = false;
                    }
                    break;
            }
        }

        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && turret.toggle) {
            togle = false;
            gamepad1.rumble(800);
        } else if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && !turret.toggle) {
            turret.toggle = true;
            gamepad1.rumble(800);

        }

        if (gamepad1.dpad_down) {
            driveBase.headingLock(45 + odometry.normiliased(), true);
        } else {
            driveBase.headingLock(45 + odometry.normiliased(), false);

        }
        if (pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(),
                    odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (!visionCollect && !gamepad1.y) {
            driveBase.drivePowers(-gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.7,
                    -gamepad1.right_stick_x);
        }

        telemetry.addData("Intake Rpm", intake.secondIntakeMotor.getVelocity());
        telemetry.addData("in zone", turret.inZone);
        telemetry.addData("odometry x", odometry.X());
        telemetry.addData("odometry y", odometry.Y());
        telemetry.addData("Heading", odometry.Heading());
        telemetry.addData("distance", turret.distance);
        telemetry.addData("diff", turret.diff);
        telemetry.addData("hood", targetHood);
        telemetry.addData("rpm", turret.targetRPM);
        telemetry.addData("limeX", Apriltag.getX());
        telemetry.addData("limeY", Apriltag.getY());
        telemetry.addData("limeH", Apriltag.getH());
        telemetry.addData("ball x ", processor.xPosCm);

        telemetry.addData("block ", intake.block);

        telemetry.addData("ball", intake.ballCount);
        telemetry.addData("turretservang ", turret.turretAngle / turret.gearRatio + 180);

        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addData("Loop Time", "%.1f ms", loopTimer.milliseconds());

        telemetry.addData("ball", intake.ballCount);
        telemetry.addData("distance velo", turret.distanceVelocity);
        telemetry.addData("distance offset", turret.ofsetDistance);
        telemetry.addData("vision angle", processor.hAngleDeg);

        telemetry.addData("turretservang ", turret.turretAngle / turret.gearRatio + 180);

        // Display velocity and acceleration data
        telemetry.addData("--- PERFORMANCE METRICS ---", "");
        telemetry.addData("Max Velocity (avg)", "%.1f cm/s", getAverageMaxVelo());
        telemetry.addData("Current Velocity", "%.1f cm/s", currentMaxVelo);
        telemetry.addData("Max Accel (avg)", "%.1f cm/s²", getAverageMaxAccel());
        telemetry.addData("Current Accel", "%.1f cm/s²", currentMaxAccel);

        telemetry.update();

    }

    /**
     * Calculate current velocity magnitude and acceleration
     */
    private void calculateVelocityAndAcceleration() {
        // Get current velocities from odometry
        double xVelo = odometry.getXVelocity(); // cm/s
        double yVelo = odometry.getYVelocity(); // cm/s

        // Calculate velocity magnitude (Pythagorean theorem)
        double currentVelo = Math.sqrt(xVelo * xVelo + yVelo * yVelo);

        // Update max velocity if current is higher
        if (currentVelo > currentMaxVelo) {
            currentMaxVelo = currentVelo;

            // Add to recent max velocities list
            recentMaxVelos.add(currentMaxVelo);
            if (recentMaxVelos.size() > AVERAGING_WINDOW) {
                recentMaxVelos.remove(0); // Remove oldest value
            }
        }

        // Calculate time delta
        double dt = veloTimer.seconds();
        veloTimer.reset();

        // Avoid division by zero
        if (dt > 0.001) {
            // Calculate acceleration components
            double xAccel = (xVelo - lastXVelo) / dt;
            double yAccel = (yVelo - lastYVelo) / dt;

            // Calculate acceleration magnitude
            double currentAccel = Math.sqrt(xAccel * xAccel + yAccel * yAccel);

            // Update max acceleration if current is higher
            if (currentAccel > currentMaxAccel) {
                currentMaxAccel = currentAccel;

                // Add to recent max accelerations list
                recentMaxAccels.add(currentMaxAccel);
                if (recentMaxAccels.size() > AVERAGING_WINDOW) {
                    recentMaxAccels.remove(0); // Remove oldest value
                }
            }
        }

        // Store current velocities for next loop
        lastXVelo = xVelo;
        lastYVelo = yVelo;
        lastHVelo = odometry.getHVelocity();
    }

    /**
     * Get average of recent max velocities
     */
    private double getAverageMaxVelo() {
        if (recentMaxVelos.isEmpty())
            return 0;

        double sum = 0;
        for (double velo : recentMaxVelos) {
            sum += velo;
        }
        return sum / recentMaxVelos.size();
    }

    /**
     * Get average of recent max accelerations
     */
    private double getAverageMaxAccel() {
        if (recentMaxAccels.isEmpty())
            return 0;

        double sum = 0;
        for (double accel : recentMaxAccels) {
            sum += accel;
        }
        return sum / recentMaxAccels.size();
    }
}