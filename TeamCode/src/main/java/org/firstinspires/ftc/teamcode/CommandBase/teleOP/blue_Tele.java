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

import java.util.ArrayList;
import java.util.List;

import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

@TeleOp


public class blue_Tele extends OpModeEX {
    private VisionPortal visionPortal;
    private LocalVision processor;
    double heading;
    boolean captureLastHeading = false;
    boolean brake = false;
    boolean rest = false;
    double targetHood = 25;
    boolean blue = true;
    boolean togle = false;
    double lastBallCount = 0;
    double currentBallCount = 0;
    PIDController headingPID = new PIDController(0.013,0,0.0032);
    ElapsedTime shooterOffWait = new ElapsedTime();

    // Variables for velocity and acceleration tracking
    private double lastXVelo = 0;
    private double lastYVelo = 0;
    private double lastHVelo = 0;
    private ElapsedTime veloTimer = new ElapsedTime();
    ElapsedTime Timer = new ElapsedTime();


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
    }

    @Override public void stop() {
        if (visionPortal != null) visionPortal.close();
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

//        driveBase.drivePowers(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        driveBase.drivePowers(-gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger), -gamepad1.right_stick_x);


//        if (turret.shootingLevel == Turret.LowMediumHigh.low &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.medium;
//        } else if (turret.shootingLevel == Turret.LowMediumHigh.medium &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.low;
//        }
//            targetHood = targetHood + gamepad1.right_stick_y/8;
//            if (intake.ballCount > 1) {
//                turret.setHoodDegrees(targetHood - 2);
//            }else {
//                turret.setHoodDegrees(targetHood);
//            }
////////
////////
//        turret.targetRPM = turret.targetRPM + gamepad1.left_stick_y*7;
//        if (!intake.InTake && intake.ballCount >2){
//            intake.reverse = true;
//        }
//
        if (intake.ballCount > 1) {
            turret.hoodCompensation = -2;
        }else {
            turret.hoodCompensation = 0;
        }
        if (intake.ballCount>0){
            shooterOffWait.reset();

        }
        if (intake.ballCount >2){
            if (togle){
                turret.toggle = true;
            }
            shooterOffWait.reset();

        }else if(intake.ballCount < 1 && shooterOffWait.milliseconds()>500 ){
            turret.toggle = false;
        }

        if (gamepad1.right_bumper ){

            intake.block = true;
            intake.InTake = true;


        }else if (currentGamepad1.left_bumper && !intake.InTake && turret.diff < 170 || turret.inZone && turret.diff < 170 && turret.toggle && turret.turretInRange && odometry.getHVelocity() < 1 && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 7){
            intake.InTake = true;
            intake.block = false;




        }else if(turret.intakeTime){
            intake.InTake = true;
        }else if (!currentGamepad1.left_bumper && !currentGamepad1.right_bumper){
            intake.InTake = false;
        }
        if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left){
            turret.turrofset -= 3;
        }
        if (!lastGamepad1.dpad_right  && currentGamepad1.dpad_right){
            turret.turrofset += 3;
        }



        if (!lastGamepad1.start && currentGamepad1.start && Apriltag.getH() != 0 && Apriltag.getH() !=180  ) {
            togle = true;
            gamepad1.rumble(800);
            rest = true;



            odometry.odo.setPosX(-Apriltag.getX(), DistanceUnit.CM);
            odometry.odo.setPosY(Apriltag.getY(), DistanceUnit.CM);
            odometry.odo.setHeading(-Apriltag.getH(), AngleUnit.DEGREES);

        }
        if (!lastGamepad1.back && currentGamepad1.back && blue){
            blue = false;
            turret.targetX = 360;
            gamepad1.rumble(800);
            turret.turrofset = 0;


        } else if (!lastGamepad1.back && currentGamepad1.back && !blue){
            blue = true;
            turret.targetX = 0;
            gamepad1.rumble(800);
            turret.turrofset = 0;


        }

        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && turret.toggle){
            togle = false;
            gamepad1.rumble(800);
        }else if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up&& !turret.toggle){
            turret.toggle = true;
            gamepad1.rumble(800);

        }


        if (gamepad1.dpad_down){
            driveBase.headingLock(45 + odometry.normiliased(),true);
        }else {
            driveBase.headingLock(45 + odometry.normiliased(),false);

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
        telemetry.addData("limeX",Apriltag.getX());
        telemetry.addData("limeY",Apriltag.getY());
        telemetry.addData("limeH",Apriltag.getH());
        telemetry.addData("ball x ",processor.xPosCm);

        telemetry.addData("block ",intake.block);


        System.out.println("Current Velocity: " + currentMaxVelo);
        System.out.println("Current Accel: " + currentMaxAccel);
        System.out.println("time: " + Timer);

        telemetry.addData("ball",intake.ballCount);
        telemetry.addData("turretservang ",turret.turretAngle/turret.gearRatio +180);




        ElapsedTime loopTimer = new ElapsedTime();


        telemetry.addData("Loop Time", "%.1f ms", loopTimer.milliseconds());


        telemetry.addData("ball",intake.ballCount);
        telemetry.addData("distance velo",turret.distanceVelocity);
        telemetry.addData("distance offset",turret.ofsetDistance);
        telemetry.addData("vision angle",processor.hAngleDeg);



        telemetry.addData("turretservang ",turret.turretAngle/turret.gearRatio +180);

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
        if (recentMaxVelos.isEmpty()) return 0;

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
        if (recentMaxAccels.isEmpty()) return 0;

        double sum = 0;
        for (double accel : recentMaxAccels) {
            sum += accel;
        }
        return sum / recentMaxAccels.size();
    }
}