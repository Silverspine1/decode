package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.AdafruitSensorDriver;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

@TeleOp


public class First_Tele extends OpModeEX {
    private VisionPortal visionPortal;
    private LocalVision processor;
    double heading;
    boolean captureLastHeading = false;
    boolean brake = false;
    double targetHood = 25;
    boolean blue = true;
    PIDController headingPID = new PIDController(0.013,0,0.0032);

    @Override
    public void initEX() {
        turret.toggle = false;
        Apriltag.limelight.pipelineSwitch(0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        processor = new LocalVision(LocalVision.TargetColor.PURPLE);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Camera + settings BEFORE build()
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set lower resolution here
        builder.setCameraResolution(new Size(640, 480));   // or 640x480 if you want

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Optional: disable RC live view to save CPU
        builder.enableLiveView(false);

        builder.addProcessor(processor);

        // Now actually create the portal
        visionPortal = builder.build();

        // Dashboard camera stream
        dashboard.startCameraStream(visionPortal, 15);





    }
    @Override public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
        @Override
    public void start() {


            Apriltag.limelight.start();

    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;
        driveBase.drivePowers(-gamepad1.right_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        driveBase.driveFieldCentric(gamepad1.right_stick_y, gamepad1.right_stick_x,gamepad1.right_trigger - gamepad1.left_trigger ,odometry.normilised);




//        if (turret.shootingLevel == Turret.LowMediumHigh.low &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.medium;
//        } else if (turret.shootingLevel == Turret.LowMediumHigh.medium &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.low;
//        }
//            targetHood = targetHood + gamepad1.right_stick_y/8;
//            turret.setHoodDegrees(targetHood);
////////
////////
//        turret.targetRPM = turret.targetRPM + gamepad1.left_stick_y*7;



        if (currentGamepad1.right_bumper  && !intake.InTake ){

                intake.block = true;
                intake.InTake = true;



                if (processor.hasTarget) {
                    driveBase.drivePowers(-gamepad1.right_stick_y + processor.distanceCm / 50, headingPID.calculate(-processor.hAngleDeg), -gamepad1.right_stick_x);
                }


        }else if (currentGamepad1.left_bumper && !intake.InTake && turret.diff < 140){
            intake.InTake = true;
            intake.block = false;

        }else if(turret.intakeTime){
            intake.InTake = true;
        }else if (!currentGamepad1.left_bumper && !currentGamepad1.right_bumper){
            intake.InTake = false;
        }

        if (!lastGamepad1.start && currentGamepad1.start && Apriltag.getH() != 0 && Apriltag.getH() !=180) {
            turret.toggle = true;
            gamepad1.rumble(800);



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
            turret.turrofset = - 4;


        }

        if (!lastGamepad1.dpad_down && currentGamepad1.dpad_down && turret.toggle){
            turret.toggle = false;
            gamepad1.rumble(800);
        }else if (!lastGamepad1.dpad_down && currentGamepad1.dpad_down&& !turret.toggle){
            turret.toggle = true;
            gamepad1.rumble(800);

        }
        if (gamepad1.dpad_left){
            intake.intakeMotor.update(1);
        }






        AdafruitSensorDriver.Reading lower = intake.lowerSensor.read();
        AdafruitSensorDriver.Reading upper = intake.upperSensor.read();

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
        System.out.println("X: " + odometry.X());
        System.out.println("Y: " + odometry.Y());












        telemetry.update();


    }
}
