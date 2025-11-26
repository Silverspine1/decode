package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandBase.AdafruitSensorDriver;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Turret;
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
            odometry.startPosition(75, 139, 0);


            Apriltag.limelight.start();

    }

    @Override
    public void loopEX() {
        turret.Auto = true;
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;
        driveBase.drivePowers(-gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger), -gamepad1.right_stick_x);

//        if (turret.shootingLevel == Turret.LowMediumHigh.low &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.medium;
//        } else if (turret.shootingLevel == Turret.LowMediumHigh.medium &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.low;
//        }
//            targetHood = targetHood + gamepad1.right_stick_y/8;
//            turret.setHoodDegrees(targetHood);
//
//
//        turret.targetRPM = turret.targetRPM + gamepad1.left_stick_y*7;



        if (gamepad1.right_bumper ){

                intake.block = true;
                intake.intake = true;



                if (processor.hasTarget) {
                    driveBase.drivePowers(-gamepad1.right_stick_y + processor.distanceCm / 50, headingPID.calculate(-processor.hAngleDeg), -gamepad1.right_stick_x);
                }


        }else if (gamepad1.left_bumper){
            intake.intake = true;
            intake.block = false;

        }else if(turret.intakeTime){
            intake.intake = true;
        }else {
            intake.intake = false;
        }
        if (!lastGamepad1.a && currentGamepad1.a){
            turret.toggle = true;
//            odometry.odo.setPosX(Apriltag.getX() + 180, DistanceUnit.CM);
//            odometry.odo.setPosY( Apriltag.getY() +180,DistanceUnit.CM);
//            odometry.odo.setHeading(180 - Apriltag.llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES), AngleUnit.DEGREES);
        }
        if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left){
            turret.turrofset -= 3;
        }
        if (!lastGamepad1.dpad_right && currentGamepad1.dpad_right){
            turret.turrofset += 3;
        }
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up){
            turret.mapOfset += -20;
        }
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up){
            turret.mapOfset += 20;
        }



        AdafruitSensorDriver.Reading lower = intake.lowerSensor.read();
        AdafruitSensorDriver.Reading upper = intake.upperSensor.read();

        telemetry.addData("in zone", turret.inZone);
        telemetry.addData("odometry x", odometry.X());
        telemetry.addData("odometry y", odometry.Y());
        telemetry.addData("Heading", odometry.Heading());
        telemetry.addData("distance", turret.distance);
        telemetry.addData("target", turret.targetRPM);
        telemetry.addData("hood", targetHood);
        telemetry.addData("count", turret.shootCount);
        telemetry.addData("T2", turret.T2);







        telemetry.update();


    }
}
