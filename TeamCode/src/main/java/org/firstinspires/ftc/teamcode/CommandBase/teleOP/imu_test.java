package org.firstinspires.ftc.teamcode.CommandBase.teleOP;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.source.doctree.SerialDataTree;

@TeleOp(name = "IMU Test", group = "Test")
public class imu_test extends LinearOpMode {

    IMU imu;
    Servo lift1;
    Servo lift2;

    @Override
    public void runOpMode() {

        // Initialize the IMU
        lift1 = hardwareMap.get(Servo.class,"lift1");
        lift2 = hardwareMap.get(Servo.class,"lift2");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set the orientation of the Control Hub on your robot.
        // Adjust LOGO_FACING_UP and USB_FACING_FORWARD to match your robot's mounting.
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        telemetry.update();

        waitForStart();

        // Reset yaw to 0 at the start
        imu.resetYaw();

        while (opModeIsActive()) {

            // Read IMU angles
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            double yaw   = angles.getYaw(AngleUnit.DEGREES);
            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double roll  = angles.getRoll(AngleUnit.DEGREES);

            // Read angular velocity
            AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // Display data
            telemetry.addLine("=== REV Internal IMU Test ===");
            telemetry.addLine();
            telemetry.addData("Yaw   (Z)", "%.2f°", yaw);
            telemetry.addData("Pitch (X)", "%.2f°", pitch);
            telemetry.addData("Roll  (Y)", "%.2f°", roll);
            telemetry.addLine();
            telemetry.update();
            if (gamepad1.dpad_up){
                lift1.setPosition(0);
                lift2.setPosition(1);
            } else if (gamepad1.dpad_down) {
                lift1.setPosition(1);
                lift2.setPosition(0);
            }else{
                lift1.setPosition(0.5);
                lift2.setPosition(0.5);
            }


        }
    }
}