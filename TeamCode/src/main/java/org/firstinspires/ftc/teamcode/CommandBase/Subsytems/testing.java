package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

@TeleOp
public class testing extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger) * 0.65, -gamepad1.right_stick_x*1));
        telemetry.addData("X",odometry.X());
        telemetry.addData("y",odometry.Y());
        telemetry.addData("Delta X",odometry.deltaX);
        telemetry.addData("backpod",odometry.currentBackPod);
        telemetry.addData("leftpod",odometry.currentLeftPod);
        telemetry.addData("Delta leftpod",odometry.deltaLeft);
        telemetry.addData("1 heading",odometry.Heading);
        telemetry.addData("2 heading",odometry.imu360);
        telemetry.addData("3 heading",odometry.Heading());
        telemetry.addData("heading IMU",odometry.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
        telemetry.addData("Delta heading",odometry.deltaHeading);

        telemetry.addData("Velocity x",odometry.currentXVelocity);
        telemetry.update();
    }
}

