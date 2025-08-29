package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        telemetry.addData("backpod",odometry.currentBackPod);
        telemetry.addData("leftpod",odometry.currentLeftPod);
        telemetry.addData("heading",odometry.Heading());
        telemetry.update();
    }
}

