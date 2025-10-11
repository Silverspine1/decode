package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class odometryTest extends OpModeEX {
    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        telemetry.addData("X",odometry.X());
        telemetry.addData("Y",odometry.Y());
        telemetry.addData("heading",odometry.Heading());

    }
}
