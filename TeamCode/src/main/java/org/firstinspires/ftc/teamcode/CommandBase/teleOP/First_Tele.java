package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
@TeleOp
public class First_Tele extends OpModeEX {
    @Override
    public void initEX() {
        odometry.startPosition(86.6, 16.6, 0);


    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.Y();
        turret.robotY = odometry.X();
        turret.robotHeading = Math.toRadians(odometry.Heading());
        driveBase.drivePowers(gamepad1.right_stick_y,(gamepad1.left_trigger - gamepad1.right_trigger),gamepad1.left_stick_x);
        if (gamepad1.right_bumper){
            intake.intakeMotor.update(-1);

        }else if (gamepad1.left_bumper){
            intake.intakeMotor.update(1);
        }else {
            intake.intakeMotor.update(0);
        }
        if (gamepad1.a){
            intake.trans.setPosition(1);
        }

        telemetry.addData("in zone",turret.inZone);
        telemetry.addData("odometry x", odometry.Y());
        telemetry.addData("odometry y", odometry.X());
        telemetry.addData("Heading",odometry.Heading());
        telemetry.update();


    }
}
