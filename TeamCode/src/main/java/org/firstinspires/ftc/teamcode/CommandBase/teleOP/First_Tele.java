package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

public class First_Tele extends OpModeEX {
    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
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


    }
}
