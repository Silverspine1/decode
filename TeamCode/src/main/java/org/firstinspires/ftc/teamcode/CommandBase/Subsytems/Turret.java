package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;

public class Turret extends OpModeEX {
    DcMotor shooterMotorOne;
    DcMotor shooterMotorTwo;
    ServoDegrees turretTurnOne = new ServoDegrees();
    ServoDegrees turretTurnTwo =new ServoDegrees();
    ServoDegrees hoodAdjust = new ServoDegrees();

    @Override
    public void initEX() {
        shooterMotorOne = hardwareMap.dcMotor.get("shooterMotorOne");
        shooterMotorTwo = hardwareMap.dcMotor.get("shooterMotorTwo");

        turretTurnOne.initServo("turretTurnOne", hardwareMap);
        turretTurnTwo.initServo("turretTurnTwo", hardwareMap);
        hoodAdjust.initServo("hoodAdjust", hardwareMap);

    }

    @Override
    public void loopEX() {

    }
}
