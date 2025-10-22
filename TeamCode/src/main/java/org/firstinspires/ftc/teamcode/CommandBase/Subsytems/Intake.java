package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Intake extends SubSystem {
    public MotorEx intakeMotor = new MotorEx();
    ServoDegrees intakeBlocker =new ServoDegrees();
    public boolean block = false;
    public Intake(OpModeEX opModeEX){
        registerSubsystem(opModeEX,defaultCommand);
   }

    @Override
    public void init() {
        intakeMotor.initMotor("intakeMotor", getOpMode().hardwareMap);
        intakeBlocker.initServo("Blocker",getOpMode().hardwareMap);
        intakeBlocker.setRange(355);
        intakeBlocker.setPosition(0);
        intakeBlocker.setDirection(Servo.Direction.REVERSE);

    }

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    @Override
    public void execute() {
        executeEX();
        if (block){
            intakeBlocker.setPosition(0);

        }else {
            intakeBlocker.setPosition(90);
        }

    }
}
