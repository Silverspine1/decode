package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Intake extends SubSystem {
    public MotorEx intakeMotor = new MotorEx();
    public Servo trans;
    double uselessBob = 3;
    public Intake(OpModeEX opModeEX){
        registerSubsystem(opModeEX,defaultCommand);
   }

    @Override
    public void init() {
        intakeMotor.initMotor("intakeMotor", getOpMode().hardwareMap);
        trans = getOpMode().hardwareMap.get(Servo.class,"trans");

    }

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    @Override
    public void execute() {
        executeEX();

    }
}
