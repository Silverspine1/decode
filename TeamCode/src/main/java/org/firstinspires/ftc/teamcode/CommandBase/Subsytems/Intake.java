package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public abstract class Intake extends SubSystem {
    public MotorEx intakeMotor = new MotorEx();
    public Servo trans;
    public Intake(OpModeEX opModeEX){
        registerSubsystem(opModeEX,null);
    }

    @Override
    public void init() {
        intakeMotor.initMotor("intakeMotor", getOpMode().hardwareMap);
        trans = getOpMode().hardwareMap.get(Servo.class,"trans");

    }


    @Override
    public void execute() {
        executeEX();

    }
}
