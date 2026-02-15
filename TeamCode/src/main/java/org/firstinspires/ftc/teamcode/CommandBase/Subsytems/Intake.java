package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Intake extends SubSystem {
    public MotorEx intakeMotor = new MotorEx();
    public MotorEx secondIntakeMotor = new MotorEx();

    Servo intakeBlocker;
    Servo intakePTO;

    // --- Three Digital Sensors for Ball Positions ---
    private DigitalChannel sensorPos1;
    private DigitalChannel sensorPos2;
    private DigitalChannel sensorPos3;

    public boolean block = true;
    public boolean InTake = false;
    boolean intakeBeforeBlock = false;

    public boolean reverse = false;
    ElapsedTime intakeTime = new ElapsedTime();


    public int ballCount = 0;
    public double intakeRPM = 0;

    public Intake(OpModeEX opModeEX){
        registerSubsystem(opModeEX,defaultCommand);
    }

    @Override
    public void init() {
        intakeMotor.initMotor("intakeMotor", getOpMode().hardwareMap);
        intakePTO = getOpMode().hardwareMap.get(Servo.class, "intakePTO");
        intakeBlocker = getOpMode().hardwareMap.get(Servo.class, "intakeBlocker");
        secondIntakeMotor.initMotor("intakeMotor2", getOpMode().hardwareMap);

        // Initialize the three sensors
        sensorPos1 = getOpMode().hardwareMap.get(DigitalChannel.class, "sensor1");
        sensorPos2 = getOpMode().hardwareMap.get(DigitalChannel.class, "sensor2");
        sensorPos3 = getOpMode().hardwareMap.get(DigitalChannel.class, "sensor3");

        sensorPos1.setMode(DigitalChannel.Mode.INPUT);
        sensorPos2.setMode(DigitalChannel.Mode.INPUT);
        sensorPos3.setMode(DigitalChannel.Mode.INPUT);

        secondIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    /**
     * Updates ballCount by checking the current state of all three sensors.
     * Since these are active-HIGH, getState() returns true when a ball is present.
     */
    private void updateBallCount() {
        int count = 0;
        if (sensorPos1.getState()) count++;
        if (sensorPos2.getState()) count++;
        if (sensorPos3.getState()) count++;

        ballCount = count;
    }

    @Override
    public void execute() {
        executeEX();

        updateBallCount();
        intakeRPM = secondIntakeMotor.getVelocity();


        if (block && intakeTime.milliseconds() >50) {
            intakeBlocker.setPosition(0.60);
            intakePTO.setPosition(0.37);
        } else if (!block ){
            intakeBlocker.setPosition(0.29);
            intakePTO.setPosition(0.52);
            intakeTime.reset();

        }

        if (InTake) {
            intakeMotor.update(-1);
            secondIntakeMotor.update(-1);
            reverse = false;

        } else {
            intakeMotor.update(0);
            secondIntakeMotor.update(0);
        }
        if (reverse){
//            intakeMotor.update(1);

        }

        if (InTake && !block) {
            secondIntakeMotor.update(-1);
        }
    }
}