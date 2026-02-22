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

    // --- Hardware Caching Fields ---
    private double lIM = 0, lSIM = 0;
    private double lIB = -1, lIP = -1;
    private final double MOTOR_EPSILON = 0.01;
    private final double SERVO_EPSILON = 0.001;

    public Intake(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
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
            () -> {
            },
            () -> {
            },
            () -> true);

    /**
     * Updates ballCount by checking the current state of all three sensors.
     * Since these are active-HIGH, getState() returns true when a ball is present.
     */
    private void updateBallCount() {
        int count = 0;
        if (sensorPos1.getState())
            count++;
        if (sensorPos2.getState())
            count++;
        if (sensorPos3.getState())
            count++;

        ballCount = count;
    }

    @Override
    public void execute() {
        executeEX();

        updateBallCount();
        intakeRPM = secondIntakeMotor.getVelocity();

        if (block && intakeTime.milliseconds() > 50) {
            if (Math.abs(0.60 - lIB) > SERVO_EPSILON) {
                intakeBlocker.setPosition(0.60);
                lIB = 0.60;
            }
            if (Math.abs(0.37 - lIP) > SERVO_EPSILON) {
                intakePTO.setPosition(0.37);
                lIP = 0.37;
            }
        } else if (!block) {
            if (Math.abs(0.29 - lIB) > SERVO_EPSILON) {
                intakeBlocker.setPosition(0.29);
                lIB = 0.29;
            }
            if (Math.abs(0.52 - lIP) > SERVO_EPSILON) {
                intakePTO.setPosition(0.52);
                lIP = 0.52;
            }
            intakeTime.reset();
        }

        if (InTake) {
            if (Math.abs(-1 - lIM) > MOTOR_EPSILON) {
                intakeMotor.update(-1);
                lIM = -1;
            }
            if (Math.abs(-1 - lSIM) > MOTOR_EPSILON) {
                secondIntakeMotor.update(-1);
                lSIM = -1;
            }
            reverse = false;
        } else {
            if (lIM != 0) {
                intakeMotor.update(0);
                lIM = 0;
            }
            if (lSIM != 0) {
                secondIntakeMotor.update(0);
                lSIM = 0;
            }
        }

        if (InTake && !block) {
            if (Math.abs(-1 - lSIM) > MOTOR_EPSILON) {
                secondIntakeMotor.update(-1);
                lSIM = -1;
            }
        }
    }
}