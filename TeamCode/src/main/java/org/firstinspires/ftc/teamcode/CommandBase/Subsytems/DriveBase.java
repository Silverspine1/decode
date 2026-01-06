package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class DriveBase extends SubSystem {

    public MotorEx LF = new MotorEx();
    public MotorEx RF = new MotorEx();
    public MotorEx RB = new MotorEx();
    public MotorEx LB = new MotorEx();
    public TouchSensor intakeSensor;
    double speed = 1;

    PIDController headingPID = new PIDController(0.025,0,0.0003);

    public IMU imu;
    public double strafeExtra = 1.0;

    double vertical;
    double turn;
    double strafe;
    public boolean tele = true;

    public DriveBase(OpModeEX opModeEX){
        registerSubsystem(opModeEX,driveCommand);
    }

    @Override
    public void init() {
        LF.initMotor("LF", getOpMode().hardwareMap);
        RF.initMotor("RF", getOpMode().hardwareMap);
        LB.initMotor("LB", getOpMode().hardwareMap);
        RB.initMotor("RB", getOpMode().hardwareMap);

        intakeSensor = getOpMode().hardwareMap.get(TouchSensor.class, "intakeSensor");
        imu = getOpMode().hardwareMap.get(IMU.class, "imu");

        imu.resetYaw();

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double headindingLockMotorPower (double headingError){
        if (headingError < -180) {
            headingError = (360 + headingError);
        } else if (headingError > 180) {
            headingError = (headingError - 360);
        }
        return headingPID.calculate(headingError);
    }

    @Override
    public void execute() {
        executeEX();
        if (!tele){
            speed = 2.2;
        }
    }

    // Robot-centric drive (no field-centric transformation)
    public Command drivePowers (double vertical, double turn, double strafe){
        this.vertical = vertical;
        this.turn = turn;
        this.strafe = strafe;
        return driveCommand;
    }

    public Command drivePowers (RobotPower power){
        this.turn = power.getPivot();
        this.strafe = -power.getVertical();
        this.vertical = power.getHorizontal();
        return driveCommand;
    }

    /**
     * Field-centric drive using IMU heading
     * @param drive Forward/backward input (+ = forward from driver's perspective)
     * @param strafe Left/right input (+ = right from driver's perspective)
     * @param turn Rotation input (+ = clockwise)
     * @param robotHeading Current robot heading in radians
     */
    public void driveFieldCentric(double drive, double strafe, double turn, double robotHeading) {
        // Rotate the movement direction counter to the robot's rotation
        // to make it field-centric (relative to driver, not robot)
        double rotatedStrafe = strafe * Math.cos(-robotHeading) - drive * Math.sin(-robotHeading);
        double rotatedDrive = strafe * Math.sin(-robotHeading) + drive * Math.cos(-robotHeading);

        // Pass the rotated values to the robot-centric drive command
        drivePowers(rotatedDrive, turn, rotatedStrafe);
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
            },
            () -> {
                // Calculate denominator for normalization
                double denominator = Math.max(speed, Math.abs(vertical) + Math.abs(strafe) + Math.abs(turn));

                // Mecanum drive calculations
                // LF = vertical - strafe - turn
                // RF = vertical + strafe + turn
                // LB = vertical + strafe - turn
                // RB = vertical - strafe + turn
                LF.update((vertical - strafe - turn) / denominator);
                RF.update((vertical + strafe + turn) / denominator);
                LB.update((vertical + (strafe * strafeExtra) - turn) / denominator);
                RB.update((vertical - (strafe * strafeExtra) + turn) / denominator);
            },
            () -> true
    );

    public void setAll(double power){
        LF.update(power);
        RF.update(power);
        LB.update(power);
        RB.update(power);
    }
}