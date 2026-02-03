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
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class DriveBase extends SubSystem {

    public MotorEx LF = new MotorEx();
    public MotorEx RF = new MotorEx();
    public MotorEx RB = new MotorEx();
    public MotorEx LB = new MotorEx();
    public TouchSensor intakeSensor;
    Servo pto1;
    Servo pto2;
    ServoDegrees baseServo =new ServoDegrees();


    public double speed = 1;
    public boolean engage = false;
    public boolean lift = false;





    PIDController headingPID = new PIDController(0.025,0,0.0003);

    public IMU imu;

    double vertikal ;
    double turn ;
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
        pto1 = getOpMode().hardwareMap.get(Servo.class, "pto1");
        pto2 = getOpMode().hardwareMap.get(Servo.class, "pto2");
        baseServo.initServo("base", getOpMode().hardwareMap);
        baseServo.setRange(180);
        baseServo.setDirection(Servo.Direction.REVERSE );




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

        if (engage){
            pto1.setPosition(0.64);
            pto2.setPosition(0.32);
            baseServo.setPosition(35);

        } else {
            pto1.setPosition(0.5);
            pto2.setPosition(0.5);
            baseServo.setPosition(0);


        }
//        if (lift){
//            baseServo.setPosition(90);
//        }else {
//            baseServo.setPosition(0);
//        }


    }


    public Command drivePowers (double vertical, double turn, double strafe){
        this.turn = turn;
        this.strafe = strafe;
        this.vertikal = vertical;

        return driveCommand;

    }

    public Command drivePowers (RobotPower power){
        this.turn = power.getPivot();
        this.strafe = -power.getVertical();
        this.vertikal = -power.getHorizontal();

        return driveCommand;
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
            },
            () -> {
                double denominator = Math.max(1.0, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));
                if (!engage) {
                    LF.update(((vertikal - strafe - turn) / denominator) * speed);
                    RF.update(((vertikal + strafe + turn) / denominator) * speed);
                    LB.update(((vertikal + strafe - turn) / denominator) * speed);
                    RB.update(((vertikal - strafe + turn) / denominator) * speed);
                } else {
                    LF.update(((vertikal - strafe - turn) / denominator) * speed);
                    RF.update(((vertikal + strafe + turn) / denominator) * speed);
                    LB.update(0);
                    RB.update(0);
                }

                System.out.println("vertikal power" + vertikal);
                System.out.println("Left front power" + LF.getPower());
            },
            () -> true
    );



    public void setAll(double power){
        LF.update(power);
        RF.update(power);
        LB.update(power);
        RB.update(power);
    }


    public void driveFieldCentric(double drive, double strafe, double turn, double robotHeading) {
        // Convert degrees → radians  (most common IMU method in FTC)
        double headingRadians = Math.toRadians(robotHeading);

        // Now rotate the translation vector opposite to the robot's heading
        double rotX = strafe * Math.cos(-headingRadians) - drive * Math.sin(-headingRadians);
        double rotY = strafe * Math.sin(-headingRadians) + drive * Math.cos(-headingRadians);

        drivePowers(rotY, turn, rotX);
    }



}