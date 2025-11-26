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

    }


    public Command drivePowers (double vertical, double turn, double strafe){
        this.turn = turn;
        this.strafe = strafe;
        this.vertikal = -vertical;

        return driveCommand;

    }

    public Command drivePowers (RobotPower power){
        this.turn = power.getPivot();
        this.strafe = -power.getVertical();
        this.vertikal = power.getHorizontal();

        return driveCommand;
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
            },
            () -> {
                 double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));

                LF.update((vertikal-(strafe)-turn)/denominator);
                RF.update((vertikal+(strafe)+turn)/denominator);
                LB.update((vertikal+(strafe*strafeExtra)-turn)/denominator);
                RB.update((vertikal-(strafe*strafeExtra)+turn)/denominator);

//                System.out.println("vertikal power" + vertikal);
//                System.out.println("Left front power" + LF.getPower());
            },
            () -> true
    );

    public Command driveFieldCentric(double vertical, double turn, double strafe){
        this.vertikal = -vertical;
        this.strafe = strafe;
        this.turn = turn;

        return driveField;
    }

    public void setAll(double power){
        LF.update(power);
        RF.update(power);
        LB.update(power);
        RB.update(power);
    }

    LambdaCommand driveField = new LambdaCommand(
            () -> {
            },
            () -> {
                double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));


                double heading = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);

                double rotX = vertikal * Math.cos(-heading) - strafe * Math.sin(-heading);
                double rotY = vertikal * Math.sin(-heading) + strafe * Math.cos(-heading);


                LF.update((rotX-rotY-turn)/denominator);
                RF.update((rotX+rotY+turn)/denominator);
                LB.update((rotX+rotY-turn)/denominator);
                RB.update((rotX-rotY+turn)/denominator);


            },
            () -> true


    );



}