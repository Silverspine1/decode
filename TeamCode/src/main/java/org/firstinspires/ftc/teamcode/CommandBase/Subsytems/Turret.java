package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

public class Turret extends SubSystem {
    public MotorEx shooterMotorOne = new MotorEx();
    public MotorEx shooterMotorTwo = new MotorEx();
    public ServoDegrees turretTurnOne = new ServoDegrees();
    public ServoDegrees turretTurnTwo =new ServoDegrees();
    public ServoDegrees hoodAdjust = new ServoDegrees();

    public double targetRPM = 2700;
    public double targetX = 200;
    public double targetY = 200;
    public double robotX;
    public double robotY;
    public double robotHeading;
    public double turretAngle;
    final double gearRatio = 0.2727;

    public boolean turretTarget = false;


    public PIDController shootPID = new PIDController(0.01,0.00004,0.0003);
    public Turret(OpModeEX opModeEX){
        registerSubsystem(opModeEX,null);
    }


    @Override
    public void init() {
        shooterMotorOne.initMotor("shooterMotorOne", getOpMode().hardwareMap);
        shooterMotorTwo.initMotor("shooterMotorTwo", getOpMode().hardwareMap);

        turretTurnOne.initServo("turretTurnOne", getOpMode().hardwareMap);
        turretTurnTwo.initServo("turretTurnTwo", getOpMode().hardwareMap);
        hoodAdjust.initServo("hoodAdjust", getOpMode().hardwareMap);

        turretTurnOne.setPosition(178);
        turretTurnTwo.setPosition(178);
    }



    @Override
    public void execute() {
        executeEX();
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        turretAngle = Math.toDegrees(Math.atan2(deltaY,deltaX) - robotHeading);
        if (turretTarget && (turretAngle)/gearRatio+178 < 178+90){
            turretTurnOne.setPosition((turretAngle)/gearRatio+178);
            turretTurnTwo.setPosition((turretAngle)/gearRatio+178);
        } else if (turretTarget && (turretAngle)/gearRatio+178 > 178-90) {
            turretTurnOne.setPosition((turretAngle)/gearRatio+178);
            turretTurnTwo.setPosition((turretAngle)/gearRatio+178);
        }else {
            turretTurnOne.setPosition(178);
            turretTurnTwo.setPosition(178);
        }


    }
}
