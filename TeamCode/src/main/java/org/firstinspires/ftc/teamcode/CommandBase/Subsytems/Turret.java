package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
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
    public double targetX = 80;
    public double targetY = 0;
    public double robotX;
    public double robotY;
    public double robotHeading;
    public double turretAngle;
    final double gearRatio = 0.7272;
    final double turretLimitAngle =80;
    public double distance;
    double distance1 = 143;
    double hoodAngle1 = 166.2;
    double power1 = 3760;
    double distance2 = 244;
    double hoodAngle2 = 110;
    double power2 = 4200;
    double power2NoHood;
    double distance3 = 320;
    double hoodAngle3 = 100;
    double power3 = 4840;
    double power3NoHood;
    double power4NoHood;
    double distance4 = 380;
    double interpolatePower;


    public boolean turretTarget = true;


    public PIDController shootPID = new PIDController(0.01,0,0.0003);
    public Turret(OpModeEX opModeEX){
        registerSubsystem(opModeEX,defaultCommand);
    }


    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    @Override
    public void init() {
        shooterMotorOne.initMotor("shooterMotorOne", getOpMode().hardwareMap);
        shooterMotorTwo.initMotor("shooterMotorTwo", getOpMode().hardwareMap);

        turretTurnOne.initServo("turretTurnOne", getOpMode().hardwareMap);
        turretTurnTwo.initServo("turretTurnTwo", getOpMode().hardwareMap);
        hoodAdjust.initServo("hoodAdjust", getOpMode().hardwareMap);
        turretTurnTwo.setRange(355);
        turretTurnOne.setRange(355);
        hoodAdjust.setRange(355);
        turretTurnTwo.setDirection(Servo.Direction.REVERSE);
        turretTurnOne.setDirection(Servo.Direction.REVERSE);


        turretTurnOne.setOffset(178);
        turretTurnTwo.setOffset(178);
        turretTurnOne.setPosition(0);
        turretTurnTwo.setPosition(0);
        hoodAdjust.setPosition(178);
    }






    @Override
    public void execute() {
        executeEX();


            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;
            distance = Math.hypot(deltaY,deltaX);
        // --- Interpolation Logic ---
        if (distance <= distance1) {

            interpolatePower = power1;

        } else if (distance <= distance2) {

            interpolatePower = power1 + (power2NoHood - power1) * (distance - distance1) / (distance2 - distance1);

        } else if (distance <= distance3) {


            interpolatePower = power2 + (power3NoHood - power2) * (distance - distance2) / (distance3 - distance2);

        } else if (distance <= distance4) {

            interpolatePower = power3 + (power4NoHood - power3) * (distance - distance3) / (distance4 - distance3);

        } else {

            if (distance4 - distance3 != 0) {

                double slope = (power4NoHood - power3) / (distance4 - distance3);
                interpolatePower = power3 + slope * (distance - distance3);

            } else {

                interpolatePower = power4NoHood;

            }
        }
//            if (distance >= distance1){
//                hoodAdjust.setPosition(hoodAngle1);
//            } else if (distance >= distance2){
//                hoodAdjust.setPosition(hoodAngle2);
//            } else if (distance >= distance3){
//                hoodAdjust.setPosition(hoodAngle3);
//            }


            // targetRPM = interpolatePower;

             if (robotHeading > Math.PI) {
                robotHeading = robotHeading - Math.PI * 2;
            }

            turretAngle = Math.toDegrees(Math.atan2(-deltaY, deltaX) - robotHeading);

            if ((turretAngle) > turretLimitAngle) {

                turretAngle = turretLimitAngle;

            } else if ((turretAngle) < -turretLimitAngle) {

                turretAngle = -turretLimitAngle;

            }

            turretTurnOne.setPosition(((turretAngle) / gearRatio) );
            turretTurnTwo.setPosition(((turretAngle) / gearRatio) );



        }
}

