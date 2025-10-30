package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public enum LowMediumHigh {
        low,
        medium,
        high
    }
    public LowMediumHigh shootingLevel = LowMediumHigh.low;

    double Xoffset = 16.6;
    double Yoffset = 16.6;
    double shootPower;


    public double targetX = 0;
    public double targetY = 0;
    public double robotX;
    public double robotY;
    public double robotHeading;

    public double targetRPM = 2700;
    public double rpm;
    public double mapOfset = 80;
    public double turrofset= 0;
    public double turretAngle;
    final double gearRatio = 0.7272;
    final double turretLimitAngle =80;


    public double distance;
    double distance1 = 120;
    double distance2 = 193;
    double distance3 = 344;
    double distance4 = 407;



    double lowHoodAngle1 = 190;
    double lowHoodAngle2 = 201;
    double lowHoodAngle3 = 192;

    double mediumHoodAngle1 = 211;
    double mediumHoodAngle2 = 181;
    double mediumHoodAngle3 = 156;

    double highHoodAngle1 = 207;
    double highHoodAngle2 = 110;
    double highHoodAngle3 = 100;


    double lowPower1 = 2300;
    double lowPower2 = 2623;
    double lowPower3 = 3055;

    double mediumPower1 = 3150;
    double mediumPower2 = 4371;
    double mediumPower3 = 4780;

    double highPower1 = 3760;
    double highPower2 = 4200;
    double highPower3 = 4840;


    double lowPower2NoHood = 2555;
    double lowPower3NoHood = 3155;
    double lowPower4NoHood = 3400;

    double mediumPower2NoHood = 4608;
    double mediumPower3NoHood = 4800;
    double mediumPower4NoHood = 5200;

    double highPower2NoHood;
    double highPower3NoHood;
    double highPower4NoHood;

    double interpolatePower;

    public ElapsedTime shootingTime = new ElapsedTime();
    ElapsedTime lookAhead = new ElapsedTime();
    ElapsedTime currentWait = new ElapsedTime();
    final double R1 = 183.286;
    final double R2 = 80.4;
    final double R3 = 80;
    final double R4 = 173.2;
    public double hoodTarget = 30;
    public double theta_4 = 30;


    public boolean inZone = false;
    boolean spunUp = false;
    public boolean intakeTime;
    boolean turretInRange = false;
    boolean intakeEnter;
    boolean currentSpike = false;
    boolean zoneResetStop = false;
    public boolean spinDown = false;
    public boolean Auto = false;
    public boolean toggle = true;




    public PIDController shootPID = new PIDController(0.013,0,0.00045);
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



        turretTurnOne.setOffset(163);
        turretTurnTwo.setOffset(163);
        turretTurnOne.setPosition(0);
        turretTurnTwo.setPosition(0);
        hoodAdjust.setPosition(0);
    }
    public double hoodDegreesToServoPoz(double theta_4){
        double K1 = R1 /R2;
        double K2 = R1 /R4;
        double K = (R1 * R1 + R2 * R2 + R4 * R4 - R3 * R3) / (2 * R2 * R4);
        double U = Math.max(54,Math.min(22,theta_4)) - 5.3;
        double A =  Math.sin(U);
        double B = K2 - Math.cos(U);
        double C = K1 * Math.cos(U) - K;
        double R = Math.sqrt(A * A + B * B);
        double psi = Math.atan2(A,B);
        double T2 = psi - Math.acos(-C/R) + 0.4;

        return Math.toDegrees(T2);


    }






    @Override
    public void execute() {
        executeEX();
        double deltaX = robotX;
        double deltaY = robotY;
        distance = Math.hypot(deltaY,deltaX);
        rpm = ((shooterMotorOne.getVelocity()/28)*60);
        if (shootPID.calculate(targetRPM,rpm) <0){
            shootPower = 0;
        }else {
            shootPower = shootPID.calculate(targetRPM,rpm);
        }
//        if (inZone && !zoneResetStop){
//            shootingTime.reset();
//            zoneResetStop = true;
//        }else if (inZone && shootingTime.milliseconds() > 1800){
//            spunUp = true;
//            intakeEnter = true;
//            zoneResetStop = false;
//
//        }else if(!inZone) {
//            spunUp = false;
//        }
//        if (spunUp && turretInRange && intakeEnter){
//            intake.reset();
//            intakeEnter = false;
//        }else if (spunUp && turretInRange && intake.milliseconds() < 1500){
//            intakeTime = true;
//        }else {
//            intakeTime = false;
//        }
//        if (spunUp && Math.abs(targetRPM - rpm )> 620){
//            currentWait.reset();
//            currentSpike = true;
//            shootingLevel = LowMediumHigh.low;
//        } else if (currentSpike && currentWait.milliseconds() > 1500) {
//            currentSpike = false;
//            shootingLevel = LowMediumHigh.medium;
//        }


        switch (shootingLevel) {
            case low:
                if (distance <= distance1) {

                    interpolatePower = lowPower1;

                } else if (distance <= distance2) {

                    interpolatePower = lowPower1 + (lowPower2NoHood - lowPower1) * (distance - distance1) / (distance2 - distance1);

                } else if (distance <= distance3) {


                    interpolatePower = lowPower2 + (lowPower3NoHood - lowPower2) * (distance - distance2) / (distance3 - distance2);

                } else if (distance <= distance4) {

                    interpolatePower = lowPower3 + (lowPower4NoHood - lowPower3) * (distance - distance3) / (distance4 - distance3);

                } else {

                    if (distance4 - distance3 != 0) {

                        double slope = (lowPower4NoHood - lowPower3) / (distance4 - distance3);
                        interpolatePower = lowPower3 + slope * (distance - distance3);

                    } else {

                        interpolatePower = lowPower4NoHood;

                    }
                }
                if (distance >= distance1 && distance < distance2) {
                   hoodTarget = hoodDegreesToServoPoz(lowHoodAngle1);

                } else if (distance >= distance2 && distance < distance3) {
                    hoodTarget = hoodDegreesToServoPoz(lowHoodAngle2);

                } else if (distance >= distance3) {
                    hoodTarget = hoodDegreesToServoPoz(lowHoodAngle3);
                }

                break;
            case medium:
                if (distance <= distance1) {

                    interpolatePower = mediumPower1;

                } else if (distance <= distance2) {

                    interpolatePower = mediumPower1 + (mediumPower2NoHood - mediumPower1) * (distance - distance1) / (distance2 - distance1);

                } else if (distance <= distance3) {


                    interpolatePower = mediumPower2 + (mediumPower3NoHood - mediumPower2) * (distance - distance2) / (distance3 - distance2);

                } else if (distance <= distance4) {

                    interpolatePower = mediumPower3 + (mediumPower4NoHood - mediumPower3) * (distance - distance3) / (distance4 - distance3);

                } else {

                    if (distance4 - distance3 != 0) {

                        double slope = (mediumPower4NoHood - mediumPower3) / (distance4 - distance3);
                        interpolatePower = mediumPower3 + slope * (distance - distance3);

                    } else {

                        interpolatePower = mediumPower4NoHood;

                    }
                }
                if (distance >= distance1 && distance < distance2) {
                    hoodAdjust.setPosition(mediumHoodAngle1);
                } else if (distance >= distance2 && distance < distance3) {
                    hoodAdjust.setPosition(mediumHoodAngle2);
                } else if (distance >= distance3) {
                    hoodAdjust.setPosition(mediumHoodAngle3);
                }
                break;
            case high:
                if (distance <= distance1) {

                    interpolatePower = highPower1;

                } else if (distance <= distance2) {

                    interpolatePower = highPower1 + (highPower2NoHood - highPower1) * (distance - distance1) / (distance2 - distance1);

                } else if (distance <= distance3) {


                    interpolatePower = highPower2 + (highPower3NoHood - highPower2) * (distance - distance2) / (distance3 - distance2);

                } else if (distance <= distance4) {

                    interpolatePower = highPower3 + (highPower4NoHood - highPower3) * (distance - distance3) / (distance4 - distance3);

                } else {

                    if (distance4 - distance3 != 0) {

                        double slope = (highPower4NoHood - highPower3) / (distance4 - distance3);
                        interpolatePower = highPower3 + slope * (distance - distance3);

                    } else {

                        interpolatePower = highPower4NoHood;

                    }
                }
                if (distance >= distance1) {
                    hoodAdjust.setPosition(highHoodAngle1);
                } else if (distance >= distance2) {
                    hoodAdjust.setPosition(highHoodAngle2);
                } else if (distance >= distance3) {
                    hoodAdjust.setPosition(highHoodAngle3);

                }


        }
        // *** Turret angle adjustment




        turretAngle = Math.toDegrees(-Math.atan2(deltaX,deltaY) + robotHeading);
        if ((turretAngle) > turretLimitAngle) {
            turretInRange = true;
            turretAngle = turretLimitAngle;

        } else if ((turretAngle) < -turretLimitAngle) {
            turretInRange = true;
            turretAngle = -turretLimitAngle;

        }else{
            turretInRange = false;
        }
//
            if ( (robotX > 130 && robotX < 240 && robotY > 260) || ((robotY + Yoffset < 200)&& (robotX + Xoffset < 200) && (robotX + Xoffset >= robotY + Yoffset))|| ((robotY + Yoffset < 108) && (robotX + Xoffset > 180) && (360- robotX+Xoffset >= robotY + Yoffset)) ){
                inZone = true;
            }else {
                inZone = false;
    }
        if (inZone && toggle) {
                targetRPM = interpolatePower + mapOfset;

                shooterMotorOne.update(shootPower);
                shooterMotorTwo.update(shootPower);
                turretTurnOne.setPosition(((turretAngle + turrofset) / gearRatio));
                turretTurnTwo.setPosition(((turretAngle + turrofset) / gearRatio));
        } else if (Auto) {
            targetRPM = interpolatePower;
            hoodTarget = hoodDegreesToServoPoz(theta_4);
            shooterMotorOne.update(shootPower);
            shooterMotorTwo.update(shootPower);
            turretTurnOne.setPosition(((turretAngle) / gearRatio));
            turretTurnTwo.setPosition(((turretAngle) / gearRatio));
            hoodAdjust.setPosition(hoodTarget);

        } else {
            shooterMotorTwo.update(0);
            shooterMotorOne.update(0);

        }



    }
}

