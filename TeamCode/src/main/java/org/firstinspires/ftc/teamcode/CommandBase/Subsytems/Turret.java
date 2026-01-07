package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
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


    public double targetX =  0;
    public double targetY = 0;
    public double robotX;
    public double robotY;
    public double robotHeading;

    public double targetRPM = 0;
    public double rpm;
    public double mapOfset = -20;
    public double turrofset= 0;
    public double turretAngle;
    final double gearRatio = 0.73;
    final double turretLimitAngle =120;


    public double distance;

    public double hoodRpmDropCompensation = 0;
    // Common distance points for interpolation
    double distance1 = 126;
    double distance2 = 234;
    double distance3 = 343;
    double distance4 = 407; // This can be used for extrapolation endpoint


    // Low power settings
    double lowHoodAngle1 = 27.7;
    double lowHoodAngle2 = 38.4;
    double lowHoodAngle3 = 42.6;
    double lowPower1 = 2039;
    double lowPower2 = 2533;
    double lowPower3 = 3127;
    // Medium power settings
    double mediumHoodAngle1 = 21;
    double mediumHoodAngle2 = 45;
    double mediumHoodAngle3 = 22;
    double mediumPower1 = 2306;
    double mediumPower2 = 2683;
    double mediumPower3 = 3401;
    double turretLast = 0;


    double interpolatedPower;
    double interpolatedHoodAngle;


    public ElapsedTime shootingTime = new ElapsedTime();
    ElapsedTime lookAhead = new ElapsedTime();
    ElapsedTime currentWait = new ElapsedTime();
    ElapsedTime turretToCenter = new ElapsedTime();

    final double R1 = 183;
    final double R2 = 63;
    final double R3 = 63;
    final double R4 = 153;


    public boolean inZone = false;

    public boolean intakeTime;
    boolean turretInRange = false;
    public boolean spinDown = false;
    public boolean Auto = false;
    public boolean toggle = true;


    double K1 = R1 /R2;
    double K2 = R1 /R4;
    public double K = (R1 * R1 + R2 * R2 + R4 * R4 - R3 * R3) / (2 * R2 * R4);

    public double U;
    public double U2;

    double A;
    double B;
    double C;
    double R;
    double psi;
    public double T2;
    public double diff = 0;





    public PIDController shootPID = new PIDController(0.009,0,0.0004);
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
        turretTurnOne.setDirection(Servo.Direction.REVERSE);
        turretTurnTwo.setDirection(Servo.Direction.REVERSE);
        turretTurnTwo.setRange(355);
        turretTurnOne.setRange(355);
        hoodAdjust.setRange(355);
        shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);




        turretTurnOne.setOffset(189);
        turretTurnTwo.setOffset(189);
        hoodAdjust.setDirection(Servo.Direction.FORWARD);
        turretTurnOne.setPosition(0);
        turretTurnTwo.setPosition(0);
        hoodAdjust.setOffset(60);
        setHoodDegrees(31);



    }


    public void setHoodDegrees(double theta){

        double U2 = Math.toRadians(theta-5);
        A = Math.sin(U2);
        B = K2 - Math.cos(U2);
        C = K1 * Math.cos(U2) - K;
        R = Math.sqrt(A * A + B * B);
        psi = Math.atan2(A, B);

        double ratio = -C / R;
        ratio = Math.max(-1, Math.min(1, ratio));
        T2 = Math.toDegrees(psi - Math.acos(ratio));

        hoodAdjust.setPosition(T2);


    }


    private double interpolateValue(double currentDistance, double d1, double v1, double d2, double v2, double d3, double v3) {
        if (currentDistance <= d1) {
            // Before the first point, hold the first value
            return v1;
        } else if (currentDistance <= d2) {
            // Between point 1 and 2
            return v1 + (v2 - v1) * (currentDistance - d1) / (d2 - d1);
        } else if (currentDistance <= d3) {
            // Between point 2 and 3
            return v2 + (v3 - v2) * (currentDistance - d2) / (d3 - d2);
        } else {
            // After the last point, extrapolate from the last two points
            if (d3 - d2 != 0) {
                double slope = (v3 - v2) / (d3 - d2);
                return v3 + slope * (currentDistance - d3);
            } else {
                return v3; // Fallback if distances are the same
            }
        }
    }


    @Override
    public void execute() {
        executeEX();
        double deltaX = robotX  - targetX;
        double deltaY = robotY  - targetY;
        distance = Math.hypot(deltaY,deltaX);
        rpm = ((shooterMotorOne.getVelocity()/28)*60);
        if (shootPID.calculate(targetRPM,rpm) <0){
            shootPower = 0;
        }else {
            shootPower = shootPID.calculate(targetRPM,rpm);
        }
        diff = Math.abs(targetRPM - rpm);
        if (hoodRpmDropCompensation<6 && diff >130) {
            hoodRpmDropCompensation =  (((lowHoodAngle2 - lowHoodAngle1 + lowHoodAngle3 - lowHoodAngle2) / 2) / ((lowPower2 - lowPower1 + lowPower3 - lowPower2) / 2)) * -diff / 2;
        }else {
            hoodRpmDropCompensation = 0;
        }
//        if (inZone && !zoneResetStop || Auto){
//            shootingTime.reset();
//            zoneResetStop = true;
//        }else if (inZone && shootingTime.milliseconds() > 1800 || Auto){
//            spunUp = true;
//            intakeEnter = true;
//            zoneResetStop = false;
//
//        }else if(!inZone && !Auto) {
//            spunUp = false;
//        }




        switch (shootingLevel) {
            case low:
                // Interpolate power for the 'low' setting
                interpolatedPower = interpolateValue(distance, distance1, lowPower1, distance2, lowPower2, distance3, lowPower3);
                // Interpolate hood angle for the 'low' setting
                interpolatedHoodAngle = interpolateValue(distance, distance1, lowHoodAngle1, distance2, lowHoodAngle2, distance3, lowHoodAngle3);
                break;
            case medium:
                // Interpolate power for the 'medium' setting
                interpolatedPower = interpolateValue(distance, distance1, mediumPower1, distance2, mediumPower2, distance3, mediumPower3);
                // Interpolate hood angle for the 'medium' setting
                interpolatedHoodAngle = interpolateValue(distance, distance1, mediumHoodAngle1, distance2, mediumHoodAngle2, distance3, mediumHoodAngle3);
                break;
            case high:

                break;
        }
        if(robotY>220){
            turrofset = 4;

        }

        // *** Turret angle adjustment
        turretAngle = Math.toDegrees(-Math.atan2(deltaX,deltaY) + robotHeading);
        if ((turretAngle) > turretLimitAngle ) {
            turretInRange = false;
            turretAngle = 0;
            turretToCenter.reset();

        } else if ((turretAngle) < -turretLimitAngle ) {
            turretInRange = false;
            turretAngle = 0;
            turretToCenter.reset();

        }else if (turretToCenter.milliseconds() > 500) {
            turretInRange = true;
        }

        if ( (robotX > 160 && robotX < 260 && robotY > 270) || ((robotY + Yoffset < 180)&& (robotX + Xoffset < 180) && (robotX + Xoffset >= robotY + Yoffset))|| ((robotY + Yoffset < 180) && (robotX + Xoffset > 180) && (360- robotX+Xoffset >= robotY + Yoffset)) ){
            inZone = true;
        }else {
            inZone = false;
        }

        if (toggle){
            targetRPM = interpolatedPower + mapOfset ;
            setHoodDegrees(Math.max(25, interpolatedHoodAngle  )); // Set hood based on interpolation

            shooterMotorOne.update(shootPower);
            shooterMotorTwo.update(shootPower);
            turretTurnOne.setPosition(((turretAngle + turrofset) / gearRatio));
            turretTurnTwo.setPosition(((turretAngle + turrofset) / gearRatio));
        } else if (Auto) {
            targetRPM = interpolatedPower = mapOfset;
            setHoodDegrees( Math.max(25, interpolatedHoodAngle + hoodRpmDropCompensation/0.4)); // Set hood based on interpolation


            shooterMotorOne.update(shootPower);
            shooterMotorTwo.update(shootPower);
            turretTurnOne.setPosition(((turretAngle+ turrofset) / gearRatio));
            turretTurnTwo.setPosition(((turretAngle+ turrofset) / gearRatio));

        } else {
            shooterMotorTwo.update(0);
            shooterMotorOne.update(0);
        }
    }
}
