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

    public double hoodCompensation = 0;
    // Common distance points for interpolation
    double distance1 = 151;
    double distance2 = 237;
    double distance3 = 340;
    double distance4 = 407;


    // Low power settings
    double lowHoodAngle1 = 30.4;
    double lowHoodAngle2 = 36.99;
    double lowHoodAngle3 = 43.5;
    double lowHoodAngle4 = 43.5;

    double lowPower1 = 1606;
    double lowPower2 = 1806;
    double lowPower3 = 2528;
    double lowPower4 = 2528;

    // Medium power settings
    double mediumHoodAngle1 = 21;
    double mediumHoodAngle2 = 45;
    double mediumHoodAngle3 = 22;
    double mediumHoodAngle4 = 22;

    double mediumPower1 = 2306;
    double mediumPower2 = 2683;
    double mediumPower3 = 3401;
    double mediumPower4 = 3401;

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
    public boolean  testOP = false;


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





    public PIDController shootPID = new PIDController(0.004,0.000,0.00);

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




        turretTurnOne.setOffset(180);
        turretTurnTwo.setOffset(180);
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


    private double interpolateValue(double currentDistance, double d1, double v1, double d2, double v2, double d3, double v3, double d4, double v4) {
        if (currentDistance <= d1) {
            // Before the first point, return the first value (Clamping)
            return v1;
        } else if (currentDistance <= d2) {
            // Between point 1 and 2
            return v1 + (v2 - v1) * (currentDistance - d1) / (d2 - d1);
        } else if (currentDistance <= d3) {
            // Between point 2 and 3
            return v2 + (v3 - v2) * (currentDistance - d2) / (d3 - d2);
        } else if (currentDistance <= d4) {
            // Between point 3 and 4
            return v3 + (v4 - v3) * (currentDistance - d3) / (d4 - d3);
        } else {
            // After the last point, return the last value (Clamping/No extrapolation)
            return v4;
        }
    }


    @Override
    public void execute() {
        executeEX();
        double deltaX = robotX  - targetX;
        double deltaY = robotY  - targetY;
        distance = Math.hypot(deltaY,deltaX);
        rpm = ((shooterMotorOne.getVelocity()/28)*60);

        shootPower = Math.max(0,shootPID.calculate(targetRPM,rpm));

        diff = Math.abs(targetRPM - rpm);
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
                // Pass distance4 and lowPower4/lowHoodAngle4
                interpolatedPower = interpolateValue(distance, distance1, lowPower1, distance2, lowPower2, distance3, lowPower3, distance4, lowPower4);
                interpolatedHoodAngle = interpolateValue(distance, distance1, lowHoodAngle1, distance2, lowHoodAngle2, distance3, lowHoodAngle3, distance4, lowHoodAngle4);
                break;
            case medium:
                // Pass distance4 and mediumPower4/mediumHoodAngle4
                interpolatedPower = interpolateValue(distance, distance1, mediumPower1, distance2, mediumPower2, distance3, mediumPower3, distance4, mediumPower4);
                interpolatedHoodAngle = interpolateValue(distance, distance1, mediumHoodAngle1, distance2, mediumHoodAngle2, distance3, mediumHoodAngle3, distance4, mediumHoodAngle4);
                break;
            case high:
                // Add logic for high if needed
                break;
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
            if (!testOP) {
                targetRPM = interpolatedPower + mapOfset;
                setHoodDegrees(Math.max(17, interpolatedHoodAngle + hoodCompensation)); // Set hood based on interpolation
            }
            shooterMotorOne.update(shootPower);
            shooterMotorTwo.update(shootPower);
            turretTurnOne.setPosition(((turretAngle + turrofset) / gearRatio));
            turretTurnTwo.setPosition(((turretAngle + turrofset) / gearRatio));
        } else if (Auto) {
            targetRPM = interpolatedPower + mapOfset;
            setHoodDegrees( Math.max(17, interpolatedHoodAngle + hoodCompensation)); // Set hood based on interpolation


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
