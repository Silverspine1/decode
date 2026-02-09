package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Turret extends SubSystem {

    public double vTangential = 0;
    public double vRadial = 0;
    public MotorEx shooterMotorOne = new MotorEx();
    public MotorEx shooterMotorTwo = new MotorEx();

    public ServoDegrees turretTurnOne = new ServoDegrees();
    public ServoDegrees turretTurnTwo =new ServoDegrees();
    public ServoDegrees hoodAdjust = new ServoDegrees();

    public static double TURRET_COMP_FACTOR = 0.9;
    public static boolean FLIP_PERPENDICULAR = true;  // Toggle on dashboard to flip tangential direction

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
    public double robotXVelo = 0;
    public double robotYVelo = 0;
    public double robotHeadingVelo = 0;

    public double targetRPM = 0;
    public double rpm;
    public double mapOfset = 0;
    public double turrofset= -1;
    public double turretAngle;
    public final double gearRatio = 0.72;
    final double turretLimitAngle =120;

    public double distance;
    public double ofsetDistance;

    public double lastDistance = 0;
    public double distanceVelocity = 0;

    public double hoodCompensation = 0;

    double distance1 = 151;
    double distance2 = 236;
    double distance3 = 336;
    double distance4 = 413;

    double lowHoodAngle1 = 35.7;
    double lowHoodAngle2 = 40.6;
    double lowHoodAngle3 = 41.8;
    double lowHoodAngle4 = 41.86;

    double lowPower1 = 1510;
    double lowPower2 = 1730;
    double lowPower3 = 2080;
    double lowPower4 = 2500;

    double lowTOF4 = 1.12;
    double lowTOF3 = 1.063;
    double lowTOF2 = 0.965;
    double lowTOF1 = 0.924;

    double mediumHoodAngle1 = 21;
    double mediumHoodAngle2 = 45;
    double mediumHoodAngle3 = 22;
    double mediumHoodAngle4 = 22;

    double mediumPower1 = 2306;
    double mediumPower2 = 2683;
    double mediumPower3 = 3401;
    double mediumPower4 = 3401;

    double mediumTOF4 = 1.12;
    double mediumTOF3 = 1.063;
    double mediumTOF2 = 0.965;
    double mediumTOF1 = 0.924;

    double turretLast = 0;

    double interpolatedPower;
    double interpolatedHoodAngle;
    public double interpolatedTOF;

    ElapsedTime turretToCenter = new ElapsedTime();
    ElapsedTime distanceTimer = new ElapsedTime();

    final double R1 = 183;
    final double R2 = 63;
    final double R3 = 63;
    final double R4 = 153;

    public boolean inZone = false;
    public boolean intakeTime;
    public boolean turretInRange = false;
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

    public boolean stopTurret = false;

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

        turretTurnOne.setOffset(181.3);
        turretTurnTwo.setOffset(195);
        hoodAdjust.setDirection(Servo.Direction.FORWARD);

        hoodAdjust.setOffset(60);

        distanceTimer.reset();
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
            return v1;
        } else if (currentDistance <= d2) {
            return v1 + (v2 - v1) * (currentDistance - d1) / (d2 - d1);
        } else if (currentDistance <= d3) {
            return v2 + (v3 - v2) * (currentDistance - d2) / (d3 - d2);
        } else if (currentDistance <= d4) {
            return v3 + (v4 - v3) * (currentDistance - d3) / (d4 - d3);
        } else {
            return v4;
        }
    }

    static double sign(double x1, double y1, double x2, double y2, double x3, double y3) {
        return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    }

    static boolean pointInTriangle(double px, double py,
                                   double x1, double y1,
                                   double x2, double y2,
                                   double x3, double y3) {
        double d1 = sign(px, py, x1, y1, x2, y2);
        double d2 = sign(px, py, x2, y2, x3, y3);
        double d3 = sign(px, py, x3, y3, x1, y1);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }

    static double[] expandTriangle(double x1, double y1,
                                   double x2, double y2,
                                   double x3, double y3, double offset) {
        double cx = (x1 + x2 + x3) / 3.0;
        double cy = (y1 + y2 + y3) / 3.0;

        double dx1 = x1 - cx, dy1 = y1 - cy;
        double len1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
        x1 += (dx1 / len1) * offset;
        y1 += (dy1 / len1) * offset;

        double dx2 = x2 - cx, dy2 = y2 - cy;
        double len2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);
        x2 += (dx2 / len2) * offset;
        y2 += (dy2 / len2) * offset;

        double dx3 = x3 - cx, dy3 = y3 - cy;
        double len3 = Math.sqrt(dx3 * dx3 + dy3 * dy3);
        x3 += (dx3 / len3) * offset;
        y3 += (dy3 / len3) * offset;

        return new double[]{x1, y1, x2, y2, x3, y3};
    }

    @Override
    public void execute() {
        executeEX();

        double deltaX = robotX - targetX;
        double deltaY = robotY - targetY;
        distance = Math.hypot(deltaY, deltaX);

        // Calculate distance velocity (rate of change of distance)
        double dt = distanceTimer.seconds();
        if (dt > 0.001) {  // Avoid division by zero
            distanceVelocity = (distance - lastDistance) / dt;
            lastDistance = distance;
            distanceTimer.reset();
        }


        rpm = ((shooterMotorOne.getVelocity()/28)*60);

        shootPower = Math.max(0, shootPID.calculate(targetRPM, rpm));
        diff = Math.abs(targetRPM - rpm);

        double ROBOT_OFFSET = 20.0;
        double[] t1 = expandTriangle(0, 0, 180, 180, 360, 0, 15);
        double[] t2 = expandTriangle(100, 360, 180, 275, 295, 360, 0);

        if (pointInTriangle(robotX, robotY, t1[0], t1[1], t1[2], t1[3], t1[4], t1[5]) ||
                pointInTriangle(robotX, robotY, t2[0], t2[1], t2[2], t2[3], t2[4], t2[5])) {
            inZone = true;
        } else {
            inZone = false;
        }

        switch (shootingLevel) {
            case low:
                interpolatedTOF = interpolateValue(distance, distance1, lowTOF1, distance2, lowTOF2, distance3, lowTOF3, distance4, lowTOF4);
                ofsetDistance = distanceVelocity * interpolatedTOF;
                interpolatedPower = interpolateValue(distance + ofsetDistance, distance1, lowPower1, distance2, lowPower2, distance3, lowPower3, distance4, lowPower4);
                interpolatedHoodAngle = interpolateValue(distance + ofsetDistance, distance1, lowHoodAngle1, distance2, lowHoodAngle2, distance3, lowHoodAngle3, distance4, lowHoodAngle4);
                break;
            case medium:
                interpolatedTOF = interpolateValue(distance, distance1, mediumTOF1, distance2, mediumTOF2, distance3, mediumTOF3, distance4, mediumTOF4);
                ofsetDistance = distanceVelocity * interpolatedTOF;
                interpolatedPower = interpolateValue(distance + ofsetDistance, distance1, mediumPower1, distance2, mediumPower2, distance3, mediumPower3, distance4, mediumPower4);
                interpolatedHoodAngle = interpolateValue(distance + ofsetDistance, distance1, mediumHoodAngle1, distance2, mediumHoodAngle2, distance3, mediumHoodAngle3, distance4, mediumHoodAngle4);
                break;
            case high:
                break;
        }

        double baseTurretAngle = Math.toDegrees(-Math.atan2(deltaX, deltaY) + robotHeading);

        // Calculate velocity components
        double compensationAngle = 0;
        if (distance > 0.01) {
            deltaX = robotX - targetX;
            deltaY = robotY - targetY;
            double dist = Math.hypot(deltaY, deltaX);
            double dirX = (dist > 0.01) ? deltaX / dist : 0;
            double dirY = (dist > 0.01) ? deltaY / dist : 0;

            // Calculate tangential and radial velocities
            vTangential = robotXVelo * dirX + robotYVelo * dirY;

            vRadial =  robotXVelo * (-dirY) + robotYVelo * dirX;

            // Calculate drift and compensation using tangential velocity
            double drift = vTangential * interpolatedTOF * TURRET_COMP_FACTOR;
            compensationAngle = Math.toDegrees(Math.atan2(drift, distance));
        }

        turretAngle = baseTurretAngle + compensationAngle;

        if ((turretAngle) > turretLimitAngle) {
            turretInRange = false;
            turretAngle = 0;
            turretToCenter.reset();
        } else if ((turretAngle) < -turretLimitAngle) {
            turretInRange = false;
            turretAngle = 0;
            turretToCenter.reset();
        } else if (turretToCenter.milliseconds() > 500) {
            turretInRange = true;
        }

        if (toggle) {
            if (!testOP) {
                targetRPM = interpolatedPower + mapOfset;
                setHoodDegrees(Math.max(17, interpolatedHoodAngle + hoodCompensation));
            }
            shooterMotorOne.update(shootPower);
            shooterMotorTwo.update(shootPower);
            if (!stopTurret) {
                turretTurnOne.setPosition(((turretAngle + turrofset) / gearRatio));
                turretTurnTwo.setPosition(((turretAngle + turrofset) / gearRatio));
            }
        } else if (Auto) {
            targetRPM = interpolatedPower + mapOfset;
            setHoodDegrees(Math.max(17, interpolatedHoodAngle + hoodCompensation));

            shooterMotorOne.update(shootPower);
            shooterMotorTwo.update(shootPower);
            if (!stopTurret) {
                turretTurnOne.setPosition(((turretAngle + turrofset) / gearRatio));
                turretTurnTwo.setPosition(((turretAngle + turrofset) / gearRatio));
            }
        } else {
            shooterMotorTwo.update(0);
            shooterMotorOne.update(0);
        }
    }
}