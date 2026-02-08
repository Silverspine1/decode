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


/**
 * TURRET WITH SHOOTING WHILE MOVING
 *
 * This version uses velocities DIRECTLY from your Odometry subsystem
 * No velocity calculation needed - your Pinpoint already does it!
 *
 * Key changes from original:
 * - Uses odometry.getXVelocity(), getYVelocity(), getHVelocity()
 * - Handles CM units from odometry (converts internally)
 * - Properly handles wrapped heading from odometry.normilised()
 * - Anti-shake filtering with speed threshold
 */
public class Turret extends SubSystem {
    public MotorEx shooterMotorOne = new MotorEx();
    public MotorEx shooterMotorTwo = new MotorEx();

    public ServoDegrees turretTurnOne = new ServoDegrees();
    public ServoDegrees turretTurnTwo = new ServoDegrees();
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

    // Velocity from odometry (you'll set these from odometry subsystem)
    public double robotVelocityX = 0; // cm/s from odometry
    public double robotVelocityY = 0; // cm/s from odometry
    public double robotAngularVelocity = 0; // rad/s from odometry

    // Shooting while moving parameters
    public boolean enableShootingWhileMoving = true;

    // Mechanical lookahead time (time for turret/hood to reach target position)
    public double mechanicalLookaheadTime = 0.15; // seconds (150ms)

    // Minimum speed threshold to use shooting while moving
    // Below this speed, we ignore velocity prediction to prevent jitter
    public double minimumSpeedForPrediction = 5.0; // cm/s - tune this value (was 50 mm/s = 5 cm/s)

    // Velocity filtering to prevent shaking when stationary
    public double velocityDeadband = 1.0; // cm/s - velocities below this are treated as 0
    public double angularVelocityDeadband = 0.02; // rad/s - angular velocities below this are treated as 0

    // Predicted positions (for debugging/telemetry)
    public double predictedRobotX;
    public double predictedRobotY;
    public double predictedRobotHeading;
    public double estimatedFlightTime;
    public double totalLookaheadTime;

    public double targetRPM = 0;
    public double rpm;
    public double mapOfset = 0;
    public double turrofset = -1;
    public double turretAngle;
    public final double gearRatio = 0.72;
    final double turretLimitAngle = 120;

    public double distance;

    public double hoodCompensation = 0;

    // ===== HARDCODED FLIGHT TIMES =====
    // These are the flight times at your calibrated distances
    // Tune these based on actual testing
    public double flightTime1 = 0.35;  // Flight time at distance1 (151cm)
    public double flightTime2 = 0.45;  // Flight time at distance2 (236cm)
    public double flightTime3 = 0.55;  // Flight time at distance3 (336cm)
    public double flightTime4 = 0.65;  // Flight time at distance4 (413cm)

    // Common distance points for interpolation (IN CM to match odometry!)
    double distance1 = 15.1; // 151mm = 15.1cm
    double distance2 = 23.6; // 236mm = 23.6cm
    double distance3 = 33.6; // 336mm = 33.6cm
    double distance4 = 41.3; // 413mm = 41.3cm

    // Low power settings
    double lowHoodAngle1 = 35.7;
    double lowHoodAngle2 = 40.6;
    double lowHoodAngle3 = 41.8;
    double lowHoodAngle4 = 41.86;

    double lowPower1 = 1510;
    double lowPower2 = 1730;
    double lowPower3 = 2080;
    double lowPower4 = 2500;

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
    public boolean turretInRange = false;
    public boolean spinDown = false;
    public boolean Auto = false;
    public boolean toggle = true;
    public boolean testOP = false;

    double K1 = R1 / R2;
    double K2 = R1 / R4;
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

    public PIDController shootPID = new PIDController(0.004, 0.000, 0.00);

    public Turret(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
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
        turretTurnOne.setPosition(0);
        turretTurnTwo.setPosition(0);
        hoodAdjust.setOffset(60);
        setHoodDegrees(31);
    }

    public void setHoodDegrees(double theta) {
        double U2 = Math.toRadians(theta - 5);
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

    /**
     * Apply deadbands to velocities to filter noise
     * Call this after getting velocities from odometry
     */
    private void applyVelocityFiltering() {
        // Filter linear velocities
        if (Math.abs(robotVelocityX) < velocityDeadband) {
            robotVelocityX = 0;
        }
        if (Math.abs(robotVelocityY) < velocityDeadband) {
            robotVelocityY = 0;
        }

        // Filter angular velocity
        if (Math.abs(robotAngularVelocity) < angularVelocityDeadband) {
            robotAngularVelocity = 0;
        }
    }

    /**
     * Gets the flight time based on distance using interpolation
     * Uses hardcoded flight times that you tune based on real-world testing
     *
     * @param currentDistance Distance to target in CM (not MM!)
     * @return Estimated flight time in seconds
     */
    private double getFlightTime(double currentDistance) {
        return interpolateValue(
                currentDistance,
                distance1, flightTime1,
                distance2, flightTime2,
                distance3, flightTime3,
                distance4, flightTime4
        );
    }

    /**
     * Predicts where the robot will be at a future time based on current velocity
     *
     * @param lookaheadTime Time into the future in seconds
     * @return Array with [predictedX, predictedY, predictedHeading] in CM and radians
     */
    private double[] predictRobotPosition(double lookaheadTime) {
        double dt = lookaheadTime;

        // Velocities are in cm/s and rad/s - perfect!
        double predX = robotX + (robotVelocityX * dt);
        double predY = robotY + (robotVelocityY * dt);
        double predHeading = robotHeading + (robotAngularVelocity * dt);

        // Normalize predicted heading to [-π, π]
        while (predHeading > Math.PI) predHeading -= 2 * Math.PI;
        while (predHeading < -Math.PI) predHeading += 2 * Math.PI;

        return new double[]{predX, predY, predHeading};
    }

    /**
     * Calculates the predicted robot position accounting for movement
     * Updates the predicted position variables for use in execute()
     */
    private void calculatePrediction() {
        // Calculate robot speed in cm/s (velocities are already in cm/s from odometry)
        double robotSpeed = Math.hypot(robotVelocityX, robotVelocityY); // cm/s

        // If feature is disabled OR robot is barely moving, don't predict
        if (!enableShootingWhileMoving || robotSpeed < minimumSpeedForPrediction) {
            // Set predicted values to current (no prediction)
            predictedRobotX = robotX;
            predictedRobotY = robotY;
            predictedRobotHeading = robotHeading;
            estimatedFlightTime = 0;
            totalLookaheadTime = 0;
            return;
        }

        // Calculate initial distance (in CM)
        double deltaX = robotX - targetX;
        double deltaY = robotY - targetY;
        double initialDistance = Math.hypot(deltaY, deltaX);

        // Get flight time from hardcoded lookup table
        double flightTime = getFlightTime(initialDistance);

        // Total lookahead = mechanical lag + flight time
        totalLookaheadTime = mechanicalLookaheadTime + flightTime;

        // Predict robot position at shot time
        double[] predictedPos = predictRobotPosition(totalLookaheadTime);
        predictedRobotX = predictedPos[0];
        predictedRobotY = predictedPos[1];
        predictedRobotHeading = predictedPos[2];

        // Calculate distance from predicted position (for refined estimate)
        deltaX = predictedRobotX - targetX;
        deltaY = predictedRobotY - targetY;
        double predictedDistance = Math.hypot(deltaY, deltaX);

        // Refine flight time with new distance
        flightTime = getFlightTime(predictedDistance);
        totalLookaheadTime = mechanicalLookaheadTime + flightTime;

        // Store for telemetry
        estimatedFlightTime = flightTime;
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

        // Apply velocity filtering to remove noise
        applyVelocityFiltering();

        // Calculate predicted position
        calculatePrediction();

        // Determine if we should use prediction based on speed
        double robotSpeed = Math.hypot(robotVelocityX, robotVelocityY); // cm/s
        boolean usePrediction = enableShootingWhileMoving && (robotSpeed >= minimumSpeedForPrediction);

        // Use predicted position if moving fast enough, otherwise use current
        double calcRobotX = usePrediction ? predictedRobotX : robotX;
        double calcRobotY = usePrediction ? predictedRobotY : robotY;

        // Calculate delta to TARGET from predicted/current position (all in CM)
        double deltaX = calcRobotX - targetX;
        double deltaY = calcRobotY - targetY;
        distance = Math.hypot(deltaY, deltaX); // Distance in CM

        rpm = ((shooterMotorOne.getVelocity() / 28) * 60);
        shootPower = Math.max(0, shootPID.calculate(targetRPM, rpm));
        diff = Math.abs(targetRPM - rpm);

        // Zone detection (using actual robot position, not predicted)
        double ROBOT_OFFSET = 1.0; // Offset in CM
        double[] t1 = expandTriangle(0, 0, 18.0, 18.0, 36.0, 0, ROBOT_OFFSET);
        double[] t2 = expandTriangle(12.0, 36.0, 18.0, 30.0, 24.0, 36.0, ROBOT_OFFSET);

        if (pointInTriangle(robotX, robotY, t1[0], t1[1], t1[2], t1[3], t1[4], t1[5]) ||
                pointInTriangle(robotX, robotY, t2[0], t2[1], t2[2], t2[3], t2[4], t2[5])) {
            inZone = true;
        } else {
            inZone = false;
        }

        // Interpolate power and hood angle based on distance
        switch (shootingLevel) {
            case low:
                interpolatedPower = interpolateValue(distance, distance1, lowPower1, distance2, lowPower2, distance3, lowPower3, distance4, lowPower4);
                interpolatedHoodAngle = interpolateValue(distance, distance1, lowHoodAngle1, distance2, lowHoodAngle2, distance3, lowHoodAngle3, distance4, lowHoodAngle4);
                break;
            case medium:
                interpolatedPower = interpolateValue(distance, distance1, mediumPower1, distance2, mediumPower2, distance3, mediumPower3, distance4, mediumPower4);
                interpolatedHoodAngle = interpolateValue(distance, distance1, mediumHoodAngle1, distance2, mediumHoodAngle2, distance3, mediumHoodAngle3, distance4, mediumHoodAngle4);
                break;
            case high:
                break;
        }

        // Calculate turret angle using CURRENT heading, NOT predicted!
        // The turret angle formula already compensates for robot rotation
        // atan2 gives us the field-relative angle to target
        // Subtracting current heading converts to robot-relative angle
        // We do NOT want to use predicted heading here!
        turretAngle = Math.toDegrees(-Math.atan2(deltaX, deltaY) + robotHeading);

        // Turret limit checking
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

        // Apply shooting parameters
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

    /**
     * Get telemetry data for debugging shooting while moving
     */
    public String getShootingWhileMovingTelemetry() {
        double robotSpeed = Math.hypot(robotVelocityX, robotVelocityY); // cm/s
        boolean isPredicting = enableShootingWhileMoving &&
                robotSpeed >= minimumSpeedForPrediction;

        return String.format(
                "SWM Enabled: %b\n" +
                        "Prediction Active: %b\n" +
                        "Robot Speed: %.1f cm/s\n" +
                        "Robot Vel: (%.1f, %.1f) cm/s\n" +
                        "Angular Vel: %.3f rad/s\n" +
                        "Flight Time: %.3f s\n" +
                        "Mech Lookahead: %.3f s\n" +
                        "Total Lookahead: %.3f s\n" +
                        "Predicted Pos: (%.1f, %.1f) cm\n" +
                        "Current Pos: (%.1f, %.1f) cm\n" +
                        "Position Offset: (%.1f, %.1f) cm",
                enableShootingWhileMoving,
                isPredicting,
                robotSpeed,
                robotVelocityX, robotVelocityY,
                robotAngularVelocity,
                estimatedFlightTime,
                mechanicalLookaheadTime,
                totalLookaheadTime,
                predictedRobotX, predictedRobotY,
                robotX, robotY,
                predictedRobotX - robotX, predictedRobotY - robotY
        );
    }
}