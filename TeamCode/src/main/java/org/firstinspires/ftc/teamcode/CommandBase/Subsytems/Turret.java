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

    // Robot velocity tracking
    public double robotVelocityX = 0;
    public double robotVelocityY = 0;
    public double robotAngularVelocity = 0;

    private double lastRobotX = 0;
    private double lastRobotY = 0;
    private double lastRobotHeading = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();

    // Velocity filtering to prevent shaking when stationary
    private static final double VELOCITY_DEADBAND = 10.0; // mm/s - ignore velocities below this
    private static final double ANGULAR_VELOCITY_DEADBAND = 0.02; // rad/s - ignore angular velocities below this
    private static final double MIN_UPDATE_TIME = 0.02; // seconds - minimum time between velocity updates

    // Shooting while moving parameters
    public boolean enableShootingWhileMoving = false;

    // Mechanical lookahead time (time for turret/hood to reach target position)
    // Tune this based on your actual mechanism response time
    public double mechanicalLookaheadTime = 0.15; // seconds (150ms)

    // Minimum speed threshold to use shooting while moving
    // Below this speed, we ignore velocity prediction to prevent jitter
    public double minimumSpeedForPrediction = 50.0; // mm/s - tune this value

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

    // Common distance points for interpolation
    double distance1 = 151;
    double distance2 = 236;
    double distance3 = 336;
    double distance4 = 413;

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

        hoodAdjust.setOffset(60);

        // Initialize velocity tracking
        velocityTimer.reset();
        lastRobotX = robotX;
        lastRobotY = robotY;
        lastRobotHeading = robotHeading;
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
     * Updates robot velocity based on position changes
     * Call this method regularly (every loop) for accurate velocity tracking
     *
     * IMPORTANT: Your odometry already provides velocity! Consider using those instead.
     * However, this method gives you control over filtering and deadbands.
     *
     * Note: robotHeading from odometry.normilised() wraps to approximately [-π, π]
     * (it's startHeading + current angle, which can wrap)
     */
    private void updateVelocity() {
        double dt = velocityTimer.seconds();

        // Only update if enough time has passed (prevents division issues and excessive noise)
        if (dt < MIN_UPDATE_TIME) {
            return;
        }

        if (dt > 0.001) { // Avoid division by zero
            // Calculate linear velocities in cm/s (your odometry uses CM not MM!)
            // Convert to mm/s for consistency with distance calculations
            double rawVelX = ((robotX - lastRobotX) / dt) * 10; // cm/s to mm/s
            double rawVelY = ((robotY - lastRobotY) / dt) * 10; // cm/s to mm/s

            // Apply deadband to prevent noise when stationary
            if (Math.abs(rawVelX) < VELOCITY_DEADBAND) {
                robotVelocityX = 0;
            } else {
                robotVelocityX = rawVelX;
            }

            if (Math.abs(rawVelY) < VELOCITY_DEADBAND) {
                robotVelocityY = 0;
            } else {
                robotVelocityY = rawVelY;
            }

            // Calculate angular velocity in rad/s
            // robotHeading wraps, so we need to handle wrap-around
            double deltaHeading = robotHeading - lastRobotHeading;

            // Handle wrap-around (e.g., from π to -π or vice versa)
            if (deltaHeading > Math.PI) {
                deltaHeading -= 2 * Math.PI;
            } else if (deltaHeading < -Math.PI) {
                deltaHeading += 2 * Math.PI;
            }

            double rawAngularVel = deltaHeading / dt;

            // Apply deadband to angular velocity
            if (Math.abs(rawAngularVel) < ANGULAR_VELOCITY_DEADBAND) {
                robotAngularVelocity = 0;
            } else {
                robotAngularVelocity = rawAngularVel;
            }

            // Update last values
            lastRobotX = robotX;
            lastRobotY = robotY;
            lastRobotHeading = robotHeading;
            velocityTimer.reset();
        }
    }

    // ===== HARDCODED FLIGHT TIMES =====
    // These are the flight times at your calibrated distances
    // Tune these based on actual testing
    public double flightTime1 = 0.35;  // Flight time at distance1 (151mm)
    public double flightTime2 = 0.45;  // Flight time at distance2 (236mm)
    public double flightTime3 = 0.55;  // Flight time at distance3 (336mm)
    public double flightTime4 = 0.65;  // Flight time at distance4 (413mm)

    /**
     * Gets the flight time based on distance using interpolation
     * Uses hardcoded flight times that you tune based on real-world testing
     *
     * @param currentDistance Distance to target in mm
     * @return Estimated flight time in seconds
     */
    private double getFlightTime(double currentDistance) {
        // Use the same interpolation as power/hood angle, but for flight time
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
     * Assumes constant velocity (reasonable for short time periods)
     *
     * @param lookaheadTime Time into the future in seconds
     * @return Array with [predictedX, predictedY, predictedHeading]
     */
    private double[] predictRobotPosition(double lookaheadTime) {
        double dt = lookaheadTime;

        // Positions are in CM from odometry, velocities are in mm/s
        // Convert velocities back to cm/s for prediction
        double predX = robotX + ((robotVelocityX / 10.0) * dt); // mm/s to cm/s
        double predY = robotY + ((robotVelocityY / 10.0) * dt); // mm/s to cm/s

        // Heading prediction (handles wrapping)
        double predHeading = robotHeading + (robotAngularVelocity * dt);

        // Normalize predicted heading to [-π, π]
        while (predHeading > Math.PI) predHeading -= 2 * Math.PI;
        while (predHeading < -Math.PI) predHeading += 2 * Math.PI;

        return new double[]{predX, predY, predHeading};
    }

    /**
     * Calculates the effective target position accounting for robot movement
     * This is where the robot will be when the ball reaches the target
     *
     * @return Array with [effectiveTargetX, effectiveTargetY]
     */
    private double[] calculateEffectiveTarget() {
        // Calculate robot speed
        double robotSpeed = Math.hypot(robotVelocityX, robotVelocityY);
        double totalSpeed = Math.hypot(robotSpeed, Math.abs(robotAngularVelocity * 100)); // Scale angular for comparison

        // If feature is disabled OR robot is barely moving, use current position
        if (!enableShootingWhileMoving || totalSpeed < minimumSpeedForPrediction) {
            // Return current position - no prediction needed
            return new double[]{targetX, targetY};
        }

        // Calculate initial distance
        double deltaX = robotX - targetX;
        double deltaY = robotY - targetY;
        double initialDistance = Math.hypot(deltaY, deltaX);

        // Get flight time from hardcoded lookup table based on distance
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

        // Return the target position (unchanged) - we'll use predicted robot position for calculations
        return new double[]{targetX, targetY};
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

        // Update robot velocity first
        updateVelocity();

        // Calculate effective target position (accounts for movement)
        double[] effectiveTarget = calculateEffectiveTarget();

        // Use predicted robot position for calculations when shooting while moving is enabled
        double calcRobotX = enableShootingWhileMoving ? predictedRobotX : robotX;
        double calcRobotY = enableShootingWhileMoving ? predictedRobotY : robotY;
        double calcRobotHeading = enableShootingWhileMoving ? predictedRobotHeading : robotHeading;

        // Calculate delta from predicted position to target
        double deltaX = calcRobotX - effectiveTarget[0];
        double deltaY = calcRobotY - effectiveTarget[1];
        distance = Math.hypot(deltaY, deltaX);

        rpm = ((shooterMotorOne.getVelocity() / 28) * 60);
        shootPower = Math.max(0, shootPID.calculate(targetRPM, rpm));
        diff = Math.abs(targetRPM - rpm);

        // Zone detection (using actual robot position, not predicted)
        double ROBOT_OFFSET = 10.0;
        double[] t1 = expandTriangle(0, 0, 180, 180, 360, 0, ROBOT_OFFSET);
        double[] t2 = expandTriangle(120, 360, 180, 300, 240, 360, ROBOT_OFFSET);

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
                // Add logic for high if needed
                break;
        }

        // Calculate turret angle using predicted position and heading
        turretAngle = Math.toDegrees(-Math.atan2(deltaX, deltaY) + calcRobotHeading);

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
     * Add this data to your telemetry output
     */
    public String getShootingWhileMovingTelemetry() {
        double robotSpeed = Math.hypot(robotVelocityX, robotVelocityY);
        boolean isPredicting = enableShootingWhileMoving &&
                robotSpeed >= minimumSpeedForPrediction;

        return String.format(
                "SWM Enabled: %b\n" +
                        "Prediction Active: %b\n" +
                        "Robot Speed: %.1f mm/s\n" +
                        "Robot Vel: (%.1f, %.1f) mm/s\n" +
                        "Angular Vel: %.3f rad/s\n" +
                        "Flight Time: %.3f s\n" +
                        "Mech Lookahead: %.3f s\n" +
                        "Total Lookahead: %.3f s\n" +
                        "Predicted Pos: (%.1f, %.1f)\n" +
                        "Current Pos: (%.1f, %.1f)\n" +
                        "Position Offset: (%.1f, %.1f)",
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