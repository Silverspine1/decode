package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.List;

import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;

/**
 * Standalone Pinpoint diagnostic with AprilTag position reading/resetting
 *
 * Controls:
 * - Left stick = drive/strafe
 * - Right stick X = turn
 * - A = Shooters ON (0.5 power)
 * - B = Shooters OFF
 * - X = Intake ON
 * - Y = Intake OFF
 * - DPAD UP = Enable continuous Limelight reading
 * - DPAD DOWN = Disable continuous reading
 * - DPAD LEFT = Enable automatic position resetting from Limelight
 * - DPAD RIGHT = Disable automatic resetting
 * - LEFT BUMPER = Perform single reset using average of last 5 valid readings
 */
@TeleOp(name = "Pinpoint Diagnostic", group = "Diagnostic")
public class PinpointDiagnostic extends LinearOpMode {

    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;

    // Tracking variables - MATCHES YOUR ORIGINAL ODOMETRY CLASS
    public double X, Y, Heading, normilised;
    private double startX = 0;
    private double startY = 0;
    private double startHeading = 0;

    private double XVelocity = 0;
    private double YVelocity = 0;
    private double HVelocity = 0;

    // Drive motors
    private DcMotor LF, RF, RB, LB;

    // Shooter motors
    private DcMotor shooterMotorOne, shooterMotorTwo;

    // Intake motors
    private DcMotor intakeMotor;

    // Motor control states
    private boolean shootersOn = false;
    private boolean intakeOn = false;

    // Limelight control states
    private boolean limelightReadingEnabled = false;
    private boolean limelightResettingEnabled = false;

    // Limelight position data
    private double limelightX = 0;
    private double limelightY = 0;
    private double limelightH = 0;
    private boolean limelightValid = false;

    // Buffer for storing recent valid readings
    private static final int READING_BUFFER_SIZE = 5;
    private List<double[]> readingBuffer = new ArrayList<>();

    // Button state tracking for single-press detection
    private boolean lastLeftBumper = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    public Servo turretTurnOne;
    @Override
    public void runOpMode() throws InterruptedException {

        // ==================== INITIALIZATION - MATCHES YOUR ODOMETRY CLASS ====================

        telemetry.addLine("Initializing Pinpoint...");
        telemetry.update();

        // Get Pinpoint from hardware map
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Initialize drive motors
        try {

            LF = hardwareMap.get(DcMotor.class, "LF");
            RF = hardwareMap.get(DcMotor.class, "RF");
            LB = hardwareMap.get(DcMotor.class, "LB");
            RB = hardwareMap.get(DcMotor.class, "RB");

            LF.setDirection(DcMotorSimple.Direction.REVERSE);
            LB.setDirection(DcMotorSimple.Direction.REVERSE);

            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addLine("Drive motors initialized");
        } catch (Exception e) {
            telemetry.addLine("Warning: Drive motors not found");
        }

        // Initialize shooter motors
        try {
            shooterMotorOne = hardwareMap.get(DcMotor.class, "shooterMotorOne");
            shooterMotorTwo = hardwareMap.get(DcMotor.class, "shooterMotorTwo");

            shooterMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);

            shooterMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            telemetry.addLine("Shooter motors initialized");
        } catch (Exception e) {
            telemetry.addLine("Warning: Shooter motors not found");
        }

        // Initialize intake motor
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addLine("Intake motor initialized");
        } catch (Exception e) {
            telemetry.addLine("Warning: Intake motor not found");
        }

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetry.addLine("Limelight initialized");
        } catch (Exception e) {
            telemetry.addLine("Warning: Limelight not found");
        }
        turretTurnOne = hardwareMap.get(Servo.class,"turretTurnOne");
        turretTurnOne.setPosition(0.5);

        telemetry.update();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // Set offsets - EXACT MATCH to your code
        odo.setOffsets(-13, 132, DistanceUnit.MM);

        // Set encoder resolution - EXACT MATCH
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions - EXACT MATCH
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        telemetry.addLine("Keep robot STATIONARY - calibrating IMU...");
        telemetry.update();

        // Reset and calibrate IMU - EXACT MATCH
        odo.resetPosAndIMU();

        // Wait for IMU stabilization - EXACT MATCH
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        // Initialize tracking variables - EXACT MATCH
        X = startX;
        Y = startY;
        Heading = startHeading;
        normilised = Math.toRadians(startHeading);

        telemetry.clear();
        telemetry.addLine("Ready - Press PLAY");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left stick = drive/strafe");
        telemetry.addLine("Right stick X = turn");
        telemetry.addLine("A = Shooters ON (0.5 power)");
        telemetry.addLine("B = Shooters OFF");
        telemetry.addLine("X = Intake ON");
        telemetry.addLine("Y = Intake OFF");
        telemetry.addLine();
        telemetry.addLine("Limelight Controls:");
        telemetry.addLine("DPAD UP = Enable reading");
        telemetry.addLine("DPAD DOWN = Disable reading");
        telemetry.addLine("DPAD LEFT = Enable auto reset");
        telemetry.addLine("DPAD RIGHT = Disable auto reset");
        telemetry.addLine("LEFT BUMPER = Single reset (avg 5)");
        telemetry.update();

        waitForStart();

        // ==================== EXECUTE - MATCHES YOUR ODOMETRY CLASS ====================

        // Reset position once at start - EXACT MATCH to your execute() behavior
        odo.setPosX(startX, DistanceUnit.CM);
        odo.setPosY(startY, DistanceUnit.CM);
        odo.setHeading(Math.toRadians(startHeading), AngleUnit.RADIANS);

        telemetry.setMsTransmissionInterval(50);

        // ==================== MAIN LOOP - EXACT MATCH TO YOUR UPDATE LAMBDA ====================

        while (opModeIsActive()) {


            if (gamepad1.start){
                turretTurnOne.setPosition(0.7);
            }else if(gamepad1.back) {
                turretTurnOne.setPosition(0.3);

            }else{
                    turretTurnOne.setPosition(0.5);


            }
            // ==================== GAMEPAD CONTROLS ====================

            // Drive controls
            double vertical = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (LF != null && RF != null && LB != null && RB != null) {
                double denominator = Math.max(1.0, Math.abs(vertical) + Math.abs(strafe) + Math.abs(turn));
                LF.setPower((vertical - strafe - turn) / denominator);
                RF.setPower((vertical + strafe + turn) / denominator);
                LB.setPower((vertical + strafe - turn) / denominator);
                RB.setPower((vertical - strafe + turn) / denominator);
            }

            // Shooter controls - A to turn on, B to turn off
            if (gamepad1.a && !shootersOn) {
                shootersOn = true;
                if (shooterMotorOne != null) shooterMotorOne.setPower(0.5);
                if (shooterMotorTwo != null) shooterMotorTwo.setPower(0.5);
            }
            if (gamepad1.b && shootersOn) {
                shootersOn = false;
                if (shooterMotorOne != null) shooterMotorOne.setPower(0);
                if (shooterMotorTwo != null) shooterMotorTwo.setPower(0);
            }

            // Intake controls - X to turn on, Y to turn off
            if (gamepad1.x && !intakeOn) {
                intakeOn = true;
                if (intakeMotor != null) intakeMotor.setPower(1.0);
            }
            if (gamepad1.y && intakeOn) {
                intakeOn = false;
                if (intakeMotor != null) intakeMotor.setPower(0);
            }

            // ==================== LIMELIGHT CONTROLS ====================

            // DPAD UP - Enable continuous reading
            if (gamepad1.dpad_up && !lastDpadUp) {
                limelightReadingEnabled = true;
            }
            lastDpadUp = gamepad1.dpad_up;

            // DPAD DOWN - Disable continuous reading
            if (gamepad1.dpad_down && !lastDpadDown) {
                limelightReadingEnabled = false;
            }
            lastDpadDown = gamepad1.dpad_down;

            // DPAD LEFT - Enable automatic resetting
            if (gamepad1.dpad_left && !lastDpadLeft) {
                limelightResettingEnabled = true;
            }
            lastDpadLeft = gamepad1.dpad_left;

            // DPAD RIGHT - Disable automatic resetting
            if (gamepad1.dpad_right && !lastDpadRight) {
                limelightResettingEnabled = false;
            }
            lastDpadRight = gamepad1.dpad_right;

            // LEFT BUMPER - Single reset using average of last 5 readings
            if (gamepad1.left_bumper && !lastLeftBumper) {
                performSingleReset();
            }
            lastLeftBumper = gamepad1.left_bumper;

            // ==================== LIMELIGHT READING AND PROCESSING ====================

            if (limelight != null && limelightReadingEnabled) {
                // Get latest result from limelight
                LLResult llResult = limelight.getLatestResult();

                if (llResult != null && llResult.isValid()) {
                    // Extract position data - MATCHES YOUR APRILTAG CLASS EXACTLY
                    double rawX = llResult.getBotpose().getPosition().y * 100; // cm
                    double rawY = llResult.getBotpose().getPosition().x * 100; // cm
                    double rawH = llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);

                    // Apply transformations - MATCHES YOUR APRILTAG CLASS
                    limelightX = rawX + 180;
                    limelightY = rawY + 180;
                    limelightH = 180 - rawH;
                    limelightValid = true;

                    // Filter out invalid readings
                    if (isValidReading(limelightX, limelightY, limelightH)) {
                        // Add to buffer for averaging
                        addToBuffer(limelightX, limelightY, limelightH);

                        // If automatic resetting is enabled, update odometry position
                        if (limelightResettingEnabled) {
                            applyLimelightPosition(limelightX, limelightY, limelightH);
                        }
                    }
                } else {
                    limelightValid = false;
                }
            }

            // ==================== ODOMETRY UPDATE ====================

            // STEP 1: Update the Pinpoint - EXACT MATCH
            odo.update();

            // STEP 2: Get velocities - EXACT MATCH
            XVelocity = odo.getVelX(DistanceUnit.CM);
            YVelocity = odo.getVelY(DistanceUnit.CM);
            HVelocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

            // STEP 3: Calculate heading - EXACT MATCH
            normilised = Math.toRadians(startHeading) + odo.getHeading(AngleUnit.RADIANS);

            // STEP 4: Calculate Heading in degrees - EXACT MATCH
            double rawHeadingDegrees = odo.getHeading(AngleUnit.DEGREES);
            if (rawHeadingDegrees < 0) {
                Heading = rawHeadingDegrees + 360;
            } else {
                Heading = rawHeadingDegrees;
            }

            // STEP 5: Update position - EXACT MATCH
            X = startX - odo.getPosX(DistanceUnit.CM);
            Y = startY + odo.getPosY(DistanceUnit.CM);

            // ==================== TELEMETRY ====================

            telemetry.addLine("=== ODOMETRY ===");
            telemetry.addData("X", "%.2f cm", X);
            telemetry.addData("Y", "%.2f cm", Y);
            telemetry.addData("Heading", "%.2f°", Heading);
            telemetry.addLine();
            telemetry.addData("X Vel", "%.2f cm/s", XVelocity);
            telemetry.addData("Y Vel", "%.2f cm/s", YVelocity);
            telemetry.addData("H Vel", "%.3f rad/s", HVelocity);
            telemetry.addLine();

            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Reading", limelightReadingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Auto Reset", limelightResettingEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Valid", limelightValid ? "YES" : "NO");
            if (limelightReadingEnabled) {
                telemetry.addData("LL X", "%.2f cm", limelightX);
                telemetry.addData("LL Y", "%.2f cm", limelightY);
                telemetry.addData("LL H", "%.2f°", limelightH);
                telemetry.addData("Buffer Size", "%d/5", readingBuffer.size());
            }
            telemetry.addLine();

            telemetry.addLine("=== MOTORS ===");
            telemetry.addData("Shooters", shootersOn ? "ON (0.5)" : "OFF");
            telemetry.addData("Intake", intakeOn ? "ON (1.0)" : "OFF");

            telemetry.update();
        }
    }

    // ==================== HELPER METHODS ====================

    /**
     * Checks if a reading is valid (filters out bad data)
     * Rejects readings at 0,0,0 or with heading near 0/180/360 without valid position
     */
    private boolean isValidReading(double x, double y, double h) {
        // Reject if position is at origin (0, 0) - likely invalid
        if (Math.abs(x) < 1.0 && Math.abs(y) < 1.0) {
            return false;
        }

        // Reject if position is at 180, 180 (your offset default) with heading at special angles
        if (Math.abs(x - 180) < 1.0 && Math.abs(y - 180) < 1.0) {
            // Check if heading is at a special angle (0, 180, 360, etc)
            double normalizedH = h % 360;
            if (Math.abs(normalizedH) < 1.0 || Math.abs(normalizedH - 180) < 1.0 || Math.abs(normalizedH - 360) < 1.0) {
                return false;
            }
        }

        // Reject if any value is NaN or infinite
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(h) ||
                Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(h)) {
            return false;
        }

        return true;
    }

    /**
     * Adds a reading to the buffer, maintaining max size
     */
    private void addToBuffer(double x, double y, double h) {
        readingBuffer.add(new double[]{x, y, h});

        // Keep only the most recent readings
        while (readingBuffer.size() > READING_BUFFER_SIZE) {
            readingBuffer.remove(0);
        }
    }

    /**
     * Applies limelight position directly to odometry
     */
    private void applyLimelightPosition(double x, double y, double h) {
        // Update the startX, startY, startHeading offsets so calculations work correctly
        startX = x - odo.getPosX(DistanceUnit.CM);
        startY = y + odo.getPosY(DistanceUnit.CM);
        startHeading = h;

        // Recalculate current position with new offsets
        X = startX - odo.getPosX(DistanceUnit.CM);
        Y = startY + odo.getPosY(DistanceUnit.CM);
        Heading = h;
    }

    /**
     * Performs a single reset using the average of the last 5 valid readings
     */
    private void performSingleReset() {
        if (readingBuffer.isEmpty()) {
            // No readings available
            return;
        }

        // Calculate average of all readings in buffer
        double avgX = 0;
        double avgY = 0;
        double avgH = 0;

        for (double[] reading : readingBuffer) {
            avgX += reading[0];
            avgY += reading[1];
            avgH += reading[2];
        }

        avgX /= readingBuffer.size();
        avgY /= readingBuffer.size();
        avgH /= readingBuffer.size();

        // Apply the averaged position
        applyLimelightPosition(avgX, avgY, avgH);

        // Clear buffer after reset
        readingBuffer.clear();
    }
}
