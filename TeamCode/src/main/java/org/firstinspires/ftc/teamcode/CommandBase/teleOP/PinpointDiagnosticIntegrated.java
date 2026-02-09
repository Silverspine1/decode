//package org.firstinspires.ftc.teamcode.CommandBase.teleOP;
//
//import android.util.Size;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
//import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.ArrayList;
//import java.util.List;
//
//import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
//
///**
// * Integrated Pinpoint Diagnostic extending OpModeEX
// *
// * FEATURE FLAGS (set in initEX):
// * - useOdometryEX: Use odometry through OpModeEX subsystem vs standalone
// * - useDriveEX: Use DriveBase subsystem vs standalone motors
// * - useTurretEX: Use Turret subsystem vs standalone shooters
// * - useIntakeEX: Use Intake subsystem vs standalone motor
// * - useAprilTagEX: Use AprilTag subsystem vs standalone Limelight
// * - useLocalVision: Enable local vision processing
// *
// * Controls:
// * - Right stick Y = forward/backward (matches TeleOp)
// * - Left/Right triggers = strafe (matches TeleOp)
// * - Right stick X = turn
// * - A = Toggle shooters/turret (same behavior as TeleOp)
// * - B = Turret OFF
// * - X = Intake ON
// * - Y = Intake OFF
// * - START = Limelight reset (like TeleOp)
// * - BACK = Switch target side (blue/red)
// * - DPAD UP = Toggle turret on/off
// * - DPAD DOWN = Ball pickup mode (with vision if enabled)
// * - DPAD LEFT/RIGHT = Turret offset adjust
// * - LEFT BUMPER = Conditional intake (like TeleOp logic)
// * - RIGHT BUMPER = Force intake block ON
// */
//@TeleOp(name = "Pinpoint Diagnostic Integrated", group = "Diagnostic")
//public class PinpointDiagnosticIntegrated extends OpModeEX {
//
//    // ==================== FEATURE FLAGS ====================
//    // Set these in initEX() to control which subsystems to use
//    private boolean useOdometryEX = false;         // Use OpModeEX odometry subsystem
//    private boolean useDriveEX = false;            // Use DriveBase subsystem
//    private boolean useTurretEX = false;           // Use Turret subsystem
//    private boolean useIntakeEX = false;           // Use Intake subsystem
//    private boolean useAprilTagEX = false;         // Use AprilTag subsystem
//    private boolean useLocalVision = false;        // Enable local vision processing
//
//    // ==================== STANDALONE COMPONENTS ====================
//    // These are used when the corresponding EX flag is false
//
//    // Standalone Pinpoint odometry
//    private GoBildaPinpointDriver standaloneOdo;
//    private double standaloneX = 0;
//    private double standaloneY = 0;
//    private double standaloneHeading = 0;
//    private double standaloneNormalized = 0;
//    private double standaloneXVel = 0;
//    private double standaloneYVel = 0;
//    private double standaloneHVel = 0;
//    private double startX = 0;
//    private double startY = 0;
//    private double startHeading = 0;
//
//    // Standalone drive motors
//    private DcMotor standaloneLF, standaloneRF, standaloneRB, standaloneLB;
//
//    // Standalone shooter motors
//    private DcMotor standaloneShooterOne, standaloneShooterTwo;
//    private boolean standaloneShootersOn = false;
//    private double standaloneShooterPower = 0.5;
//
//    // Standalone intake motor
//    private DcMotor standaloneIntakeMotor;
//    private boolean standaloneIntakeOn = false;
//    private boolean standaloneIntakeBlock = true;
//
//    // ==================== VISION ====================
//    private VisionPortal visionPortal;
//    private LocalVision processor;
//    private PIDController visionHeadingPID = new PIDController(0.013, 0, 0.0032);
//
//    // ==================== CONTROL STATES ====================
//    private boolean blue = true;  // Blue side = true, Red side = false
//    private boolean toggle = false;  // Master turret toggle
//    private double targetHood = 25;
//    private double lastBallCount = 0;
//    private double currentBallCount = 0;
//
//    // Limelight reset state
//    private boolean limelightAutoReset = false;
//    private boolean rest = false;
//
//    // Limelight position buffer
//    private static final int READING_BUFFER_SIZE = 5;
//    private List<double[]> readingBuffer = new ArrayList<>();
//
//    // Timing
//    private ElapsedTime shooterOffWait = new ElapsedTime();
//
//    @Override
//    public void initEX() {
//
//        // ==================== SET FEATURE FLAGS HERE ====================
//        // Change these to switch between standalone and OpModeEX subsystems
//        useOdometryEX = false;      // Use OpModeEX odometry (odometry subsystem)
//        useDriveEX = false;         // Use DriveBase subsystem
//        useTurretEX = false;        // Use Turret subsystem
//        useIntakeEX = false;        // Use Intake subsystem
//        useAprilTagEX = false;      // Use AprilTag subsystem (Limelight)
//        useLocalVision = false;     // Enable local vision processing
//
//        // ==================== SETUP DASHBOARD ====================
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//
//        telemetry.addLine("========================================");
//        telemetry.addLine("PINPOINT DIAGNOSTIC - INTEGRATED");
//        telemetry.addLine("========================================");
//        telemetry.addLine();
//        telemetry.addLine("Feature Flags:");
//        telemetry.addData("Odometry EX", useOdometryEX);
//        telemetry.addData("Drive EX", useDriveEX);
//        telemetry.addData("Turret EX", useTurretEX);
//        telemetry.addData("Intake EX", useIntakeEX);
//        telemetry.addData("AprilTag EX", useAprilTagEX);
//        telemetry.addData("Local Vision", useLocalVision);
//        telemetry.addLine();
//        telemetry.update();
//
//        // ==================== ODOMETRY INITIALIZATION ====================
//
//        if (!useOdometryEX) {
//            // Initialize standalone Pinpoint (OpModeEX odometry won't be used)
//            telemetry.addLine("Initializing standalone Pinpoint...");
//            telemetry.update();
//
//            standaloneOdo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//
//            // Set offsets - EXACT MATCH to your Odometry class
//            standaloneOdo.setOffsets(-13, 132, DistanceUnit.MM);
//
//            // Set encoder resolution
//            standaloneOdo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//
//            // Set encoder directions
//            standaloneOdo.setEncoderDirections(
//                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
//                    GoBildaPinpointDriver.EncoderDirection.FORWARD
//            );
//
//            telemetry.addLine("Keep robot STATIONARY - calibrating IMU...");
//            telemetry.update();
//
//            // Reset and calibrate IMU
//            standaloneOdo.resetPosAndIMU();
//
//            // Wait for IMU stabilization
//            try {
//                Thread.sleep(300);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//
//            // Initialize tracking variables
//            standaloneX = startX;
//            standaloneY = startY;
//            standaloneHeading = startHeading;
//            standaloneNormalized = Math.toRadians(startHeading);
//
//            telemetry.addLine("Standalone odometry initialized");
//        } else {
//            // OpModeEX odometry is already initialized by parent class
//            telemetry.addLine("Using OpModeEX odometry subsystem");
//        }
//        telemetry.update();
//
//        // ==================== DRIVE INITIALIZATION ====================
//
//        if (!useDriveEX) {
//            // Initialize standalone drive motors
//            telemetry.addLine("Initializing standalone drive motors...");
//            telemetry.update();
//
//            try {
//                standaloneLF = hardwareMap.get(DcMotor.class, "LF");
//                standaloneRF = hardwareMap.get(DcMotor.class, "RF");
//                standaloneLB = hardwareMap.get(DcMotor.class, "LB");
//                standaloneRB = hardwareMap.get(DcMotor.class, "RB");
//
//                standaloneLF.setDirection(DcMotorSimple.Direction.REVERSE);
//                standaloneLB.setDirection(DcMotorSimple.Direction.REVERSE);
//
//                standaloneLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                standaloneRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                standaloneLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                standaloneRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                telemetry.addLine("Standalone drive motors initialized");
//            } catch (Exception e) {
//                telemetry.addLine("WARNING: Drive motors not found");
//                telemetry.addLine("Error: " + e.getMessage());
//            }
//        } else {
//            // OpModeEX DriveBase is already initialized by parent class
//            telemetry.addLine("Using OpModeEX DriveBase subsystem");
//        }
//        telemetry.update();
//
//        // ==================== TURRET/SHOOTER INITIALIZATION ====================
//
//        if (!useTurretEX) {
//            // Initialize standalone shooter motors
//            telemetry.addLine("Initializing standalone shooter motors...");
//            telemetry.update();
//
//            try {
//                standaloneShooterOne = hardwareMap.get(DcMotor.class, "shooterMotorOne");
//                standaloneShooterTwo = hardwareMap.get(DcMotor.class, "shooterMotorTwo");
//
//                standaloneShooterOne.setDirection(DcMotorSimple.Direction.REVERSE);
//                standaloneShooterTwo.setDirection(DcMotorSimple.Direction.REVERSE);
//
//                standaloneShooterOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                standaloneShooterTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//                telemetry.addLine("Standalone shooter motors initialized");
//            } catch (Exception e) {
//                telemetry.addLine("WARNING: Shooter motors not found");
//                telemetry.addLine("Error: " + e.getMessage());
//            }
//        } else {
//            // OpModeEX Turret is already initialized by parent class
//            turret.toggle = false;
//            telemetry.addLine("Using OpModeEX Turret subsystem");
//        }
//        telemetry.update();
//
//        // ==================== INTAKE INITIALIZATION ====================
//
//        if (!useIntakeEX) {
//            // Initialize standalone intake motor
//            telemetry.addLine("Initializing standalone intake motor...");
//            telemetry.update();
//
//            try {
//                standaloneIntakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//                standaloneIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                telemetry.addLine("Standalone intake motor initialized");
//            } catch (Exception e) {
//                telemetry.addLine("WARNING: Intake motor not found");
//                telemetry.addLine("Error: " + e.getMessage());
//            }
//        } else {
//            // OpModeEX Intake is already initialized by parent class
//            telemetry.addLine("Using OpModeEX Intake subsystem");
//        }
//        telemetry.update();
//
//        // ==================== APRILTAG/LIMELIGHT INITIALIZATION ====================
//
//        if (useAprilTagEX) {
//            // OpModeEX AprilTag is already initialized by parent class
//            Apriltag.limelight.pipelineSwitch(0);
//            telemetry.addLine("Using OpModeEX AprilTag subsystem");
//        } else {
//            telemetry.addLine("Using standalone Limelight (if available)");
//            // Limelight is accessed directly through hardware map when needed
//        }
//        telemetry.update();
//
//        // ==================== LOCAL VISION INITIALIZATION ====================
//
//        if (useLocalVision) {
//            telemetry.addLine("Initializing local vision...");
//            telemetry.update();
//
//            processor = new LocalVision(LocalVision.TargetColor.BOTH);
//
//            VisionPortal.Builder builder = new VisionPortal.Builder();
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//            builder.setCameraResolution(new Size(640, 480));
//            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//            builder.enableLiveView(false);
//            builder.addProcessor(processor);
//
//            visionPortal = builder.build();
//            dashboard.startCameraStream(visionPortal, 15);
//
//            telemetry.addLine("Local vision initialized");
//        } else {
//            telemetry.addLine("Local vision disabled");
//        }
//        telemetry.update();
//
//        // ==================== FINAL SETUP ====================
//
//        telemetry.clear();
//        telemetry.addLine("========================================");
//        telemetry.addLine("INITIALIZATION COMPLETE");
//        telemetry.addLine("========================================");
//        telemetry.addLine();
//        telemetry.addLine("Active Systems:");
//        telemetry.addData("Odometry", useOdometryEX ? "OpModeEX" : "Standalone");
//        telemetry.addData("Drive", useDriveEX ? "OpModeEX DriveBase" : "Standalone");
//        telemetry.addData("Turret", useTurretEX ? "OpModeEX Turret" : "Standalone Shooters");
//        telemetry.addData("Intake", useIntakeEX ? "OpModeEX Intake" : "Standalone");
//        telemetry.addData("AprilTag", useAprilTagEX ? "OpModeEX Limelight" : "Standalone");
//        telemetry.addData("Vision", useLocalVision ? "Enabled" : "Disabled");
//        telemetry.addLine();
//        telemetry.addLine("Ready - Press PLAY");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        // Start OpModeEX AprilTag if enabled
//        if (useAprilTagEX) {
//            Apriltag.limelight.start();
//        }
//
//        // Reset standalone odometry position if using standalone
//        if (!useOdometryEX && standaloneOdo != null) {
//            standaloneOdo.setPosX(startX, DistanceUnit.CM);
//            standaloneOdo.setPosY(startY, DistanceUnit.CM);
//            standaloneOdo.setHeading(Math.toRadians(startHeading), AngleUnit.RADIANS);
//        }
//
//        telemetry.setMsTransmissionInterval(50);
//    }
//
//    @Override
//    public void loopEX() {
//
//        // ==================== ODOMETRY UPDATE ====================
//
//        // Get current odometry values (from either source)
//        double currentX, currentY, currentHeading, currentNormalized;
//        double currentXVel, currentYVel, currentHVel;
//
//        if (useOdometryEX) {
//            // Use OpModeEX odometry
//            currentX = odometry.X();
//            currentY = odometry.Y();
//            currentHeading = odometry.Heading();
//            currentNormalized = odometry.normiliased();
//            currentXVel = odometry.getXVelocity();
//            currentYVel = odometry.getYVelocity();
//            currentHVel = odometry.getHVelocity();
//        } else {
//            // Update standalone odometry - EXACT MATCH to your Odometry class
//            if (standaloneOdo != null) {
//                standaloneOdo.update();
//
//                standaloneXVel = standaloneOdo.getVelX(DistanceUnit.CM);
//                standaloneYVel = standaloneOdo.getVelY(DistanceUnit.CM);
//                standaloneHVel = standaloneOdo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
//
//                standaloneNormalized = Math.toRadians(startHeading) + standaloneOdo.getHeading(AngleUnit.RADIANS);
//
//                double rawHeadingDegrees = standaloneOdo.getHeading(AngleUnit.DEGREES);
//                if (rawHeadingDegrees < 0) {
//                    standaloneHeading = rawHeadingDegrees + 360;
//                } else {
//                    standaloneHeading = rawHeadingDegrees;
//                }
//
//                standaloneX = startX - standaloneOdo.getPosX(DistanceUnit.CM);
//                standaloneY = startY + standaloneOdo.getPosY(DistanceUnit.CM);
//            }
//
//            currentX = standaloneX;
//            currentY = standaloneY;
//            currentHeading = standaloneHeading;
//            currentNormalized = standaloneNormalized;
//            currentXVel = standaloneXVel;
//            currentYVel = standaloneYVel;
//            currentHVel = standaloneHVel;
//        }
//
//        // ==================== UPDATE TURRET POSITION (if using TurretEX) ====================
//
//        if (useTurretEX) {
//            turret.robotX = currentX;
//            turret.robotY = currentY;
//            turret.robotHeading = currentNormalized;
//            turret.robotVelocityX = currentXVel;
//            turret.robotVelocityY = currentYVel;
//            turret.robotAngularVelocity = currentHVel;
//        }
//
//        // ==================== BALL COUNT TRACKING ====================
//
//        lastBallCount = currentBallCount;
//        if (useIntakeEX) {
//            currentBallCount = intake.ballCount;
//        } else {
//            // Standalone doesn't have ball counting, default to 0
//            currentBallCount = 0;
//        }
//
//        // ==================== DRIVE CONTROLS ====================
//
//        // TeleOp style controls: Right stick Y, triggers for strafe, right stick X for turn
//        double vertical = -gamepad1.right_stick_y;
//        double strafe = (gamepad1.left_trigger - gamepad1.right_trigger);
//        double turn = -gamepad1.right_stick_x;
//
//        // Override for ball pickup mode with vision
//        boolean visionDriveActive = false;
//        if (gamepad1.dpad_down && useLocalVision && processor != null && processor.hasTarget) {
//            visionDriveActive = true;
//            vertical = -gamepad1.right_stick_y + processor.distanceCm / 50;
//            turn = visionHeadingPID.calculate(-processor.hAngleDeg);
//            strafe = -gamepad1.right_stick_x;
//
//            // Auto-enable intake in vision mode
//            if (useIntakeEX) {
//                intake.block = true;
//                intake.InTake = true;
//            } else {
//                standaloneIntakeBlock = true;
//                standaloneIntakeOn = true;
//            }
//        }
//
//        if (useDriveEX) {
//            // Use OpModeEX DriveBase
//            driveBase.drivePowers(vertical, strafe, turn);
//        } else {
//            // Use standalone drive motors
//            if (standaloneLF != null && standaloneRF != null && standaloneLB != null && standaloneRB != null) {
//                double denominator = Math.max(1.0, Math.abs(vertical) + Math.abs(strafe) + Math.abs(turn));
//                standaloneLF.setPower((vertical - strafe - turn) / denominator);
//                standaloneRF.setPower((vertical + strafe + turn) / denominator);
//                standaloneLB.setPower((vertical + strafe - turn) / denominator);
//                standaloneRB.setPower((vertical - strafe + turn) / denominator);
//            }
//        }
//
//        // ==================== INTAKE CONTROLS ====================
//
//        // RIGHT BUMPER - Force intake ON (like TeleOp)
//        if (gamepad1.right_bumper) {
//            if (useIntakeEX) {
//                intake.block = true;
//                intake.InTake = true;
//            } else {
//                standaloneIntakeBlock = true;
//                standaloneIntakeOn = true;
//            }
//        }
//        // LEFT BUMPER - Conditional intake (matches TeleOp logic)
//        else if (currentGamepad1.left_bumper) {
//            if (useIntakeEX) {
//                if (!intake.InTake && useTurretEX && turret.diff < 170 ||
//                    useTurretEX && turret.inZone && turret.diff < 170 && turret.toggle && turret.turretInRange) {
//                    intake.InTake = true;
//                    intake.block = false;
//                }
//            } else {
//                standaloneIntakeOn = true;
//                standaloneIntakeBlock = false;
//            }
//        }
//        // Turret intakeTime trigger (only if using TurretEX)
//        else if (useTurretEX && turret.intakeTime) {
//            if (useIntakeEX) {
//                intake.InTake = true;
//            } else {
//                standaloneIntakeOn = true;
//            }
//        }
//        // Turn off when no buttons pressed
//        else if (!currentGamepad1.left_bumper && !currentGamepad1.right_bumper && !visionDriveActive) {
//            if (useIntakeEX) {
//                intake.InTake = false;
//            } else {
//                standaloneIntakeOn = false;
//            }
//        }
//
//        // Apply standalone intake motor power
//        if (!useIntakeEX && standaloneIntakeMotor != null) {
//            standaloneIntakeMotor.setPower(standaloneIntakeOn ? 1.0 : 0.0);
//        }
//
//        // ==================== TURRET/SHOOTER CONTROLS ====================
//
//        // Hood compensation based on ball count
//        if (useTurretEX) {
//            if (currentBallCount > 1) {
//                turret.hoodCompensation = -2;
//            } else {
//                turret.hoodCompensation = 0;
//            }
//        }
//
//        // Auto-enable shooters when ball count > 2
//        if (currentBallCount > 2) {
//            if (toggle) {
//                if (useTurretEX) {
//                    turret.toggle = true;
//                } else {
//                    standaloneShootersOn = true;
//                }
//            }
//            shooterOffWait.reset();
//        } else if (currentBallCount < 1 && shooterOffWait.milliseconds() > 500) {
//            if (useTurretEX) {
//                turret.toggle = false;
//            } else {
//                standaloneShootersOn = false;
//            }
//        }
//
//        // DPAD UP - Toggle turret/shooters
//        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up) {
//            if (useTurretEX) {
//                if (turret.toggle) {
//                    toggle = false;
//                    turret.toggle = false;
//                } else {
//                    toggle = true;
//                    turret.toggle = true;
//                }
//            } else {
//                standaloneShootersOn = !standaloneShootersOn;
//            }
//            gamepad1.rumble(800);
//        }
//
//        // DPAD LEFT/RIGHT - Turret offset adjust (only if using TurretEX)
//        if (useTurretEX) {
//            if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left) {
//                turret.turrofset -= 3;
//            }
//            if (!lastGamepad1.dpad_right && currentGamepad1.dpad_right) {
//                turret.turrofset += 3;
//            }
//        }
//
//        // Apply standalone shooter power
//        if (!useTurretEX) {
//            if (standaloneShooterOne != null) {
//                standaloneShooterOne.setPower(standaloneShootersOn ? standaloneShooterPower : 0);
//            }
//            if (standaloneShooterTwo != null) {
//                standaloneShooterTwo.setPower(standaloneShootersOn ? standaloneShooterPower : 0);
//            }
//        }
//
//        // ==================== APRILTAG/LIMELIGHT RESET ====================
//
//        // START button - Limelight reset (like TeleOp)
//        if (!lastGamepad1.start && currentGamepad1.start || currentBallCount > 2) {
//            double limelightH = 0;
//            double limelightX = 0;
//            double limelightY = 0;
//            boolean validReset = false;
//
//            if (useAprilTagEX) {
//                limelightH = Apriltag.getH();
//                limelightX = Apriltag.getX();
//                limelightY = Apriltag.getY();
//                validReset = (limelightH != 0 && limelightH != 180);
//            } else {
//                // Could implement standalone Limelight reading here if needed
//                // For now, skip
//            }
//
//            if (validReset) {
//                toggle = true;
//                gamepad1.rumble(800);
//                rest = true;
//
//                if (useOdometryEX) {
//                    odometry.odo.setPosX(-limelightX, DistanceUnit.CM);
//                    odometry.odo.setPosY(limelightY, DistanceUnit.CM);
//                    odometry.odo.setHeading(-limelightH, AngleUnit.DEGREES);
//                } else if (standaloneOdo != null) {
//                    standaloneOdo.setPosX(-limelightX, DistanceUnit.CM);
//                    standaloneOdo.setPosY(limelightY, DistanceUnit.CM);
//                    standaloneOdo.setHeading(-limelightH, AngleUnit.DEGREES);
//                }
//            }
//        }
//
//        // ==================== TARGET SIDE SWITCH ====================
//
//        // BACK button - Switch between blue and red side
//        if (!lastGamepad1.back && currentGamepad1.back) {
//            blue = !blue;
//            gamepad1.rumble(800);
//
//            if (useTurretEX) {
//                if (blue) {
//                    turret.targetX = 0;
//                } else {
//                    turret.targetX = 360;
//                }
//                turret.turrofset = 0;
//            }
//        }
//
//        // ==================== HANG CONTROLS ====================
//
//        // A button - Toggle engage (matches TeleOp)
//        if (!lastGamepad1.a && currentGamepad1.a) {
//            if (useDriveEX) {
//                driveBase.engage = !driveBase.engage;
//            }
//        }
//
//        // ==================== TELEMETRY ====================
//
//        telemetry.addLine("=== ACTIVE SYSTEMS ===");
//        telemetry.addData("Odometry", useOdometryEX ? "OpModeEX" : "Standalone");
//        telemetry.addData("Drive", useDriveEX ? "OpModeEX" : "Standalone");
//        telemetry.addData("Turret", useTurretEX ? "OpModeEX" : "Standalone");
//        telemetry.addData("Intake", useIntakeEX ? "OpModeEX" : "Standalone");
//        telemetry.addData("AprilTag", useAprilTagEX ? "OpModeEX" : "Standalone");
//        telemetry.addLine();
//
//        telemetry.addLine("=== ODOMETRY ===");
//        telemetry.addData("X", "%.2f cm", currentX);
//        telemetry.addData("Y", "%.2f cm", currentY);
//        telemetry.addData("Heading", "%.2f°", currentHeading);
//        telemetry.addData("Normalized", "%.3f rad", currentNormalized);
//        telemetry.addData("X Vel", "%.2f cm/s", currentXVel);
//        telemetry.addData("Y Vel", "%.2f cm/s", currentYVel);
//        telemetry.addData("H Vel", "%.3f rad/s", currentHVel);
//        telemetry.addLine();
//
//        if (useTurretEX) {
//            telemetry.addLine("=== TURRET (OpModeEX) ===");
//            telemetry.addData("Toggle", turret.toggle);
//            telemetry.addData("In Zone", turret.inZone);
//            telemetry.addData("Distance", "%.1f cm", turret.distance);
//            telemetry.addData("Diff", "%.1f°", turret.diff);
//            telemetry.addData("Turret Angle", "%.1f°", turret.turretAngle / turret.gearRatio + 180);
//            telemetry.addData("Target RPM", "%.0f", turret.targetRPM);
//            telemetry.addData("Hood Comp", "%.1f", turret.hoodCompensation);
//            telemetry.addData("Turret Offset", "%.1f", turret.turrofset);
//            telemetry.addLine();
//            telemetry.addLine("=== Movement Debug ===");
//            telemetry.addLine(turret.getShootingWhileMovingTelemetry());
//        } else {
//            telemetry.addLine("=== SHOOTERS (Standalone) ===");
//            telemetry.addData("Shooters", standaloneShootersOn ? "ON" : "OFF");
//            telemetry.addData("Power", "%.2f", standaloneShooterPower);
//        }
//        telemetry.addLine();
//
//        if (useIntakeEX) {
//            telemetry.addLine("=== INTAKE (OpModeEX) ===");
//            telemetry.addData("Intake", intake.InTake ? "ON" : "OFF");
//            telemetry.addData("Block", intake.block ? "CLOSED" : "OPEN");
//            telemetry.addData("Ball Count", intake.ballCount);
//            telemetry.addData("Intake RPM", intake.secondIntakeMotor.getVelocity());
//        } else {
//            telemetry.addLine("=== INTAKE (Standalone) ===");
//            telemetry.addData("Intake", standaloneIntakeOn ? "ON" : "OFF");
//            telemetry.addData("Block", standaloneIntakeBlock ? "CLOSED" : "OPEN");
//        }
//        telemetry.addLine();
//
//        if (useAprilTagEX) {
//            telemetry.addLine("=== LIMELIGHT (OpModeEX) ===");
//            telemetry.addData("X", "%.2f cm", Apriltag.getX());
//            telemetry.addData("Y", "%.2f cm", Apriltag.getY());
//            telemetry.addData("H", "%.2f°", Apriltag.getH());
//            telemetry.addLine();
//        }
//
//        if (useLocalVision && processor != null) {
//            telemetry.addLine("=== LOCAL VISION ===");
//            telemetry.addData("Has Target", processor.hasTarget);
//            if (processor.hasTarget) {
//                telemetry.addData("Ball X", "%.1f cm", processor.xPosCm);
//                telemetry.addData("Distance", "%.1f cm", processor.distanceCm);
//                telemetry.addData("H Angle", "%.1f°", processor.hAngleDeg);
//            }
//            telemetry.addLine();
//        }
//
//        telemetry.addLine("=== CONTROL STATE ===");
//        telemetry.addData("Side", blue ? "BLUE (0,0)" : "RED (360,360)");
//        telemetry.addData("Master Toggle", toggle);
//        telemetry.addData("Vision Drive", visionDriveActive ? "ACTIVE" : "OFF");
//        if (useDriveEX) {
//            telemetry.addData("Engage", driveBase.engage);
//        }
//
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        // Close vision portal if it was initialized
//        if (visionPortal != null) {
//            visionPortal.close();
//        }
//    }
//}
