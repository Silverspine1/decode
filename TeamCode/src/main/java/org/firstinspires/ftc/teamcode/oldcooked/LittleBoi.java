package org.firstinspires.ftc.teamcode.oldcooked;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class LittleBoi extends LinearOpMode {

    // Drive motors
    private DcMotor LF, RF, LB, RB;

    // Shooter motors
    private DcMotor shooterMotorOne, shooterMotorTwo;

    // Intake motors
    private DcMotor intakeMotor;
    private DcMotor Intake2;

    // Servos
    private Servo turretTurnOne, turretTurnTwo, hoodAdjust, trans;

    // Sensors
    private TouchSensor intakeSensor;
    private IMU imu;

    // Constants
    private final double gearRatio = 0.7272;
    private final double turretLimitAngle = 80;

    // Shooter control
    private double targetRPM = 2700;
    private double shootPower = 0;
    private boolean shootHim = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        shooterMotorOne = hardwareMap.get(DcMotor.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotor.class, "shooterMotorTwo");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");

        turretTurnOne = hardwareMap.get(Servo.class, "turretTurnOne");
        turretTurnTwo = hardwareMap.get(Servo.class, "turretTurnTwo");
        hoodAdjust = hardwareMap.get(Servo.class, "hoodAdjust");
        trans = hardwareMap.get(Servo.class, "trans");

        intakeSensor = hardwareMap.get(TouchSensor.class, "intakeSensor");
        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse motors
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        turretTurnOne.setDirection(Servo.Direction.REVERSE);
        turretTurnTwo.setDirection(Servo.Direction.REVERSE);

        // Set all motors to run without encoders
        for (DcMotor motor : new DcMotor[]{LF, RF, LB, RB, shooterMotorOne, shooterMotorTwo, intakeMotor, Intake2}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Reset IMU
        imu.resetYaw();

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {

            // --- Driving (robot-centric mecanum) ---
            double vertical = -gamepad1.right_stick_y;
            double strafe = gamepad1.left_trigger - gamepad1.right_trigger;
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(1.0, Math.abs(vertical) + Math.abs(strafe) + Math.abs(turn));
            double lfPower = (vertical - strafe - turn) / denominator;
            double rfPower = (vertical + strafe + turn) / denominator;
            double lbPower = (vertical + (strafe * 1.2) - turn) / denominator;
            double rbPower = (vertical - (strafe * 1.2) + turn) / denominator;

            LF.setPower(lfPower);
            RF.setPower(rfPower);
            LB.setPower(lbPower);
            RB.setPower(rbPower);

            // --- Shooter Control ---
            if (gamepad1.a) {
                double stickPower = gamepad1.left_stick_y;
                shooterMotorOne.setPower(stickPower);
                shooterMotorTwo.setPower(stickPower);
            } else {
                shooterMotorOne.setPower(0);
                shooterMotorTwo.setPower(0);
            }

            // --- Intake Control (intakeMotor) ---
            if (gamepad1.b) {
                intakeMotor.setPower(-1);
            } else if (gamepad1.x) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            // --- Intake2 (second intake)
            if (gamepad1.dpad_up) {
                Intake2.setPower(1);
            } else {
                Intake2.setPower(0);
            }

            // Telemetry
            telemetry.addData("Shooter Target RPM", targetRPM);
            telemetry.addData("Shooter Power", shootPower);
            telemetry.update();
        }
    }
}
