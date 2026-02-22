package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

@TeleOp
public class Teleop extends OpMode {

    List<LynxModule> allHubs;

    Servo turret;
    Servo hood;

    Servo parkPTO1;
    Servo parkPTO2;

    Servo stopper;

    Servo intakePTO;

    DcMotorEx intake;

    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;

    DcMotorEx shooter1;
    DcMotorEx shooter2;

    ColorSensor colorSensor;

    double targetIntakePower = 0;

    double blockPath = 0.55;
    double openPath = 0.38;

    double shooterPower;
    double targetShooterVelocity = 0;

    double engageTransfer = 0.4;
    double disengageTransfer = 0.5;

    double engageParkPTO1 = 0.465;
    double disengageParkPTO1 = 0.515;

    double engageParkPTO2 = 0.435;
    double disengageParkPTO2 = 0.535;

    double turretPosition = 0.5;
    double hoodPosition = 0;

    boolean shooting = false;

    ElapsedTime intakeStartUpWait = new ElapsedTime();

    PIDController firstPID = new PIDController(0.005, 0, 0);
    PIDController secondPID = new PIDController(0.08, 0, 0.0005);

    double currentSum = 0;
    double lastSum = 0;

    int greenCollected = 0;
    int purpleCollected = 0;

    @Override
    public void init() {
        // Enable bulk caching for all LynxModule hubs
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(Servo.class, "turret");
        hood = hardwareMap.get(Servo.class, "hood");

        parkPTO1 = hardwareMap.get(Servo.class, "dtpto1");
        parkPTO2 = hardwareMap.get(Servo.class, "dtpto2");
        parkPTO2.setDirection(Servo.Direction.REVERSE);

        stopper = hardwareMap.get(Servo.class, "stopper");
        intakePTO = hardwareMap.get(Servo.class, "intakePTO");

        colorSensor = hardwareMap.get(ColorSensor.class, "colourSensor");

        stopper.setPosition(openPath);
        intakePTO.setPosition(disengageTransfer);
        hood.setPosition(hoodPosition);

        // Throttle telemetry to 5 Hz
        telemetry.setMsTransmissionInterval(200);
    }

    @Override
    public void loop() {
        // Clear bulk cache at the start of every loop
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        fullControlInSequences();
        manuelControlsForTests();
    }

    private void fullControlInSequences() {
        driveCode();

        // Cache sensor reads at top (avoids duplicate USB transactions)
        double cachedRed = colorSensor.red();
        double cachedGreen = colorSensor.green();
        double cachedBlue = colorSensor.blue();
        double cachedSum = cachedRed + cachedGreen + cachedBlue;
        double cachedVelocity = shooter1.getVelocity();
        double cachedCurrent = intake.getCurrent(CurrentUnit.MILLIAMPS);

        lastSum = currentSum;
        currentSum = cachedSum;

        if (gamepad1.a) {
            targetIntakePower = 0;
            targetShooterVelocity = 0;
            shooting = false;
        }

        if (targetIntakePower > 0.5 && cachedCurrent > 4000 && intakeStartUpWait.milliseconds() > 500 && !shooting) {
            targetIntakePower = 0;
        }

        if (gamepad1.right_bumper) {
            targetIntakePower = 0.9;
            stopper.setPosition(blockPath);
            targetShooterVelocity = 0;
            intakePTO.setPosition(disengageTransfer);
            targetShooterVelocity = 0;
            intakeStartUpWait.reset();
            shooting = false;
        }

        if (gamepad1.left_bumper && cachedVelocity > 500 && Math.abs(targetShooterVelocity - cachedVelocity) < 200) {
            targetIntakePower = 0.8;
            stopper.setPosition(openPath);
            intakePTO.setPosition(engageTransfer);
            intakeStartUpWait.reset();
            shooting = true;
        } else if (gamepad1.left_bumper) {
            targetShooterVelocity = 1600;
        }

        if (cachedVelocity < 300) {
            shooterPower = firstPID.calculate((targetShooterVelocity - cachedVelocity) / 10);
        } else {
            shooterPower = secondPID.calculate((targetShooterVelocity - cachedVelocity) / 10);
        }

        updateMotorPowers();

        // green range > 400 < 2000
        // purple range > 2000
        if (currentSum > 400 && currentSum < 2000 && lastSum < 400) {
            greenCollected++;
        } else if (currentSum > 2000 && lastSum < 2000) {
            purpleCollected++;
        }

        telemetry.addData("Sensor green", greenCollected);
        telemetry.addData("Sensor purple", purpleCollected);
        telemetry.addData("Sum", cachedSum);
        telemetry.addData("intake current", cachedCurrent);
        telemetry.addData("velocity", cachedVelocity);
        telemetry.update();
    }

    private void driveCode() {
        double vertical = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;

        double denominator = Math.max(1, Math.abs(vertical) + Math.abs(strafe) + Math.abs(turn));

        LF.setPower((vertical - (strafe) - turn) / denominator);
        RF.setPower((vertical + (strafe) + turn) / denominator);
        LB.setPower((vertical + (strafe) - turn) / denominator);
        RB.setPower((vertical - (strafe) + turn) / denominator);
    }

    private void updateMotorPowers() {
        shooter1.setPower(shooterPower);
        shooter2.setPower(shooterPower);
        intake.setPower(targetIntakePower);
    }

    private void manuelControlsForTests() {
        if (gamepad1.x) {
            turretPosition -= 0.01;
        } else if (gamepad1.b) {
            turretPosition += 0.01;
        }

        turret.setPosition(turretPosition);

        if (gamepad1.dpad_left) {
            hoodPosition = Range.clip(hoodPosition - 0.01, 0, 1);
        } else if (gamepad1.dpad_right) {
            hoodPosition = Range.clip(hoodPosition + 0.01, 0, 1);
        }

        hood.setPosition(hoodPosition);

        if (gamepad1.dpad_up) {
            parkPTO1.setPosition(engageParkPTO1);
            parkPTO2.setPosition(engageParkPTO2);
        } else if (gamepad1.dpad_down) {
            parkPTO1.setPosition(disengageParkPTO1);
            parkPTO2.setPosition(disengageParkPTO2);
        }
    }
}
