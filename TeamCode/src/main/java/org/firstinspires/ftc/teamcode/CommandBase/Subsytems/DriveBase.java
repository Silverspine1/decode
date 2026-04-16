package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class DriveBase extends SubSystem {

    public MotorEx LF = new MotorEx();
    public MotorEx RF = new MotorEx();
    public MotorEx RB = new MotorEx();
    public MotorEx LB = new MotorEx();
    public TouchSensor intakeSensor;
    public Servo base1;
    public Servo base2;

    public double speed = 1;
    public boolean engage = false;

    boolean liftBasePlate = false;
    boolean liftBasePlateDone = false;

    // ==================== PID Controllers for Smooth Stopping ====================
    private final PIDController forwardPID = new PIDController(0.007, 0.0, 0.0008);
    private final PIDController strafePID  = new PIDController(0.002, 0.0, 0.0008);
    private final PIDController headingPID = new PIDController(0.028, 0.0, 0.0005);

    private double targetForward = 0;
    private double targetStrafe = 0;
    private double targetHeading = 0;

    private boolean brakeMode = false;
    private final double INPUT_THRESHOLD = 0.08; // sticks must be below this to trigger brake

    public double vertikal;
    public double turn;
    public double strafe;

    public boolean tele = false;
    public double fieldVelY = 0;
    public double fieldVelX = 0;

    public DriveBase(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, driveCommand);
    }

    @Override
    public void init() {
        LF.initMotor("LF", getOpMode().hardwareMap);
        RF.initMotor("RF", getOpMode().hardwareMap);
        LB.initMotor("LB", getOpMode().hardwareMap);
        RB.initMotor("RB", getOpMode().hardwareMap);

        base1 = getOpMode().hardwareMap.get(Servo.class, "base1");
        base2 = getOpMode().hardwareMap.get(Servo.class, "base2");
        intakeSensor = getOpMode().hardwareMap.get(TouchSensor.class, "intakeSensor");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void driveFieldCentric(double drive, double strafeInput, double turnInput, double robotHeading) {
        double headingRadians = Math.toRadians(robotHeading);

        double rotX = strafeInput * Math.cos(-headingRadians) - drive * Math.sin(-headingRadians);
        double rotY = strafeInput * Math.sin(-headingRadians) + drive * Math.cos(-headingRadians);

        if (tele) {
            boolean driverIsMoving = Math.abs(drive) > INPUT_THRESHOLD ||
                    Math.abs(strafeInput) > INPUT_THRESHOLD ||
                    Math.abs(turnInput) > INPUT_THRESHOLD;

            if (!driverIsMoving) {
                if (!brakeMode) {
                    targetHeading = robotHeading;
                    brakeMode = true;
                }


                double robotVelX = fieldVelX * Math.cos(-headingRadians) - fieldVelY * Math.sin(-headingRadians);
                double robotVelY = fieldVelX * Math.sin(-headingRadians) + fieldVelY * Math.cos(-headingRadians);

                double forwardPower = forwardPID.calculate(robotVelY, 0);
                double strafePower  = strafePID.calculate(robotVelX, 0);

                drivePowers(forwardPower, 0, strafePower);
            }
            else {
                brakeMode = false;
                double finalRotX = Math.abs(rotX) < 0.25 ? 0 : rotX;

                drivePowers(rotY, turnInput, finalRotX);
            }
        }
        else {
            double finalRotX = Math.abs(rotX) < 0.25 ? 0 : rotX;
            drivePowers(rotY, turnInput, finalRotX);
        }
    }

    private double getHeadingError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    public Command drivePowers(double vertical, double turn, double strafe) {
        this.vertikal = vertical;
        this.turn = turn;
        this.strafe = strafe;
        return driveCommand;
    }

    public Command drivePowers(RobotPower power) {
        this.turn = power.getPivot();
        this.strafe = -power.getVertical();
        this.vertikal = -power.getHorizontal();
        return driveCommand;
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {},
            () -> {
                long start = System.nanoTime();

                double denominator = Math.max(1.0, Math.abs(vertikal) + Math.abs(strafe) + Math.abs(turn));

                double lfT = ((vertikal - strafe - turn) / denominator) * speed;
                double rfT = ((vertikal + strafe + turn) / denominator) * speed;
                double lbT = ((vertikal + strafe - turn) / denominator) * speed;
                double rbT = ((vertikal - strafe + turn) / denominator) * speed;

                if (!engage) {
                    LF.update(lfT);
                    RF.update(rfT);
                    LB.update(lbT);
                    RB.update(rbT);
                } else {
                    LF.update(lfT);
                    RF.update(rfT);
                    LB.update(0);
                    RB.update(0);
                }

                ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.DRIVE_BASE, System.nanoTime() - start);
            },
            () -> true);

    @Override
    public void execute() {
        long start = System.nanoTime();
        executeEX();

        if (!liftBasePlate && !liftBasePlateDone) {
            base2.setPosition(1);
            base1.setPosition(0);
            liftBasePlate = true;
        } else if (liftBasePlate && !liftBasePlateDone) {
            base2.setPosition(0.5);
            base1.setPosition(0.5);
            liftBasePlateDone = true;
        }

        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.DRIVE_BASE, System.nanoTime() - start);
    }

    public void setAll(double power) {
        LF.update(power);
        RF.update(power);
        LB.update(power);
        RB.update(power);
    }
}