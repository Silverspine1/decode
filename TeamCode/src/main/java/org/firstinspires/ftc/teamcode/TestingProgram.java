package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp
public class TestingProgram extends OpModeEX {

    follower follow = new follower();
    pathsManager paths = new pathsManager();
    boolean pathing = false;
    double targetHeading;
    public IMU imu;
    double targetRPM = 2700;
    boolean shootHim = false;
    double shootpower;


    public final sectionBuilder[] setPoint = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(60,60), new Vector2D(120, 0)),
    };


    @Override
    public void initEX() {
        odometry.startPosition(0, 0, 0);
        targetHeading = 0;
        paths.addNewPath("setPoint");
        paths.buildPath(setPoint);


    }

    @Override
    public void loopEX() {
        driveBase.driveFieldCentric(gamepad1.right_stick_y,(gamepad1.left_trigger - gamepad1.right_trigger),-gamepad1.right_stick_x);
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = Math.toRadians(odometry.Heading());

        if (gamepad1.a){
            turret.shooterMotorOne.update(gamepad1.left_stick_y);
            turret.shooterMotorTwo.update(gamepad1.left_stick_y);
        }
        if (gamepad1.y){
           shootHim = true;

        }
        double rpm = -(turret.shooterMotorOne.getVelocity()/28)*60;

        if (turret.shootPID.calculate(targetRPM,rpm) <0){
            shootpower = 0;
        }else {
            shootpower = turret.shootPID.calculate(targetRPM,rpm);
        }

        if (shootHim) {
            targetRPM += (gamepad1.left_stick_y*14);
            turret.shooterMotorOne.update(Math.abs(shootpower));
            turret.shooterMotorTwo.update(Math.abs(shootpower));
        }
        if (pathing && follow.isFinished(5,5)){
            pathing = false;
        }
        if (gamepad1.b){
            intake.intakeMotor.update(-1);
        }else if(gamepad1.x) {
            intake.intakeMotor.update(1);
        }else {
            intake.intakeMotor.update(0);
        }
        if (gamepad1.dpad_up){
            intake.trans.setPosition(1);
        }else {
            intake.trans.setPosition(0.5);
        }

        if (pathing) {

            odometry.queueCommand(odometry.updateLineBased);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());

            driveBase.queueCommand(driveBase.drivePowers(currentPower));

        }
        telemetry.addData("odometry x", odometry.X());
        telemetry.addData("odometry y", odometry.Y());
        telemetry.addData("Heading",odometry.Heading());
        telemetry.addData("left pod",odometry.leftPod.getCurrentPosition());
        telemetry.addData("back pod",odometry.backPod.getCurrentPosition());
        telemetry.addData("Intake Sensor",driveBase.intakeSensor.isPressed());
        telemetry.addData("shoot rpm",rpm);
        telemetry.addData("target",targetRPM);
        telemetry.addData("power", shootpower);
        telemetry.update();




    }

}
