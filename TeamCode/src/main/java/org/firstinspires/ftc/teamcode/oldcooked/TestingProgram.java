package org.firstinspires.ftc.teamcode.oldcooked;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;


public class TestingProgram extends OpModeEX {

    private static final Logger log = LoggerFactory.getLogger(TestingProgram.class);
    follower follow = new follower();
    pathsManager paths = new pathsManager();
    boolean pathing = false;
    double targetHeading;
    public IMU imu;
    double targetRPM = 2700;
    boolean shootHim = false;
    double shootpower;
    double hood = 178;


    public final sectionBuilder[] setPoint = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(60,60), new Vector2D(120, 0)),
    };


    @Override
    public void initEX() {
        odometry.startPosition(86.6, 16.6, 0);
        targetHeading = 0;
        paths.addNewPath("setPoint");
        paths.buildPath(setPoint);


    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = Math.toRadians(odometry.Heading());
        if (Math.abs(gamepad1.right_stick_y)>0){
            turret.hoodAdjust.setPosition(hood += gamepad1.right_stick_y);
        }

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
        if (gamepad1.dpad_left){
            turret.turretTurnOne.setPosition(0);
            turret.turretTurnTwo.setPosition(0);
        }


        telemetry.addData("odometry x", odometry.X());
        telemetry.addData("odometry y", odometry.Y());
        telemetry.addData("Heading",odometry.Heading());
        telemetry.addData("rpm",turret.rpm);
        telemetry.addData("power", shootpower);
        telemetry.addData("ditance",turret.distance);
        telemetry.addData("hood angle",hood);
        telemetry.addData("shoot zone",turret.inZone);
        telemetry.update();

telemetry.addData("tc",aprilTagInput.limelight.getLatestResult());


    }

}
