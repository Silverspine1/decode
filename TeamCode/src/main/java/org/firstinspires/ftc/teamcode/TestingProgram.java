package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    double shoopower;


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

        if (gamepad1.a){
            driveBase.shoot.update(gamepad1.left_stick_y);
            driveBase.shoot2.update(gamepad1.left_stick_y);
        }
        if (gamepad1.y){
           shootHim = true;

        }
        double rpm = -(driveBase.shoot.getVelocity()/28)*60;

        if (driveBase.shootPID.calculate(targetRPM,rpm) <0){
            shoopower = 0;
        }else {
            shoopower = driveBase.shootPID.calculate(targetRPM,rpm);
        }

        if (shootHim) {
            targetRPM += (gamepad1.left_stick_y*14);
            driveBase.shoot.update(Math.abs(shoopower));
            driveBase.shoot2.update(Math.abs(shoopower));
        }
        if (pathing && follow.isFinished(5,5)){
            pathing = false;
        }
        if (gamepad1.b){
            driveBase.intake.update(-1);
        }else if(gamepad1.x) {
            driveBase.intake.update(1);
        }else {
            driveBase.intake.update(0);
        }
        if (gamepad1.dpad_up){
            driveBase.trans.setPosition(1);
        }else {
            driveBase.trans.setPosition(0.5);
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
        telemetry.addData("power",shoopower);
        telemetry.update();




    }

}
