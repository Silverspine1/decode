package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    public final sectionBuilder[] setPoint = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(0, 5), new Vector2D(0,10), new Vector2D(0, 15)),
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
        driveBase.drivePowers(-gamepad1.right_stick_y,(gamepad1.left_trigger - gamepad1.right_trigger),-gamepad1.right_stick_x);

        if (gamepad1.a){
            follow.setPath(paths.returnPath("setPoint"));
            pathing = true;

        }
        if (gamepad1.b){
            driveBase.intake.update(-1);
        }else {
            driveBase.intake.update(0);
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
        telemetry.update();




    }

}
