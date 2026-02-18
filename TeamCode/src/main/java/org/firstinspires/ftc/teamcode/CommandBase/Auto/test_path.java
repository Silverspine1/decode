package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class test_path extends OpModeEX {
    pathsManager paths =new pathsManager(new RobotConfig(0.018, 0.004, 0.020, 0.005, 0.04, 0.004, 0.065, 0.004
            , 0.008, 0.0005, 0.012, 0.002,  200, 273, 270, 320));



    follower follow = new follower(new RobotConfig(0.000, 0.0, 0.028, 0.007, 0.06, 0.005, 0.075, 0.005
            , 0.008, 0.0005, 0.012, 0.002, 152, 210, 200, 280));
    double targetHeading = 0;
    boolean pathing = false;
    private final sectionBuilder[] shoot = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(0, 0),  new Vector2D(100, 0)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(0, 0, 0);

        paths.addNewPath("shoot");
        paths.buildPath(shoot);


    }

    @Override
    public void loopEX() {
        follow.setPath(paths.returnPath("shoot"));
        pathing =true;

        if (pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else {
            driveBase.queueCommand(driveBase.drivePowers(0,0,0));
        }

    }
}
