package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
@Autonomous
public class Far_Auto extends OpModeEX {
    pathsManager paths = new pathsManager();

    follower follow = new follower();
    double targetHeading;
    boolean pathing = false;
    boolean built = true;

    ElapsedTime shootTime = new ElapsedTime();


    enum AutoState {
        preload,
        driveToCollect,
        firstShootDone,
        collect1,
        driveToShoot1,
        Collect2,
        driveToShoot2
    }

    Close_Auto.AutoState state = Close_Auto.AutoState.preload;


    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(156, 335), new Vector2D(154, 263), new Vector2D(50, 268)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(50, 269), new Vector2D(154, 263), new Vector2D(156, 335)),
    };
    private final sectionBuilder[] Collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(156, 335), new Vector2D(144, 194), new Vector2D(50, 210)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(50, 210), new Vector2D(144, 194), new Vector2D(156, 335)),
    };
    @Override
    public void initEX() {
        odometry.startPosition(160, 342, 0);
        paths.addNewPath("collect1");
        paths.buildPath(collect1);
        paths.addNewPath("driveToShoot1");
        paths.buildPath(driveToShoot1);
        paths.addNewPath("Collect2");
        paths.buildPath(Collect2);
        paths.addNewPath("driveToShoot2");
        paths.buildPath(driveToShoot2);

    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;

        if (built && state == Close_Auto.AutoState.preload) {
            turret.shootingTime.reset();
            built = false;
            pathing = false;
            turret.spinDown=false;


        }
        if (turret.shootingTime.milliseconds() > 1800 && state == Close_Auto.AutoState.preload) {
            built = true;
            pathing = false;
            shootTime.reset();
            intake.intakeMotor.update(-1);
            state = Close_Auto.AutoState.firstShootDone;
            turret.spinDown = true;


        }

        if (shootTime.milliseconds() > 1000 && built && state == Close_Auto.AutoState.firstShootDone) {
            state = Close_Auto.AutoState.collect1;
            follow.setPath(paths.returnPath("collect1"));
            state = Close_Auto.AutoState.collect1;
            intake.block = true;
            targetHeading = 270;
            built = false;
            turret.targetRPM = 0;
            pathing = true;
            intake.intakeMotor.update(-1);
        }
        if (pathing && follow.isFinished(2, 2) && state == Close_Auto.AutoState.collect1){
            state = Close_Auto.AutoState.driveToShoot1;
            follow.setPath(paths.returnPath("driveToShoot1"));
            targetHeading = 310;
            intake.block = false;
            pathing = true;
            intake.intakeMotor.update(0);
            turret.spinDown = false;
            state = Close_Auto.AutoState.driveToShoot1;

        }
        if (pathing && follow.isFinished(2, 2) && state == Close_Auto.AutoState.driveToShoot1){
            pathing = false;
            shootTime.reset();
            intake.intakeMotor.update(-1);
            built = true;


        }
        if (shootTime.milliseconds() > 1000 && state == Close_Auto.AutoState.driveToShoot1 && built){
            turret.spinDown = true;
            targetHeading = 270;
            intake.block = true;
            state = Close_Auto.AutoState.Collect2;
            follow.setPath(paths.returnPath("Collect2"));
            pathing = true;
            built = false;

        }
        if (pathing && follow.isFinished(2, 2) && state == Close_Auto.AutoState.Collect2){
            state = Close_Auto.AutoState.driveToShoot2;
            follow.setPath(paths.returnPath("driveToShoot2"));
            intake.block = false;
            intake.intakeMotor.update(0);
            turret.spinDown = false;

            targetHeading = 310;
        }
        if (pathing && follow.isFinished(2, 2) && state == Close_Auto.AutoState.driveToShoot2){
            pathing = false;
            intake.intakeMotor.update(-1);
            shootTime.reset();
            if (shootTime.milliseconds() > 1000){
                requestOpModeStop();

            }

        }
        if (pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));

        }else {
            driveBase.queueCommand(driveBase.drivePowers(0,0,0));
        }


    }


}
