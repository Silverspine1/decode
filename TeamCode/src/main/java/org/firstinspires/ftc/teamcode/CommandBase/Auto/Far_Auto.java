package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
@Autonomous
public class Far_Auto extends OpModeEX {
    pathsManager paths = new pathsManager();

    follower follow = new follower(new RobotConfig(0.023, 0.007, 0.030, 0.008, 0.065, 0.0055, 0.078, 0.0054, 0.022, 0.0005, 0.012, 0.002, 173,200,260,260));
    double targetHeading;
    boolean pathing = false;
    boolean built = true;
    double velo = 2;

    ElapsedTime shootTime = new ElapsedTime();


    enum AutoState {
        preload,
        driveToCollect,
        firstShootDone,
        collect1,
        driveToShoot1,
        Collect2,
        driveToShoot2,
        finished
    }

    AutoState state = AutoState.preload;


    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(156, 335), new Vector2D(154, 273), new Vector2D(45, 278)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(45, 269), new Vector2D(154, 263), new Vector2D(146, 325)),
    };
    private final sectionBuilder[] Collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(146, 325), new Vector2D(144, 190), new Vector2D(45, 217)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(45, 210), new Vector2D(144, 194), new Vector2D(146, 325)),
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

        if (built && state == AutoState.preload) {
            turret.shootingTime.reset();
            built = false;
            pathing = false;
            turret.spinDown=false;


        }
        if (turret.shootingTime.milliseconds() > 1800 && state == AutoState.preload) {
            built = true;
            pathing = false;
            shootTime.reset();
            intake.intakeMotor.update(-0.7);
            state = AutoState.firstShootDone;
            turret.spinDown = true;


        }

        if (shootTime.milliseconds() > 2500 && built && state == AutoState.firstShootDone) {
            state = AutoState.collect1;
            follow.setPath(paths.returnPath("collect1"));
            intake.block = true;
            targetHeading = 270;
            built = false;
            turret.targetRPM = 0;
            pathing = true;
            intake.intakeMotor.update(-1);
        }
        if (pathing && follow.isFinished(4, 4) && state == AutoState.collect1 && Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
            state = AutoState.driveToShoot1;
            follow.setPath(paths.returnPath("driveToShoot1"));
            targetHeading = 310;
            intake.block = false;
            pathing = true;
            intake.intakeMotor.update(0);
            turret.spinDown = false;
            state = AutoState.driveToShoot1;

        }
        if (pathing && follow.isFinished(4, 4) && state == AutoState.driveToShoot1  && Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
            pathing = false;
            shootTime.reset();
            intake.intakeMotor.update(-0.7);
            built = true;


        }
        if (shootTime.milliseconds() > 2500 && state == AutoState.driveToShoot1 && built){
            turret.spinDown = true;
            targetHeading = 270;
            intake.block = true;
            state = AutoState.Collect2;
            follow.setPath(paths.returnPath("Collect2"));
            intake.intakeMotor.update(-1);

            pathing = true;
            built = false;

        }
        if (pathing && follow.isFinished(4, 4) && state == AutoState.Collect2  && Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
            state = AutoState.driveToShoot2;
            follow.setPath(paths.returnPath("driveToShoot2"));
            intake.block = false;
            intake.intakeMotor.update(0);
            turret.spinDown = false;

            targetHeading = 310;
        }
        if (pathing && follow.isFinished(4, 4) && state == AutoState.driveToShoot2 && Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
            pathing = false;
            intake.intakeMotor.update(-0.7);
            shootTime.reset();
            state = AutoState.finished;


        }
        if (shootTime.milliseconds() > 2500 && state == AutoState.finished){
            requestOpModeStop();

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
