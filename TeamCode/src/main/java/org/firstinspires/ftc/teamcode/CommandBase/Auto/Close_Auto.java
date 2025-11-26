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
public class Close_Auto extends OpModeEX {
    pathsManager paths =new pathsManager(new RobotConfig(0.022, 0.006, 0.028, 0.007, 0.06, 0.005, 0.075, 0.005, 0.022, 0.0005, 0.012, 0.002, 173,200,260,260));



    follower follow = new follower();
    double targetHeading;
    boolean pathing = false;
    boolean built = true;
    boolean Intake = false;
    boolean manuel = true;
    double lookAheadTime = 1;



    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intkeTime = new ElapsedTime();


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

    private final sectionBuilder[] preload = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(84, 28), new Vector2D(107, 75), new Vector2D(135, 128 )),
    };

    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(135, 128), new Vector2D(95, 157), new Vector2D(40, 151)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(40, 151), new Vector2D(99, 152), new Vector2D(145, 150)),
    };
    private final sectionBuilder[] Collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(150 , 150), new Vector2D(117, 218), new Vector2D(33, 211)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(33, 211), new Vector2D(126, 197), new Vector2D(160, 180)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(75, 22, 0);
        turret.Auto = true;


        paths.addNewPath("preload");
        paths.buildPath(preload);
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
        turret.robotX = odometry.X() - odometry.getXVelocity() * lookAheadTime;
        turret.robotY = odometry.Y() - odometry.getYVelocity() * lookAheadTime;
        turret.robotHeading = odometry.normilised;



        if (built && state == AutoState.preload) {
            follow.setPath(paths.returnPath("preload"));
            targetHeading = 310;
            built = false;
            pathing = true;
            turret.spinDown = false;


        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.preload) {
            built = true;
            pathing = false;
            intake.block = false;
            state = AutoState.firstShootDone;
            intake.intake = true;
            shootTime.reset();



        }


        if (built && state == AutoState.firstShootDone && shootTime.milliseconds() > 250) {
            built = false;
            state = AutoState.collect1;
            follow.setPath(paths.returnPath("collect1"));
            targetHeading = 270;
            state = AutoState.collect1;
            pathing = true;

        }
        if (pathing && follow.isFinished(2, 2) && state == AutoState.collect1){
            state = AutoState.driveToShoot1;
            follow.setPath(paths.returnPath("driveToShoot1"));
            targetHeading = 270;
            pathing = true;
            turret.spinDown = false;
            state = AutoState.driveToShoot1;

        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.driveToShoot1){
            pathing = false;
            built = true;
            shootTime.reset();

        }

        if (state == AutoState.driveToShoot1 && built && shootTime.milliseconds() > 250){
            state = AutoState.Collect2;
            follow.setPath(paths.returnPath("Collect2"));
            targetHeading = 270;
            pathing = true;
            built = false;

        }
        if (pathing && follow.isFinished(2, 2) && state == AutoState.Collect2){
            state = AutoState.driveToShoot2;
            follow.setPath(paths.returnPath("driveToShoot2"));
            targetHeading = 270;
            turret.spinDown = false;
        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.driveToShoot2){
            pathing = false;
            state = AutoState.finished;
            shootTime.reset();



        }

        if (state == AutoState.driveToShoot1 && follow.isFinished(12,12)){
            intake.block = false;
        }
        if (state == AutoState.driveToShoot2 && follow.isFinished(12,12)){
            intake.block = false;
        }
        if (state == AutoState.Collect2 && odometry.X() < 90){
            intake.block = true;
        }
        if (state == AutoState.Collect2 && odometry.X() < 90) {
            intake.block = true;
        }
        if (state == AutoState.finished && shootTime.milliseconds() >600){
            requestOpModeStop();

        }
        if (pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));

        }else {
            driveBase.queueCommand(driveBase.drivePowers(0,0,0));
        }
        System.out.println(odometry.X());
        System.out.println(odometry.X());



    }


}