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
public class Close_Auto extends OpModeEX {
    pathsManager paths = new pathsManager();

    follower follow = new follower();
    double targetHeading;
    boolean pathing = false;
    boolean built = true;
    boolean Intake = false;

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
            () -> paths.addPoints(new Vector2D(135, 128), new Vector2D(95, 157), new Vector2D(45, 151)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(45, 151), new Vector2D(99, 152), new Vector2D(150, 150)),
    };
    private final sectionBuilder[] Collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(150 , 150), new Vector2D(117, 209), new Vector2D(45, 211)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(45, 211), new Vector2D(126, 197), new Vector2D(179, 180)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(75, 22, 0);


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
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;
        if (Intake){
            intkeTime.reset();
            intake.intakeMotor.update(-1);
            if (intkeTime.milliseconds() > 2500){
                Intake = false;
            }
        }else {
            intake.intakeMotor.update(0);
        }

        if (built && state == AutoState.preload) {
            follow.setPath(paths.returnPath("preload"));
            targetHeading = 310;
            built = false;
            pathing = true;
            turret.spinDown = false;


        }
        if (pathing && follow.isFinished(5, 5) && state == AutoState.preload && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) < 8) {
            built = true;
            pathing = false;
            shootTime.reset();
            Intake = true;
            state = AutoState.firstShootDone;


        }

        if (shootTime.milliseconds() > 1000 && built && state == AutoState.firstShootDone) {
            intake.block = true;
            built = false;
            state = AutoState.collect1;
            follow.setPath(paths.returnPath("collect1"));
            targetHeading = 270;
            state = AutoState.collect1;
            pathing = true;
            Intake = true;

        }
        if (pathing && follow.isFinished(2, 2) && state == AutoState.collect1){
            state = AutoState.driveToShoot1;
            intake.block = false;
            follow.setPath(paths.returnPath("driveToShoot1"));
            targetHeading = 270;
            pathing = true;
            turret.spinDown = false;
            state = AutoState.driveToShoot1;

        }
        if (pathing && follow.isFinished(5, 5) && state == AutoState.driveToShoot1 && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) < 8){
            pathing = false;
            shootTime.reset();
            Intake = true;
            built = true;


        }
        if (shootTime.milliseconds() > 1000 && state == AutoState.driveToShoot1 && built){
            turret.spinDown = true;
            state = AutoState.Collect2;
            Intake = true;
            intake.block = true;
            follow.setPath(paths.returnPath("Collect2"));
            targetHeading = 258;
            pathing = true;
            built = false;

        }
        if (pathing && follow.isFinished(2, 2) && state == AutoState.Collect2){
            state = AutoState.driveToShoot2;
            intake.block = false;
            follow.setPath(paths.returnPath("driveToShoot2"));
            targetHeading = 270;
            turret.spinDown = false;
        }
        if (pathing && follow.isFinished(5, 5) && state == AutoState.driveToShoot2 && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) < 8){
            pathing = false;
            Intake = true;
            shootTime.reset();
            state = AutoState.finished;


        }
        if (shootTime.milliseconds() > 1000 && state == AutoState.finished){
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