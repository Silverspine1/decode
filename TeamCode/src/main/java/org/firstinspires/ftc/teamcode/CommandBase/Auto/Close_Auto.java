package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class Close_Auto extends OpModeEX {
    pathsManager paths = new pathsManager();

    follower follow = new follower();
    double targetHeading;
    boolean pathing = false;
    boolean built = true;

    ElapsedTime shootTime = new ElapsedTime();


    enum AutoState {
        preload,
        driveToCollect,
        collect1,
        driveToShoot1,
        Collect2,
        driveToShoot2
    }

    AutoState state = AutoState.preload;

    private final sectionBuilder[] preload = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(84, 28), new Vector2D(108, 44), new Vector2D(120, 89)),
    };

    private final sectionBuilder[] driveToCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(120, 89), new Vector2D(135, 136), new Vector2D(107, 151)),
    };
    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(107, 151), new Vector2D(77, 151), new Vector2D(50, 152)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(50, 151), new Vector2D(104, 134), new Vector2D(150, 120)),
    };
    private final sectionBuilder[] Collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(150, 120), new Vector2D(144, 217), new Vector2D(50, 210)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(50, 220), new Vector2D(144, 217), new Vector2D(150, 120)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(86.6, 16.6, 0);
        paths.addNewPath("driveToCollect");
        paths.buildPath(driveToCollect);
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

        if (built && state == AutoState.preload) {
            paths.setCurrentPath("preload");
            targetHeading = 315;
            built = false;
            pathing = true;
            turret.targetRPM = 2900;
            turret.hoodAdjust.setPosition(90);


        }
        if (pathing && follow.isFinished(5, 5) && state == AutoState.preload) {
            built = true;
            pathing = false;
            shootTime.reset();
            intake.intakeMotor.update(-1);
            if (shootTime.milliseconds() > 3000 && built){
                state = AutoState.driveToCollect;
                paths.setCurrentPath("driveToCollect");
                intake.intakeMotor.update(0);
                turret.targetRPM = 0;
                targetHeading = 270;
                pathing = true;
            }



        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.driveToCollect) {
            state = AutoState.collect1;
            paths.setCurrentPath("collect1");
            state = AutoState.collect1;
            targetHeading = 270;
            pathing = true;
            intake.intakeMotor.update(-1);
        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.collect1){
            state = AutoState.driveToShoot1;
            paths.setCurrentPath("driveToShoot1");
            targetHeading = 310;
            pathing = true;
            intake.intakeMotor.update(0);
            turret.targetRPM = 2900;
            turret.hoodAdjust.setPosition(90);
        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.driveToShoot1){
            pathing = false;
            shootTime.reset();
            intake.intakeMotor.update(-1);
            if (shootTime.milliseconds() > 3000){
                turret.targetRPM = 0;
                targetHeading = 270;
                state = AutoState.Collect2;
                paths.setCurrentPath("Collect2");
                pathing = true;

            }

        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.Collect2){
            state = AutoState.driveToShoot2;
            paths.setCurrentPath("driveToShoot2");
            intake.intakeMotor.update(0);
            turret.targetRPM = 2900;
            turret.hoodAdjust.setPosition(90);
            targetHeading = 310;
        }
        if (pathing && follow.isFinished(10, 10) && state == AutoState.driveToShoot2){
            pathing = false;
            intake.intakeMotor.update(-1);
            shootTime.reset();
            if (shootTime.milliseconds() > 3000){
                requestOpModeStop();

            }

        }
        if (pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));

        }


    }


}