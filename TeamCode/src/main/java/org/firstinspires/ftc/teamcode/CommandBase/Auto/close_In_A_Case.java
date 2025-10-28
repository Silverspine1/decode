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
public class close_In_A_Case extends OpModeEX {
    pathsManager paths =new pathsManager(new RobotConfig(0.023, 0.007, 0.030, 0.008, 0.065, 0.0055, 0.078, 0.0054, 0.022, 0.0005, 0.012, 0.002, 173,200,260,260));



    follower follow = new follower();
    double targetHeading;
    boolean pathing = false;
    boolean built = true;
    boolean Intake = false;
    boolean manuel = true;
    double lookAheadTime = 0;
    double shootWait = 3000;
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

    Close_Auto.AutoState state = Close_Auto.AutoState.preload;

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
            () -> paths.addPoints(new Vector2D(150 , 150), new Vector2D(117, 228), new Vector2D(28, 207)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(33, 211), new Vector2D(126, 197), new Vector2D(160, 170)),
    };
    @Override
    public void initEX() {
        odometry.startPosition(75, 22, 0);
        turret.Auto = true;
        driveBase.tele= false;


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
        switch (state) {
            case preload:
                if (built ) {
                    follow.setPath(paths.returnPath("preload"));
                    targetHeading = 310;
                    built = false;
                    pathing = true;
                    turret.spinDown = false;


                }
                if (pathing && follow.isFinished(3, 3) && Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo) {
                    built = true;
                    pathing = false;
                    intake.block = false;
                    state = Close_Auto.AutoState.firstShootDone;
                    intake.intakeMotor.update(-0.75);
                    shootTime.reset();


                }
                break;
                case firstShootDone:
                    if (built  && shootTime.milliseconds() > shootWait) {
                        built = false;
                        state = Close_Auto.AutoState.collect1;
                        follow.setPath(paths.returnPath("collect1"));
                        intake.block = true;
                        targetHeading = 270;
                        state = Close_Auto.AutoState.collect1;
                        pathing = true;

                    }
                    break;
                    case collect1:
                        if (pathing && follow.isFinished(7, 7)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
                            state = Close_Auto.AutoState.driveToShoot1;
                            follow.setPath(paths.returnPath("driveToShoot1"));
                            targetHeading = 270;
                            pathing = true;
                            turret.spinDown = false;
                            state = Close_Auto.AutoState.driveToShoot1;

                        }
                        break;
                        case driveToShoot1:
                            if (pathing && follow.isFinished(5, 5)) {
                                pathing = false;
                                built = true;
                                shootTime.reset();

                            }

                            if ( built && shootTime.milliseconds() > shootWait){
                                state = Close_Auto.AutoState.Collect2;
                                follow.setPath(paths.returnPath("Collect2"));
                                targetHeading = 270;
                                intake.block = true;
                                pathing = true;
                                built = false;

                            }
                            if (state == Close_Auto.AutoState.driveToShoot1 && follow.isFinished(12,12)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
                                intake.block = false;
                            }
                            break;
                            case Collect2:
                                if (pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                                    state = Close_Auto.AutoState.driveToShoot2;
                                    follow.setPath(paths.returnPath("driveToShoot2"));
                                    targetHeading = 270;
                                    turret.spinDown = false;
                                }
                                break;
                                case driveToShoot2:
                                    if (pathing && follow.isFinished(4, 4)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                                        intake.block = false;
                                        pathing = false;
                                        state = Close_Auto.AutoState.finished;
                                        shootTime.reset();

                                    }
                                    break;
                                    case finished:
                                        if (shootTime.milliseconds() > shootWait) {
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
        System.out.println(odometry.X());
        System.out.println(odometry.X());

    }
}
