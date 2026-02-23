package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous

public class back_In_A_Case extends OpModeEX {
    pathsManager paths = new pathsManager(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01,
            0.0005, 0.012, 0.002, 170, 193, 270, 920));

    follower follow = new follower(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01, 0.0005,
            0.012, 0.002, 150, 193, 270, 920));
    PIDController headingPID = new PIDController(0.012, 0, 0.0030);
    PIDController x = new PIDController(0.06, 0, 0.0030);

    private VisionPortal visionPortal;
    private LocalVision processor;

    enum AutoState {
        preLoad,
        collect1,
        driveToShoot1,
        collect2,
        driveToShoot2,
        collect3,
        gate,
        driveToShoot3,
        firstBackCollect,
        driveToShootBack,
        backCollect,
        finished
    }

    AutoState state = AutoState.preLoad;

    double targetHeading;
    boolean pathing = false;
    boolean built = true;
    boolean intakeOff = false;
    boolean Preload = false;
    boolean visionCollect = false;
    boolean ballShot = false;
    boolean collectDone = false;
    boolean reset = false;
    boolean intakePathSelected = false;
    boolean afterGateCollect = false;
    boolean dontWaitForPoz = false;
    boolean p3Qued = true;
    boolean PIDAtGate = false;
    boolean alreadyFailed = false;
    double IntakeOffWait = 200;
    double Xdist = 110;
    boolean ballsInIntake = false;

    double lookAheadTime = 0;
    double shootWait = 700;
    double velo = 15;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime ballshot = new ElapsedTime();
    ElapsedTime gateInTakeTime = new ElapsedTime();
    ElapsedTime maxToGetToShoot = new ElapsedTime();
    ElapsedTime waitForTurretToTarget = new ElapsedTime();
    ElapsedTime endPath = new ElapsedTime();
    ElapsedTime ballCollectWait = new ElapsedTime();


    private final sectionBuilder[] shoot = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(170, 330), new Vector2D(161, 324)),
    };

    private final sectionBuilder[] collect1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(155, 330), new Vector2D(126, 254), new Vector2D(77, 278)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(43, 282), new Vector2D(104, 261), new Vector2D(130, 306)),
    };
    private final sectionBuilder[] collect2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(145, 310), new Vector2D(132, 218), new Vector2D(72, 210)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(117, 150), new Vector2D(70, 255), new Vector2D(49, 208.5)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(57, 210), new Vector2D(117, 150)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(130, 155), new Vector2D(78, 146)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(40, 150), new Vector2D(117, 148)),
    };
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(117, 148), new Vector2D(164, 293), new Vector2D(82, 315)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(158, 327)),
    };
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(40, 218), new Vector2D(112, 300)),
    };
    private final sectionBuilder[] movePath = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(100, 300)),
    };
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(72, 340), new Vector2D(115, 320)),
    };
    private final sectionBuilder[] tryAgain = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(105, 320), new Vector2D(136, 320)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(161, 342, 270);
        turret.Auto = true;
        driveBase.tele = false;
        follow.setHeadingOffset(90);

        paths.addNewPath("shoot");
        paths.buildPath(shoot);
        paths.addNewPath("collect1");
        paths.buildPath(collect1);
        paths.addNewPath("driveToShoot1");
        paths.buildPath(driveToShoot1);
        paths.addNewPath("collect2");
        paths.buildPath(collect2);
        paths.addNewPath("driveToShoot2");
        paths.buildPath(driveToShoot2);
        paths.addNewPath("collect3");
        paths.buildPath(collect3);
        paths.addNewPath("gate");
        paths.buildPath(gate);
        paths.addNewPath("driveToShoot3");
        paths.buildPath(driveToShoot3);
        paths.addNewPath("firstBackCollect");
        paths.buildPath(firstBackCollect);
        paths.addNewPath("firstDriveToShootBack");
        paths.buildPath(firstDriveToShootBack);
        paths.addNewPath("driveToShootBack");
        paths.buildPath(driveToShootBack);
        paths.addNewPath("movePath");
        paths.buildPath(movePath);
        paths.addNewPath("S1");
        paths.buildPath(S1);
        paths.addNewPath("tryAgain");
        paths.buildPath(tryAgain);

        Apriltag.limelight.pipelineSwitch(0);

        processor = new LocalVision(LocalVision.TargetColor.BOTH);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(false);
        builder.addProcessor(processor);
        visionPortal = builder.build();

        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;

        if (!intake.InTake && intake.ballCount > 2) {
            intake.reverse = true;
        }
        if (!reset) {
            gameTime.reset();
            reset = true;
        }

        if (intakeOff && intake.ballCount > 2 && intakeoff.milliseconds() > IntakeOffWait) {
            intake.InTake = false;
            intakeOff = false;
        }
        if (intake.ballCount < 1 && !ballShot) {
            ballShot = true;
            ballshot.reset();
        }

        if (visionCollect) {
            if (intake.ballCount > 2 && !ballsInIntake) {
                ballCollectWait.reset();
                ballsInIntake = true;
            } else if (ballsInIntake && intake.ballCount < 2) {
                ballsInIntake = false;
            }


            if (maxWait.milliseconds() > 400 && follow.isFinished(10, 100)
                    || maxWait.milliseconds() > 1500
                    || ballsInIntake && ballCollectWait.milliseconds() > 200) {
                collectDone = true;
            }

            intake.block = true;
            intake.InTake = true;
        }

        if (intake.ballCount > 1) {
            turret.hoodCompensation = -2;
        } else {
            turret.hoodCompensation = 0;
        }

        switch (state) {
            case preLoad:
                if (!Preload) {
                    preload.reset();
                    Preload = true;
                    follow.setPath(paths.returnPath("shoot"));
                    pathing = true;
                    driveBase.speed = 1;
                    turret.mapOfset = 45;
                }
                if (built && preload.milliseconds() > 1480 || built && turret.diff < 100 && turret.rpm > 1200) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }
                if (pathing && follow.isFinished(10, 10)) {
                    pathing = false;
                }
                if (!built && shootTime.milliseconds() > 420) {
                    follow.setPath(paths.returnPath("collect1"));
                    turret.mapOfset = 38;
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    pathing = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect1;
                }
                break;

            case collect1:
                if (pathing && odometry.X() < 107) {
                    targetHeading = 270;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)) {
                    state = AutoState.driveToShoot1;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    turret.turrofset = 0;
                    follow.usePathHeadings(true);
                    follow.setHeadingOffset(-90);
                    follow.setHeadingLookAheadDistance(100);
                }
                break;

            case driveToShoot1:
                if (built && follow.isFinished(15, 15) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < velo / 1.6) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }
                if (!built && shootTime.milliseconds() > shootWait
                        && (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())
                        + Math.abs(odometry.getHVelocity())) < velo
                        || !built && ballShot && (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())
                        + Math.abs(odometry.getHVelocity())) < velo) {
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(130);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.InTake = true;
                    turret.turrofset = -1;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect2;
                }
                break;

            case collect2:
                if (pathing && odometry.X() < 135) {
                    targetHeading = 286;
                    follow.usePathHeadings(false);
                }
                if (follow.isFinished(8, 8)) {
                    state = AutoState.driveToShoot2;
                    follow.setPath(paths.returnPath("driveToShoot2"));
                    follow.setHeadingOffset(-90);
                    targetHeading = 275;
                    built = true;
                    intake.InTake = true;
                }
                break;

            case driveToShoot2:
                shootWait = 1600;
                if (built && follow.isFinished(15, 15) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < velo) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }
                if (follow.isFinished(15, 15) && !built && shootTime.milliseconds() > shootWait * 1.5
                        || follow.isFinished(10, 10) && !built && ballShot) {
                    targetHeading = 270;
                    follow.setPath(paths.returnPath("collect3"));
                    follow.usePathHeadings(false);
                    intake.InTake = true;
                    pathing = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect3;
                    driveBase.speed = 1;
                }
                break;

            case collect3:
                if (pathing && odometry.X() < 65) {
                    targetHeading = 270;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)) {
                    state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    driveBase.speed = 1;
                    targetHeading = 270;
                }
                break;

            case driveToShoot3:
                if (built && follow.isFinished(10, 10) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < velo) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }
                if (!built && shootTime.milliseconds() > 420) {
                    follow.setPath(paths.returnPath("gate"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    turret.stopTurret = true;
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    state = AutoState.gate;
                }
                break;

            case gate:
                if (odometry.X() < 140) {
                    follow.usePathHeadings(false);
                    targetHeading = 297;
                }
                if (follow.isFinished(12, 12) && !built) {
                    pathing = false;
                    gateInTakeTime.reset();
                    built = true;
                }
                if (!pathing) {
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - 297), 0);
                    PIDAtGate = true;
                }
                if (built && gateInTakeTime.milliseconds() > 900) {
                    turret.stopTurret = false;
                    pathing = true;
                    PIDAtGate = false;
                    follow.setPath(paths.returnPath("firstDriveToShootBack"));
                    follow.usePathHeadings(false);
                    state = AutoState.driveToShootBack;
                    afterGateCollect = true;
                    driveBase.speed = 1;
                    targetHeading = 308;
                    maxToGetToShoot.reset();
                    turret.mapOfset = 65;
                    intakeoff.reset();
                    intakeOff = true;
                    built = false;
                }
                break;

            case driveToShootBack:
                if (afterGateCollect && odometry.Y() > 305) {
                    afterGateCollect = false;
                    dontWaitForPoz = true;
                    pathing = false;
                    p3Qued = true;
                    waitForTurretToTarget.reset();
                }
                if (afterGateCollect && odometry.Y() > 270) {
                    targetHeading = 292;
                }
                if (follow.isFinished(10, 10) && Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                        + Math.abs(odometry.getHVelocity() * 2) < 18) {
                    pathing = false;
                }
                if (!pathing && odometry.X() > 117 && !built && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) + Math.abs(odometry.getHVelocity() * 2) < 2 && !dontWaitForPoz
                        || !built && dontWaitForPoz && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) + Math.abs(odometry.getHVelocity() * 2) < 4 && waitForTurretToTarget.milliseconds() > 350) {
                    if (dontWaitForPoz) {
                        shootWait = 700;
                    } else {
                        shootWait = 500;
                    }
                    shootTime.reset();
                    follow.usePathHeadings(false);
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    intake.block = false;
                    intake.InTake = true;
                    IntakeOffWait = 200;
                }
                if (dontWaitForPoz || afterGateCollect) {
                    maxToGetToShoot.reset();
                }
                if (maxToGetToShoot.milliseconds() > 2400) {
                    follow.setPath(paths.returnPath("tryAgain"));
                    pathing = true;
                    maxToGetToShoot.reset();
                    alreadyFailed = true;
                }
                if (alreadyFailed && maxToGetToShoot.milliseconds() > 2200) {
                    follow.usePathHeadings(false);
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    intake.block = true;
                    intake.InTake = true;
                    ballsInIntake = false;
                    state = AutoState.backCollect;
                    driveBase.speed = 1;
                    collectDone = false;
                    maxWait.reset();
                    alreadyFailed = false;
                }
                if (built && shootTime.milliseconds() > shootWait) {
                    state = AutoState.backCollect;
                    driveBase.speed = 1;
                    collectDone = false;
                    ballsInIntake = false;
                    maxWait.reset();
                }
                break;

            case backCollect:
                if (built && !collectDone && !visionCollect) {
                    visionCollect = true;


                    final sectionBuilder[] p1Dynamic = new sectionBuilder[] {
                            () -> paths.addPoints(
                                    new Vector2D(odometry.X(), odometry.Y()),
                                    new Vector2D(140, 340),
                                    new Vector2D(60, 340))
                    };
                    paths.addNewPath("p1Dynamic");
                    paths.buildPath(p1Dynamic);
                    follow.setPath(paths.returnPath("p1Dynamic"));

                    targetHeading = 270;
                    intake.block = true;
                    intake.InTake = true;
                    ballCollectWait.reset();
                    maxWait.reset();
                }
                if (gameTime.milliseconds() > 27600 && !pathing) {
                    built = false;
                    visionCollect = false;
                    pathing = true;
                    intake.block = true;
                    follow.setPath(paths.returnPath("movePath"));
                }
                if (!built && follow.isFinished(5, 5)) {
                    pathing = false;
                    visionCollect = false;
                }
                if (built && collectDone) {
                    visionCollect = false;
                    state = AutoState.driveToShootBack;
                    final sectionBuilder[] S1 = new sectionBuilder[] {
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(115, 320)),
                    };
                    paths.addNewPath("S1");
                    paths.buildPath(S1);
                    follow.setPath(paths.returnPath("S1"));
                    turret.turrofset = 0;
                    intakePathSelected = false;
                    maxToGetToShoot.reset();
                    ballShot = false;
                    targetHeading = 270;
                    intakeoff.reset();
                    intakeOff = true;
                    pathing = true;
                    built = false;
                }
                break;

            case finished:
                requestOpModeStop();
        }

        if (pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (visionCollect) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(-currentPower.getHorizontal(), currentPower.getPivot(), x.calculate(processor.xPosCm)));
        } else if (!PIDAtGate) {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }
    }
}