package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous

public class back_In_A_Case_Hype_auto_Red extends OpModeEX {
    pathsManager paths = new pathsManager(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01,
            0.0005, 0.012, 0.002, 200, 273, 270, 320));

    follower follow = new follower(new RobotConfig(0.015, 0.004, 0.016, 0.005, 0.02, 0.004, 0.055, 0.004, 0.01, 0.0005,
            0.012, 0.002, 200, 273, 270, 320));

    PIDController headingPID = new PIDController(0.012, 0, 0.0030);
    PIDController forward = new PIDController(0.010, 0, 0.0030);

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

    enum shootPath {
        S1,
        S2,
        S3,
    }

    AutoState state = AutoState.preLoad;
    shootPath shootState = shootPath.S2;

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
    boolean p3Qued = false;
    boolean PIDAtGate = false;
    boolean preQue = true;
    boolean alreadyFailed = false;
    boolean waitAtEnd = false;
    double IntakeOffWait = 200;
    double Xdist = 110;
    double cycleCount = 0;
    boolean failSafeHasHappend = false;

    double lookAheadTime = 0;
    double shootWait = 700;
    double velo = 15;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime ballshot = new ElapsedTime();
    ElapsedTime extraTurnWait = new ElapsedTime();
    ElapsedTime maxToGetToShoot = new ElapsedTime();
    ElapsedTime waitForTurretToTarget = new ElapsedTime();
    ElapsedTime endPath = new ElapsedTime();

    private final sectionBuilder[] shoot = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(190, 330), new Vector2D(199, 324)),
    };

    private final sectionBuilder[] collect1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(205, 330), new Vector2D(234, 254), new Vector2D(283, 278)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(317, 282), new Vector2D(256, 261), new Vector2D(230, 306)),

    };
    private final sectionBuilder[] collect2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(215, 310), new Vector2D(228, 218), new Vector2D(291, 209)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(243, 150), new Vector2D(290, 255), new Vector2D(310, 197)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(303, 210), new Vector2D(243, 150)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(230, 155), new Vector2D(287, 146)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(320, 150), new Vector2D(243, 148)),
    };

    private final sectionBuilder[] firstBackCollect = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(243, 148), new Vector2D(196, 293), new Vector2D(278, 315)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(202, 327)),
    };
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(320, 218), new Vector2D(248, 300)),
    };
    private final sectionBuilder[] movePath = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(260, 300)),
    };
    private final sectionBuilder[] p1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(245, 317), new Vector2D(220, 340), new Vector2D(298, 340)),
    };
    private final sectionBuilder[] p2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(245, 317), new Vector2D(298, 311)),
    };
    private final sectionBuilder[] p3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(245, 317), new Vector2D(298, 284)),
    };
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(304, 340), new Vector2D(245, 320)),
    };
    private final sectionBuilder[] S2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(304, 311), new Vector2D(245, 320)),
    };
    private final sectionBuilder[] S3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(304, 284), new Vector2D(245, 320)),
    };
    private final sectionBuilder[] pEsh = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(230, 306), new Vector2D(220, 340), new Vector2D(302, 340)),
    };
    private final sectionBuilder[] tryAgain = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(255, 320), new Vector2D(224, 320)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(199, 342, 90);
        turret.Auto = true;
        turret.targetX = 360;
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
        paths.addNewPath("p1");
        paths.buildPath(p1);
        paths.addNewPath("p2");
        paths.buildPath(p2);
        paths.addNewPath("p3");
        paths.buildPath(p3);
        paths.addNewPath("pEsh");
        paths.buildPath(pEsh);
        paths.addNewPath("S1");
        paths.buildPath(S1);
        paths.addNewPath("S2");
        paths.buildPath(S2);
        paths.addNewPath("S3");
        paths.buildPath(S3);
        paths.addNewPath("tryAgain");
        paths.buildPath(tryAgain);

        Apriltag.limelight.pipelineSwitch(0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        processor = new LocalVision(LocalVision.TargetColor.BOTH);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Camera + settings BEFORE build()
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set lower resolution here
        builder.setCameraResolution(new Size(640, 480));

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Optional: disable RC live view to save CPU
        builder.enableLiveView(false);

        // Add BOTH processors
        builder.addProcessor(processor);

        // Now actually create the portal
        visionPortal = builder.build();

        // Dashboard camera stream
        dashboard.startCameraStream(visionPortal, 15);
        turret.turrofset = -3.5;

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
            if (processor.hAngleDeg < -16 && !intakePathSelected) {
                follow.setPath(paths.returnPath("p3"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S3;
                targetHeading = 90;
                intake.block = false;
                intake.InTake = true;
                maxWait.reset();

            } else if (processor.hAngleDeg > 0.1 && !intakePathSelected) {
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S1;
                targetHeading = 90;
                intake.block = true;
                intake.InTake = true;
                maxWait.reset();

            } else if (!intakePathSelected && processor.hAngleDeg < 0.1) {
                follow.setPath(paths.returnPath("p2"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S2;
                targetHeading = 90;
                intake.block = true;
                intake.InTake = true;
                maxWait.reset();

            }
            if (follow.isFinished(5, 10) && !waitAtEnd || maxWait.milliseconds() > 1700 && !waitAtEnd) {
                waitAtEnd = true;
                endPath.reset();
            }
            if (waitAtEnd && endPath.milliseconds() > 180) {
                waitAtEnd = false;
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
                    turret.mapOfset = 40;

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
                if (!built && shootTime.milliseconds() > 500) {
                    follow.setPath(paths.returnPath("collect1"));
                    turret.mapOfset = 30;
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    pathing = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect1;
                }
                break;
            case collect1:
                if (pathing && odometry.X() > 253) {
                    targetHeading = 90;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)) {
                    state = AutoState.driveToShoot1;
                    turret.turrofset = -4.5;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    follow.usePathHeadings(true);
                    follow.setHeadingOffset(-90);
                    follow.setHeadingLookAheadDistance(100);
                }
                break;
            case driveToShoot1:

                if (built && follow.isFinished(15, 15)
                        && (Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())
                                + Math.abs(odometry.getHVelocity())) < velo / 1.6
                        && extraTurnWait.milliseconds() > 400) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }

                if (!built && shootTime.milliseconds() > 450 && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < velo) {
                    follow.usePathHeadings(false);
                    pathing = false;
                    intake.InTake = true;
                    turret.turrofset = -5;
                    collectDone = false;
                    built = true;
                    intake.block = true;
                    turret.mapOfset = 40;
                    ballShot = false;
                    maxToGetToShoot.reset();
                    state = AutoState.backCollect;
                }
                break;
            case driveToShootBack:
                if (afterGateCollect && odometry.Y() > 305) {
                    afterGateCollect = false;
                    dontWaitForPoz = true;
                    pathing = false;
                    p3Qued = true;
                    waitForTurretToTarget.reset();
                    IntakeOffWait = 350;

                }
                if (shootState == shootPath.S3) {
                    Xdist = 237;
                } else {
                    Xdist = 243;
                }

                if (afterGateCollect && odometry.Y() > 270) {
                    targetHeading = 68;
                }
                if (follow.isFinished(10, 10) && Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                        + Math.abs(odometry.getHVelocity() * 2) < 18) {
                    pathing = false;
                }
                if (!pathing && odometry.X() < Xdist && !built
                        && Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                                + Math.abs(odometry.getHVelocity() * 2) < 8) {

                    shootWait = 450;

                    shootTime.reset();
                    follow.usePathHeadings(false);
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    intake.block = false;
                    intake.InTake = true;
                    cycleCount += 1;
                    IntakeOffWait = 350;

                }
                if (!failSafeHasHappend && cycleCount == 4) {
                    preQue = true;

                }

                if (dontWaitForPoz || afterGateCollect) {
                    maxToGetToShoot.reset();
                }
                if (maxToGetToShoot.milliseconds() > 2400) {
                    follow.setPath(paths.returnPath("tryAgain"));
                    pathing = true;
                    maxToGetToShoot.reset();
                    failSafeHasHappend = true;
                    alreadyFailed = true;

                }
                if (alreadyFailed && maxToGetToShoot.milliseconds() > 2200) {
                    follow.usePathHeadings(false);
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    intake.block = true;
                    intake.InTake = true;
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
                    maxWait.reset();

                }

                break;
            case backCollect:
                if (built && !collectDone) {
                    if (!preQue) {
                        visionCollect = true;
                    } else if (preQue) {
                        follow.setPath(paths.returnPath("pEsh"));
                        targetHeading = 90;
                        if (!pathing) {
                            maxWait.reset();
                        }
                        pathing = true;
                        intakePathSelected = true;
                        shootState = shootPath.S1;
                        visionCollect = true;
                        preQue = false;

                    }
                }
                if (built && collectDone) {
                    visionCollect = false;
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
                    follow.setPath(paths.returnPath(shootState.name()));
                    turret.turrofset = -5;

                    intakePathSelected = false;
                    maxToGetToShoot.reset();

                    ballShot = false;
                    targetHeading = 90;

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
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(),
                    odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (!PIDAtGate) {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }
        telemetry.addData("block ", intake.block);
        telemetry.update();

    }
}