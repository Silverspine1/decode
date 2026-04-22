package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Intake;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.LocalVision;
import org.firstinspires.ftc.vision.VisionPortal;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name="", group="Blue")

public class back_OneSpike extends OpModeEX {
    pathsManager paths = new pathsManager(new RobotConfig(
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2, 0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 700));

    follower follow = new follower(new RobotConfig(
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2, 0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 700));

    PIDController headingPID = new PIDController(0.009, 0, 0.0030);
    PIDController x = new PIDController(0.06, 0, 0.0030);
    PIDController y = new PIDController(0.013, 0, 0.0030);

    private VisionPortal visionPortal;
    private LocalVision processor;
    private FtcDashboard dashboard;

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

    double frontOffset      = 8.0;
    double sideOffsetDeg    = 0.0;
    double robotHalfWidth   = 17.5;
    double wallSafetyMargin = 10.0;
    double wallBuffer        = 20.0;
    double maxWallAvoidPower = 0.40;
    double wallVelGain       = 0.018;
    double wallLookAheadSecs = 0.12;

    final double WALL_Y = 270;

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
    boolean ballsInIntake = false;
    boolean driveBackToShoot = false;
    boolean waitAtEnd = false;

    boolean HoldHeadingWhileShooting = false;
    boolean p1Pathing = false;

    double backCycles = 0;
    double shootWait = 700;
    double extraShootDrive = 0;
    // gateTurnX: 360 - 248 = 112
    // gateAngle: abs(58 - 360) = 302
    double gateTolX = 10; double gateTolY = 8; double gateTurnX = 112; double gateAngle = 302; double gateTime = 1200;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime ballshot = new ElapsedTime();
    ElapsedTime gateInTakeTime = new ElapsedTime();
    ElapsedTime maxToGetToShoot = new ElapsedTime();
    ElapsedTime ballCollectWait = new ElapsedTime();
    ElapsedTime endPath = new ElapsedTime();
    ElapsedTime stage1Timer = new ElapsedTime();

    // All Vector2D x values: 360 - redX
    // shoot:         (190,330)->(170,330)   (193,308)->(167,308)
    private final sectionBuilder[] shoot = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(170, 330), new Vector2D(167, 312)),
    };
    // driveToShoot1: (287,273)->(73,273)   (235,338)->(125,338)
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(73, 273), new Vector2D(125, 338)),
    };
    // collect2: (212,305)->(148,305)  (240,240)->(120,240)  (297,211)->(63,211)
    private final sectionBuilder[] collect2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(148, 305), new Vector2D(120, 240), new Vector2D(63, 211)),
    };
    // gate: (222,170)->(138,170)  (311,212)->(49,212)
    private final sectionBuilder[] gate = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(138, 170), new Vector2D(49, 212)),
    };
    // gateFromBack: (222,325)->(138,325)  (306,232)->(54,232)
    private final sectionBuilder[] gateFromBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(138, 325), new Vector2D(54, 232)),
    };
    // driveToShoot2: (313,206)->(47,206)  (217,150)->(143,150)
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(47, 206), new Vector2D(143, 150)),
    };
    // collect3: (220,150)->(140,150)  (294,158)->(66,158)
    private final sectionBuilder[] collect3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(140, 150), new Vector2D(66, 158)),
    };
    // driveToShoot3Stage1: (282,150)->(78,150)  (248,150)->(112,150)
    private final sectionBuilder[] driveToShoot3Stage1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(78, 150), new Vector2D(112, 150)),
    };
    // driveToShoot3Stage2: (248,150)->(112,150)  (200,172)->(160,172)
    private final sectionBuilder[] driveToShoot3Stage2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(112, 150), new Vector2D(160, 172)),
    };
    // firstBackCollect: (243,148)->(117,148)  (196,293)->(164,293)  (274,315)->(86,315)
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(117, 148), new Vector2D(164, 293), new Vector2D(86, 315)),
    };
    // driveToShootBack: (308,329)->(52,329)  (202,327)->(158,327)
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(158, 327)),
    };
    // firstDriveToShootBack: (320,208)->(40,208)  (230,330)->(130,330)
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(40, 208), new Vector2D(130, 330)),
    };
    // movePath: (308,329)->(52,329)  (260,300)->(100,300)
    private final sectionBuilder[] movePath = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(100, 300)),
    };
    // S1: (288,340)->(72,340)  (252,320)->(108,320)
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(72, 340), new Vector2D(108, 320)),
    };
    // tryAgain: (255,320)->(105,320)  (224,320)->(136,320)
    private final sectionBuilder[] tryAgain = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(105, 320), new Vector2D(136, 320)),
    };

    private void enterGateFromBack() {
        visionCollect  = false;
        collectDone    = false;
        ballsInIntake  = false;
        alreadyFailed  = false;
        pathing        = true;
        built          = false;
        PIDAtGate      = false;
        follow.setPath(paths.returnPath("gateFromBack"));
        follow.usePathHeadings(true);
        follow.setHeadingLookAheadDistance(100);
        follow.setHeadingOffset(90);
        intake.block   = true;
        state          = AutoState.gate;
    }

    @Override
    public void initEX() {
        // Start position: 360 - 192.5 = 167.5, heading stays 0
        odometry.startPosition(167.5, 342, 0);
        odometry.odo.setHeading(90, AngleUnit.DEGREES); // Blue heading

        turret.Auto = true;
        // turret.targetX = 360 removed — blue uses default (0)
        driveBase.tele = false;
        follow.setHeadingOffset(90);
        paths.addNewPath("shoot");
        paths.buildPath(shoot);
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
        paths.addNewPath("driveToShoot3Stage1");
        paths.buildPath(driveToShoot3Stage1);
        paths.addNewPath("driveToShoot3Stage2");
        paths.buildPath(driveToShoot3Stage2);
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
        paths.addNewPath("gateFromBack");
        paths.buildPath(gateFromBack);

        Apriltag.limelight.pipelineSwitch(0);

        processor = new LocalVision(LocalVision.TargetColor.BOTH);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(false);
        builder.addProcessor(processor);
        visionPortal = builder.build();
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(visionPortal, 4);
        intake.auto = true;
    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;

        if (!reset) {
            gameTime.reset();
            reset = true;
            odometry.odo.setHeading(90, AngleUnit.DEGREES); // Blue
        }

        if (intakeOff && intakeoff.milliseconds() > 600) {
            intake.InTake = false;
            intakeOff = false;
        } else if (intake.ballCount < 3 && !intakeOff) {
            intakeoff.reset();
        }

        if (intake.ballCount < 1 && !ballShot) {
            ballShot = true;
            ballshot.reset();
        }
        if (intake.ballCount > 2 && !ballsInIntake) {
            ballCollectWait.reset();
            ballsInIntake = true;
        } else if (ballsInIntake && intake.ballCount < 2) {
            ballsInIntake = false;
        }

        if (visionCollect) {
            // Vision angle comparisons: flip operators, remove negatives
            // Red: < -22  →  Blue: > 22
            if (processor.hAngleDeg > 22 && !intakePathSelected) {
                final sectionBuilder[] p3 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(49, 293)),
                };
                paths.addNewPath("p3");
                paths.buildPath(p3);
                follow.setPath(paths.returnPath("p3"));
                pathing = true;
                intakePathSelected = true;
                follow.setHeadingOffset(90);
                follow.usePathHeadings(true);
                extraShootDrive = 7;
                intake.block = false;
                intake.InTake = true;
                maxWait.reset();

                // Red: > -6  →  Blue: < 6
            } else if (processor.hAngleDeg < 6 && !intakePathSelected) {
                final sectionBuilder[] p1 = new sectionBuilder[]{
                        // Red: (odometry.X(), y), (220,337), (310 - r/8, 337)
                        // Blue: (odometry.X(), y), (140,337), (50 + r/8, 337)
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(140, 337), new Vector2D(49 + processor.radiusPixels / 8, 337)),
                };
                paths.addNewPath("p1");
                paths.buildPath(p1);
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                intakePathSelected = true;
                follow.setHeadingOffset(90);
                follow.usePathHeadings(false);
                targetHeading = 260; // abs(100 - 360) = 260
                intake.block = true;
                intake.InTake = true;
                extraShootDrive = 0;
                p1Pathing = true;
                maxWait.reset();

                // Red: < -6 && > -22  →  Blue: > 6 && < 22
            } else if (!intakePathSelected && processor.hAngleDeg > 6 && processor.hAngleDeg < 22) {
                final sectionBuilder[] p2 = new sectionBuilder[]{
                        // Red: (310 - r/8, 320)  →  Blue: (50 + r/8, 320)
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(49 + processor.radiusPixels / 8, 320)),
                };
                paths.addNewPath("p2");
                paths.buildPath(p2);
                follow.setPath(paths.returnPath("p2"));
                pathing = true;
                intakePathSelected = true;
                follow.setHeadingOffset(90);
                follow.usePathHeadings(true);
                intake.block = true;
                intake.InTake = true;
                extraShootDrive = 0;
                maxWait.reset();
            }

            if (follow.isFinished(5, 10) || maxWait.milliseconds() > 1400) {
                collectDone = true;
            }
            if (endPath.milliseconds() > 20 && waitAtEnd) {
                waitAtEnd = false;
                collectDone = true;
            }

            intake.block = true;
            intake.InTake = true;
        }

        switch (state) {
            case preLoad:
                if (!Preload) {
                    preload.reset();
                    intake.poz = Intake.intakePoz.gatePoz;
                    Preload = true;
                    odometry.odo.setHeading(90, AngleUnit.DEGREES); // Blue
                    follow.setPath(paths.returnPath("shoot"));
                    pathing = true;
                    driveBase.speed = 1;
                    turret.mapOfset = 5;
                    turret.turrofset = 2;
                    turret.StopSWM = true;

                    targetHeading = 270; // abs(90 - 360) = 270
                }
                if (built && turret.diff < 120 && turret.rpm > 1000) {
                    intake.InTake = true;
                }
                if (built && preload.milliseconds() > 1400 || built && turret.diff < 60 && turret.rpm > 1200 && odometry.getYVelocity() < 8) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }
                if (pathing && follow.isFinished(10, 10)) {
                    pathing = false;
                }
                if (!built && shootTime.milliseconds() > 400) {
                    final sectionBuilder[] collect1 = new sectionBuilder[] {
                            // Red target was (278,270) → Blue: (360-278,270) = (82,270)
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(82, 270)),
                    };
                    paths.addNewPath("collect1");
                    paths.buildPath(collect1);
                    follow.setPath(paths.returnPath("collect1"));
                    turret.StopSWM = false;
                    turret.mapOfset = -35;
                    targetHeading = 278; // abs(82 - 360) = 278
                    turret.turrofset = 4; // flip sign: -3.5 → +3.5

                    pathing = true;
                    built = true;
                    intake.InTake = true;
                    intake.block = true;
                    state = AutoState.collect1;
                }
                break;

            case collect1:
                // Red: odometry.X() > 275  →  Blue: odometry.X() < 85  (360-275=85, flip operator)
                if (pathing && odometry.X() < 85) {
                    targetHeading = 305; // abs(55 - 360) = 305
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)) {
                    state = AutoState.driveToShoot1;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    follow.usePathHeadings(true);
                    intakeoff.reset();
                    intakeOff = true;
                    follow.setHeadingOffset(-90);
                    follow.setHeadingLookAheadDistance(100);
                }
                break;

            case driveToShoot1:
                if (follow.isFinished(22, 34)) {
                    follow.usePathHeadings(false);
                    targetHeading = 285; // abs(75 - 360) = 285
                }
                if (built && follow.isFinished(22, 22) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 19) {
                    intake.InTake = true;
                    built = false;
                    pathing = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }
                if (!built && shootTime.milliseconds() > 320 || !built && ballShot) {
                    driveBase.speed = 1.4;
                    collectDone = false;
                    ballsInIntake = false;
                    intake.holdUp = false;
                    maxWait.reset();
                    HoldHeadingWhileShooting = false;
                    built = true;
                    turret.turrofset = 1.7; // flip sign: -4.5 → +4.5
                    turret.mapOfset = -75;
                    state = AutoState.backCollect;
                }
                break;

            case driveToShootBack:
                if (afterGateCollect && odometry.Y() > 230) {
                    intake.InTake = false;
                }
                if (afterGateCollect && odometry.Y() > 270) {
                    targetHeading = 282; // abs(78 - 360) = 282
                }
                // Red: odometry.X() < 284  →  Blue: odometry.X() > 76  (360-284=76, flip operator)
                if (odometry.X() > 76 && intake.poz == Intake.intakePoz.normalPoz && shootTime.milliseconds() > 500 && !(backCycles == 0)) {
                    intake.poz = Intake.intakePoz.up;
                    intake.InTake = false;
                    intake.holdUp = true;
                }
                if (follow.isFinished(20, 25) && Math.abs(Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity()))
                        + Math.abs(odometry.getHVelocity() * 2) < 45) {
                    pathing = false;
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - 270), 0); // Blue: 270
                    HoldHeadingWhileShooting = true;
                }
                // Red: odometry.X() < 250  →  Blue: odometry.X() > 110  (360-250=110, flip operator)
                if (follow.isFinished(20, 25) && odometry.X() > 110 && !built
                        && Math.abs(Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())) + Math.abs(odometry.getHVelocity() * 2) < 28
                        && !dontWaitForPoz) {
                    shootWait = 380;
                    shootTime.reset();
                    follow.usePathHeadings(false);
                    pathing = false;
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - 270), 0); // Blue: 270
                    HoldHeadingWhileShooting = true;

                    gateTime = 1200;
                    backCycles += 1;
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    afterGateCollect = false;
                    intake.block = false;
                    intake.InTake = true;
                    IntakeOffWait = 200;
                }
                if (built && shootTime.milliseconds() > 120) {
                    intake.poz = Intake.intakePoz.normalPoz;
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
                    backCycles += 1;
                    driveBase.speed = 1.4;
                    if (backCycles >= 2) {
                        enterGateFromBack();
                        gateAngle = 307; // abs(53 - 360) = 307
                    } else {
                        follow.usePathHeadings(false);
                        dontWaitForPoz = false;
                        built = true;
                        pathing = false;
                        intake.InTake = true;
                        ballsInIntake = false;
                        collectDone = false;
                        maxWait.reset();
                        intake.holdUp = false;
                        alreadyFailed = false;
                        intake.block = true;
                        state = AutoState.backCollect;
                    }
                }
                if (built && shootTime.milliseconds() > shootWait) {
                    driveBase.speed = 1.4;
                    collectDone = false;
                    ballsInIntake = false;
                    intake.holdUp = false;
                    maxWait.reset();
                    HoldHeadingWhileShooting = false;
                    state = AutoState.backCollect;
                    turret.turrofset += 0.1;

                }
                break;

            case backCollect:
                if (built && !collectDone) {
                    if (!(backCycles == 0)) {
                        p1Pathing = false;
                        visionCollect = true;
                    } else if (!pathing) {
                        final sectionBuilder[] p2 = new sectionBuilder[]{
                                // Red target was (320,342) → Blue: (360-320,342) = (40,342)
                                () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(40, 342)),
                        };
                        paths.addNewPath("p2");
                        paths.buildPath(p2);
                        follow.setPath(paths.returnPath("p2"));
                        follow.setHeadingOffset(90);
                        follow.usePathHeadings(true);
                        if (!pathing) {
                            maxWait.reset();
                        }
                        pathing = true;
                        intakePathSelected = true;
                        p3Qued = false;
                        intake.block = true;
                    }
                }
                // Red: odometry.X() > 275  →  Blue: odometry.X() < 85  (360-275=85, flip operator)
                if (backCycles == 0 && odometry.X() < 85) {
                    driveBase.speed = 0.4;
                }
                if (backCycles == 0) {
                }
                if (backCycles == 0 && follow.isFinished(5, 10)) {
                    collectDone = true;
                }

                if (built && collectDone) {
                    visionCollect = false;
                    state = AutoState.driveToShootBack;
                    final sectionBuilder[] S1 = new sectionBuilder[] {
                            // Red: (odometry.X(), y), (250 - extraShootDrive, 330)
                            // Blue: (odometry.X(), y), (110 + extraShootDrive, 330)  [360-250=110, and extra flips direction]
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(110 + extraShootDrive, 330)),
                    };
                    driveBase.speed = 1;

                    paths.addNewPath("S1");
                    paths.buildPath(S1);
                    follow.setPath(paths.returnPath("S1"));

                    follow.usePathHeadings(false);
                    targetHeading = 270; // abs(90 - 360) = 270

                    intakePathSelected = false;
                    maxToGetToShoot.reset();

                    ballShot = false;
                    targetHeading = 270;

                    pathing = true;
                    built = false;
                }
                break;

            case finished:
                requestOpModeStop();
        }

        if (pathing && p1Pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(-currentPower.getHorizontal(), currentPower.getPivot(), -currentPower.getVertical() / 10));
        } else if (pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (!PIDAtGate && !HoldHeadingWhileShooting) {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }
    }
}