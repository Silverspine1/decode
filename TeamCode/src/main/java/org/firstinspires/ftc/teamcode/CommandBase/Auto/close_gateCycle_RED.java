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

@Autonomous(name="close_gateCycle", group="Red")

public class close_gateCycle_RED extends OpModeEX {
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
        gateShoot,
        gate,
        driveToShootBack,
        finished
    }

    double frontOffset     = 8.0;
    double sideOffsetDeg   = 0.0;
    double robotHalfWidth  = 17.5;
    double wallSafetyMargin = 10.0;
    double wallBuffer       = 20.0;
    double maxWallAvoidPower = 0.40;
    double wallVelGain      = 0.018;
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

    double balls = 0;
    double shootWait = 700;
    // gateTurnX: 360 - 112 = 248
    // gateAngle: abs(295 - 360) = 65
    double gateTolX = 10; double gateTolY = 8; double gateTurnX = 248; double gateAngle = 65; double gateTime = 1100;
    boolean stage1Done = false;

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


    // All Vector2D x values: 360 - oldX
    // shoot:       (47.5,84)->(312.5,84)   (125,120)->(235,120)
    private final sectionBuilder[] shoot = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(312.5, 84), new Vector2D(235, 120)),
    };
    // driveToShoot1: (60,140)->(300,140)   (110,154)->(250,154)
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(300, 140), new Vector2D(250, 154)),
    };
    // collect2: (134,154)->(226,154)  (116,230)->(244,230)  (45,210)->(315,210)
    private final sectionBuilder[] collect2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(226, 154), new Vector2D(244, 230), new Vector2D(315, 210)),
    };
    // gate: (130,150)->(230,150)  (38,209)->(322,209) -- NOTE: blue drives left toward gate; red drives right
    private final sectionBuilder[] gate = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(230, 150), new Vector2D(322, 209)),
    };
    // gateFromBack: (138,325)->(222,325)  (55,222)->(305,222)
    private final sectionBuilder[] gateFromBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(222, 325), new Vector2D(305, 222)),
    };
    // driveToShoot2: (49,210)->(311,210)  (138,150)->(222,150)
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(311, 210), new Vector2D(222, 150)),
    };
    // collect3: (140,150)->(220,150)  (70,151)->(290,151)
    private final sectionBuilder[] collect3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(220, 150), new Vector2D(290, 151)),
    };
    // driveToShoot3Stage1: (78,150)->(282,150)  (112,150)->(248,150)
    private final sectionBuilder[] driveToShoot3Stage1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(282, 150), new Vector2D(248, 150)),
    };
    // driveToShoot3Stage2: (112,150)->(248,150)  (160,172)->(200,172)
    private final sectionBuilder[] driveToShoot3Stage2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(248, 150), new Vector2D(200, 172)),
    };
    // firstBackCollect: (117,148)->(243,148)  (164,293)->(196,293)  (86,315)->(274,315)
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(243, 148), new Vector2D(196, 293), new Vector2D(274, 315)),
    };
    // driveToShootBack: (52,329)->(308,329)  (158,327)->(202,327)
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(202, 327)),
    };
    // firstDriveToShootBack: (40,208)->(320,208)  (130,330)->(230,330)
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(320, 208), new Vector2D(230, 330)),
    };
    // movePath: (52,329)->(308,329)  (100,300)->(260,300)
    private final sectionBuilder[] movePath = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(260, 300)),
    };
    // S1: (72,340)->(288,340)  (108,320)->(252,320)
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(288, 340), new Vector2D(252, 320)),
    };
    // tryAgain: (105,320)->(255,320)  (136,320)->(224,320)
    private final sectionBuilder[] tryAgain = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(255, 320), new Vector2D(224, 320)),
    };

    private void enterGate() {
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
        // Start position: X = 360 - 47.5 = 312.5, heading = abs(0 - 360) = 0 (stays 0)
        odometry.startPosition(312.5, 84, 0);
        odometry.odo.setHeading(270, AngleUnit.DEGREES); // RED

        turret.Auto = true;
        turret.targetX = 360; // RED alliance
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
            odometry.odo.setHeading(270, AngleUnit.DEGREES); // RED
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
            // Vision angle comparisons: flip operators AND flip sign of threshold
            // Blue: > 22  →  Red: < -22
            if (processor.hAngleDeg < -22 && !intakePathSelected) {
                final sectionBuilder[] p3 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(360 - odometry.X(), odometry.Y()), new Vector2D(305, 293)),
                };
                paths.addNewPath("p3");
                paths.buildPath(p3);
                follow.setPath(paths.returnPath("p3"));
                pathing = true;
                intakePathSelected = true;
                follow.setHeadingOffset(90);
                follow.usePathHeadings(true);
                intake.block = false;
                intake.InTake = true;
                maxWait.reset();

            // Blue: < 6  →  Red: > -6
            } else if (processor.hAngleDeg > -6 && !intakePathSelected) {
                final sectionBuilder[] p1 = new sectionBuilder[]{
                        // X coords: 360 - odometry.X() for current pos; 360-140=220, 360-(55+r/8)=305-r/8
                        () -> paths.addPoints(new Vector2D(360 - odometry.X(), odometry.Y()), new Vector2D(220, 337), new Vector2D(305 - processor.radiusPixels / 8, 337)),
                };
                paths.addNewPath("p1");
                paths.buildPath(p1);
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                intakePathSelected = true;
                follow.setHeadingOffset(90);
                follow.usePathHeadings(false);
                targetHeading = 90; // abs(270 - 360) = 90
                intake.block = true;
                intake.InTake = true;
                p1Pathing = true;
                maxWait.reset();

            // Blue: > 6 && < 22  →  Red: < -6 && > -22
            } else if (!intakePathSelected && processor.hAngleDeg < -6 && processor.hAngleDeg > -22) {
                final sectionBuilder[] p2 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(360 - odometry.X(), odometry.Y()), new Vector2D(305 - processor.radiusPixels / 8, 320)),
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
                maxWait.reset();
            }

            if (follow.isFinished(5, 10) || maxWait.milliseconds() > 1400) {
                collectDone = true;
            }
            if (endPath.milliseconds() > 40 && waitAtEnd) {
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
                    odometry.odo.setHeading(270, AngleUnit.DEGREES); // RED
                    follow.setPath(paths.returnPath("shoot"));
                    pathing = true;
                    driveBase.speed = 1;
                    turret.TURRET_COMP_FACTOR = 0.95;
                    turret.turrofset = 8; // flip sign: -8 → +8

                    targetHeading = 105; // abs(255 - 360) = 105
                }
                if (built && turret.diff < 120 && turret.rpm > 1000) {
                    intake.InTake = true;
                }
                if (built && preload.milliseconds() > 1400 || built && turret.diff < 60 && turret.rpm > 1200 && odometry.getYVelocity() < 8) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    balls += 3;
                    shootTime.reset();
                }
                if (pathing && follow.isFinished(10, 10)) {
                    pathing = false;
                }
                if (!built && shootTime.milliseconds() > 400) {
                    final sectionBuilder[] collect1 = new sectionBuilder[] {
                            // current X → 360 - odometry.X(); target was (52,158) → (308,158)
                            () -> paths.addPoints(new Vector2D(360 - odometry.X(), odometry.Y()), new Vector2D(308, 158)),
                    };
                    paths.addNewPath("collect1");
                    paths.buildPath(collect1);
                    follow.setPath(paths.returnPath("collect1"));
                    driveBase.speed = 1;
                    targetHeading = 105; // abs(255 - 360) = 105

                    pathing = true;
                    built = true;
                    intake.InTake = true;
                    intake.block = true;
                    state = AutoState.collect1;
                }
                break;

            case collect1:
                if (pathing && follow.isFinished(10, 10)) {
                    state = AutoState.driveToShoot1;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    driveBase.speed = 1;

                    follow.usePathHeadings(true);
                    intakeoff.reset();
                    intakeOff = true;
                    follow.setHeadingOffset(-90);
                    follow.setHeadingLookAheadDistance(100);
                }
                break;

            case driveToShoot1:
                if (follow.isFinished(17, 35)) {
                    follow.usePathHeadings(false);
                    targetHeading = 135; // abs(225 - 360) = 135
                }
                if (built && follow.isFinished(22, 22) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 80) {
                    intake.InTake = true;
                    built = false;
                    pathing = false;
                    intake.block = false;
                    shootTime.reset();
                    balls += 3;
                    ballShot = false;
                }
                if (!built && shootTime.milliseconds() > 380
                        || !built && ballShot) {
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(160);
                    follow.setHeadingOffset(90);
                    turret.turrofset = 6; // flip sign: -6 → +6
                    turret.mapOfset = 10;

                    pathing = true;
                    intake.InTake = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect2;
                }
                break;

            case collect2:
                if (pathing && follow.isFinished(12, 12)) {
                    targetHeading = 108; // abs(252 - 360) = 108
                    follow.usePathHeadings(false);
                }
                if (follow.isFinished(8, 8)) {
                    state = AutoState.driveToShoot2;
                    follow.setPath(paths.returnPath("driveToShoot2"));
                    follow.setHeadingOffset(-90);
                    intakeoff.reset();
                    intakeOff = true;
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(130);
                    built = true;
                }
                break;

            case driveToShoot2:
                if (follow.isFinished(43, 43)) {
                    follow.usePathHeadings(false);
                    targetHeading = 130; // abs(230 - 360) = 130
                }
                if (built && follow.isFinished(20, 20) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 80) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    intake.InTake = true;
                    pathing = false;
                    balls += 3;
                    ballShot = false;
                }
                if (follow.isFinished(15, 15) && !built && shootTime.milliseconds() > 380) {
                    follow.setPath(paths.returnPath("gate"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    state = AutoState.gate;
                    turret.turrofset = 1; // flip sign: -1 → +1
                    turret.mapOfset = 10;
                }
                break;

            case gate:
                // X comparison: blue < 112  →  red > 248  (360 - 112 = 248, flip operator)
                if (odometry.X() > gateTurnX) {
                    follow.usePathHeadings(false);
                    targetHeading = gateAngle; // 65
                    intake.poz = Intake.intakePoz.gatePoz;
                    intake.InTake = true;
                }
                if (follow.isFinished(gateTolX, gateTolY) && !built) {
                    pathing = false;
                    gateInTakeTime.reset();
                    built = true;
                }

                if (!pathing) {
                    // headingPID: blue - 295  →  red - 65
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - gateAngle), 0);
                    PIDAtGate = true;
                }

                // X comparison: blue < 122  →  red > 238  (360 - 122 = 238, flip operator)
                if (odometry.X() > 238 && balls == 24) {
                    state = AutoState.finished;
                }

                if (built && gateInTakeTime.milliseconds() > gateTime || built && ballsInIntake && ballCollectWait.milliseconds() > 260 && gateInTakeTime.milliseconds() > 300) {
                    pathing = true;
                    PIDAtGate = false;
                    follow.setPath(paths.returnPath("driveToShoot2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(-90);
                    state = AutoState.gateShoot;
                    afterGateCollect = true;
                    driveBase.speed = 1;
                    maxToGetToShoot.reset();
                    intake.poz = Intake.intakePoz.normalPoz;
                    intakeOff = true;
                    built = true;
                }
                break;

            case gateShoot:
                if (follow.isFinished(30, 43)) {
                    follow.usePathHeadings(false);
                    targetHeading = 130; // abs(230 - 360) = 130
                }
                if (built && follow.isFinished(20, 20) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 60) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    intake.InTake = true;
                    pathing = false;
                    ballShot = false;
                    balls += 3;
                }
                if (!built && shootTime.milliseconds() > 400) {
                    follow.setPath(paths.returnPath("gate"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    state = AutoState.gate;
                    turret.turrofset = 1; // flip sign: -1 → +1
                    turret.mapOfset = 10;
                }
                break;

            case finished:
                requestOpModeStop();
        }

        if (pathing && p1Pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(-currentPower.getHorizontal(), currentPower.getPivot(), -currentPower.getVertical() / 15));
        } else if (pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (!PIDAtGate && !HoldHeadingWhileShooting) {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }
    }
}
