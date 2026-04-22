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

@Autonomous(name="", group="Red")

public class back_twospike_RED extends OpModeEX {
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

    double backCycles = 0;
    double shootWait = 700;
    double gateTolX = 10; double gateTolY = 8; double gateTurnX = 248; double gateAngle = 58; double gateTime = 1150;
    boolean stage1Done = false;
    double extraShootDrive = 0;


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

    private final sectionBuilder[] shoot = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(190, 330), new Vector2D(193, 298)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(287, 273), new Vector2D(220, 299)),
    };
    private final sectionBuilder[] collect2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(212, 305), new Vector2D(240, 240), new Vector2D(300, 211)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(222, 170), new Vector2D(312.5, 213)),
    };
    private final sectionBuilder[] gateFromBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(222, 325), new Vector2D(308, 233)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(313, 206), new Vector2D(204, 176)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(220, 150), new Vector2D(294, 158)),
    };
    private final sectionBuilder[] driveToShoot3Stage1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(282, 150), new Vector2D(248, 150)),
    };
    private final sectionBuilder[] driveToShoot3Stage2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(248, 150), new Vector2D(200, 172)),
    };
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(243, 148), new Vector2D(196, 293), new Vector2D(274, 315)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(202, 327)),
    };
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(320, 208), new Vector2D(230, 330)),
    };
    private final sectionBuilder[] movePath = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(260, 300)),
    };
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(288, 340), new Vector2D(252, 320)),
    };
    private final sectionBuilder[] tryAgain = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(255, 320), new Vector2D(224, 320)),
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
        odometry.startPosition(192.5, 342, 0);           // 360 - 167.5
        odometry.odo.setHeading(270, AngleUnit.DEGREES); // Red heading

        turret.Auto = true;
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

        turret.targetX = 360;
    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;

        if (!reset) {
            gameTime.reset();
            reset = true;
            odometry.odo.setHeading(270, AngleUnit.DEGREES);
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

        if (visionCollect){
            if (processor.hAngleDeg < -22 && !intakePathSelected){
                final sectionBuilder[] p3 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(310, 293)),
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
                extraShootDrive = 7;


                maxWait.reset();

            } else if (processor.hAngleDeg > -6 && !intakePathSelected) {
                final sectionBuilder[] p1 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(220, 337), new Vector2D(310 - processor.radiusPixels /8, 337)),
                };
                paths.addNewPath("p1");
                paths.buildPath(p1);
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                intakePathSelected = true;
                follow.setHeadingOffset(90);
                follow.usePathHeadings(false);
                targetHeading = 100;
                intake.block = true;
                intake.InTake = true;
                p1Pathing = true;
                maxWait.reset();
                extraShootDrive = 0;


            } else if (!intakePathSelected && processor.hAngleDeg < -6 && processor.hAngleDeg > -22) {
                final sectionBuilder[] p2 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(310 - processor.radiusPixels /8, 320)),
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

            if (follow.isFinished(5,10) || maxWait.milliseconds() > 1400 ){
                collectDone = true;
            }
            if (endPath.milliseconds() > 40 && waitAtEnd){
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
                    odometry.odo.setHeading(270, AngleUnit.DEGREES);
                    follow.setPath(paths.returnPath("shoot"));
                    pathing = true;
                    driveBase.speed = 1;
                    turret.mapOfset = 75;
                    turret.turrofset = -1;
                    turret.StopSWM = true;

                    targetHeading = 90;
                }
                if (built && turret.diff < 120 && turret.rpm > 1000){
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
                if (!built && shootTime.milliseconds() > 380) {
                    final sectionBuilder[] collect1 = new sectionBuilder[] {
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(286, 270)),
                    };
                    paths.addNewPath("collect1");
                    paths.buildPath(collect1);
                    follow.setPath(paths.returnPath("collect1"));
                    turret.StopSWM = false;
                    turret.mapOfset = -20;
                    targetHeading = 82;
                    turret.turrofset = 0;

                    pathing = true;
                    built = true;
                    intake.InTake = true;
                    intake.block = true;
                    state = AutoState.collect1;
                }
                break;

            case collect1:
                if (pathing && odometry.X() > 275 ) {
                    targetHeading = 65;
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
                if (follow.isFinished(17, 35)) {
                    follow.usePathHeadings(false);
                    targetHeading = 50;
                }
                if (built && follow.isFinished(22, 22) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 18) {
                    intake.InTake = true;
                    built = false;
                    pathing = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }
                if (!built && shootTime.milliseconds() > 380 || !built && ballShot) {
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(160);
                    follow.setHeadingOffset(90);
                    turret.turrofset = 5;
                    turret.mapOfset = 15;

                    pathing = true;
                    intake.InTake = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect2;
                }
                break;

            case collect2:
                if (pathing && follow.isFinished(12,12)) {
                    targetHeading = 108;
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
                    targetHeading = 110;
                }
                if (built && follow.isFinished(20, 20) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 100) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    intake.InTake = true;
                    pathing = false;
                    ballShot = false;
                }
                if (follow.isFinished(15, 15) && !built && shootTime.milliseconds() > 340) {
                    follow.setPath(paths.returnPath("gate"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    state = AutoState.gate;
                    turret.turrofset = -1.5;
                    turret.mapOfset = -40;

                }
                break;



            case gate:
                if (odometry.X() > gateTurnX) {                 // Flipped operator
                    follow.usePathHeadings(false);
                    targetHeading = gateAngle;
                    intake.poz = Intake.intakePoz.gatePoz;
                    intake.InTake = true;
                }
                if (follow.isFinished(gateTolX, gateTolY) && !built) {
                    pathing = false;
                    gateInTakeTime.reset();
                    built = true;
                }
                if (!pathing) {
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - gateAngle), 0);
                    PIDAtGate = true;
                }

                if (built && gateInTakeTime.milliseconds() > gateTime || built && ballsInIntake && ballCollectWait.milliseconds() > 260 && gateInTakeTime.milliseconds() > 700) {
                    pathing = true;
                    PIDAtGate = false;
                    follow.setPath(paths.returnPath("firstDriveToShootBack"));
                    follow.usePathHeadings(false);
                    state = AutoState.driveToShootBack;
                    afterGateCollect = true;
                    driveBase.speed = 1;
                    targetHeading = 52;
                    maxToGetToShoot.reset();
                    intake.poz = Intake.intakePoz.normalPoz;

                    built = false;
                }
                break;

            case driveToShootBack:
                if (afterGateCollect && odometry.Y() > 230) {
                    intake.InTake = false;
                }
                if (afterGateCollect && odometry.Y() > 270) {
                    targetHeading = 78;
                }
                if (odometry.X() < 284 && intake.poz == Intake.intakePoz.normalPoz && shootTime.milliseconds() > 500){  // 360-76
                    intake.poz = Intake.intakePoz.up;
                    intake.InTake = false;
                    intake.holdUp = true;
                }
                if (follow.isFinished(20, 25) && Math.abs( Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity()))
                        + Math.abs(odometry.getHVelocity() * 2) < 50) {
                    pathing = false;
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - 90), 0);  // Changed 270 → 90
                    HoldHeadingWhileShooting = true;
                }
                if (follow.isFinished(20, 25) && odometry.X() < 250 && !built
                        && Math.abs( Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity())) + Math.abs(odometry.getHVelocity() * 2) < 35
                        && !dontWaitForPoz) {
                    shootWait = 380;
                    shootTime.reset();
                    follow.usePathHeadings(false);
                    pathing = false;
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - 90), 0);
                    HoldHeadingWhileShooting = true;

                    gateTime = 1100;
                    backCycles += 1;
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    afterGateCollect = false;
                    intake.block = false;
                    intake.InTake = true;
                    IntakeOffWait = 200;
                }
                if (built && shootTime.milliseconds() > 120){
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
                    driveBase.speed = 1;
                    if (backCycles >= 2) {
                        enterGateFromBack();
                        gateAngle  = 53;
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
                        state = AutoState.backCollect;
                    }
                }
                if (built && shootTime.milliseconds() > shootWait) {
                    driveBase.speed = 1;
                    collectDone = false;
                    ballsInIntake = false;
                    intake.holdUp = false;
                    maxWait.reset();
                    HoldHeadingWhileShooting = false;

                    if (backCycles == 2) {
                        gateTolX = 8;
                        gateTolY = 8;
                        gateTurnX = 286;                        // 360-74
                        enterGateFromBack();
                        gateAngle  = 50;                        // 360-305
                    } else {
                        state = AutoState.backCollect;
                    }
                }
                break;

            case backCollect:
                if (built && !collectDone ){
                    p1Pathing = false;
                    visionCollect = true;
                    driveBase.speed = 1.4;
                }

                if (built && collectDone ){
                    visionCollect = false;
                    state = AutoState.driveToShootBack;
                    final sectionBuilder[] S1 = new sectionBuilder[] {
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(240, 330)),
                    };
                    paths.addNewPath("S1");
                    paths.buildPath(S1);
                    follow.setPath(paths.returnPath("S1"));
                    driveBase.speed = 1;
                    follow.usePathHeadings(false);
                    targetHeading = 90;

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

        if (pathing && p1Pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(-currentPower.getHorizontal(),currentPower.getPivot(),-currentPower.getVertical() / 10));
        } else if (pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (!PIDAtGate && !HoldHeadingWhileShooting) {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }
    }
}