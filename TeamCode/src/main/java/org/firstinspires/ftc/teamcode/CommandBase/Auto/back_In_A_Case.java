package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous

public class back_In_A_Case extends OpModeEX {
    pathsManager paths = new pathsManager(new RobotConfig(0.02, 0.004, 0.02, 0.005, 0.08, 0.004, 0.09, 0.004, 0.01, 0.0005,
            0.012, 0.002, 130, 240, 650, 1400));

    follower follow = new follower(new RobotConfig(0.02, 0.004, 0.02, 0.005, 0.08, 0.004, 0.09, 0.004, 0.01, 0.0005,
            0.012, 0.002, 130, 240, 650, 1400));
    PIDController headingPID = new PIDController(0.012, 0, 0.0030);
    PIDController x = new PIDController(0.06, 0, 0.0030);
    PIDController y = new PIDController(0.015, 0, 0.0030);

//    private VisionPortal visionPortal;
//    private LocalVision processor;
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
    boolean driveBackToShoot = false;

    double backCycles = 0;
    double shootWait = 700;
    double velo = 16;
    double gateTolX = 10; double gateTolY = 8; double gateTurnX = 85;double gateAngle = 298;
    double targetPos = 44;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime ballshot = new ElapsedTime();
    ElapsedTime gateInTakeTime = new ElapsedTime();
    ElapsedTime maxToGetToShoot = new ElapsedTime();
    ElapsedTime ballCollectWait = new ElapsedTime();
    ElapsedTime waitAfterCollected = new ElapsedTime();


    private final sectionBuilder[] shoot = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(170, 330), new Vector2D(170, 300)),
    };
    private final sectionBuilder[] collect1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(170, 310), new Vector2D(126, 275), new Vector2D(74, 273)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(73, 273), new Vector2D(148, 305)),
    };
    private final sectionBuilder[] collect2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(163, 300), new Vector2D(126, 210), new Vector2D(68, 204)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(138, 170), new Vector2D(61, 199.5)),
    };
    private final sectionBuilder[] gateFromBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(138, 325), new Vector2D(55, 210)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(47, 206), new Vector2D(157, 150)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(140, 150), new Vector2D(70, 146)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(78, 150), new Vector2D(138, 170)),
    };
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(117, 148), new Vector2D(164, 293), new Vector2D(86, 315)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(158, 327)),
    };
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(40, 208), new Vector2D(130, 332)),
    };
    private final sectionBuilder[] movePath = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(100, 300)),
    };
    private final sectionBuilder[] S1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(72, 340), new Vector2D(108, 320)),
    };
    private final sectionBuilder[] tryAgain = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(105, 320), new Vector2D(136, 320)),
    };

    // resets all relevant state cleanly.
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
        odometry.startPosition(167.5, 342, 0);
        odometry.odo.setHeading(90, AngleUnit.DEGREES);

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
        paths.addNewPath("gateFromBack");
        paths.buildPath(gateFromBack);

        Apriltag.limelight.pipelineSwitch(0);

//        processor = new LocalVision(LocalVision.TargetColor.BOTH);
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        builder.setCameraResolution(new Size(640, 480));
//        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//        builder.enableLiveView(false);
//        builder.addProcessor(processor);
//        visionPortal = builder.build();
//        dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        dashboard.startCameraStream(visionPortal, 4);
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
            odometry.odo.setHeading(90, AngleUnit.DEGREES);
        }

        if (intake.ballCount > 2 && intakeoff.milliseconds() > 500 && intake.block || intakeOff && intakeoff.milliseconds() > 600) {
            intake.InTake = false;
            intakeOff = false;
        }else if (intake.ballCount <3 && !intakeOff){
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


            if (maxWait.milliseconds() > 400 && odometry.X() < targetPos - 5
                    || maxWait.milliseconds() > 1100
                    || ballsInIntake
                    || odometry.Y() < 245) {
                if (!driveBackToShoot){
                    driveBackToShoot = true;
                    waitAfterCollected.reset();
                }

            }
            if (driveBackToShoot && waitAfterCollected.milliseconds() > 130){
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
                    odometry.odo.setHeading(90, AngleUnit.DEGREES);
                    follow.setPath(paths.returnPath("shoot"));
                    pathing = true;
                    driveBase.speed = 1;
                    turret.TURRET_COMP_FACTOR = 0;
                    turret.mapOfset = 15;
                    turret.turrofset = -1;


                    targetHeading = 270;
                }
                if (built && turret.diff < 150 && turret.rpm > 1000){
                    intake.InTake = true;
                }
                if (built && preload.milliseconds() > 1480 || built && turret.diff < 80 && turret.rpm > 1200) {
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
                    follow.usePathHeadings(true);
                    turret.TURRET_COMP_FACTOR = 0.95;
                    turret.mapOfset = -15;
                    turret.turrofset = -0.5;


                    pathing = true;
                    built = true;
                    intake.InTake = true;
                    intake.block = true;
                    state = AutoState.collect1;
                }
                break;

            case collect1:
                if (pathing && odometry.X() < 107) {
                    targetHeading = 295;
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
                    targetHeading = 310;
                }
                if (built && follow.isFinished(22, 22) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 5) {
                    intake.InTake = true;
                    built = false;
                    pathing = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }
                if (!built && shootTime.milliseconds() > 460
                        || !built && ballShot) {
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(160);
                    turret.turrofset = 0;
                    follow.setHeadingOffset(90);
                    turret.mapOfset = 0;

                    pathing = true;
                    intake.InTake = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect2;
                }
                break;

            case collect2:
                if (pathing && odometry.X() < 95) {
                    targetHeading = 245;
                    follow.usePathHeadings(false);
                }
                if (follow.isFinished(8, 8)) {
                    state = AutoState.driveToShoot2;
                    follow.setPath(paths.returnPath("driveToShoot2"));
                    follow.setHeadingOffset(-90);
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(130);
                    built = true;
                }
                break;

            case driveToShoot2:
                if (follow.isFinished(43, 43)) {
                    follow.usePathHeadings(false);
                    targetHeading = 270;
                }
                if (built && follow.isFinished(20, 20) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 50) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    intake.InTake = true;
                    pathing = false;
                    ballShot = false;
                }
                if (follow.isFinished(15, 15) && !built && shootTime.milliseconds() > 340) {
                    targetHeading = 270;
                    follow.setPath(paths.returnPath("collect3"));
                    follow.usePathHeadings(false);
                    intake.InTake = true;
                    pathing = true;
                    built = true;
                    intake.block = true;
                    intake.InTake = true;
                    state = AutoState.collect3;
                    turret.turrofset = -1;
                    driveBase.speed = 1;
                }
                break;

            case collect3:
                if (pathing && follow.isFinished(20, 10)) {
                    targetHeading = 295;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)) {
                    state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    driveBase.speed = 1;
                    intakeoff.reset();
                    intakeOff = true;
                    targetHeading = 270;
                }
                break;

            case driveToShoot3:
                if (built && follow.isFinished(18, 18) && (Math.abs(odometry.getXVelocity())
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 45) {
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    intake.InTake = true;
                    shootTime.reset();
                    ballShot = false;
                    pathing = false;
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
                }
                break;

            case gate:
                if (odometry.X() < gateTurnX) {
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

                if (built && gateInTakeTime.milliseconds() > 1150 || built && ballsInIntake && ballCollectWait.milliseconds() > 180 && gateInTakeTime.milliseconds() > 300) {
                    pathing = true;
                    PIDAtGate = false;
                    follow.setPath(paths.returnPath("firstDriveToShootBack"));
                    follow.usePathHeadings(false);
                    state = AutoState.driveToShootBack;
                    afterGateCollect = true;
                    driveBase.speed = 1;
                    targetHeading = 308;
                    maxToGetToShoot.reset();
                    intake.poz = Intake.intakePoz.normalPoz;

                    built = false;
                    turret.mapOfset = 0;
                    turret.turrofset = -0.5;
                }
                break;

            case driveToShootBack:
                if (afterGateCollect && odometry.Y() > 330) {
                    intake.InTake = false;
                }
                if (afterGateCollect && odometry.Y() > 280) {
                    targetHeading = 278;
                }
                if (odometry.X() > 64 && intake.poz == Intake.intakePoz.normalPoz && shootTime.milliseconds() > 500){
                    intake.poz = Intake.intakePoz.up;
                    intake.InTake = false;
                    intake.holdUp = true;
                }
                if (follow.isFinished(15, 30) && Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                        + Math.abs(odometry.getHVelocity() * 2) < 30) {
                    pathing = false;
                }
                if (!pathing && odometry.X() > 110 && !built
                        && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) + Math.abs(odometry.getHVelocity() * 2) < 20
                        && !dontWaitForPoz) {
                    shootWait = 400;
                    shootTime.reset();
                    follow.usePathHeadings(false);
                    backCycles += 1;
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    afterGateCollect = false;
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
                // alreadyFailed recovery — redirect to gate on cycle 2, else backCollect
                if (alreadyFailed && maxToGetToShoot.milliseconds() > 2200) {
                    backCycles += 1;
                    driveBase.speed = 1;
                    if (backCycles >= 2) {
                        enterGateFromBack();
                        gateAngle  = 307;
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
                    if (backCycles == 2) {
                        gateTolX = 8;
                        gateTolY = 8;
                        gateTurnX = 74;
                        enterGateFromBack();
                        gateAngle  = 305;

                    } else {
                        state = AutoState.backCollect;
                    }
                    if (backCycles == 1 ){
                        targetPos = 55;
                    }else if(backCycles == 3){
                        targetPos = 56;
                    }else{
                        targetPos = 44;
                    }
                }
                break;

            case backCollect:
                if (built && !collectDone && !visionCollect) {
                    visionCollect = true;
                    targetHeading = 270;
                    driveBackToShoot = false;
                    intake.block = true;
                    intake.poz = Intake.intakePoz.normalPoz;
                    intake.InTake = true;
                    ballCollectWait.reset();
                    maxWait.reset();
                }
                if (built && collectDone) {
                    visionCollect = false;
                    state = AutoState.driveToShootBack;
                    final sectionBuilder[] S1 = new sectionBuilder[] {
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(113, 333)),
                    };
                    paths.addNewPath("S1");
                    paths.buildPath(S1);
                    follow.setPath(paths.returnPath("S1"));
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


        if (pathing) {
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (visionCollect) {
            odometry.queueCommand(odometry.update);
            driveBase.queueCommand(driveBase.drivePowers(-y.calculate(targetPos, odometry.X()), headingPID.calculate(odometry.Heading() - 270), -x.calculate(0)));
        } else if (!PIDAtGate) {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }
    }
}