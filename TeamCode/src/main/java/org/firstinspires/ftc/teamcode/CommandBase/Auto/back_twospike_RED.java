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

@Autonomous

public class back_twospike_RED extends OpModeEX {
    pathsManager paths = new pathsManager(new RobotConfig(
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2, 0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 800));

    follower follow = new follower(new RobotConfig(
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2, 0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 800));

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

    double frontOffset     = 8.0;   // distance from robot center to intake face (along -X at heading 0/360)
    double sideOffsetDeg   = 0.0;   // sideways camera bias in degrees (+ = right of camera center)
    double robotHalfWidth  = 17.5;  // half the robot's width parallel to the wall
    double wallSafetyMargin = 10.0;  // extra clearance beyond robotHalfWidth
    double wallBuffer       = 20.0; // field units from safe limit where avoidance activates
    double maxWallAvoidPower = 0.40; // max Y power applied for wall avoidance
    double wallVelGain      = 0.018; // velocity feedforward gain (predictive braking)
    double wallLookAheadSecs = 0.12; // seconds ahead to predict wall approach

    final double WALL_Y = 270; // field Y of the wall

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
    boolean HoldHeadingWhileShooting = false;
    boolean waitAtEnd = false;

    double backCycles = 0;
    double shootWait = 700;
    double velo = 16;
    double gateTolX = 10; double gateTolY = 8; double gateTurnX = 286;double gateAngle = 65; double gateTime = 1120;
    double targetPos = 44;
    boolean stage1Done = false;
    double collectAngle = 90;

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


    private final sectionBuilder[] shoot = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(190, 330), new Vector2D(190, 305)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(287, 273), new Vector2D(220, 296)),
    };
    private final sectionBuilder[] collect2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(212, 305), new Vector2D(238, 230), new Vector2D(301, 210)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(207, 150), new Vector2D(305, 205.5)),
    };
    private final sectionBuilder[] gateFromBack = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(222, 325), new Vector2D(303, 221)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(313, 206), new Vector2D(207, 150)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[] {
            () -> paths.addPoints(new Vector2D(220, 150), new Vector2D(290, 146)),
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
            () -> paths.addPoints(new Vector2D(320, 208), new Vector2D(230, 332)),
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
        follow.setHeadingOffset(-90);
        intake.block   = true;
        state          = AutoState.gate;
    }

    @Override
    public void initEX() {
        odometry.startPosition(192.5, 342, 0);
        odometry.odo.setHeading(270, AngleUnit.DEGREES);

        turret.Auto = true;
        turret.targetX = 0;
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

        if (visionCollect){
            if (processor.hAngleDeg < -16 && !intakePathSelected){
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
                maxWait.reset();

            } else if (processor.hAngleDeg > -1  && !intakePathSelected) {
                final sectionBuilder[] p1 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(220, 340), new Vector2D(310, 340)),
                };
                paths.addNewPath("p1");
                paths.buildPath(p1);
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                intakePathSelected = true;
                follow.setHeadingOffset(90);
                follow.usePathHeadings(true);
                intake.block = true;
                intake.InTake = true;
                maxWait.reset();


            }else if(!intakePathSelected && processor.hAngleDeg < -1 && processor.hAngleDeg > -16 ) {
                final sectionBuilder[] p2 = new sectionBuilder[]{
                        () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(310, 311)),
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
            if (follow.isFinished(5,10)  || maxWait.milliseconds() > 1400 ){
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
                    turret.TURRET_COMP_FACTOR = 0;
                    turret.mapOfset = -50;
                    turret.turrofset = -3;


                    targetHeading = 90;
                }
                if (built && turret.diff < 100 && turret.rpm > 1000){
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
                if (!built && shootTime.milliseconds() > 450) {
                    final sectionBuilder[] collect1 = new sectionBuilder[] {
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(286, 267)),
                    };
                    paths.addNewPath("collect1");
                    paths.buildPath(collect1);
                    follow.setPath(paths.returnPath("collect1"));
                    turret.TURRET_COMP_FACTOR = 0.95;
                    turret.mapOfset = 20;
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
                        + Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity())) < 20) {
                    intake.InTake = true;
                    built = false;
                    pathing = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }
                if (!built && shootTime.milliseconds() > 490
                        || !built && ballShot) {
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(160);
                    follow.setHeadingOffset(90);
                    turret.turrofset = 3.5;
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
                if (follow.isFinished(15, 15) && !built && shootTime.milliseconds() > 510) {
                    follow.setPath(paths.returnPath("gate"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    state = AutoState.gate;
                    turret.turrofset = -3;
                    turret.mapOfset = -35;

                }
                break;

            case gate:
                if (odometry.X() > gateTurnX) {
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

                if (built && gateInTakeTime.milliseconds() > gateTime || built && ballsInIntake && ballCollectWait.milliseconds() > 180 && gateInTakeTime.milliseconds() > 300) {
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
                if (afterGateCollect && odometry.Y() > 330) {
                    intake.InTake = false;
                }
                if (afterGateCollect && odometry.Y() > 280) {
                    targetHeading = 82;
                }
                if (odometry.X() < 278 && intake.poz == Intake.intakePoz.normalPoz && shootTime.milliseconds() > 500){
                    intake.poz = Intake.intakePoz.up;
                    intake.InTake = false;
                    intake.holdUp = true;
                }
                if (follow.isFinished(15, 30) && Math.abs(odometry.getXVelocity() + odometry.getYVelocity())
                        + Math.abs(odometry.getHVelocity() * 2) < 50) {
                    pathing = false;
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - 90), 0);
                    HoldHeadingWhileShooting = true;

                }
                if (follow.isFinished(15, 30) && odometry.X() < 250 && !built
                        && Math.abs(odometry.getXVelocity() + odometry.getYVelocity()) + Math.abs(odometry.getHVelocity() * 2) < 32
                        && !dontWaitForPoz) {
                    shootWait = 420;
                    shootTime.reset();
                    follow.usePathHeadings(false);
                    pathing = false;
                    driveBase.drivePowers(0, headingPID.calculate(odometry.Heading() - 90), 0);
                    HoldHeadingWhileShooting = true;

                    gateTime = 1150;
                    backCycles += 1;
                    dontWaitForPoz = false;
                    built = true;
                    pathing = false;
                    afterGateCollect = false;
                    intake.block = false;
                    intake.InTake = true;
                    IntakeOffWait = 200;
                }
                if  (built && shootTime.milliseconds() > 120){
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
                // alreadyFailed recovery — redirect to gate on cycle 2, else backCollect
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
                        gateTurnX = 286;
                        enterGateFromBack();
                        gateAngle  = 55;

                    } else {
                        state = AutoState.backCollect;
                    }
                }
                break;

            case backCollect:
                if (built && !collectDone ){
                    if (!p3Qued){
                        visionCollect = true;
                    }else {
                        final sectionBuilder[] p2 = new sectionBuilder[]{
                                () -> paths.addPoints(new Vector2D(245, 317), new Vector2D(305, 311)),
                        };
                        paths.addNewPath("p2");
                        paths.buildPath(p2);
                        follow.setPath(paths.returnPath("p2"));
                        follow.setHeadingOffset(90);
                        follow.usePathHeadings(true);
                        if (!pathing){
                            maxWait.reset();
                        }
                        pathing = true;
                        intakePathSelected = true;
                        visionCollect = true;
                        p3Qued = false;


                    }
                }




                if (built && collectDone ){
                    visionCollect = false;
                    state = AutoState.driveToShootBack;
                    final sectionBuilder[] S1 = new sectionBuilder[] {
                            () -> paths.addPoints(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(247, 333)),
                    };
                    paths.addNewPath("S1");
                    paths.buildPath(S1);
                    follow.setPath(paths.returnPath("S1"));
                    follow.setHeadingOffset(-90);


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
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else if (!PIDAtGate && !HoldHeadingWhileShooting) {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }
    }
}
