package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    pathsManager paths =new pathsManager(new RobotConfig(0.018, 0.004, 0.020, 0.005, 0.04, 0.004, 0.065, 0.004
            , 0.022, 0.0005, 0.012, 0.002, 200, 173, 80, 140));



    follower follow = new follower();
    PIDController headingPID = new PIDController(0.012,0,0.0030);
    PIDController forward = new PIDController(0.010,0,0.0030);


    private VisionPortal visionPortal;
    private LocalVision processor;

    enum AutoState{
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

    double lookAheadTime = 0;
    double shootWait = 1200;
    double velo = 8;
    double cycleTarget = 4;
    double cycle ;
    double turn;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();
    ElapsedTime gameTime = new ElapsedTime();


    private final sectionBuilder[] shoot = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(190, 330),  new Vector2D(190, 328)),
    };


    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(205, 330), new Vector2D(234, 254), new Vector2D(298, 268)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(317, 270), new Vector2D(256, 261), new Vector2D(228, 332)),

    };
    private final sectionBuilder[] collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(215, 310), new Vector2D(228, 190), new Vector2D(292, 210)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(300, 223), new Vector2D(290, 215), new Vector2D(298, 178)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325, 210), new Vector2D(243, 150)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(220, 155), new Vector2D(273, 146)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(320, 150), new Vector2D(243, 148)),
    };

    private final sectionBuilder[] firstBackCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(243, 148), new Vector2D(196, 293), new Vector2D(290, 330)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(210, 327)),
    };
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(210, 327)),
    };
    private final sectionBuilder[] movePath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(260, 300)),
    };
    private final sectionBuilder[] p1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(210, 317), new Vector2D(315, 350)),
    };
    private final sectionBuilder[] p2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(210, 317), new Vector2D(315, 317)),
    };
    private final sectionBuilder[] p3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(210, 317), new Vector2D(315, 284)),
    };
    @Override
    public void initEX() {
        odometry.startPosition(191, 346, 10);
        turret.Auto = true;
        driveBase.tele= false;
        follow.setHeadingOffset(90);
        turret.turrofset = -3;

        turret.targetX = 360;



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
        paths.buildPath(firstDriveToShootBack,100);
        paths.addNewPath("driveToShootBack");
        paths.buildPath(driveToShootBack, 100);
        paths.addNewPath("movePath");
        paths.buildPath(movePath);
        paths.addNewPath("p1");
        paths.buildPath(p1);
        paths.addNewPath("p2");
        paths.buildPath(p2);
        paths.addNewPath("p3");
        paths.buildPath(p3);

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


    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;
        if (!intake.InTake && intake.ballCount >2){
            intake.reverse = true;
        }
        if (!reset){
            gameTime.reset();
            reset = true;
        }

        if (intakeOff && intakeoff.milliseconds() > 1200 || intakeOff && intake.ballCount >2){
            intake.InTake = false;
            intakeOff = false;

        }
        if(intake.ballCount <1 && !ballShot){
            ballShot = true;
        }

        if (visionCollect){
            if (processor.hAngleDeg < 8){
                follow.setPath(paths.returnPath("p3"));
                pathing = true;
                targetHeading = 90;
            } else if (processor.hAngleDeg > -8) {
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                targetHeading = 90;

            }else {
                follow.setPath(paths.returnPath("p2"));
                pathing = true;
                targetHeading = 90;

            }
            if (follow.isFinished(5,5)){
                collectDone = true;
            }


            intake.block = true;
            intake.InTake = true;



        }
        switch (state) {
            case preLoad:
                if (!Preload){
                    preload.reset();
                    Preload = true;
                    follow.setPath(paths.returnPath("shoot"));
                    pathing = true;

                }
                if (built && preload.milliseconds() >1400 ){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }
                if (pathing && follow.isFinished(10,10)){
                    pathing = false;
                }
                if (!built && shootTime.milliseconds() > shootWait){
                    follow.setPath(paths.returnPath("collect1"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    pathing = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect1;
                }
                break;
            case collect1:
                if (pathing && odometry.X() > 295){
                    targetHeading = 90;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)){
                    state = AutoState.driveToShoot1;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    follow.usePathHeadings(true);
                    follow.setHeadingOffset(-90);
                    follow.setHeadingLookAheadDistance(100);
                }
                break;
            case driveToShoot1:
                if (built && follow.isFinished(10,10) && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot =false;
                }

                if (follow.isFinished(10,10) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo || follow.isFinished(10,10) && !built && ballShot && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo  ){
                    follow.setPath(paths.returnPath("firstBackCollect"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    collectDone = false;
                    state = AutoState.firstBackCollect;
                }
                break;
            case collect2:
                if (pathing && odometry.X() > 275){
                    targetHeading = 90;
                    follow.usePathHeadings(false);
                }

                if (pathing && follow.isFinished(10, 10)){
                    state = AutoState.gate;
                    follow.setPath(paths.returnPath("gate"));
                    follow.usePathHeadings(false);
                    intakeoff.reset();
                    intakeOff = true;
                    targetHeading = 90;
                    built = true;
                }
                break;
            case gate:
                if (follow.isFinished(8,8)){
                    state = AutoState.driveToShoot2;
                    follow.setPath(paths.returnPath("driveToShoot2"));
                    follow.setHeadingOffset(-90);


                }
                break;
            case driveToShoot2:
                if (built && follow.isFinished(15,15) &&  (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo/0.6){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                }

                if (follow.isFinished(15,15) && !built && shootTime.milliseconds() > shootWait*1.5 && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo || follow.isFinished(10,10) && !built && ballShot && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo  ){                    targetHeading = 90;
                    follow.setPath(paths.returnPath("collect3"));
                    follow.usePathHeadings(false);
                    pathing = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect3;
                }



                break;
            case collect3:
                if (pathing && odometry.X() > 295){
                    targetHeading = 90;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)){
                    state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    targetHeading = 90;
                }
                break;
            case driveToShoot3:
                if (built && follow.isFinished(15,15) && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    pathing = false;
                    shootTime.reset();
                    ballShot =false;
                }

                if (follow.isFinished(15,15) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo || follow.isFinished(10,10) && !built && ballShot && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo  ){                    follow.setPath(paths.returnPath("firstBackCollect"));
                    follow.setPath(paths.returnPath("firstBackCollect"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    state = AutoState.firstBackCollect;
                }
                break;
            case firstBackCollect:
                if (!built && follow.isFinished(5,5) ){
                    pathing = false;
                    built = true;
                    maxWait.reset();
                    forward.reset();
                    intake.block = true;
                    turret.stopTurret = true;
                    turret.mapOfset = 0;




                }


                if ( built && maxWait.milliseconds() > 1000 || built && collectDone ){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("firstDriveToShootBack"));
                    follow.usePathHeadings(false);
                    visionCollect = false;
                    intakeOff = false;
                    ballShot = false;
                    pathing = true;
                    turret.stopTurret = false;
                    built = false;
                }
                break;
            case driveToShootBack:
                if (intakeoff.milliseconds() > 400){
                    targetHeading = 5;

                }
                if (pathing && follow.isFinished(   10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< 4  ){
                    pathing = false;
                    shootTime.reset();
                    built = true;
                    intake.block = false;
                    intake.InTake = true;
                    turret.turrofset -= 0.1;

                }


                if (built && shootTime.milliseconds() > shootWait ){
                    state = AutoState.backCollect;
                    collectDone = false;
                    maxWait.reset();

                }


                break;
            case backCollect:

                if (built && maxWait.milliseconds() < 2200 || intake.ballCount>2 ){
                    visionCollect = true;
                }
                if (built && maxWait.milliseconds() > 2200 || intake.ballCount>2){
                    visionCollect = false;
                }
                if (gameTime.milliseconds() > 27200 && !pathing){
                    built = false;
                    visionCollect = false;
                    pathing = true;
                    follow.setPath(paths.returnPath("movePath"));

                }
                if (!built &&  follow.isFinished(5,5)){
                    pathing = false;
                    visionCollect = false;
                }

                if (built && maxWait.milliseconds() > 2200 ||  built && collectDone){
                    visionCollect = false;
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("driveToShootBack"));
                    ballShot = false;
                    targetHeading = 5;
                    intakeoff.reset();
                    intakeOff = true;


                    pathing = true;
                    built = false;
                }






                break;
            case finished:
                requestOpModeStop();



        }



        if (pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else {
            driveBase.queueCommand(driveBase.drivePowers(0,0,0));
        }
        telemetry.addData("block ",intake.block);
        telemetry.update();



        System.out.println(odometry.X());
        System.out.println(odometry.X());

    }
}