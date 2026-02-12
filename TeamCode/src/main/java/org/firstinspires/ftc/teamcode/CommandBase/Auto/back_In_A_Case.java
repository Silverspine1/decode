package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    pathsManager paths =new pathsManager(new RobotConfig(0.018, 0.004, 0.020, 0.005, 0.04, 0.004, 0.065, 0.004
            , 0.022, 0.0005, 0.012, 0.002, 200, 273, 270, 320));



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
    enum shootPath{
        S1,
        S2,
        S3
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

    double lookAheadTime = 0;
    double shootWait = 700;
    double velo = 18;



    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime ballshot = new ElapsedTime();



    private final sectionBuilder[] shoot = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(170, 330),  new Vector2D(170, 328)),
    };


    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(155, 330), new Vector2D(126, 254), new Vector2D(73, 268)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(43, 270), new Vector2D(104, 261), new Vector2D(132, 332)),

    };
    private final sectionBuilder[] collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(145, 310), new Vector2D(132, 190), new Vector2D(68, 210)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(60, 223), new Vector2D(70, 215), new Vector2D(56, 178)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(35, 210), new Vector2D(127, 150)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(130, 155), new Vector2D(73, 146)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(40, 150), new Vector2D(127, 148)),
    };

    private final sectionBuilder[] firstBackCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(117, 148), new Vector2D(164, 293), new Vector2D(82, 315)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(150, 327)),
    };
    private final sectionBuilder[] firstDriveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(82, 314), new Vector2D(130, 317)),
    };
    private final sectionBuilder[] movePath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(100, 300)),
    };
    private final sectionBuilder[] p1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(130, 317), new Vector2D(60, 340)),
    };
    private final sectionBuilder[] p2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(130, 317), new Vector2D(60, 317)),
    };
    private final sectionBuilder[] p3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(130, 317), new Vector2D(60, 284)),
    };
    private final sectionBuilder[] S1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(56, 340), new Vector2D(130, 320)),
    };
    private final sectionBuilder[] S2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(56, 320), new Vector2D(130, 320)),
    };
    private final sectionBuilder[] S3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(56, 284), new Vector2D(130, 320)),
    };
    @Override
    public void initEX() {
        odometry.startPosition(169, 346, 270);
        turret.Auto = true;
        driveBase.tele= false;
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
        paths.addNewPath("S1");
        paths.buildPath(S1);
        paths.addNewPath("S2");
        paths.buildPath(S2);
        paths.addNewPath("S3");
        paths.buildPath(S3);


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

        if (intakeOff && intake.ballCount >2 && intakeoff.milliseconds() >200){
            intake.InTake = false;
            intakeOff = false;

        }
        if(intake.ballCount <1 && !ballShot){
            ballShot = true;
            ballshot.reset();
        }

        if (visionCollect){
            if (processor.hAngleDeg >8 && !intakePathSelected){
                follow.setPath(paths.returnPath("p3"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S3;
                targetHeading = 270;
                maxWait.reset();

            } else if (processor.hAngleDeg <-8 && !intakePathSelected) {
                follow.setPath(paths.returnPath("p1"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S1;
                targetHeading = 270;
                maxWait.reset();


            }else if(!intakePathSelected) {
                follow.setPath(paths.returnPath("p2"));
                pathing = true;
                intakePathSelected = true;
                shootState = shootPath.S2;
                targetHeading = 270;
                maxWait.reset();


            }
            if (follow.isFinished(5,10) || maxWait.milliseconds() > 1000){
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
                    driveBase.speed = 1;

                }
                if (built && preload.milliseconds() >1400|| built && turret.diff < 100 && turret.rpm > 1200 ){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }
                if (pathing && follow.isFinished(10,10)){
                    pathing = false;
                }
                if (!built && shootTime.milliseconds() > 550){
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
                if (pathing && odometry.X() < 93){
                    targetHeading = 270;
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
                if (built && follow.isFinished(10,10) && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo ){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot =false;
                }

                if ( !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo || !built && ballShot && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo  ){
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.InTake = true;

                    built = true;
                    intake.block = true;
                    state = AutoState.collect2;
                }
                break;
            case collect2:
                if (pathing && odometry.X() < 85){
                    targetHeading = 270;
                    follow.usePathHeadings(false);
                }

                if (pathing && follow.isFinished(10, 10)){
                    state = AutoState.gate;
                    follow.setPath(paths.returnPath("gate"));
                    follow.usePathHeadings(false);
                    intakeoff.reset();
                    intakeOff = true;
                    targetHeading = 270;
                    built = true;
                }
                break;
            case gate:
                if (follow.isFinished(8,8)){
                    state = AutoState.driveToShoot2;
                    follow.setPath(paths.returnPath("driveToShoot2"));
                    turret.turrofset = -8;
                    turret.mapOfset = 50;
                    follow.setHeadingOffset(-90);
                }
                break;
            case driveToShoot2:
                shootWait = 1600;
                if (built && follow.isFinished(15,15) && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo || built && odometry.X() > 127){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot = false;
                    turret.turrofset = -10;


                }

                if (follow.isFinished(15,15) && !built && shootTime.milliseconds() > shootWait*1.5 || follow.isFinished(10,10) && !built && ballShot && ballshot.milliseconds() > 150 ){
                    targetHeading = 270;
                    follow.setPath(paths.returnPath("collect3"));
                    follow.usePathHeadings(false);
                    pathing = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect3;
                    driveBase.speed = 1;
                    turret.turrofset = -10;

                }



                break;
            case collect3:
                if (pathing && odometry.X() < 65){
                    targetHeading = 270;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)){
                    state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    driveBase.speed = 1;

                    targetHeading = 270;
                }
                break;
            case driveToShoot3:
                if (built && follow.isFinished(10,10) && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo || built && odometry.X() > 127){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                    ballShot =false;
                }

                if (!built && shootTime.milliseconds() > 800    ){
                    follow.setPath(paths.returnPath("firstBackCollect"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(120);
                    turret.stopTurret = true;
                    follow.setHeadingOffset(90);
                    pathing = true;
                    intake.block = true;
                    built = false;
                    state = AutoState.firstBackCollect;
                    turret.turrofset = 0;
                    turret.mapOfset = 0;
                }
                break;
            case firstBackCollect:
                if (!built && follow.isFinished(15,15) ){
                    pathing = false;
                    built = true;
                    maxWait.reset();
                    intake.block = true;
                    turret.stopTurret = true;
                    turret.mapOfset = 0;




                }


                if (built && maxWait.milliseconds() > 1000 ){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("firstDriveToShootBack"));
                    follow.usePathHeadings(false);
                    visionCollect = false;
                    intakeoff.reset();
                    intakeOff = true;
                    ballShot = false;
                    pathing = true;
                    turret.stopTurret = false;
                    built = false;
                }
                break;
            case driveToShootBack:
                if (!built && pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< 15){
                    shootTime.reset();
                    shootWait = 600;
                    built = true;
                    pathing = false;
                    intake.block = false;
                    intake.InTake = true;

                }


                if (built && shootTime.milliseconds() > shootWait ){
                    state = AutoState.backCollect;
                    driveBase.speed = 0.9;
                    collectDone = false;
                    maxWait.reset();

                }


                break;
            case backCollect:

                if (built && !collectDone ){
                    visionCollect = true;
                }
                if (built && collectDone){
                    visionCollect = false;
                }
                if (gameTime.milliseconds() > 27600 && !pathing){
                    built = false;
                    visionCollect = false;
                    pathing = true;
                    intake.block = true;
                    follow.setPath(paths.returnPath("movePath"));

                }
                if (!built &&  follow.isFinished(5,5)){
                    pathing = false;
                    visionCollect = false;
                }

                if (built && collectDone ){
                    visionCollect = false;
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath(shootState.name()));
                    intakePathSelected = false;
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
