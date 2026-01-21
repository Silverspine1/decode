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

public class back_In_A_Case extends OpModeEX {
    pathsManager paths =new pathsManager(new RobotConfig(0.022, 0.006, 0.028, 0.007, 0.06, 0.005, 0.075, 0.005, 0.022, 0.0005, 0.012, 0.002, 200, 173, 80, 140));



    follower follow = new follower();
    PIDController headingPID = new PIDController(0.012,0,0.0030);


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
    double lookAheadTime = 0;
    double shootWait = 1500;
    double velo = 6;
    double cycleTarget = 4;
    double cycle ;
    double turn;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();
    ElapsedTime ballShotTimer = new ElapsedTime();

    private final sectionBuilder[] shoot = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(170, 330),  new Vector2D(170, 305)),
    };


    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(155, 330), new Vector2D(126, 254), new Vector2D(58, 268)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(43, 270), new Vector2D(104, 261), new Vector2D(142, 310)),
    };
    private final sectionBuilder[] collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(145, 310), new Vector2D(132, 190), new Vector2D(55, 220)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(85, 223), new Vector2D(105, 215), new Vector2D(55, 178)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(35, 210), new Vector2D(140, 165)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(140, 155), new Vector2D(70, 148)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(40, 150), new Vector2D(140, 130)),
    };
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(140, 150), new Vector2D(110, 300)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(136, 329)),
    };
    @Override
    public void initEX() {
        odometry.startPosition(169, 346, 350);
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
        paths.addNewPath("driveToShootBack");
        paths.buildPath(driveToShootBack);

        Apriltag.limelight.pipelineSwitch(0);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

// Create TWO processors - one for each color
        processor = new LocalVision(LocalVision.TargetColor.PURPLE);
        LocalVision greenProcessor = new LocalVision(LocalVision.TargetColor.GREEN);

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
        builder.addProcessor(greenProcessor);

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

        if (intakeOff && intakeoff.milliseconds() > 850 || intakeOff && intake.ballCount >2){
            intake.InTake = false;
            intakeOff = false;

        }
        if(intake.ballCount <1 && !ballShot){
            ballShot = true;
        }

        if (visionCollect){
            if (headingPID.calculate(-processor.hAngleDeg) > 0){
                turn = headingPID.calculate(-processor.hAngleDeg);
            }else {
                turn = 0;
            }

            driveBase.drivePowers(0.15 + Math.pow(processor.distanceCm / 65,2), turn, 0);
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
                if (built && preload.milliseconds() >2200){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
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
                if (pathing && odometry.X() < 65){
                    targetHeading = 270;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset();
                    intakeOff = true;
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
            if (pathing && odometry.X() < 65){
                targetHeading = 270;
                follow.usePathHeadings(false);
            }

            if (pathing && follow.isFinished(10, 10)){
                state = AutoState.gate;
                follow.setPath(paths.returnPath("gate"));
                follow.usePathHeadings(false);
                targetHeading = 270;
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

                if (follow.isFinished(15,15) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo || follow.isFinished(10,10) && !built && ballShot && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo  ){                    targetHeading = 270;
                    follow.setPath(paths.returnPath("collect3"));
                    follow.usePathHeadings(false);
                    pathing = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect3;
                }



            break;
            case collect3:
                if (pathing && odometry.X() < 65){
                    targetHeading = 270;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset();
                    intakeOff = true;
                    state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    targetHeading = 270;
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
                    targetHeading = 220;
                    follow.setHeadingOffset(90);
                    pathing = true;
                    built = false;
                    state = AutoState.firstBackCollect;
                }
            break;
            case firstBackCollect:
                if (!built && follow.isFinished(15,15)){
                    pathing = false;
                    built = true;
                    maxWait.reset();


                }
                if (built && maxWait.milliseconds() < 2200 || intake.ballCount>2){
                    visionCollect = true;
                    intake.block = true;

                }
                if (built && maxWait.milliseconds() > 2200|| intake.ballCount>2 ){
                    visionCollect = false;
                }
                //|| intake.ballCount>2


                if ( built && maxWait.milliseconds() > 2200 || intake.ballCount>2 && built ){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("driveToShootBack"));
                    follow.usePathHeadings(false);
                    visionCollect = false;
                    targetHeading = 280;
                    ballShot = false;
                    pathing = true;
                    built = false;
                }
                break;
            case driveToShootBack:
                if (pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                    pathing = false;
                    shootTime.reset();
                    built = true;
                    intake.block = false;
                    intake.InTake = true;

                }


                if (built && shootTime.milliseconds() > shootWait ){
                    state = AutoState.backCollect;
                    maxWait.reset();

                }


                break;
            case backCollect:
                if (built && maxWait.milliseconds() < 2200 || intake.ballCount>2){
                    visionCollect = true;
                }
                if (built && maxWait.milliseconds() > 2200 || intake.ballCount>2){
                    visionCollect = false;
                }

                if (built && maxWait.milliseconds() > 2200 || intake.ballCount>2 && built ){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("driveToShootBack"));
                    ballShot = false;
                    targetHeading = 280;


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
        }else if (!visionCollect){
            driveBase.queueCommand(driveBase.drivePowers(0,0,0));
        }
        telemetry.addData("block ",intake.block);
        telemetry.update();



        System.out.println(odometry.X());
        System.out.println(odometry.X());

    }
}
