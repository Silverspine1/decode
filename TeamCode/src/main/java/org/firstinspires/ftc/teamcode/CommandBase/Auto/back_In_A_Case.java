package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

public class back_In_A_Case extends OpModeEX {
    pathsManager paths =new pathsManager(new RobotConfig(0.022, 0.006, 0.028, 0.007, 0.06, 0.005, 0.075, 0.005, 0.022, 0.0005, 0.012, 0.002, 173,200,260,260));



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
    boolean manuel = true;
    double lookAheadTime = 0;
    double shootWait = 1800;
    double velo = 2;
    double cycleTarget = 4;
    double cycle ;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();

    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(155, 330), new Vector2D(104, 261), new Vector2D(48, 270)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(48, 270), new Vector2D(104, 261), new Vector2D(155, 330)),
    };
    private final sectionBuilder[] collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(155, 330), new Vector2D(128, 195), new Vector2D(76.5, 210)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(76.5, 210), new Vector2D(44, 209), new Vector2D(35, 188)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(35, 188), new Vector2D(140, 155)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(140, 155), new Vector2D(40, 150)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(40, 150), new Vector2D(140, 150)),
    };
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(140, 150), new Vector2D(80, 300)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(134, 329)),
    };
    @Override
    public void initEX() {
        odometry.startPosition(155, 330, 0);
        turret.Auto = true;
        driveBase.tele= false;
        follow.setHeadingOffset(90);

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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        processor = new LocalVision(LocalVision.TargetColor.PURPLE);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Camera + settings BEFORE build()
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set lower resolution here
        builder.setCameraResolution(new Size(640, 480));   // or 640x480 if you want

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Optional: disable RC live view to save CPU
        builder.enableLiveView(false);

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
        if (intakeOff && intakeoff.milliseconds() > 1000){
            intake.InTake = false;
            intakeOff = false;

        }
        switch (state) {
            case preLoad:
                if (built){
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
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset();
                    intakeOff = true;
                    state = AutoState.driveToShoot1;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                }
            break;
            case driveToShoot1:
                if (built){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }

                if (follow.isFinished(10,10) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo){
                follow.setPath(paths.returnPath("collect2"));
                follow.usePathHeadings(true);
                follow.setHeadingLookAheadDistance(100);
                pathing = true;
                built = true;
                intake.block = true;
                state = AutoState.collect2;
            }
        break;
        case collect2:
            if (pathing && follow.isFinished(10, 10)){
                intakeoff.reset();
                intakeOff = true;
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

            }
        break;
            case driveToShoot2:
                if (built){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }

                if (follow.isFinished(10,10) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo) {
                    follow.setPath(paths.returnPath("collect3"));
                    pathing = true;
                    built = true;
                    targetHeading = 270;
                    intake.block = true;
                    state = AutoState.collect3;
                }



            break;
            case collect3:
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset();
                    intakeOff = true;
                    state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    targetHeading = 270;
                }
            break;
            case driveToShoot3:
                if (follow.isFinished(10,10) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo) {
                    follow.setPath(paths.returnPath("firstBackCollect"));
                    pathing = true;
                    built = true;
                    intake.block = true;
                    targetHeading = 250;
                    state = AutoState.firstBackCollect;
                }
            break;
            case firstBackCollect:
                if (pathing && follow.isFinished(10, 10)){
                    pathing = false;
                    built = true;
                    if (processor.hasTarget) {
                        intake.InTake =true;
                        intake.block= true;
                        driveBase.drivePowers(-gamepad1.right_stick_y + processor.distanceCm / 70, headingPID.calculate(-processor.hAngleDeg), -gamepad1.right_stick_x);
                        maxWait.reset();
                    }

                }
                if (built && maxWait.milliseconds() > 1500 || !(intake.upperBall == org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Intake.BallColor.NONE) && !(intake.lowerBall == org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Intake.BallColor.NONE)){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("driveToShootBack"));
                    targetHeading = 270;
                    pathing = true;
                    built = false;
                }
                break;
            case driveToShootBack:
                if (pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                    pathing = false;
                    shootTime.reset();
                    built = true;

                }
                if ( follow.isFinished(12,12)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
                    intake.block = false;
                }
                if (cycle > cycleTarget){
                    state = AutoState.finished;

                }
                if (built && shootTime.milliseconds() > shootWait){
                    state = AutoState.backCollect;
                }


                break;
            case backCollect:
                if (processor.hasTarget  ) {
                    driveBase.drivePowers(-gamepad1.right_stick_y + processor.distanceCm / 60, headingPID.calculate(-processor.hAngleDeg), -gamepad1.right_stick_x);
                    intake.block = true;
                }
                if (built && maxWait.milliseconds() > 1500 || !(intake.upperBall == org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Intake.BallColor.NONE) && !(intake.lowerBall == org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Intake.BallColor.NONE)){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("driveToShootBack"));
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
        System.out.println(odometry.X());
        System.out.println(odometry.X());

    }
}
