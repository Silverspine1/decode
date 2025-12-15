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
@Autonomous(name = "back_In_A_Case_RED") // Renamed for clarity on Driver Station

public class back_In_A_Case_Red extends OpModeEX {
    // RobotConfig should be the same
    pathsManager paths =new pathsManager(new RobotConfig(0.020, 0.005, 0.024, 0.005, 0.05, 0.004, 0.07, 0.004, 0.020, 0.0004, 0.01, 0.0015, 220,200,250,430));

    follower follow = new follower();
    PIDController headingPID = new PIDController(0.012,0,0.0030);


    private VisionPortal visionPortal;
    private LocalVision processor;

    // State machine logic remains the same
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
    double lookAheadTime = 0;
    double shootWait = 1000;
    double velo = 2;
    double cycleTarget = 4;
    double cycle ;

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();


    // --- MIRRORED PATHS FOR RED ALLIANCE (x -> 360 - x) ---

    // Blue Path: (155, 330), (120, 258), (33, 270)
    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(205, 330), new Vector2D(240, 258), new Vector2D(327, 274)),
    };
    // Blue Path: (33, 270), (104, 261), (145, 310)
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(327, 270), new Vector2D(256, 261), new Vector2D(215, 310)),
    };
    // Blue Path: (145, 310), (128, 195), (26, 212)
    private final sectionBuilder[] collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(215, 310), new Vector2D(232, 195), new Vector2D(334, 216)),
    };
    // Blue Path: (70, 210), (100, 215), (35, 180)
    private final sectionBuilder[] gate = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(290, 210), new Vector2D(260, 215), new Vector2D(325, 180)),
    };
    // Blue Path: (35, 210), (140, 165)
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325, 210), new Vector2D(220, 165)),
    };
    // Blue Path: (140, 155), (30, 150)
    private final sectionBuilder[] collect3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(220, 155), new Vector2D(330, 154)),
    };
    // Blue Path: (40, 150), (140, 150)
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(320, 150), new Vector2D(220, 150)),
    };
    // Blue Path: (140, 150), (80, 300)
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(220, 150), new Vector2D(280, 300)),
    };
    // Blue Path: (52, 329), (134, 329)
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(226, 329)),
    };
    @Override
    public void initEX() {
        // Mirrored start position: x -> 360-x. HEADING REMAINS THE SAME.
        odometry.startPosition(360 - 163, 344, 0); // Corrected heading to 0
        turret.Auto = true;
        driveBase.tele= false;
        turret.targetX = 360;

        // HEADING OFFSET REMAINS THE SAME as the blue auto.
        follow.setHeadingOffset(90);

        // Path building logic remains the same
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

        // Vision and dashboard setup remains the same
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        processor = new LocalVision(LocalVision.TargetColor.PURPLE);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(false);
        builder.addProcessor(processor);
        visionPortal = builder.build();
        dashboard.startCameraStream(visionPortal, 15);
    }

    @Override
    public void loopEX() {
        // Turret and intake timer logic remains the same
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;
        if (intakeOff && intakeoff.milliseconds() > 850){
            intake.InTake = false;
            intakeOff = false;
        }

        // The state machine flow remains the same.
        // Target Headings remain the same as blue side because the world coordinate system doesn't change.
        switch (state) {
            case preLoad:
                if (!Preload){
                    preload.reset();
                    Preload = true;
                }
                if (built && preload.milliseconds() >2900 && turret.diff < 140){
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
                if (pathing && follow.isFinished(15,15)){
                    targetHeading = 270; // Remains 270
                }
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset();
                    intakeOff = true;
                    state = AutoState.driveToShoot1;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    follow.usePathHeadings(true);
                    follow.setHeadingOffset(-90); // Remains -90
                    follow.setHeadingLookAheadDistance(100);
                }
                break;
            case driveToShoot1:
                if (built && follow.isFinished(10,10) && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }

                if (follow.isFinished(10,10) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo){
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingLookAheadDistance(100);
                    follow.setHeadingOffset(90); // Remains 90
                    pathing = true;
                    intake.InTake = true;
                    built = true;
                    intake.block = true;
                    state = AutoState.collect2;
                }
                break;
            case collect2:
                if (pathing && follow.isFinished(15,15)){
                    targetHeading = 90;
                }

                if (pathing && follow.isFinished(10, 10)){
                    state = AutoState.driveToShoot2;
                    follow.setPath(paths.returnPath("driveToShoot2"));
                    follow.usePathHeadings(false);
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
                if (built && follow.isFinished(10,10) && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }

                if (follow.isFinished(10,10) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo) {
                    follow.setPath(paths.returnPath("collect3"));
                    follow.usePathHeadings(false);
                    pathing = true;
                    built = true;
                    targetHeading = 90;
                    intake.block = true;
                    state = AutoState.collect3;
                }
                break;
            case collect3:
                if (pathing && follow.isFinished(15,15)){
                    targetHeading = 90;
                }
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset();
                    intakeOff = true;
                    state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    targetHeading = 90;
                }
                break;
            case driveToShoot3:
                if (built && follow.isFinished(10,10) && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo){
                    intake.InTake = true;
                    built = false;
                    intake.block = false;
                    shootTime.reset();
                }

                if (follow.isFinished(10,10) && !built && shootTime.milliseconds() > shootWait && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity()) + Math.abs(odometry.getHVelocity()))< velo) {
                    follow.setPath(paths.returnPath("collect3"));
                    follow.usePathHeadings(false);
                    pathing = true;
                    built = true;
                    targetHeading = 90;
                    intake.block = true;
                    state = AutoState.finished;
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
                    targetHeading = 270; // Remains 270
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

        // Path following logic remains the same
        if (pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else {
            driveBase.queueCommand(driveBase.drivePowers(0,0,0));
        }

        telemetry.addData("X", odometry.X());
        telemetry.addData("Y", odometry.Y());
        telemetry.update();
    }
}