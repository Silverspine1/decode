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

@Autonomous(name = "back_In_A_Case_Hype_RED")
public class back_In_A_Case_Hype_auto_Red extends OpModeEX {
    pathsManager paths = new pathsManager(new RobotConfig(0.022, 0.006, 0.028, 0.007, 0.06, 0.005, 0.075, 0.005, 0.022, 0.0005, 0.012, 0.002, 200, 173, 120, 280));

    follower follow = new follower();
    PIDController headingPID = new PIDController(0.012, 0, 0.0030);

    private VisionPortal visionPortal;
    private LocalVision processor;

    enum AutoState {
        preLoad, collect1, driveToShoot1, collect2, driveToShoot2, collect3, gate, driveToShoot3, firstBackCollect, driveToShootBack, backCollect, finished
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

    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime intakeoff = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();
    ElapsedTime preload = new ElapsedTime();

    // MIRRORED PATHS (360 - X)
    private final sectionBuilder[] shoot = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(190, 330),  new Vector2D(190, 305)),
    };

    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(205, 330), new Vector2D(234, 254), new Vector2D(302, 268)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(317, 270), new Vector2D(256, 261), new Vector2D(230, 300)),
    };
    private final sectionBuilder[] collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(215, 310), new Vector2D(228, 190), new Vector2D(305, 220)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(275, 223), new Vector2D(255, 215), new Vector2D(305, 170)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325, 210), new Vector2D(220, 165)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(220, 155), new Vector2D(298, 148)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(320, 150), new Vector2D(206, 130)),
    };
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(220, 150), new Vector2D(250, 300)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(308, 329), new Vector2D(218, 329)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(191, 346, 10);
        turret.Auto = true;
        driveBase.tele = false;
        follow.setHeadingOffset(90);
        turret.targetX = 360;


        paths.addNewPath("shoot"); paths.buildPath(shoot);
        paths.addNewPath("collect1"); paths.buildPath(collect1);
        paths.addNewPath("driveToShoot1"); paths.buildPath(driveToShoot1);
        paths.addNewPath("collect2"); paths.buildPath(collect2);
        paths.addNewPath("driveToShoot2"); paths.buildPath(driveToShoot2);
        paths.addNewPath("collect3"); paths.buildPath(collect3);
        paths.addNewPath("gate"); paths.buildPath(gate);
        paths.addNewPath("driveToShoot3"); paths.buildPath(driveToShoot3);
        paths.addNewPath("firstBackCollect"); paths.buildPath(firstBackCollect);
        paths.addNewPath("driveToShootBack"); paths.buildPath(driveToShootBack);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        processor = new LocalVision(LocalVision.TargetColor.PURPLE);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(processor);
        visionPortal = builder.build();
    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;

        if (visionCollect){
            driveBase.drivePowers(Math.pow(processor.distanceCm / 65, 2), headingPID.calculate(-processor.hAngleDeg), 0);
            intake.block = true; intake.InTake = true;
        }

        switch (state) {
            case preLoad:
                if (!Preload){ preload.reset(); Preload = true; follow.setPath(paths.returnPath("shoot")); pathing = true; }
                if (built && preload.milliseconds() > 2200){ intake.InTake = true; built = false; intake.block = false; shootTime.reset(); }
                if (!built && shootTime.milliseconds() > shootWait){
                    follow.setPath(paths.returnPath("collect1"));
                    follow.usePathHeadings(true);
                    pathing = true; built = true; intake.block = true; state = AutoState.collect1;
                }
                break;

            case collect1:
                if (pathing && odometry.X() > 295){
                    targetHeading = 90;
                    follow.usePathHeadings(false);
                }
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset(); intakeOff = true; state = AutoState.driveToShoot1;
                    follow.setPath(paths.returnPath("driveToShoot1"));
                    follow.usePathHeadings(true);
                    follow.setHeadingOffset(-90);
                }
                break;

            case driveToShoot1:
                if (follow.isFinished(10,10) && !built && (shootTime.milliseconds() > shootWait || ballShot) && (Math.abs(odometry.getXVelocity())+ Math.abs(odometry.getYVelocity())) < velo){
                    follow.setPath(paths.returnPath("collect2"));
                    follow.usePathHeadings(true);
                    follow.setHeadingOffset(90);
                    pathing = true; intake.InTake = true; built = true; intake.block = true; state = AutoState.collect2;
                }
                break;

            case collect2:
                if (pathing && odometry.X() > 295){ targetHeading = 90; follow.usePathHeadings(false); }
                if (pathing && follow.isFinished(10, 10)){
                    state = AutoState.gate; follow.setPath(paths.returnPath("gate"));
                    targetHeading = 90; built = true;
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
                if (follow.isFinished(15,15) && !built && (shootTime.milliseconds() > shootWait || ballShot) && (Math.abs(odometry.getXVelocity())) < velo){
                    targetHeading = 90;
                    follow.setPath(paths.returnPath("collect3"));
                    pathing = true; built = true; intake.block = true; state = AutoState.collect3;
                }
                break;

            case collect3:
                if (pathing && odometry.X() > 295){ targetHeading = 90; follow.usePathHeadings(false); }
                if (pathing && follow.isFinished(10, 10)){
                    intakeoff.reset(); intakeOff = true; state = AutoState.driveToShoot3;
                    follow.setPath(paths.returnPath("driveToShoot3"));
                    targetHeading = 90;
                }
                break;

            case driveToShoot3:
                if (follow.isFinished(15,15) && !built && (shootTime.milliseconds() > shootWait || ballShot)){
                    follow.setPath(paths.returnPath("firstBackCollect"));
                    targetHeading = 140;
                    follow.setHeadingOffset(90);
                    pathing = true; built = false; state = AutoState.firstBackCollect;
                }
                break;

            case firstBackCollect:
                if (built && (maxWait.milliseconds() > 2200 || intake.ballCount > 2)){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("driveToShootBack"));
                    visionCollect = false;
                    targetHeading = 80;
                    pathing = true; built = false;
                }
                break;

            case driveToShootBack:
                if (built && shootTime.milliseconds() > shootWait){
                    state = AutoState.backCollect; maxWait.reset();
                }
                break;

            case backCollect:
                if (built && (maxWait.milliseconds() > 2200 || intake.ballCount > 2)){
                    state = AutoState.driveToShootBack;
                    follow.setPath(paths.returnPath("driveToShootBack"));
                    targetHeading = 80;
                    pathing = true; built = false;
                }
                break;

            case finished:
                requestOpModeStop();
        }

        if (pathing){
            odometry.queueCommand(odometry.update);
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        } else if (!visionCollect){
            driveBase.queueCommand(driveBase.drivePowers(0,0,0));
        }
    }
}
