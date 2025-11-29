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
@Autonomous
public class close_In_A_Case extends OpModeEX {
    pathsManager paths =new pathsManager(new RobotConfig(0.022, 0.006, 0.028, 0.007, 0.06, 0.005, 0.075, 0.005, 0.022, 0.0005, 0.012, 0.002, 173,200,260,260));



    follower follow = new follower();
    PIDController headingPID = new PIDController(0.012,0,0.0030);


    private VisionPortal visionPortal;
    private LocalVision processor;


    double targetHeading;
    boolean pathing = false;
    boolean built = true;
    boolean Intake = false;
    boolean manuel = true;
    double lookAheadTime = 0;
    double shootWait = 1800 ;
    double velo = 2;
    double cycleTarget = 4;
    double cycle ;




    ElapsedTime shootTime = new ElapsedTime();
    ElapsedTime maxWait = new ElapsedTime();


    enum AutoState {
        preload,
        driveToCollect,
        firstShootDone,
        collect1,
        driveToShoot1,
        Collect2,
        driveToShoot2,
        collect3,
        gate,
        driveToShoot3,
        firstBackCollect,
        driveToShootBack,
        backCollect,

        finished
    }

    AutoState state = AutoState.preload;

    private final sectionBuilder[] preload = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(84, 28), new Vector2D(107, 75), new Vector2D(135, 128 )),
    };

    private final sectionBuilder[] collect1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(135, 128), new Vector2D(95, 157), new Vector2D(40, 151)),
    };
    private final sectionBuilder[] driveToShoot1 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(40, 151), new Vector2D(99, 152), new Vector2D(145, 150)),
    };
    private final sectionBuilder[] Collect2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(150 , 150), new Vector2D(117, 215), new Vector2D(36, 205)),
    };
    private final sectionBuilder[] driveToShoot2 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(33, 211), new Vector2D(160, 150)),
    };
    private final sectionBuilder[] collect3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(160, 180), new Vector2D(148, 270), new Vector2D(30, 265)),
    };
    private final sectionBuilder[] gate = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(40, 270), new Vector2D(110, 200), new Vector2D(20, 181)),
    };
    private final sectionBuilder[] driveToShoot3 = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(20, 181), new Vector2D(160, 181)),
    };
    private final sectionBuilder[] firstBackCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(160, 180), new Vector2D(137, 263), new Vector2D(105, 290)),
    };
    private final sectionBuilder[] driveToShootBack = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(52, 329), new Vector2D(134, 329)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(75, 22, 0);
        turret.Auto = true;
        driveBase.tele= false;
        follow.setHeadingOffset(90);


        paths.addNewPath("preload");
        paths.buildPath(preload);
        paths.addNewPath("collect1");
        paths.buildPath(collect1);
        paths.addNewPath("driveToShoot1");
        paths.buildPath(driveToShoot1);
        paths.addNewPath("Collect2");
        paths.buildPath(Collect2);
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
    @Override public void stop() {
        if (visionPortal != null) visionPortal.close();
    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X() - odometry.getXVelocity() * lookAheadTime;
        turret.robotY = odometry.Y() - odometry.getYVelocity() * lookAheadTime;
        turret.robotHeading = odometry.normilised;

        switch (state) {
            case preload:
                if (built ) {
                    follow.setPath(paths.returnPath("preload"));
                    targetHeading = 310;
                    built = false;
                    pathing = true;


                }
                if (pathing && follow.isFinished(8, 8) && Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo) {
                    built = true;
                    pathing = false;
                    intake.block = false;
                    state = AutoState.firstShootDone;
                    intake.InTake =true;
                    shootTime.reset();


                }
                break;
                case firstShootDone:
                    if (built  && shootTime.milliseconds() > shootWait) {
                        built = false;
                        state = AutoState.collect1;
                        follow.setPath(paths.returnPath("collect1"));
                        intake.block = true;
                        targetHeading = 270;
                        state = AutoState.collect1;
                        pathing = true;

                    }
                    break;
                    case collect1:
                        if (pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
                            state = AutoState.driveToShoot1;
                            follow.setPath(paths.returnPath("driveToShoot1"));
                            targetHeading = 270;
                            pathing = true;
                            state = AutoState.driveToShoot1;
                            built = false;

                        }
                        break;
                        case driveToShoot1:
                            if (pathing && follow.isFinished(10, 10)) {
                                pathing = false;
                                built = true;
                                intake.block = false;
                                shootTime.reset();

                            }

                            if ( built && shootTime.milliseconds() > shootWait){
                                state = AutoState.Collect2;
                                follow.usePathHeadings(true);
                                follow.setHeadingLookAheadDistance(100);
                                follow.setPath(paths.returnPath("Collect2"));
                                intake.block = true;
                                pathing = true;
                                built = false;

                            }
                            if ( follow.isFinished(12,12)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
                                intake.block = false;
                            }
                            break;
                            case Collect2:
                                if (pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                                    state = AutoState.driveToShoot2;
                                    follow.setPath(paths.returnPath("driveToShoot2"));
                                    follow.usePathHeadings(false);
                                    targetHeading = 260;
                                    built = false;
                                }
                                break;
                                case driveToShoot2:
                                    if (pathing && follow.isFinished(8, 8)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                                        pathing = false;
                                        shootTime.reset();
                                        built = true;

                                    }
                                    if ( follow.isFinished(12,12)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
                                        intake.block = false;
                                    }
                                    if ( built && shootTime.milliseconds() > shootWait){
                                        state = AutoState.finished;
//                                        follow.setPath(paths.returnPath("collect3"));
                                        follow.usePathHeadings(true);
                                        follow.setHeadingLookAheadDistance(130);
                                        intake.block = true;
                                        pathing = true;
                                        built = false;

                                    }

                                    break;
                                    case collect3:
                                        if (pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                                            state = AutoState.gate;
                                            follow.setPath(paths.returnPath("gate"));
                                            follow.usePathHeadings(false);

                                            targetHeading = 0;
                                            built = false;
                                        }

                                        break;
                                        case gate:
                                            if (follow.isFinished(18,18 )&&built==false){
                                                state = AutoState.driveToShoot3;
                                                follow.setPath(paths.returnPath("driveToShoot3"));
                                                targetHeading = 270;

                                            }
                                            break;
                                            case driveToShoot3:
                                                if (pathing && follow.isFinished(10, 10)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo ){
                                                    pathing = false;
                                                    shootTime.reset();
                                                    built = true;

                                                }
                                                if ( follow.isFinished(12,12)&& Math.abs(odometry.getXVelocity() +odometry.getYVelocity())< velo){
                                                    intake.block = false;
                                                }
                                                if ( built && shootTime.milliseconds() > shootWait){
                                                    state = AutoState.firstBackCollect;
                                                    follow.setPath(paths.returnPath("firstBackCollect"));
                                                    targetHeading = 250;
                                                    intake.block = true;
                                                    pathing = true;
                                                    built = false;

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
