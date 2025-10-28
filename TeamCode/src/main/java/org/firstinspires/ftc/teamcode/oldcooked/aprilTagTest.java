//package org.firstinspires.ftc.teamcode.oldcooked;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
//
//@TeleOp
//
//public class aprilTagTest extends OpModeEX {
//    @Override
//    public void initEX() {
//
//
//    }
//
//    @Override
//    public void start() {
//
//        Apriltag.limelight.start();
//
//    }
//
//    @Override
//    public void loopEX() {
//        telemetry.addData("Limelight Connected?", Apriltag.limelight.isConnected());
//        if (Apriltag.limelight.isConnected()) {
//            telemetry.addData("con", "connected");
//        }
//        if (Apriltag.llResult.isValid()) {
//            int count = Apriltag.llResult.getFiducialResults().size();
//            telemetry.addData("Fiducials Detected", count);
//            Pose3D botpose = Apriltag.llResult.getBotpose();
//
//                telemetry.addData( "tx", Apriltag.llResult.getTx());
//                telemetry.addData("ty", Apriltag.llResult.getTy());
//                telemetry.addData("ta", Apriltag.llResult.getTa());
//                telemetry.addData("Tag ID", Apriltag.llResult.getFiducialResults().get(0).getFiducialId());
//                telemetry.addData("yaw",Apriltag.llResult.getFiducialResults().get(0).getRobotPoseFieldSpace());
//                telemetry.addData("post",Apriltag.llResult.getBotpose().getPosition());
//
//                telemetry.addData("botpose", botpose.toString());
//            telemetry.update();
//        }
//
//
//            telemetry.addData("yessss", "7777");
//        }
//
//
//
//    }
//
