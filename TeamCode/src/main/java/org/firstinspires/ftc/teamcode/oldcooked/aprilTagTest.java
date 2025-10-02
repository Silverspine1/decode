package org.firstinspires.ftc.teamcode.oldcooked;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

@TeleOp
public class aprilTagTest extends OpModeEX {
    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        telemetry.addData("Limelight Connected?", Apriltag.limelight.isConnected());
        if (Apriltag.llresult != null) {
            int count = Apriltag.llresult.getFiducialResults().size();
            telemetry.addData("Fiducials Detected", count);

            telemetry.update();
            if (count > 0) {
                telemetry.addData("tx", Apriltag.getTx());
                telemetry.addData("ty", Apriltag.getTy());
                telemetry.addData("ta", Apriltag.getTa());
                telemetry.addData("Tag ID", Apriltag.llresult.getFiducialResults().get(0 ).getFiducialId());
            } else {
                telemetry.addData("Tags", "None detected");
            }
        } else {
            telemetry.addData("Limelight", "Result is NULL");
        }


        telemetry.update();

    }
}
