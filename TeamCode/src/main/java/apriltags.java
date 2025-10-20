import static com.qualcomm.hardware.limelightvision.LLResultTypes.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class apriltags extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult> detectorResults;
    private java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducialResults;
    private JSONObject jsonData;

    protected apriltags(JSONObject json) throws JSONException {
        this.jsonData = json;
        this.fiducialResults = new ArrayList();
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


    }

    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        return this.fiducialResults;
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.addData("fi", llResult.getFiducialResults());
            telemetry.addData("de", llResult.getDetectorResults());
            JSONArray fiducialArray = this.jsonData.optJSONArray("FiducialResults");
            if (fiducialArray != null) {
                for (int i = 0; i < fiducialArray.length(); i++) {
                    JSONObject detection = fiducialArray.optJSONObject(i);
                    if (detection == null) continue;

                    int tagId = detection.optInt("fID", -1); // -1 = invalid if missing

                    if (tagId == 1) {
                        telemetry.addLine("Detected AprilTag ID: 1");
                    } else if (tagId == 2) {
                        telemetry.addLine("Detected AprilTag ID: 2");
                    } else if (tagId == 3) {
                        telemetry.addLine("Detected AprilTag ID: 3");
                    } else {
                        telemetry.addLine("Detected AprilTag ID: " + tagId);

                    }

                }

            }
        }
    }
}












