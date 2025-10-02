package org.firstinspires.ftc.teamcode.oldcooked;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.json.JSONException;
import org.json.JSONObject;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class AprilTags extends SubSystem {
   public  Limelight3A limelight;
   public IMU imu;
   public JSONObject jsonData;
           public java.util.List<LLResultTypes.FiducialResult> fiducialResults;
           public java.util.List<LLResultTypes.DetectorResult> detectorResults;
   public AprilTags(JSONObject json) throws JSONException{
       this.jsonData = json;
       this.fiducialResults = new ArrayList<>();

   }
   public LLResult llresult = getLatestResult();
   public AprilTags(OpModeEX opModeEX) {
       registerSubsystem(opModeEX, defaultCommand);
   }



    @Override
    public void init() {
        limelight= getOpMode().hardwareMap.get(Limelight3A.class,"limelight" );
        limelight.pipelineSwitch(0);
        LLResult llresult = getLatestResult();
RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
    public LLResult getLatestResult() {
        if (limelight != null) {
            return limelight.getLatestResult();
        }
        return null;
    }


    public double getTx() {
        LLResult llresult = getLatestResult();
        return (llresult != null && llresult.isValid()) ? llresult.getTx() : 0.0;
    }
    public double getTy() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    public double getTa() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTa() : 0.0;
    }


    public void start(){
       limelight.start();
    }
    public Command defaultCommand = new LambdaCommand(
            () -> {
            },
            () -> {
            },
            () -> true
    );
    @Override
    public void execute() {


    }
}
