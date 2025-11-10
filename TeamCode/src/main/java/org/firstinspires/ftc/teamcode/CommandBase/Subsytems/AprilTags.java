package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.json.JSONException;
import org.json.JSONObject;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class AprilTags extends SubSystem {
   public LLResult llResult;
   public Limelight3A limelight;
   public IMU imu;
   public double theata4;
   double cameraOffset = 0.2;
   double YOffset = 0;
   double XOffset = 0;
   double X;
   double Y;

    public JSONObject jsonData;
           public java.util.List<LLResultTypes.FiducialResult> fiducialResults;
           public java.util.List<LLResultTypes.DetectorResult> detectorResults;
   public AprilTags(JSONObject json) throws JSONException{
       this.jsonData = json;
       this.fiducialResults = new ArrayList<>();

   }

   Pose3D botPose;

   public AprilTags(OpModeEX opModeEX) {
       registerSubsystem(opModeEX, defaultCommand);
   }



    @Override
    public void init() {
        limelight = getOpMode().hardwareMap.get(Limelight3A.class,"limelight" );
        limelight.pipelineSwitch(0);
      //  LLResult llresult = getLatestResult();
RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }


    public void start(){

        limelight.start();
    }

    public double getX() {

        return X;
    }
    public double getY() {

        return Y;
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






        LLResult llResult = limelight.getLatestResult();
        this.llResult = llResult;
        X = llResult.getBotpose().getPosition().y*100;
        Y = llResult.getBotpose().getPosition().x*100;
    }
}
