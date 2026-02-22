package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    double X;
    double Y;
    double H;
    boolean valid = false;
    public boolean enabled = false; // Gating: only update when actually requested

    public JSONObject jsonData;
    public java.util.List<LLResultTypes.FiducialResult> fiducialResults;
    public java.util.List<LLResultTypes.DetectorResult> detectorResults;

    public AprilTags(JSONObject json) throws JSONException {
        this.jsonData = json;
        this.fiducialResults = new ArrayList<>();

    }

    Pose3D botPose;

    public AprilTags(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

    @Override
    public void init() {
        limelight = getOpMode().hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void start() {

        limelight.start();
    }

    public double getX() {

        return X + 180;
    }

    public double getY() {

        return Y + 180;
    }

    public double getH() {
        return H;
    }

    public boolean getValid() {

        return valid;
    }

    public Command defaultCommand = new LambdaCommand(
            () -> {
            },
            () -> {
            },
            () -> true);

    @Override
    public void execute() {
        // Only run if explicitly enabled by the OpMode (e.g. during recalibration)
        if (!enabled) {
            valid = false;
            return;
        }

        LLResult lr = limelight.getLatestResult();
        if (lr == null) {
            valid = false;
            return;
        }
        this.llResult = lr;
        X = lr.getBotpose().getPosition().y * 100;
        Y = lr.getBotpose().getPosition().x * 100;
        H = lr.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
        if (lr.isValid()) {
            valid = true;
        }
        H = 180 - H;
    }
}
