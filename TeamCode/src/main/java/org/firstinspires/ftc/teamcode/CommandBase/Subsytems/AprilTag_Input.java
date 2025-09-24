package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class AprilTag_Input extends SubSystem {
    Limelight3A limelight;
    private IMU imu;
     LLResultTypes.FiducialResult fiducialResult;
    public double X;
    public double y;
    public double heading;


    public AprilTag_Input(OpModeEX opModeEX){
        registerSubsystem(opModeEX,defaultCommand);
    }

    @Override
    public void init() {
        limelight = getOpMode().hardwareMap.get(Limelight3A.class, "limelight");



    }
    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    @Override
    public void execute() {
        executeEX();
        limelight.updateRobotOrientation(heading);

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid() );
        Pose3D botpose = llResult.getBotpose_MT2();

    }


    }

