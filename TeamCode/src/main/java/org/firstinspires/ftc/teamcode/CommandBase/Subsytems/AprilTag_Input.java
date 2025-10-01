package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;


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
    public Limelight3A limelight;
    private IMU imu;
    public LLResultTypes.FiducialResult fiducialResult;
    public  LLResult llResult;
    public double X;
    public double y;
    public double heading;
    public double XOffset;
    public double YOffset;


    public AprilTag_Input(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

    @Override
    public void init() {
        limelight = getOpMode().hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.UP);


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
        executeEX();
        limelight.updateRobotOrientation(heading);

        LLResult llResult = limelight.getLatestResult();


    }
}




