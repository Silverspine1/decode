package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Tracking extends SubSystem {

    private Limelight3A limelight;
    public LLResult llResult;
    public double distance ;
    public double horizontalAngle ;
    public double verticalAngle ;
    public double angleToBall ;
    public double xPosition ;
    public double yPosition ;
    public double pixelsFromBottom ;
    public double validFlag ;
    double lastH;
    boolean a = false;
    public Tracking(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }





    @Override
    public void init() {
        // Initialize Limelight
        limelight = getOpMode().hardwareMap.get(Limelight3A.class,"limelight" );

        // Start the Limelight
        limelight.pipelineSwitch(5);
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
        LLResult llResult = limelight.getLatestResult();
        this.llResult = llResult;
        if (!a){
            a =true;
            limelight.pipelineSwitch(5);

        }
        if (horizontalAngle > 0 && !(horizontalAngle ==0)) {
            lastH = horizontalAngle + 5;
        }else if (horizontalAngle < 0 && !(horizontalAngle ==0)) {
            lastH = horizontalAngle - 5;
        }



        // Get the Python data array
        double[] pythonData = llResult.getPythonOutput();

        if (pythonData != null && pythonData.length >= 8) {
            // Extract all the values
            distance = pythonData[0];        // cm
            horizontalAngle = pythonData[1]; // degrees
            verticalAngle = pythonData[2];   // degrees
            angleToBall = pythonData[3];     // degrees
            xPosition = pythonData[4];       // cm
            yPosition = pythonData[5];       // cm
            pixelsFromBottom = pythonData[6];
            validFlag = pythonData[7];// 1 = valid, 0 = invalid
        }else {
            validFlag = 0;
        }
    }
    public double GetDistance(){
        return distance;
    }
    public double GetValid(){
        return validFlag;
    }
    public double GetAngle(){
        return horizontalAngle;
    }
    public  double GetLastH(){
        return lastH;
    }
}
