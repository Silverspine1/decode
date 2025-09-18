package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.util.ArrayList;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Odometry extends SubSystem {



    public DcMotorEx leftPod;
    public DcMotorEx rightPod;
    public DcMotorEx backPod;

    public IMU imu;


    double X, Y, Heading;
    int startHeading;
    public double otherHeading;

    double lastRightPod, lastLeftPod, lastBackPod;
    public double currentRightPod, currentLeftPod, currentBackPod;
    public double rightPodPos, leftPodPos, backPodPos;

    double podTicks = 2000;
    double wheelRadius = 1.5;
    double trackWidth = 12.5;
    double backPodOffset = 17.5;

    double ticksPerCM = ((2.0 * Math.PI) * wheelRadius)/podTicks;
    double cmPerDegreeX = (double) (2) / 360;
    double cmPerDegreeY = (((2 * Math.PI) * backPodOffset))/ 360;

    double currentXVelocity = 0;
    double currentYVelocity = 0;

    boolean sampleReset = true;

    public boolean isRunningDistanceSensorReset() {
        return runningDistanceSensorReset;
    }



    boolean runningDistanceSensorReset = false;
    int resetCounter = 0;

    public Odometry(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, updateLineBased);
    }

    public void startPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        this.startHeading = Heading;
        this.Heading = Math.toRadians(Heading);
    }

    @Override
    public void init() {
        leftPod = getOpMode().hardwareMap.get(DcMotorEx.class, "RF");
        rightPod = getOpMode().hardwareMap.get(DcMotorEx.class, "RB");
        backPod = getOpMode().hardwareMap.get(DcMotorEx.class, "LF");

        imu = getOpMode().hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();


    }

    public double headingError(double targetHeading){
        return Heading-targetHeading;
    }

    @Override
    public void execute() {

        executeEX();
        updateVelocity();



    }

    public double X (){
        return X;
    }

    public double Y (){
        return Y;
    }

    public double Heading (){
        return Math.toDegrees(Heading);
    }

    public double getYVelocity(){
        return currentYVelocity;
    }

    public double getXVelocity(){
        return currentXVelocity;
    }

    public void updateVelocity(){
        double RRXError = ticksPerCM * ((-rightPod.getVelocity()+(-leftPod.getVelocity()))/2);
        double RRYError = ticksPerCM * -backPod.getVelocity();

//        currentXVelocity = RRXError;
//        currentYVelocity = RRYError;

        currentXVelocity = RRXError * Math.cos(Heading) - RRYError * Math.sin(Heading);
        currentYVelocity = RRXError * Math.sin(Heading) + RRYError * Math.cos(Heading);

    }

    public LambdaCommand update = new LambdaCommand(
            () -> {},
            () -> {
                //need to code this
                //prob some constant accel loc code
            },
            () -> false
    );

    public void offsetY(double offset){
        Y += offset;
    }



    public LambdaCommand updateLineBased = new LambdaCommand(
            () -> {},
            () -> {

//                System.out.println("execute odometry update line based");

                lastBackPod = currentBackPod;
                lastLeftPod = currentLeftPod;
                lastRightPod = currentRightPod;

                currentBackPod = -backPod.getCurrentPosition();
                currentLeftPod = -leftPod.getCurrentPosition();
                currentRightPod = -rightPod.getCurrentPosition();

                double deltaRight = currentRightPod - lastRightPod;
                double deltaLeft = currentLeftPod - lastLeftPod;
                double deltaBack = currentBackPod - lastBackPod;

                double deltaHeading = (ticksPerCM * (deltaRight - deltaLeft)) / (trackWidth+0.22);
                Heading += deltaHeading;

                if (Math.toDegrees(Heading) < 0){
                    Heading = Math.toRadians(360 - Math.toDegrees(Heading));
                } else if (Math.toDegrees(Heading) > 360) {
                    Heading = Math.toRadians(Math.toDegrees(Heading) - 360);
                }

                double deltaX = ((((deltaRight+deltaLeft)*ticksPerCM)/2)) + (Math.toDegrees(deltaHeading) * cmPerDegreeX);
                double deltaY = (ticksPerCM * deltaBack) - (Math.toDegrees(deltaHeading) * cmPerDegreeY);

//                X += deltaX;
//                Y += deltaY;

                X += deltaX * Math.cos(Heading) - deltaY * Math.sin(Heading);
                Y += deltaX * Math.sin(Heading) + deltaY * Math.cos(Heading);

                //4165 back pod 180

//                updatePodReadings();
//                leftPod.update(0);
//                rightPod.update(0);
//                backPod.update(0);

//                System.out.println(Math.abs(rightPod.getTimeCompleted() - leftPod.getTimeCompleted())/1000000);

            },
            () -> true
    );

    public Command resetPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        startHeading = Heading;
        return resetPosition;
    }

    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> false
    );

    private void updatePodReadings(){
//        leftPod.updatePosition();
//        rightPod.updatePosition();
//        backPod.updatePosition();
    }


}