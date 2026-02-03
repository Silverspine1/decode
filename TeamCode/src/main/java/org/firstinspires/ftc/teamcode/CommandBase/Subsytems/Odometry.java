package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver.Register;


import java.util.ArrayList;

import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;


public class Odometry extends SubSystem {

    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    Register[] onlyPosition = {
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
            Register.X_VELOCITY,
            Register.Y_VELOCITY,
    };

    DcMotorEx leftPod;
    DcMotorEx rightPod;
    DcMotorEx backPod;

    public double X, Y, Heading,normilised;
    double startX, startY, startHeading;

    double XVelocity = 0;
    double YVelocity = 0;
    double HVelocity = 0;
    boolean resetAtStart = false;

   public Odometry(OpModeEX opModeEX) {
       registerSubsystem(opModeEX, update);
    }

    public void startPosition(double X, double Y, int Heading) {
        this.startX = X;
        this.startY = Y;
        this.Heading = Heading;
    }

    @Override
    public void init() {
        /*
         * Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         */
        odo = getOpMode().hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        //Here we set the bulk read scope we created earlier.

        /*
         Another new feature to the Pinpoint is on-device error detection. This allows the device
         to ensure that the most recent read hasn't been corrupted. We use CRC8 error detection
         which is a lightweight, and accurate option. The sending device does a polynomial division
         on the contents of the transmission, and sends the result of that calculation along with
         the data. The receiving device then repeats that calculation and compares it. If the two
         results do not match, then the previous read of that data is repeated, and a
         "FAULT_BAD_READ" flag is thrown.
         */

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-13, 132, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public double headingError(double targetHeading) {
        return Heading - targetHeading;
    }

    @Override
    public void execute() {
        executeEX();
        if (!resetAtStart){
            odo.setPosX(0,DistanceUnit.CM);
            odo.setPosY(0,DistanceUnit.CM);
            odo.setHeading(0,AngleUnit.DEGREES);
            resetAtStart = true;

        }


    }

    public double X() {
        return X;
    }

    public double Y() {
        return Y;
    }

    public double Heading() {
        return 360 - Heading;
    }
    public double normiliased() {
        return normilised;
    }

    public double getYVelocity() {
        return YVelocity;
    }

    public double getXVelocity() {
        return XVelocity;
    }
    public double getHVelocity() {
        return HVelocity;
    }


    public LambdaCommand update = new LambdaCommand(
            () -> {
            },
            () -> {

                odo.update();


                XVelocity = odo.getVelX(DistanceUnit.CM);
                YVelocity = odo.getVelY(DistanceUnit.CM);
                HVelocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);


                Heading = startHeading + odo.getHeading(AngleUnit.DEGREES);
                normilised = startHeading + odo.getHeading(AngleUnit.RADIANS) ;

                if (odo.getHeading(AngleUnit.DEGREES) <0) {
                    Heading = odo.getHeading(AngleUnit.DEGREES) + 360;
                } else{
                    Heading = odo.getHeading(AngleUnit.DEGREES);
                }

                X =  startX-odo.getPosX(DistanceUnit.CM);
                Y =  startY + odo.getPosY(DistanceUnit.CM);
            },
            () -> false
    );

    public void offsetY(double offset) {
        Y += offset;
    }
}


