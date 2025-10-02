package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

public class Turret extends SubSystem {
    public MotorEx shooterMotorOne = new MotorEx();
    public MotorEx shooterMotorTwo = new MotorEx();

    public ServoDegrees turretTurnOne = new ServoDegrees();
    public ServoDegrees turretTurnTwo =new ServoDegrees();
    public ServoDegrees hoodAdjust = new ServoDegrees();

    enum lowMediumHigh{
        low,
        medium,
        high
    }
    public lowMediumHigh shootingLevel = lowMediumHigh.low;

    public boolean inZone = false;
    double Xoffset = 16.6;
    double Yoffset = 16.6;
    double shootPower;


    public double targetX = 0;
    public double targetY = 0;
    public double robotX;
    public double robotY;
    public double robotHeading;

    double targetRPM = 2700;
    double turretAngle;
    final double gearRatio = 0.7272;
    final double turretLimitAngle =80;


    public double distance;
    double distance1 = 143;
    double distance2 = 244;
    double distance3 = 320;
    double distance4 = 380;



    double lowHoodAngle1 = 166.2;
    double lowHoodAngle2 = 110;
    double lowHoodAngle3 = 100;

    double mediumHoodAngle1 = 166.2;
    double mediumHoodAngle2 = 110;
    double mediumHoodAngle3 = 100;

    double highHoodAngle1 = 166.2;
    double highHoodAngle2 = 110;
    double highHoodAngle3 = 100;


    double lowPower1 = 3760;
    double lowPower2 = 4200;
    double lowPower3 = 4840;

    double mediumPower1 = 3760;
    double mediumPower2 = 4200;
    double mediumPower3 = 4840;

    double highPower1 = 3760;
    double highPower2 = 4200;
    double highPower3 = 4840;


    double lowPower2NoHood;
    double lowPower3NoHood;
    double lowPower4NoHood;

    double mediumPower2NoHood;
    double mediumPower3NoHood;
    double mediumPower4NoHood;

    double highPower2NoHood;
    double highPower3NoHood;
    double highPower4NoHood;

    double interpolatePower;


    public boolean turretTarget = true;


    public PIDController shootPID = new PIDController(0.01,0,0.0003);
    public Turret(OpModeEX opModeEX){
      registerSubsystem(opModeEX,defaultCommand);
    }


    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

    @Override
    public void init() {
        shooterMotorOne.initMotor("shooterMotorOne", getOpMode().hardwareMap);
        shooterMotorTwo.initMotor("shooterMotorTwo", getOpMode().hardwareMap);

        turretTurnOne.initServo("turretTurnOne", getOpMode().hardwareMap);
        turretTurnTwo.initServo("turretTurnTwo", getOpMode().hardwareMap);
        hoodAdjust.initServo("hoodAdjust", getOpMode().hardwareMap);
        turretTurnTwo.setRange(355);
        turretTurnOne.setRange(355);
        hoodAdjust.setRange(355);
        turretTurnTwo.setDirection(Servo.Direction.REVERSE);
        turretTurnOne.setDirection(Servo.Direction.REVERSE);


        turretTurnOne.setOffset(178);
        turretTurnTwo.setOffset(178);
        turretTurnOne.setPosition(0);
        turretTurnTwo.setPosition(0);
        hoodAdjust.setPosition(168);
    }






    @Override
    public void execute() {
        executeEX();
        double rpm = -(shooterMotorOne.getVelocity()/28)*60;
        if (shootPID.calculate(targetRPM,rpm) <0){
            shootPower = 0;
        }else {
            shootPower = shootPID.calculate(targetRPM,rpm);
        }

        switch (shootingLevel) {
            case low:
                if (distance <= distance1) {

                    interpolatePower = lowPower1;

                } else if (distance <= distance2) {

                    interpolatePower = lowPower1 + (lowPower2NoHood - lowPower1) * (distance - distance1) / (distance2 - distance1);

                } else if (distance <= distance3) {


                    interpolatePower = lowPower2 + (lowPower3NoHood - lowPower2) * (distance - distance2) / (distance3 - distance2);

                } else if (distance <= distance4) {

                    interpolatePower = lowPower3 + (lowPower4NoHood - lowPower3) * (distance - distance3) / (distance4 - distance3);

                } else {

                    if (distance4 - distance3 != 0) {

                        double slope = (lowPower4NoHood - lowPower3) / (distance4 - distance3);
                        interpolatePower = lowPower3 + slope * (distance - distance3);

                    } else {

                        interpolatePower = lowPower4NoHood;

                    }
                }
                if (distance >= distance1) {
                    hoodAdjust.setPosition(lowHoodAngle1);
                } else if (distance >= distance2) {
                    hoodAdjust.setPosition(lowHoodAngle2);
                } else if (distance >= distance3) {
                    hoodAdjust.setPosition(lowHoodAngle3);
                }

                break;
            case medium:
                if (distance <= distance1) {

                    interpolatePower = mediumPower1;

                } else if (distance <= distance2) {

                    interpolatePower = mediumPower1 + (mediumPower2NoHood - mediumPower1) * (distance - distance1) / (distance2 - distance1);

                } else if (distance <= distance3) {


                    interpolatePower = mediumPower2 + (mediumPower3NoHood - mediumPower2) * (distance - distance2) / (distance3 - distance2);

                } else if (distance <= distance4) {

                    interpolatePower = mediumPower3 + (mediumPower4NoHood - mediumPower3) * (distance - distance3) / (distance4 - distance3);

                } else {

                    if (distance4 - distance3 != 0) {

                        double slope = (mediumPower4NoHood - mediumPower3) / (distance4 - distance3);
                        interpolatePower = mediumPower3 + slope * (distance - distance3);

                    } else {

                        interpolatePower = mediumPower4NoHood;

                    }
                }
                if (distance >= distance1) {
                    hoodAdjust.setPosition(mediumHoodAngle1);
                } else if (distance >= distance2) {
                    hoodAdjust.setPosition(mediumHoodAngle2);
                } else if (distance >= distance3) {
                    hoodAdjust.setPosition(mediumHoodAngle3);


                }
                break;
                case high:
                    if (distance <= distance1) {

                        interpolatePower = highPower1;

                    } else if (distance <= distance2) {

                        interpolatePower = highPower1 + (highPower2NoHood - highPower1) * (distance - distance1) / (distance2 - distance1);

                    } else if (distance <= distance3) {


                        interpolatePower = highPower2 + (highPower3NoHood - highPower2) * (distance - distance2) / (distance3 - distance2);

                    } else if (distance <= distance4) {

                        interpolatePower = highPower3 + (highPower4NoHood - highPower3) * (distance - distance3) / (distance4 - distance3);

                    } else {

                        if (distance4 - distance3 != 0) {

                            double slope = (highPower4NoHood - highPower3) / (distance4 - distance3);
                            interpolatePower = highPower3 + slope * (distance - distance3);

                        } else {

                            interpolatePower = highPower4NoHood;

                        }
                    }
                    if (distance >= distance1) {
                        hoodAdjust.setPosition(highHoodAngle1);
                    } else if (distance >= distance2) {
                        hoodAdjust.setPosition(highHoodAngle2);
                    } else if (distance >= distance3) {
                        hoodAdjust.setPosition(highHoodAngle3);

                    }


        }

            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;
            distance = Math.hypot(deltaY,deltaX);


             if (robotHeading > Math.PI) {
                robotHeading = robotHeading - Math.PI * 2;
            }

            turretAngle = Math.toDegrees(Math.atan2(deltaY, -deltaX) - robotHeading);

            if ((turretAngle) > turretLimitAngle) {

                turretAngle = turretLimitAngle;

            } else if ((turretAngle) < -turretLimitAngle) {

                turretAngle = -turretLimitAngle;

            }
            if ( (robotX > 120 && robotX < 240 && robotY > 320) || ((robotY - Yoffset < 180)&& (robotX - Xoffset < 180) && (robotX - Xoffset >= robotY - Yoffset))|| ((robotY + Yoffset < 180) && (robotX + Xoffset > 180) && (360- robotX+Xoffset >= robotY + Yoffset)) ){
                inZone = true;
            }else {
                inZone = false;
            }
            if (inZone) {
                targetRPM = interpolatePower;
                shooterMotorOne.update(shootPower);
                shooterMotorTwo.update(shootPower);
                turretTurnOne.setPosition(((turretAngle) / gearRatio));
                turretTurnTwo.setPosition(((turretAngle) / gearRatio));
            }else{
                shooterMotorTwo.update(0);
                shooterMotorOne.update(0);
            }
            //robotX > 120 && robotX < 240 && robotY > 320 ||
        //|| ((robotY + Yoffset < 180) && (robotX - Xoffset > 180) && (360- robotX-Xoffset >= robotY + Yoffset))



        }
}

