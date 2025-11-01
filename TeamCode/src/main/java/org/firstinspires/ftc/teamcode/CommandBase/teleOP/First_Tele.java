package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Turret;

@TeleOp
public class First_Tele extends OpModeEX {
    double drivepower = 0.5;
    double targetHood = 25;

    @Override
    public void initEX() {
        turret.toggle = false;
       Apriltag.limelight.pipelineSwitch(0);

    }
        @Override
    public void start() {

        Apriltag.limelight.start();

    }

    @Override
    public void loopEX() {
        turret.Auto = true;
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;
      //  driveBase.drivePowers(-gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger), -gamepad1.right_stick_x);
//        if (turret.shootingLevel == Turret.LowMediumHigh.low &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.medium;
//        } else if (turret.shootingLevel == Turret.LowMediumHigh.medium &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
//            turret.shootingLevel = Turret.LowMediumHigh.low;
//        }
        if (Math.abs(gamepad1.right_stick_y)>0){
            targetHood += gamepad1.right_stick_y/8;
            turret.setHoodDegrees(targetHood);
        }
//
 //        turret.targetRPM += gamepad1.left_stick_y*7;

        if (gamepad1.right_bumper){
            intake.block = true;
            intake.intakeMotor.update(-1);

        }else if (gamepad1.left_bumper){
            intake.intakeMotor.update(-1);
            intake.block = false;
        }else if(turret.intakeTime){
            intake.intakeMotor.update(-1);
        }else {
            intake.intakeMotor.update(0);
        }
        if (!lastGamepad1.a && currentGamepad1.a){
            turret.toggle = true;
            odometry.odo.setPosX(Apriltag.llResult.getBotpose().getPosition().y - 180 +38, DistanceUnit.CM);
            odometry.odo.setPosY( Apriltag.llResult.getBotpose().getPosition().x-180 -35,DistanceUnit.CM);
            odometry.odo.setHeading(180 - Apriltag.llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES), AngleUnit.DEGREES);
        }
        if (!lastGamepad2.dpad_left && currentGamepad2.dpad_left){
            turret.turrofset += -3;
        }
        if (!lastGamepad2.dpad_right && currentGamepad2.dpad_right){
            turret.turrofset += 3;
        }
        if (!lastGamepad2.dpad_up && currentGamepad2.dpad_up){
            turret.mapOfset += -20;
        }
        if (!lastGamepad2.dpad_up && currentGamepad2.dpad_up){
            turret.mapOfset += 20;
        }







        telemetry.addData("in zone",turret.inZone);
        telemetry.addData("odometry x", odometry.X());
        telemetry.addData("odometry y", odometry.Y());
        telemetry.addData("Heading",odometry.Heading());
        telemetry.addData("target",turret.targetRPM);
        telemetry.addData("ditance",turret.distance);
        telemetry.addData("angle",turret.theta_4 - 5.3 );

        telemetry.addData("Total Pose",-Apriltag.llResult.getBotpose().getPosition().x * 100 + 180 -38);
        telemetry.addData("Total Pose",Apriltag.llResult.getBotpose().getPosition().y * 100 +180 +35);

        telemetry.addData("hood U",turret.U2);
        telemetry.addData("hood T2",targetHood);



        telemetry.update();


    }
}
