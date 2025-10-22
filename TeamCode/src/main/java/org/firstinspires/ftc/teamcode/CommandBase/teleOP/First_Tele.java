package org.firstinspires.ftc.teamcode.CommandBase.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.Subsytems.Turret;

@TeleOp
public class First_Tele extends OpModeEX {
    double drivepower = 0.5;
    double hood = 168;
    @Override
    public void initEX() {
        odometry.startPosition(75, 139, 0);


    }

    @Override
    public void loopEX() {
        turret.robotX = odometry.X();
        turret.robotY = odometry.Y();
        turret.robotHeading = odometry.normilised;
        driveBase.drivePowers(-gamepad1.right_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger), -gamepad1.right_stick_x);
        if (turret.shootingLevel == Turret.LowMediumHigh.low &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
            turret.shootingLevel = Turret.LowMediumHigh.medium;
        } else if (turret.shootingLevel == Turret.LowMediumHigh.medium &&currentGamepad1.dpad_up && !lastGamepad1.dpad_up){
            turret.shootingLevel = Turret.LowMediumHigh.low;
        }
//        if (Math.abs(gamepad1.right_stick_y)>0){
//            turret.hoodAdjust.setPosition(hood += gamepad1.right_stick_y);
//        }
        if (gamepad1.dpad_up){
            drivepower += 0.01;
        }
        if (gamepad1.dpad_left){
            driveBase.drivePowers(drivepower,0,0);
        }else {
            driveBase.drivePowers(0,0,0);
        }
//        turret.targetRPM += gamepad1.left_stick_y*7;
        if (gamepad1.right_bumper){
            intake.intakeMotor.update(-1);

        }else if (gamepad1.left_bumper){
            intake.intakeMotor.update(1);
        }else if(turret.intakeTime){
            intake.intakeMotor.update(-1);
        }else {
            intake.intakeMotor.update(0);
        }
        if (gamepad1.a){
            intake.trans.setPosition(1);
        }


        telemetry.addData("in zone",turret.inZone);
        telemetry.addData("odometry x", odometry.X());
        telemetry.addData("odometry y", odometry.Y());
        telemetry.addData("Heading",odometry.Heading());
        telemetry.addData("target",turret.targetRPM);
        telemetry.addData("ditance",turret.distance);
        telemetry.addData("hood angle",hood);
        telemetry.addData("drive",drivepower);
        telemetry.update();


    }
}
