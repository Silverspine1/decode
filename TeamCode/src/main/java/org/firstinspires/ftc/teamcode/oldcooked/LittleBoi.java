package org.firstinspires.ftc.teamcode.oldcooked;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp
public class LittleBoi extends OpModeEX {

    double targetRPM = 2700;
    boolean shootHim = false;
    double shootpower;





    @Override
    public void initEX() {


    }

    @Override
    public void loopEX() {
        driveBase.drivePowers(gamepad1.right_stick_y,(gamepad1.left_trigger - gamepad1.right_trigger),-gamepad1.right_stick_x);


        if (gamepad1.a){
            turret.shooterMotorOne.update(gamepad1.left_stick_y);
//            turret.shooterMotorTwo.update(gamepad1.left_stick_y);
        }
        if (gamepad1.y){
            shootHim = true;

        }
        double rpm = -(turret.shooterMotorOne.getVelocity()/28)*60;

        if (turret.shootPID.calculate(targetRPM,rpm) <0){
            shootpower = 0;
        }else {
            shootpower = turret.shootPID.calculate(targetRPM,rpm);
        }

        if (shootHim) {
            targetRPM += (gamepad1.left_stick_y*14);
            turret.shooterMotorOne.update(Math.abs(shootpower));
//            turret.shooterMotorTwo.update(Math.abs(shootpower));
        }
//        if (gamepad1.b){
//            intake.intakeMotor.update(-1);
//        }else if(gamepad1.x) {
//            intake.intakeMotor.update(1);
//        }else {
//            intake.intakeMotor.update(0);
//        }
//        if (gamepad1.dpad_up){
//            intake.trans.setPosition(1);
//        }else {
//            intake.trans.setPosition(0.5);
//        }


        telemetry.addData("shoot rpm",rpm);
        telemetry.addData("target",targetRPM);
        telemetry.addData("power", shootpower);
        telemetry.update();




    }

}
