package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp
public class test_path extends OpModeEX {

    // --- CHANGE FOR EACH TEST RUN -------------------------------------------
    // This becomes the filename: /sdcard/FIRST/datalogs/<LOG_NAME>.csv
    // Examples:  "ratio_2p0"  "ratio_3p5"  "ratio_5p0"
    private static final String LOG_NAME = "ratio_2p5";

    // --- PATH DISTANCE -------------------------------------------------------
    // Change PATH_END_Y to set how far the robot drives for this test.
    // Use the same value for ALL 3 ratio tests so results are comparable.
    // Unit = whatever your odometry reports (mm, inches, ticks, etc.)
    //   Short  ->  -300
    //   Medium ->  -700  (default, use your average match distance)
    //   Long   ->  -1200
    private static final double PATH_END_Y = 95;
    // -------------------------------------------------------------------------

    pathsManager paths = new pathsManager(new RobotConfig(
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2, 0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 700));

    follower follow = new follower(new RobotConfig(
            0.02, 0.004, 0.02, 0.009, 0.08, 0.004,
            0.2, 0.004, 0.01, 0.0005, 0.012, 0.002,
            130, 181, 650, 700));

    double  targetHeading = 0;
    boolean pathing       = false;
    boolean setHeading    = false;

    // --- LOGGING STATE -------------------------------------------------------
    private FileWriter  csvWriter = null;
    private ElapsedTime pathTimer = new ElapsedTime();
    private boolean     logging   = false;
    private double      prevVel   = 0;
    double  howLong = 0;
    private double      prevTime  = 0;
    // -------------------------------------------------------------------------

    private final sectionBuilder[] path = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(0, PATH_END_Y)),
    };

    @Override
    public void initEX() {
        odometry.startPosition(0, 0, 0);
        targetHeading = 0;

        paths.addNewPath("path");
        paths.buildPath(path);
    }

    @Override
    public void loopEX() {

        // Start path + logging on dpad_up
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up) {
            follow.setPath(paths.returnPath("path"));
            pathing = true;

            openLog();
            pathTimer.reset();
            prevVel  = 0;
            prevTime = 0;
            logging  = true;
        }

        // Detect path finish
        if (pathing && follow.isFinished(5, 5) && Math.abs(odometry.getYVelocity()) < 4) {
            pathing = false;

            // Final zero-velocity row so the decel tail is captured
            if (logging) {
                writeRow(pathTimer.seconds(), 0.0, 0.0, odometry.Y());
                closeLog();
                logging = false;
            }
        }

        // Main drive loop
        if (pathing) {
            odometry.queueCommand(odometry.update);

            RobotPower currentPower = follow.followPathAuto(
                    targetHeading,
                    odometry.Heading(),
                    odometry.X(),
                    odometry.Y(),
                    odometry.getXVelocity(),
                    odometry.getYVelocity());

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
            telemetry.addData("vert", currentPower.getHorizontal());

            // Log this sample
            if (logging) {
                double t   = pathTimer.seconds();
                double vel = odometry.getYVelocity();
                double dt  = t - prevTime;
                double acc = (dt > 0.001) ? (vel - prevVel) / dt : 0.0;
                howLong = t;



                writeRow(t, vel, acc, odometry.Y());

                prevVel  = vel;
                prevTime = t;
            }

        } else {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }

        telemetry.addData("Y pos",   odometry.Y());
        telemetry.addData("Y vel",   odometry.getYVelocity());
        telemetry.addData("logging", logging ? "YES - " + LOG_NAME : "idle");
        telemetry.addData("t",   howLong);


    }

    // Open CSV and write header
    private void openLog() {
        try {
            File dir = new File("/sdcard/FIRST/datalogs");
            if (!dir.exists()) dir.mkdirs();

            csvWriter = new FileWriter(new File(dir, LOG_NAME + ".csv"), false);
            csvWriter.write("time_s,velocity,acceleration,position_y\n");
            csvWriter.flush();

        } catch (IOException e) {
            telemetry.addData("LOG ERROR", e.getMessage());
            csvWriter = null;
            logging   = false;
        }
    }

    // Write one data row — flush every row so a crash doesn't lose data
    private void writeRow(double t, double vel, double acc, double posY) {
        if (csvWriter == null) return;
        try {
            csvWriter.write(String.format("%.4f,%.4f,%.4f,%.4f\n", t, vel, acc, posY));
            csvWriter.flush();
        } catch (IOException e) {
            telemetry.addData("LOG WRITE ERR", e.getMessage());
        }
    }

    private void closeLog() {
        if (csvWriter == null) return;
        try { csvWriter.close(); } catch (IOException ignored) {}
        csvWriter = null;
    }

    // Safety net: close on OpMode stop even if path never finished
    @Override
    public void stop() {
        if (logging) {
            writeRow(pathTimer.seconds(), 0.0, 0.0, 0.0);
            closeLog();
            logging = false;
        }
    }
}