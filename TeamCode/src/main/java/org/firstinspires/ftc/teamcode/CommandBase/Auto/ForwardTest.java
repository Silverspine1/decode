package org.firstinspires.ftc.teamcode.CommandBase.Auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

/**
 * Single test path: drive 120cm straight forward using the exact PID values
 * from the competition autos (close_gateCycle). Place the robot with room
 * ahead, dpad_up to run. Telemetry shows live and final error.
 */
@TeleOp(name = "ForwardTest (120cm)", group = "tuning")
public class ForwardTest extends OpModeEX {

    private static final double DISTANCE = 120;   // cm forward
    private static final double START_X = 180, START_Y = 120;

    // exact auto config (close_gateCycle.java)
    private static final RobotConfig CONFIG = new RobotConfig(0.02, 0.004, 0.02, 0.009, 0.08, 0.004, 0.2, 0.004, 0.01, 0.0005, 0.012, 0.002, 130, 181, 650, 700);

    private final pathsManager paths = new pathsManager(CONFIG);
    private final follower follow = new follower(CONFIG);

    private boolean running = false;
    private double targetX, targetY;
    private final ElapsedTime runTimer = new ElapsedTime();

    @Override
    public void initEX() {
        odometry.startPosition((int) START_X, (int) START_Y, 0);
        driveBase.tele = false;
        driveBase.speed = 1;
        telemetry.addLine("ForwardTest: 120cm forward, auto PID values.");
        telemetry.addLine("dpad_up = run,  B = stop");
    }

    @Override
    public void loopEX() {
        boolean start = currentGamepad1.dpad_up && !lastGamepad1.dpad_up;
        boolean stop  = currentGamepad1.b && !lastGamepad1.b;

        odometry.queueCommand(odometry.update);

        if (stop) {
            running = false;
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }

        if (start && !running) {
            buildPath();
            running = true;
            runTimer.reset();
        }

        if (running) {
            RobotPower p = follow.followPathAuto(0, odometry.Heading(),
                    odometry.X(), odometry.Y(),
                    odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(p));
        } else {
            driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
        }

        telemetry.addData("running", running);
        telemetry.addData("pos", String.format("%.1f, %.1f  h=%.1f",
                odometry.X(), odometry.Y(), odometry.Heading()));
        if (targetY != 0) {
            telemetry.addData("error X/Y", String.format("%.1f / %.1f cm",
                    odometry.X() - targetX, odometry.Y() - targetY));
        }
        telemetry.addData("vel", String.format("%.0f cm/s",
                Math.hypot(odometry.getXVelocity(), odometry.getYVelocity())));
        if (running) telemetry.addData("time", String.format("%.2f s", runTimer.seconds()));
    }

    private void buildPath() {
        double cx = odometry.X(), cy = odometry.Y();
        targetX = cx;
        targetY = cy + DISTANCE;

        final Vector2D[] pts = new Vector2D[]{
                new Vector2D(cx, cy),
                new Vector2D(targetX, targetY)
        };
        sectionBuilder[] section = new sectionBuilder[]{
                () -> paths.addPoints(pts[0], pts[1])
        };
        paths.addNewPath("fwd120");
        paths.buildPath(section);

        follow.setPath(paths.returnPath("fwd120"));
        follow.usePathHeadings(false);
    }
}
