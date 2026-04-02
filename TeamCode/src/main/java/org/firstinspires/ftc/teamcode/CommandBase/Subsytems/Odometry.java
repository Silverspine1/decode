package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver.Register;

import java.util.ArrayDeque;
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

    public double X, Y, Heading, normilised;
    double startX, startY, startHeading;

    double XVelocity = 0;
    double YVelocity = 0;
    double HVelocity = 0;

    // ── Raw acceleration (Δvel / Δtime) ──────────────────────────────────────
    private double prevXVel = 0;
    private double prevYVel = 0;
    private double prevHVel = 0;
    private long   prevNanos = 0;         // nanosecond timestamp of last update

    // ── Rolling-window smoothed acceleration ─────────────────────────────────
    // Tune this constant: larger window → smoother but laggier response.
    // At ~10 ms loop time, ACCEL_WINDOW = 15 averages ~150 ms of history.
    private static final int ACCEL_WINDOW = 5;

    private final ArrayDeque<Double> xAccelSamples = new ArrayDeque<>(ACCEL_WINDOW + 1);
    private final ArrayDeque<Double> yAccelSamples = new ArrayDeque<>(ACCEL_WINDOW + 1);
    private final ArrayDeque<Double> hAccelSamples = new ArrayDeque<>(ACCEL_WINDOW + 1);

    public double XAccel = 0;   // smoothed X acceleration  (cm/s²)
    public double YAccel = 0;   // smoothed Y acceleration  (cm/s²)
    public double HAccel = 0;   // smoothed H acceleration  (rad/s²)

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
        odo = getOpMode().hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // --- Pinpoint I2C Optimization (Phase 2) ---
        // Only read what we actually use. Excluding LOOP_TIME, etc.
        odo.setBulkReadScope(
                Register.X_POSITION,
                Register.Y_POSITION,
                Register.H_ORIENTATION,
                Register.X_VELOCITY,
                Register.Y_VELOCITY,
                Register.H_VELOCITY);

        odo.setOffsets(29, 134, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    public double headingError(double targetHeading) {
        return Heading - targetHeading;
    }

    @Override
    public void execute() {
        long start = System.nanoTime();
        executeEX();

        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.ODOMETRY, System.nanoTime() - start);
    }

    public double X() { return X; }
    public double Y() { return Y; }
    public double Heading() { return 360 - Heading; }
    public double normiliased() { return normilised; }
    public double getYVelocity() { return YVelocity; }
    public double getXVelocity() { return XVelocity; }
    public double getHVelocity() { return HVelocity; }

    // ── Smoothed acceleration getters ─────────────────────────────────────────
    public double getXAccel() { return XAccel; }
    public double getYAccel() { return YAccel; }
    public double getHAccel() { return HAccel; }

    // ── Helper: push a sample into a window deque and return the new average ──
    private double pushAndAverage(ArrayDeque<Double> window, double sample) {
        window.addLast(sample);
        if (window.size() > ACCEL_WINDOW) {
            window.pollFirst();          // evict the oldest sample
        }
        double sum = 0;
        for (double v : window) sum += v;
        return sum / window.size();
    }

    public LambdaCommand update = new LambdaCommand(
            () -> {
            },
            () -> {
                odo.update();

                // ── Velocity ──────────────────────────────────────────────────
                XVelocity = odo.getVelX(DistanceUnit.CM);
                YVelocity = -odo.getVelY(DistanceUnit.CM);
                HVelocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

                // ── Averaged Acceleration ─────────────────────────────────────
                // Compute Δtime in seconds; guard against the very first tick
                // (prevNanos == 0) and near-zero dt to avoid divide-by-zero.
                long nowNanos = System.nanoTime();
                double dt = (prevNanos == 0) ? 0.0 : (nowNanos - prevNanos) * 1e-9;
                prevNanos = nowNanos;

                if (dt > 1e-6) {   // only update when we have a real time delta
                    double rawXAccel =  (XVelocity - prevXVel) / dt;
                    double rawYAccel =  (YVelocity - prevYVel) / dt;
                    double rawHAccel =  (HVelocity - prevHVel) / dt;

                    XAccel = pushAndAverage(xAccelSamples, rawXAccel);
                    YAccel = pushAndAverage(yAccelSamples, rawYAccel);
                    HAccel = pushAndAverage(hAccelSamples, rawHAccel);
                }

                prevXVel = XVelocity;
                prevYVel = YVelocity;
                prevHVel = HVelocity;

                // ── Heading ───────────────────────────────────────────────────
                // Cache heading reads (avoids redundant unit conversions)
                double rawDeg = odo.getHeading(AngleUnit.DEGREES);
                double rawRad = odo.getHeading(AngleUnit.RADIANS);

                Heading = startHeading + rawDeg;
                normilised = startHeading + rawRad;

                if (startHeading + rawDeg < 0) {
                    Heading = startHeading + rawDeg + 360;
                } else {
                    Heading = startHeading + rawDeg;
                }

                // ── Position ──────────────────────────────────────────────────
                X = startX + odo.getPosX(DistanceUnit.CM);
                Y = startY - odo.getPosY(DistanceUnit.CM);
            },
            () -> false);

    public void offsetY(double offset) {
        Y += offset;
    }
}