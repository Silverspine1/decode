package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.AdafruitSensorDriver;
import org.firstinspires.ftc.teamcode.CommandBase.ColourSensorEx;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.util.ArrayList;
import java.util.List;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Intake extends SubSystem {
    public MotorEx intakeMotor = new MotorEx();
    public MotorEx secondIntakeMotor = new MotorEx();

    Servo intakeBlocker;
    Servo intakePTO;
    public boolean block = true;
    public boolean InTake = false;
    private boolean intakeTogle;

    public ColourSensorEx lowerSensor = new ColourSensorEx();
    public ColourSensorEx upperSensor = new ColourSensorEx();


    public int ballCount = 0;

    public List<BallColor> ballOrder = new ArrayList<>();

    private boolean isBallCurrentlyDetected = false;
    private static final int MAX_BALLS = 3;


    public Intake(OpModeEX opModeEX){
        registerSubsystem(opModeEX,defaultCommand);
    }

    public BallColor upperBall = BallColor.NONE;
    public BallColor lowerBall = BallColor.NONE;

    private static final double COLOR_DOMINANCE_RATIO = 1.08;
    private static final int MIN_CLEAR_FOR_BALL = 300;

    @Override
    public void init() {
        intakeMotor.initMotor("intakeMotor", getOpMode().hardwareMap);
        intakeMotor.initMotor("intakeMotor", getOpMode().hardwareMap);

        intakePTO = getOpMode().hardwareMap.get(Servo.class, "intakePTO");
        intakeBlocker = getOpMode().hardwareMap.get(Servo.class, "intakeBlocker");
        lowerSensor.initSensor("LowerSensor",getOpMode().hardwareMap);
        upperSensor.initSensor("UpperSensor", getOpMode().hardwareMap); // Init the second sensor
    }

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );
    public enum BallColor {
        NONE,
        GREEN,
        PURPLE
    }

    /**
     * Resets the ball counter and the stored order of colors.
     * Call this at the start of a match or after purging the intake.
     */
    public void resetIntakeCounter() {
        ballCount = 0;
        ballOrder.clear();
        isBallCurrentlyDetected = false;
    }

    /**
     * Processes the reading from the lower sensor to detect and classify a new ball.
     * This method implements a rising-edge detector.
     * @param lowerReading The latest reading from the lower Adafruit sensor.
     */
    public void classifyAndCountLowerBall(AdafruitSensorDriver.Reading lowerReading) {
        // First, classify the color of whatever is in front of the sensor right now.
        BallColor currentlySeenColor = classifySingle(lowerReading);
        boolean isBallNowPresent = currentlySeenColor != BallColor.NONE;

        // RISING-EDGE LOGIC: Check if a ball just appeared when there wasn't one before.
        if (isBallNowPresent && !isBallCurrentlyDetected) {
            // A new ball has entered the sensor's range.
            // Only count and add if we haven't reached the max capacity.
            if (ballCount < MAX_BALLS) {
                ballCount++;
                ballOrder.add(currentlySeenColor); // Add the detected color to our list
            }
        }

        // Update the state for the next loop cycle.
        this.isBallCurrentlyDetected = isBallNowPresent;
    }

    // This method is now used for the upper sensor or for general-purpose classification
    public void classifyBalls(AdafruitSensorDriver.Reading upperReading,
                              AdafruitSensorDriver.Reading lowerReading) {
        upperBall = classifySingle(upperReading);
        lowerBall = classifySingle(lowerReading);
    }

    private BallColor classifySingle(AdafruitSensorDriver.Reading r) {
        if (r == null || r.clear < MIN_CLEAR_FOR_BALL) return BallColor.NONE;

        double red = r.red;
        double green = r.green;
        double blue = r.blue;

        // prevent false classification when all colors are similar
        double max = Math.max(red, Math.max(green, blue));
        double min = Math.min(red, Math.min(green, blue));
        if (max - min < 100) return BallColor.NONE;

        boolean redHigh = red >= COLOR_DOMINANCE_RATIO * green &&
                red >= COLOR_DOMINANCE_RATIO * blue;
        boolean greenHigh = green >= COLOR_DOMINANCE_RATIO * red &&
                green >= COLOR_DOMINANCE_RATIO * blue;

        if (greenHigh) return BallColor.GREEN;
        // For purple, you might check if blue is high or if red and blue dominate green.
        // I'm assuming your redHigh check correctly identifies purple.
        if (redHigh) return BallColor.PURPLE;
        return BallColor.NONE;
    }

    @Override
    public void execute() {
        executeEX();
        if (block){
            intakeBlocker.setPosition(0.56);
            intakePTO.setPosition(0.52);

        }else {
            intakeBlocker.setPosition(0.36);
            intakePTO.setPosition(0.43);
        }
        if (InTake){
            intakeMotor.update(-1);
        }else {
            intakeMotor.update(0);
        }

    }
}
