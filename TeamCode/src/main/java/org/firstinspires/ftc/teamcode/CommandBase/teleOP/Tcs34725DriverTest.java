//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.CommandBase.AdafruitSensorDriver;
//
//@TeleOp(name = "AdaColour Debug Test", group = "Sensor")
//public class Tcs34725DriverTest extends OpMode {
//
//    private AdafruitSensorDriver sensor;
//
//    @Override
//    public void init() {
//        // Name MUST match the XML: <TcsSensor name="tcs" .../>
//        sensor = hardwareMap.get(AdafruitSensorDriver.class, "LowerSensor");
//        sensor = hardwareMap.get(AdafruitSensorDriver.class, "UpperSensor");
//
//        telemetry.addLine("AdafruitSensorDriver init() complete");
//        telemetry.addData("Connected", sensor.isConnected());
//        telemetry.addData("ID (hex)", "0x%02X", sensor.readId());
//        telemetry.addData("Status (hex)", "0x%02X", sensor.readStatus());
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        AdafruitSensorDriver.Reading r = sensor.readCRGB();
//
//        telemetry.addLine("TCS34725 raw CRGB:");
//        telemetry.addData("Clear", r.clear);
//        telemetry.addData("Red",   r.red);
//        telemetry.addData("Green", r.green);
//        telemetry.addData("Blue",  r.blue);
//
//        telemetry.addLine();
//        telemetry.addData("Connected", sensor.isConnected());
//        telemetry.addData("ID (hex)", "0x%02X", sensor.readId());
//        telemetry.addData("Status (hex)", "0x%02X", sensor.readStatus());
//
//        telemetry.update();
//    }
//}