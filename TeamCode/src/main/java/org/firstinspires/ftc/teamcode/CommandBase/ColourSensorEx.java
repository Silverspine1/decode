package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.CommandBase.AdafruitSensorDriver;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ColourSensorEx {

    private AdafruitSensorDriver sensor;
    private ExecutorService executor;

    public void initSensor(String deviceName, HardwareMap hardwareMap) {
        sensor = hardwareMap.get(AdafruitSensorDriver.class, deviceName);
        this.executor = Executors.newFixedThreadPool(2);

    }

    public boolean isReady() {
        return sensor != null && sensor.isConnected();
    }

    public AdafruitSensorDriver.Reading read() {
        if (sensor == null) return null;
        return sensor.readCRGB();
    }

    public AdafruitSensorDriver getRawSensor() {
        return sensor;
    }
}