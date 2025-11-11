package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Minimal Adafruit TCS34725 driver for FTC, modeled on Adafruit_TCS34725.
 */
@I2cDeviceType
@DeviceProperties(
        name = "Adafruit TCS34725 (Custom)",
        xmlTag = "TcsSensor",              // KEEP THIS so existing XML <TcsSensor ...> still works
        description = "Custom Adafruit Color Sensor"
)
public class AdafruitSensorDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // 7-bit I2C address of TCS34725
    private static final I2cAddr I2C_ADDR = I2cAddr.create7bit(0x29);

    // Command bit
    private static final int CMD = 0x80;

    // Registers (datasheet addresses, before OR-ing with CMD)
    private static final int REG_ENABLE  = 0x00;
    private static final int REG_ATIME   = 0x01;
    private static final int REG_ID      = 0x12;
    private static final int REG_STATUS  = 0x13;
    private static final int REG_CDATAL  = 0x14; // clear low, then R,G,B

    // ENABLE bits
    private static final int ENABLE_PON  = 0x01;
    private static final int ENABLE_AEN  = 0x02;

    // Gain values (same as Adafruit)
    public static final int GAIN_1X  = 0x00;
    public static final int GAIN_4X  = 0x01;
    public static final int GAIN_16X = 0x02;
    public static final int GAIN_60X = 0x03;

    // Integration time register value, default 50 ms (0xEB from Adafruit lib)
    private int integrationTime = 0xEB;
    private int gain            = GAIN_16X;

    public static class Reading {
        public int clear, red, green, blue;
    }

    // Public constructor used by the SDK
    public AdafruitSensorDriver(I2cDeviceSynch deviceClient) {
        this(deviceClient, true);
    }

    // Internal constructor
    public AdafruitSensorDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2C_ADDR);
        this.deviceClient.setLogging(false);
        registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize() {
        // Check that the chip responds with a known ID
        int id = read8(REG_ID);
        if (id != 0x4D && id != 0x44 && id != 0x10) {
            return false;
        }

        // Set integration time and gain
        write8(REG_ATIME, integrationTime);
        write8(0x0F, gain); // CONTROL register

        // Enable RGBC
        enableChip();

        // Dummy read to start conversions
        read8(REG_STATUS);

        return true;
    }

    private void enableChip() {
        write8(REG_ENABLE, ENABLE_PON);
        sleepMs(3);
        write8(REG_ENABLE, ENABLE_PON | ENABLE_AEN);

        int delayMs = ((256 - integrationTime) * 12) / 5 + 1;
        sleepMs(delayMs);
    }

    public boolean isConnected() {
        int id = read8(REG_ID);
        return (id == 0x4D || id == 0x44 || id == 0x10);
    }

    public int readId() {
        return read8(REG_ID);
    }

    public int readStatus() {
        return read8(REG_STATUS);
    }

    public Reading readCRGB() {
        byte[] buf = deviceClient.read(CMD | REG_CDATAL, 8);

        Reading r = new Reading();
        r.clear = ((buf[1] & 0xFF) << 8) | (buf[0] & 0xFF);
        r.red   = ((buf[3] & 0xFF) << 8) | (buf[2] & 0xFF);
        r.green = ((buf[5] & 0xFF) << 8) | (buf[4] & 0xFF);
        r.blue  = ((buf[7] & 0xFF) << 8) | (buf[6] & 0xFF);
        return r;
    }


    private void write8(int reg, int value) {
        deviceClient.write8(CMD | reg, (byte) value);
    }

    private int read8(int reg) {
        return deviceClient.read8(CMD | reg) & 0xFF;
    }

    private void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) { }
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit TCS34725 Color Sensor";
    }
}