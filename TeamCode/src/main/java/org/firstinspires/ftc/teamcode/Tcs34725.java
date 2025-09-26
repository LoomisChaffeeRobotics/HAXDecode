
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

// The class name 'Tcs34725' now matches the filename 'Tcs34725.java'
public class Tcs34725 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements HardwareDevice {

    // 7-bit I2C address for TCS34725
    public static final I2cAddr I2C_ADDRESS = I2cAddr.create7bit(0x29);

    // Command bit to OR with register address
    private static final int CMD = 0x80;

    // Registers
    private static final int ENABLE  = 0x00;
    private static final int ATIME   = 0x01;
    private static final int CONTROL = 0x0F;
    private static final int ID      = 0x12;
    private static final int STATUS  = 0x13;
    private static final int CDATA   = 0x14; // order: C,R,G,B (16-bit each, little-endian)

    // ENABLE bits
    private static final int PON = 0x01;
    private static final int AEN = 0x02;

    public enum Gain {
        GAIN_1X(0x00),
        GAIN_4X(0x01),
        GAIN_16X(0x02),
        GAIN_60X(0x03);
        public final int code;
        Gain(int c) { this.code = c; }
    }

    private int integrationTimeMs = 154; // default (~ATIME 0xD5)
    private Gain gain = Gain.GAIN_4X;

    public static class Rgbc {
        public final int r, g, b, c;
        public Rgbc(int r, int g, int b, int c) {
            this.r = r; this.g = g; this.b = b; this.c = c;
        }
        /** Simple normalized 0..1 (guarding divide by zero). */
        public float normR() { return c > 0 ? (float)r / (float)c : 0f; }
        public float normG() { return c > 0 ? (float)g / (float)c : 0f; }
        public float normB() { return c > 0 ? (float)b / (float)c : 0f; }
    }

    public Tcs34725(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2C_ADDRESS);

        // Set a read window that covers STATUS through BDATAH (0x13..0x1B)
        ReadWindow window = new ReadWindow(CMD | STATUS, (0x1B - 0x13) + 1, ReadMode.REPEAT);
        this.deviceClient.setReadWindow(window);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        // Read ID just to validate presence (not strictly required)
        int id = read8(ID);

        // Power-on then enable RGBC
        write8(ENABLE, PON);
        sleepMs(3);
        write8(ENABLE, PON | AEN);

        // Default integration time & gain
        setIntegrationTimeMs(integrationTimeMs);
        setGain(gain);

        // Wait one integration period for first valid sample
        sleepMs(integrationTimeMs + 10);
        return true;
    }

    /** ms must be within ~2.4..614.4ms. ATIME = 256 - (ms/2.4). */
    public void setIntegrationTimeMs(int ms) {
        if (ms < 3) ms = 3;
        if (ms > 614) ms = 614;
        this.integrationTimeMs = ms;
        int atime = 256 - Math.round(ms / 2.4f);
        if (atime < 0) atime = 0;
        if (atime > 255) atime = 255;
        write8(ATIME, atime & 0xFF);
        // give sensor time to apply
        sleepMs(3);
    }

    public void setGain(Gain gain) {
        this.gain = gain;
        write8(CONTROL, gain.code & 0x03);
        sleepMs(3);
    }

    /** Blocking read of RGBC (16-bit each), in register order C,R,G,B. */
    public Rgbc readRgbc() {
        // Read 8 bytes starting at CDATA (little-endian words)
        byte[] buf = deviceClient.read(CMD | CDATA, 8);
        int c = u16(buf[0], buf[1]);
        int r = u16(buf[2], buf[3]);
        int g = u16(buf[4], buf[5]);
        int b = u16(buf[6], buf[7]);
        return new Rgbc(r, g, b, c);
    }

    /** Returns true if a new RGBC sample is ready (STATUS.AVALID). */
    public boolean isDataValid() {
        int status = read8(STATUS);
        return (status & 0x01) != 0; // AVALID bit
    }

    /** Convenience: wait for next sample (up to ~2x integration time). */
    public Rgbc readNextRgbc() {
        long timeoutMs = Math.max(50, 2L * integrationTimeMs);
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < timeoutMs) {
            if (isDataValid()) break;
            sleepMs(2);
        }
        return readRgbc();
    }

    private int read8(int reg) {
        return deviceClient.read8(CMD | reg) & 0xFF;
    }

    private void write8(int reg, int val) {
        deviceClient.write8(CMD | reg, val & 0xFF);
    }

    private static int u16(byte lo, byte hi) {
        return ((hi & 0xFF) << 8) | (lo & 0xFF);
    }

    private static void sleepMs(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "TCS34725 RGBC Color Sensor";
    }

    @Override
    public void close() {
        super.close();
    }
}