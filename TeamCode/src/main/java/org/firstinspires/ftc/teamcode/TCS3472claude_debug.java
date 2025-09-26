package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TCS3472 Color Sensor Test")
public class TCS3472claude_debug extends LinearOpMode {

    // TCS3472 I2C address (7-bit, typically 0x29)
    private static final I2cAddr SENSOR_ADDRESS = I2cAddr.create7bit(0x29);

    // Register addresses
    private static final int COMMAND_BIT = 0x80;
    private static final int ENABLE = 0x00;
    private static final int ATIME = 0x01;
    private static final int CONTROL = 0x0F;
    private static final int CDATA = 0x14; // Clear data low byte, next 7 registers are color data

    private I2cDeviceSynch sensor;

    @Override
    public void runOpMode() {
        // Initialize the I2C device
        sensor = new I2cDeviceSynchImplOnSimple(hardwareMap.i2cDeviceSynch.get("colorSensor"), false);
        sensor.engage();

        // Power on and enable RGBC
        sensor.write8(COMMAND_BIT | ENABLE, 0x03); // Power on + RGBC enable
        sleep(10);

        // Set integration time (atime, lower = longer, here 0xD5 = 24ms)
        sensor.write8(COMMAND_BIT | ATIME, 0xD5);

        // Set gain (CONTROL register, 0x01 = 4x gain)
        sensor.write8(COMMAND_BIT | CONTROL, 0x01);

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            int c = read16(CDATA);
            int r = read16(CDATA + 2);
            int g = read16(CDATA + 4);
            int b = read16(CDATA + 6);

            telemetry.addData("Clear", c);
            telemetry.addData("Red", r);
            telemetry.addData("Green", g);
            telemetry.addData("Blue", b);

            String detectedColor = detectColor(r, g, b);
            telemetry.addData("Detected Color", detectedColor);

            telemetry.update();
            sleep(100);
        }
        sensor.close();
    }

    // Helper to read 16-bit values (little-endian)
    private int read16(int reg) {
        byte[] data = sensor.read(COMMAND_BIT | reg, 2);
        return ((data[1] & 0xFF) << 8) | (data[0] & 0xFF);
    }

    // Simple color detection algorithm
    private String detectColor(int r, int g, int b) {
        if (r > g && r > b) {
            return "Red";
        } else if (g > r && g > b) {
            return "Green";
        } else if (b > r && b > g) {
            return "Blue";
        } else {
            return "Unknown";
        }
    }
}
