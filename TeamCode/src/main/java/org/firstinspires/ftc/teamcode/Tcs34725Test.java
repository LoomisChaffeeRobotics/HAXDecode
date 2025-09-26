package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "TCS34725 Test", group = "Test")
public class Tcs34725Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Match the Robot Config name for your I2C device (see section below)
        I2cDeviceSynch i2c = hardwareMap.get(I2cDeviceSynch.class, "tcs34725");

        Tcs34725 sensor = new Tcs34725(i2c);

        // Optional: tweak integration time / gain for your lighting
        sensor.setIntegrationTimeMs(154);               // ~150 ms
        sensor.setGain(Tcs34725.Gain.GAIN_4X);          // try 1X/4X/16X/60X

        telemetry.addLine("TCS34725 initialized. Press PLAY.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            Tcs34725.Rgbc rgbc = sensor.readNextRgbc();

            telemetry.addData("C", rgbc.c);
            telemetry.addData("R", rgbc.r);
            telemetry.addData("G", rgbc.g);
            telemetry.addData("B", rgbc.b);

            // Simple normalized values (0..1 relative to clear)
            telemetry.addData("nR", "%.3f", rgbc.normR());
            telemetry.addData("nG", "%.3f", rgbc.normG());
            telemetry.addData("nB", "%.3f", rgbc.normB());

            // A naive "dominant color" example for students
            String dom = "none";
            if (rgbc.r > rgbc.g && rgbc.r > rgbc.b) dom = "RED";
            else if (rgbc.g > rgbc.r && rgbc.g > rgbc.b) dom = "GREEN";
            else if (rgbc.b > rgbc.r && rgbc.b > rgbc.g) dom = "BLUE";
            telemetry.addData("Dominant", dom);

            telemetry.update();
        }
    }
}

