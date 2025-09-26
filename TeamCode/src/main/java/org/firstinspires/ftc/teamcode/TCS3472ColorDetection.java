package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

@TeleOp(name = "TCS3472 Color Sensor", group = "Sensor")
public class TCS3472ColorDetection extends LinearOpMode {
    private ModernRoboticsI2cColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor");

        colorSensor.enableLed(true);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press START to detect colors");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read color values
            float red = colorSensor.red();
            float green = colorSensor.green();
            float blue = colorSensor.blue();
            float alpha = colorSensor.alpha();

            // Calculate dominant color
            String dominantColor = getDominantColor(red, green, blue);

            // Display color values on telemetry
            telemetry.addData("Raw Values", "R:%.0f G:%.0f B:%.0f A:%.0f", red, green, blue, alpha);
            telemetry.addData("Dominant Color", dominantColor);
            telemetry.addData("Status", "Detecting colors...");
            telemetry.update();

            sleep(100);
        }
    }

    private String getDominantColor(float red, float green, float blue) {
        if (red > green && red > blue && red > 50) {
            return "RED";
        } else if (green > red && green > blue && green > 50) {
            return "GREEN";
        } else if (blue > red && blue > green && blue > 50) {
            return "BLUE";
        } else if (red > 50 && green > 50 && blue > 50) {
            return "WHITE";
        } else if (red < 10 && green < 10 && blue < 10) {
            return "BLACK";
        } else {
            return "OTHER";
        }
    }
}