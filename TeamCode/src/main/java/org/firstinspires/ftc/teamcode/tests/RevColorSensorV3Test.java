package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "REV Color Sensor V3 Test", group = "Sensor")
public class RevColorSensorV3Test extends LinearOpMode {

    private ColorSensor colorSensor;
    private NormalizedColorSensor normalizedColorSensor;

    @Override
    public void runOpMode() {
        // Initialize the REV Color Sensor V3
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        // Enable LED for better readings
        colorSensor.enableLed(true);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Sensor", "REV Color Sensor V3 (TCS34725)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read raw RGB values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();

            // Read normalized values (0-1)
            NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();

            // Convert to HSV for color detection
            float[] hsvValues = new float[3];
            android.graphics.Color.RGBToHSV(
                    (int)(colors.red * 255),
                    (int)(colors.green * 255),
                    (int)(colors.blue * 255),
                    hsvValues
            );

            // Display telemetry
            telemetry.addLine("=== REV Color Sensor V3 ===");
            telemetry.addData("Red", "%d (%.3f)", red, colors.red);
            telemetry.addData("Green", "%d (%.3f)", green, colors.green);
            telemetry.addData("Blue", "%d (%.3f)", blue, colors.blue);
            telemetry.addData("Alpha", "%d (%.3f)", alpha, colors.alpha);

            telemetry.addLine("--- HSV Values ---");
            telemetry.addData("Hue", "%.1f°", hsvValues[0]);
            telemetry.addData("Saturation", "%.1f%%", hsvValues[1] * 100);
            telemetry.addData("Value", "%.1f%%", hsvValues[2] * 100);

            // Detect color based on hue
            String colorName = getColorName(hsvValues[0]);
            telemetry.addData("Detected Color", colorName);

            telemetry.update();
            sleep(50);
        }

        // Clean up
        colorSensor.enableLed(false);
    }

    private String getColorName(float hue) {
        // Hue ranges for different colors (adjust based on your testing)
        if (hue < 30 || hue > 330) return "RED";
        else if (hue >= 30 && hue < 90) return "YELLOW";
        else if (hue >= 90 && hue < 150) return "GREEN";
        else if (hue >= 150 && hue < 210) return "CYAN";
        else if (hue >= 210 && hue < 270) return "BLUE";
        else if (hue >= 270 && hue < 330) return "MAGENTA";
        else return "UNKNOWN";
    }
}
