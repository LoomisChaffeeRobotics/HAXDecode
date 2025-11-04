package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class colorTesting extends OpMode {
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    Telemetry telemetry;
    double hue;
    final float[] hsvValues = new float[3];
    String curColor = "white";
    static String[] slotColor = {"white", "white", "white"};
    public static int pointer = 0;
    float greenMin = 100;
    float greenMax = 180;
    float purpleMin = 200;
    float purpleMax = 310;
    float gain = 1;
    public void setGain(float gain) {
        colorSensor.setGain(gain);
    }
    String readColor() {
        colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        hue = hsvValues[0];
        if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) > 8) {
            return "white";
        } else if (hue > greenMin && hue < greenMax) {
            return "green";
        } else if (hue > purpleMin && hue < purpleMax) {
            return "purple";
        } else {
            return "white";
        }
    }
    void updateColors() {
        String curColor = readColor();
        slotColor[pointer] = curColor;
    }
    public void updateTelemetry() {
        telemetry.addData("hue", hue);
        telemetry.addData("curColor", curColor);
        telemetry.addData("gain", gain);
        telemetry.addData("Distance1 (cm)", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        setGain(1);
    }
    @Override
    public void loop() {
        if (gamepad1.dpad_up && !gamepad1.dpadUpWasPressed()){
            gain += 0.05;
            setGain(gain);
        }
        else if (gamepad1.dpad_down && !gamepad1.dpadDownWasPressed()){
            gain -= 0.05;
            setGain(gain);
        }
        curColor = readColor();
    }
}
