package org.firstinspires.ftc.teamcode.subsystems.revolver;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorTracker {
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    public boolean arrived = false;
    double hue;
    public static int pointer = 0;
    final float[] hsvValues = new float[3];
    String curColor = "white";
    static String[] slotColor = {"white", "white", "white"};
    //-----------define colors--------------
    float greenMin = 100;
    float greenMax = 160;
    float purpleMin = 200;
    float purpleMax = 310;
    // functions
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
    static int findNearestWhite() {
        if (slotColor[pointer].equals("white")) {
            return pointer;
        }
        else if (slotColor[(pointer + 1) % 3].equals("white")) {
            return (pointer + 1) % 3;
        } else if (slotColor[(pointer + 2) % 3].equals("white")) {
            return (pointer + 2) % 3;
        }
        return pointer;
    }

    int findNearestColor(String Col) {
        if (slotColor[(pointer + 1) % 3].equals(Col)) {
            return (pointer + 1) % 3;
        } else if (slotColor[(pointer + 2) % 3].equals(Col)) {
            return (pointer + 2) % 3;
        }
        return pointer;
    }
    boolean colorAvailable(String Col) {
        if (slotColor[0].equals(Col) || slotColor[1].equals(Col) || slotColor[2].equals(Col)) {
            return true;
        } else {
            return false;
        }
    }
    int findNearestBall() {
        if (slotColor[(pointer + 1) % 3].equals("green") || slotColor[(pointer + 1) % 3].equals("purple")) {
            return (pointer + 1) % 3;
        } else if (slotColor[(pointer + 2) % 3].equals("green") || slotColor[(pointer + 2) % 3].equals("purple")) {
            return (pointer + 2) % 3;
        }
        return pointer;
    }
    boolean emptyAvailable() {
        if (slotColor[0].equals("white") || slotColor[1].equals("white") || slotColor[2].equals("white")) {
            return true;
        } else {
            return false;
        }
    }
    boolean ballAvailable() {
        if (!slotColor[0].equals("white") || !slotColor[1].equals("white") || !slotColor[2].equals("white")) {
            return true;
        } else {
            return false;
        }
    }
    void removeFiredBall(int pointer) {
        slotColor[pointer] = "white";
    }
    void updateColors() {
        String curColor = readColor();
        slotColor[pointer] = curColor;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("hue", hue);
        telemetry.addData("curColor", curColor);
        telemetry.addData("Distance1 (cm)", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        dist = hardwareMap.get(DistanceSensor.class, "dist");
        setGain(2.5F);
    }

    public void loop() {
        curColor = readColor();
        if (arrived) {
            updateColors();
        }
    }
}