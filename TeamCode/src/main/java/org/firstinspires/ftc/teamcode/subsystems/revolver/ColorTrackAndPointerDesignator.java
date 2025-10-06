package org.firstinspires.ftc.teamcode.subsystems.revolver;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorTrackAndPointerDesignator {
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    boolean UPPRESSED = false;
    boolean DOWNPRESSED = false;
    boolean APRESSED = false;
    boolean LEFTPRESSED = false;
    boolean RIGHTPRESSED = false;
    double hue;
    public static int pointer = 0;
    final float[] hsvValues = new float[3];
    String curColor = "white";
    static String[] slotColor = {"white", "white", "white"};
    //-----------define colors--------------
    float greenMin = 100;
    float greenMax = 180;
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

    void updateColors() {
        String curColor = readColor();
        slotColor[pointer] = curColor;
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
        if (slotColor[pointer % 3].equals(Col)) {
            return pointer;
        }
        else if (slotColor[(pointer + 1) % 3].equals(Col)) {
            return (pointer + 1) % 3;
        } else if (slotColor[(pointer + 2) % 3].equals(Col)) {
            return (pointer + 2) % 3;
        }
        return pointer;
    }

    int findNearestBall() {
        if (slotColor[pointer].equals("green") || slotColor[pointer].equals("purple")) {
            return pointer;
        }
        else if (slotColor[(pointer + 1) % 3].equals("green") || slotColor[(pointer + 1) % 3].equals("purple")) {
            return (pointer + 1) % 3;
        } else if (slotColor[(pointer + 2) % 3].equals("green") || slotColor[(pointer + 2) % 3].equals("purple")) {
            return (pointer + 2) % 3;
        }
        return pointer;
    }

    boolean emptyAvailble() {
        if (slotColor[0].equals("white") || slotColor[1].equals("white") || slotColor[2].equals("white")) {
            return true;
        } else {
            return false;
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("hue", hue);
        telemetry.addData("curColor", curColor);
        telemetry.addData("pointer", pointer);
        telemetry.addData("slot1", slotColor[0]);
        telemetry.addData("slot2", slotColor[1]);
        telemetry.addData("slot3", slotColor[2]);
        telemetry.addData("Distance1 (cm)", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        setGain(1);
    }

    public void loop(boolean arrived) {
        curColor = readColor();
        if (arrived) {
            updateColors();
        }
    }
}