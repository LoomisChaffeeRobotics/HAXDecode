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
    public boolean arrived;
    NormalizedColorSensor colorSensor;
    DistanceSensor colorDist;
    NormalizedRGBA colors;
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
    double distance;
    // functions
    public void setGain(float gain) {
        colorSensor.setGain(gain);
    }
    String readColor() {
        colors = colorSensor.getNormalizedColors(); // Get colors
        distance = colorDist.getDistance(DistanceUnit.CM);

        Color.colorToHSV(colors.toColor(), hsvValues);
        hue = hsvValues[0]; //Read current color in HSV

        if (distance < 7) {
            if (hue > greenMin && hue < greenMax) {
                return "green"; // hue is within green range --> return current color is green
            } else if (hue > purpleMin && hue < purpleMax) {
                return "purple"; //hue is within purple range --> return current color is purple
            } else {
                return "white"; //else return white
            }
        } else {
            return "white";
        }
    }
    static int findNearestWhite() {
        if (slotColor[pointer].equals("white")) {
            return pointer; // if we are at empty slot, we return current location
        }
        else if (slotColor[(pointer + 1) % 3].equals("white")) {
            return (pointer + 1) % 3; // if next slow is empty we return the next location
        } else if (slotColor[(pointer + 2) % 3].equals("white")) {
            return (pointer + 2) % 3; // if next slow is empty we return the next location
        }
        return pointer; // if nothing is white, we return current locations
    }

    int findNearestColor(String Col) {
        if (slotColor[pointer].equals(Col)){
            return pointer;
        }
        if (slotColor[(pointer + 1) % 3].equals(Col)) {
            return (pointer + 1) % 3;
        } else if (slotColor[(pointer + 2) % 3].equals(Col)) {
            return (pointer + 2) % 3;
        }
        return pointer;
    }
    boolean colorAvailable(String Col) {
        if(slotColor[0].equals(Col) || slotColor[1].equals(Col) || slotColor[2].equals(Col)) {
            return true;
        } else {
            return false;
        }
    } // We get either "white", "green", "purple" as input parameter. We retur if is any slots are in that color
    int findNearestBall() {
        if (slotColor[pointer].equals("green") || slotColor[pointer].equals("purple")){
            return pointer;
        }
        else if (slotColor[(pointer + 1) % 3].equals("green") || slotColor[(pointer + 1) % 3].equals("purple")) {
            return (pointer + 1) % 3;
        } else if (slotColor[(pointer + 2) % 3].equals("green") || slotColor[(pointer + 2) % 3].equals("purple")) {
            return (pointer + 2) % 3;
        }
        return pointer;
    } //Shouldn't we check if our current position has a ball too? (Depending on how this ftn is used)
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
    }//Returns if there is a ball in the drum or not
    void removeFiredBall(int pointer) {
        slotColor[pointer] = "white";
    }
    void addGreen(int pointer) {
        slotColor[pointer] = "green";
    }
    void addPurple(int pointer) {
        slotColor[pointer] = "purple";
    }
    //resets the slot's color after shooting
    void updateColors() {
        String curColor = readColor();
        if (colorAvailable("white")){
            slotColor[pointer] = curColor;
        }
    }
    //Read the color at this moment, input the color into the array/list

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("hue", hue);
        telemetry.addData("curColor", curColor);
        telemetry.addData("colList0", slotColor[0]);
        telemetry.addData("colList1", slotColor[1]);
        telemetry.addData("colList2", slotColor[2]);
        telemetry.addData("Distance1 (cm)", distance);
    }
    //Add data to telemetry

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorDist = (DistanceSensor) colorSensor;
        slotColor[0] = "white";
        slotColor[1] = "white";
        slotColor[2] = "white";
//        dist = hardwareMap.get(DistanceSensor.class, "dist");
        setGain(2.5F);
    }

    public void loop(boolean AArrived, boolean intaking) {
        arrived = AArrived;
        curColor = readColor();
        if (AArrived && intaking) {
            updateColors();
        }
    }
    //update consistently
}