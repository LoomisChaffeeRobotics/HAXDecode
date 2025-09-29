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
    NormalizedColorSensor colorSens1;
    NormalizedColorSensor colorSens2;
    NormalizedColorSensor colorSens3;
    NormalizedRGBA colors;
    public float gain = 2;
    boolean UPPRESSED = false;
    boolean DOWNPRESSED = false;
    boolean APRESSED = false;
    boolean LEFTPRESSED = false;
    boolean RIGHTPRESSED = false;
    double hue;
    public int pointer = 0;
    final float[] hsvValues = new float[3];
    String[] slotColor = {"white", "white", "white"};
    //-----------define colors--------------
    float greenMin = 100;
    float greenMax = 180;
    float purpleMin = 200;
    float purpleMax = 299;
    // functions
    public void setGain(){
        colorSens1.setGain(gain);
        colorSens2.setGain(gain);
        colorSens3.setGain(gain);
    }
    void readColor(NormalizedColorSensor colorS, int slot){
        colors = colorSens1.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        hue = hsvValues[0];
        if (((DistanceSensor) colorS).getDistance(DistanceUnit.CM) > 8){
            slotColor[slot] = "white";
        }
        else if (hue > greenMin && hue < greenMax){
            slotColor[slot] = "green";
        }
        else if (hue > purpleMin && hue < purpleMax){
            slotColor[slot] = "purple";
        }
        else{
            slotColor[slot] = "white";
        }
    }
    void updateColors() {
        readColor(colorSens1, 0);
        readColor(colorSens2, 1);
        readColor(colorSens3, 2);
    }
    public double findNearest360(double num) {
        return Math.round(num / 360.0) * 360;
    }
    int findNearestWhite(){
        if (slotColor[(pointer + 1) % 3].equals("white")){
            return (pointer + 1) % 3;
        }
        else if (slotColor[(pointer + 2) % 3].equals("white")){
            return (pointer + 2) % 3;
        }
        return pointer;
    }
    int findNearestColor(String Col){
        if (slotColor[(pointer + 1) % 3].equals(Col)){
            return (pointer + 1) % 3;
        }
        else if (slotColor[(pointer + 2) % 3].equals(Col)){
            return (pointer + 2) % 3;
        }
        return pointer;
    }
    int findNearestBall(){
        if (slotColor[(pointer + 1) % 3].equals("green") || slotColor[(pointer + 1) % 3].equals("purple")){
            return (pointer + 1) % 3;
        }
        else if (slotColor[(pointer + 2) % 3].equals("green") || slotColor[(pointer + 2) % 3].equals("purple")){
            return (pointer + 2) % 3;
        }
        return pointer;
    }
    boolean emptyAvailble(){
        if (slotColor[0].equals("white") || slotColor[1].equals("white") || slotColor[2].equals("white")){
            return true;
        }
        else{
            return false;
        }
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("gain", gain);
        telemetry.addData("hue", hue);
        telemetry.addData("pointer", pointer);
        telemetry.addData("slot1", slotColor[0]);
        telemetry.addData("slot2", slotColor[1]);
        telemetry.addData("slot3", slotColor[2]);
        telemetry.addData("Distance1 (cm)", ((DistanceSensor) colorSens1).getDistance(DistanceUnit.CM));
        telemetry.addData("Distance2 (cm)", ((DistanceSensor) colorSens2).getDistance(DistanceUnit.CM));
        telemetry.addData("Distance3 (cm)", ((DistanceSensor) colorSens3).getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    public void init(HardwareMap hardwareMap) {
        colorSens1 = hardwareMap.get(NormalizedColorSensor.class, "color1");
        colorSens2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        colorSens3 = hardwareMap.get(NormalizedColorSensor.class, "color3");
        setGain();
    }
    public void loop() {
        updateColors();
    }
}
