package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Revolver extends OpMode{
    //-----------------functions-------------
    int sign(double num){
        if (num > 0){
            return 1;
        }
        else if (num < 0){
            return -1;
        }
        return 0;
    }
    void setGain(){
        colorSens1.setGain(gain);
        colorSens2.setGain(gain);
        colorSens3.setGain(gain);
    }
    void pull_trigger(){
        //codeforpulltrigger
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
        if (pid.arrived){
            readColor(colorSens1, 0);
            readColor(colorSens2, 1);
            readColor(colorSens3, 2);
        }
    }
    void waitForPid(){
        while (!pid.arrived){
            continue;
        }
    }
    double findNearest360(double num) {
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
    void updateTelemetry(){
        telemetry.addData("gain", gain);
        telemetry.addData("hue", hue);
        telemetry.addData("pointer", pointer);
        telemetry.addData("target", pid.target);
        telemetry.addData("curPos", curPos);
        telemetry.addData("slot1", slotColor[0]);
        telemetry.addData("slot2", slotColor[1]);
        telemetry.addData("slot3", slotColor[2]);
        telemetry.addData("P", pid.PID_P);
        telemetry.addData("I", pid.PID_I);
        telemetry.addData("D", pid.PID_D);
        telemetry.addData("kP", pid.Kp);
        telemetry.addData("speed", pid.velo);
        telemetry.addData("Distance1 (cm)", ((DistanceSensor) colorSens1).getDistance(DistanceUnit.CM));
        telemetry.addData("Distance2 (cm)", ((DistanceSensor) colorSens2).getDistance(DistanceUnit.CM));
        telemetry.addData("Distance3 (cm)", ((DistanceSensor) colorSens3).getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    //---------------------Define stuff------------------
    DcMotor revSpin;
    DcMotor trigger;
    NormalizedColorSensor colorSens1;
    NormalizedColorSensor colorSens2;
    NormalizedColorSensor colorSens3;
    NormalizedRGBA colors;
    PID pid = new PID();
    //--------------------------variables------------------
    float gain = 2;
    boolean UPPRESSED = false;
    boolean DOWNPRESSED = false;
    boolean APRESSED = false;
    boolean LEFTPRESSED = false;
    boolean RIGHTPRESSED = false;
    int Manual = 1;
    double hue;
    int pointer = 0;
    double curPos = 0;
    String tarColor = "green";
    final float[] hsvValues = new float[3];
    double[] slotTarget = {0, 120, -120};
    String[] slotColor = {"white", "white", "white"};
    public enum revMode {
        CONTFIRE,
        AUTOIN,
        FIRECOLOR,
        HP
    }
    private revMode curMode = revMode.AUTOIN;
    //-----------define colors--------------
    float greenMin = 100;
    float greenMax = 180;
    float purpleMin = 200;
    float purpleMax = 299;
    //------------------------------init---------------------------------
    @Override
    public void init() {
        pid.init();
        pid.setCoefficients(1, 0, 0);
        pid.iMax = 0.3;
        pid.iRange = 0.5;
        pid.errorTol = 4;
        pid.dTol = 2;
        pid.target = 0;
        colorSens1 = hardwareMap.get(NormalizedColorSensor.class, "color1");
        colorSens2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        colorSens3 = hardwareMap.get(NormalizedColorSensor.class, "color3");
        revSpin = hardwareMap.get(DcMotor.class, "Spin");
        trigger = hardwareMap.get(DcMotor.class, "trig");
        revSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setGain();
    }
    //-------------------------------loop--------------------------------
    @Override
    public void loop() {
        //---------------Adjust target values------------
        curPos = revSpin.getCurrentPosition();
        //--------------------change gain per loop-------------------------
        if (gamepad1.dpad_up && !UPPRESSED){
            gain += 0.05;
            setGain();
        }
        else if (gamepad1.dpad_down && !DOWNPRESSED){
            gain -= 0.05;
            setGain();
        }
        //---------------------spin revolver---------------------------
        if (gamepad1.dpad_right && !RIGHTPRESSED){
            pointer = (pointer + 1) % 3;
        }
        else if (gamepad1.dpad_left && !LEFTPRESSED){
            pointer = (pointer + 2) % 3;;
        }
        //--------------------REVOVLER CONTROL------------------------
        //Run this part as a task in a whiletrue(hopefully)
        if (Manual == 1){
            //manually control everything -> incase stuff breaks
        }
        else{
            if (curMode == revMode.AUTOIN){
                //intake code
                pointer = findNearestWhite();
            }
            else if (curMode == revMode.CONTFIRE){
                if (pid.arrived) {
                    if (slotColor[pointer].equals("white")) {
                        pointer = findNearestBall();
                    }
                    else{
                        pull_trigger();
                    }
                }
            }
            else if (curMode == revMode.FIRECOLOR){
                if (slotColor[pointer].equals(tarColor)){
                    pull_trigger();
                }
                else{
                    pointer = findNearestColor(tarColor);
                    waitForPid();
                    pull_trigger();
                }
                curMode = revMode.AUTOIN;
            }
            else if (curMode == revMode.HP){
                if (emptyAvailble()) {
                    pointer = (findNearestWhite() + 1) % 3;
                }
                else{
                    curMode = revMode.AUTOIN;
                }
            }
        }
        //-------------------------------set target---------------------
        pid.target = findNearest360(curPos) + slotTarget[pointer];
        //-----------------------loop actions-------------------------
        pid.update(curPos);
        revSpin.setPower(pid.velo);

        APRESSED = gamepad1.a;
        UPPRESSED = gamepad1.dpad_up;
        DOWNPRESSED = gamepad1.dpad_down;
        LEFTPRESSED = gamepad1.dpad_left;
        RIGHTPRESSED = gamepad1.dpad_right;

        updateColors();

        updateTelemetry();
    }
}
