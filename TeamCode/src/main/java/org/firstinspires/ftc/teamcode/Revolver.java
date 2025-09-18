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
    String detectColor() {
        colors = colorSens.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        hue = hsvValues[0];
        if (((DistanceSensor) colorSens).getDistance(DistanceUnit.CM) > 8){
            curColor = "white";
        }
        else if (hue > greenMin && hue < greenMax){
            curColor = "green";
        }
        else if (hue > purpleMin && hue < purpleMax){
            curColor = "purple";
        }
        else{
            curColor = "white";
        }
        return curColor;
    }
    void shiftTarget(double times){
        double ticks = fullCircle * times;
        slotTarget[0] += ticks;
        slotTarget[1] += ticks;
        slotTarget[2] += ticks;
    }
    double calcTimes(){
        if (sign(curPos) == 1){
            return Math.floor(curPos / fullCircle);
        }
        else if (sign(curPos) == -1){
            return Math.ceil(curPos / fullCircle);
        }
        return 0;
    }
    void updateColors(){
        if ((pid.velo < 0.2) && pid.arrived && (slotColor[pointer].equals("white"))){
            slotColor[pointer] = detectColor();
        }
    }
    //---------------------Define stuff------------------
    DcMotor revSpin;
    NormalizedColorSensor colorSens;
    NormalizedRGBA colors;
    PID pid = new PID();
    //--------------------------variables------------------
    float gain = 2;
    boolean UPPRESSED = false;
    boolean DOWNPRESSED = false;
    boolean APRESSED = false;
    boolean LEFTPRESSED = false;
    boolean RIGHTPRESSED = false;
    double hue;
    int pointer = 0;
    double curPos = 0;
    double fullCircle = 1400;
    String curColor = "white";
    final float[] hsvValues = new float[3];
    double[] slotTarget = {0, 120, 240};
    String[] slotColor = {"white", "white", "white"};
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
        colorSens = hardwareMap.get(NormalizedColorSensor.class, "color");
        revSpin = hardwareMap.get(DcMotor.class, "Spin");
        revSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorSens.setGain(gain);
        if (colorSens instanceof SwitchableLight) {
            ((SwitchableLight)colorSens).enableLight(true);
        }
    }
    //-------------------------------loop--------------------------------
    @Override
    public void loop() {
        //---------------Adjust target values------------
        curPos = revSpin.getCurrentPosition();
        shiftTarget(calcTimes());
        //--------------------change gain per loop-------------------------
        if (gamepad1.dpad_up && !UPPRESSED){
            gain += 0.05;
            colorSens.setGain(gain);
        }
        else if (gamepad1.dpad_down && !DOWNPRESSED){
            gain -= 0.05;
            colorSens.setGain(gain);
        }
        //-----------------------toggle lights-----------------------
        if (gamepad1.a && !APRESSED){
            if (colorSens instanceof SwitchableLight) {
                SwitchableLight light = (SwitchableLight)colorSens;
                light.enableLight(!light.isLightOn());
            }
        }
        //---------------------spin revolver---------------------------
        if (gamepad1.dpad_right && !RIGHTPRESSED){
            pointer = (pointer + 1) % 3;
        }
        else if (gamepad1.dpad_left && !LEFTPRESSED){
            pointer = (pointer - 1 + 3) % 3;;
        }
        //-------------------------------end cycle---------------------
        if ( Math.abs(curPos - slotTarget[pointer]) > (fullCircle / 2) ) {
            pid.target = sign(slotTarget[pointer]) * fullCircle + slotTarget[pointer];
        }

        pid.update(curPos);
        revSpin.setPower(pid.velo);
        pid.target = slotTarget[pointer];

        APRESSED = gamepad1.a;
        UPPRESSED = gamepad1.dpad_up;
        DOWNPRESSED = gamepad1.dpad_down;
        LEFTPRESSED = gamepad1.dpad_left;
        RIGHTPRESSED = gamepad1.dpad_right;

        updateColors();
        detectColor();

        telemetry.addData("gain", gain);
        telemetry.addData("hue", hue);
        telemetry.addData("color", curColor);
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
        telemetry.addData("Distance (cm)", ((DistanceSensor) colorSens).getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
