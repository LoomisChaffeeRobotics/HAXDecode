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
        if (((DistanceSensor) colorSens).getDistance(DistanceUnit.CM) < 3){
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
        if (pid.velo <= stopDev){
            slotColor[pointer] = detectColor();
        }
    }
    //---------------------Define stuff------------------
    DcMotor revSpin;
    NormalizedColorSensor colorSens;
    NormalizedRGBA colors;
    PID pid = new PID();
    //--------------------------variables------------------
    float gain = 1;
    boolean UPPRESSED = false;
    boolean DOWNPRESSED = false;
    boolean APRESSED = false;
    double hue;
    int pointer = 0;
    double curPos = 0;
    double fullCircle = 1400;
    String curColor = "white";
    final float[] hsvValues = new float[3];
    double[] slotTarget = {0, 120, 240};
    double stopDev = 0.15;
    String[] slotColor = {"white", "white", "white"};
    //-----------define colors--------------
    float greenMin = 0;
    float greenMax = 0;
    float purpleMin = 0;
    float purpleMax = 0;
    float target = 0;
    //------------------------------init---------------------------------
    @Override
    public void init() {
        pid.init();
        pid.setCoefficients(0, 0, 0);
        pid.iMax = 0.3;
        pid.iRange = 0.5;
        pid.errorTol = 4;
        pid.target = target;
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
        if (gamepad1.dpad_left){
            pointer = (pointer + 1) % 3;
        }
        else if (gamepad1.dpad_right){
            pointer = (pointer - 1) % 3;
        }
        //-------------------------------end cycle---------------------
        if ( Math.abs(curPos - slotTarget[pointer]) > (fullCircle / 2) ) {
            pid.target = sign(slotTarget[pointer]) * fullCircle + slotTarget[pointer];
        }

        pid.update(curPos);
        revSpin.setPower(pid.velo);

        APRESSED = gamepad1.a;
        UPPRESSED = gamepad1.dpad_up;
        DOWNPRESSED = gamepad1.dpad_down;

        updateColors();

        telemetry.addData("gain", gain);
        telemetry.addData("hue", hue);
        telemetry.addData("color", curColor);
        telemetry.addData("pointer", pointer);
        telemetry.addData("target", target);
        telemetry.addData("Distance (cm)", ((DistanceSensor) colorSens).getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
