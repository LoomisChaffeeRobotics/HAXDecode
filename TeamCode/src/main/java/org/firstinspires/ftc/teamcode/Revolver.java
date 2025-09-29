package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.revolver.ColorTrackAndPointerDesignator;
import org.firstinspires.ftc.teamcode.subsystems.revolver.PointerControl;

//@TeleOp
public class Revolver {
    ColorTrackAndPointerDesignator pointerDesignator = new ColorTrackAndPointerDesignator();
    PointerControl controller = new PointerControl();
    int pointer = pointerDesignator.pointer;
    public enum revMode {
        CONTFIRE,
        AUTOIN,
        FIRECOLOR,
        HP
    }
    private revMode curMode = revMode.AUTOIN;

    //------------------------------init---------------------------------
//    @Override
    public void init(HardwareMap hardwareMap) {
        pointerDesignator.init(hardwareMap);
    }
    //-------------------------------loop--------------------------------
//    @Override
    public void loop(Gamepad gamepad1, Telemetry t) {
        pointerDesignator.loop();
        controller.loop(pointer);
        //--------------------change gain per loop-------------------------
        if (gamepad1.dpad_up && !gamepad1.dpadUpWasPressed()){
            pointerDesignator.gain += 0.05;
            pointerDesignator.setGain();
        }
        else if (gamepad1.dpad_down && !gamepad1.dpadDownWasPressed()){
            pointerDesignator.gain -= 0.05;
            pointerDesignator.setGain();
        }
        //---------------------spin revolver---------------------------

        if (gamepad1.dpad_right && !gamepad1.dpadRightWasPressed()){
            pointer = (pointer + 1) % 3;
        }
        else if (gamepad1.dpad_left && !gamepad1.dpadLeftWasPressed()){
            pointer = (pointer + 2) % 3;;
        }
        //--------------------REVOVLER CONTROL------------------------
        //Run this part as a task in a whiletrue(hopefully)
        /*
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
        */
        pointerDesignator.updateTelemetry(t);
        controller.updateTelemetry(t);
    }
}
