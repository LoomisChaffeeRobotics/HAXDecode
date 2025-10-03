package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.revolver.ColorTrackAndPointerDesignator;
import org.firstinspires.ftc.teamcode.subsystems.revolver.PointerControl;

@TeleOp
public class Main extends OpMode {
    PointerControl pController = new PointerControl();
    //public static int pointer = 0;
    public static float gain = 1;
    @Override
    public void init() {
        pController.init(hardwareMap);
    }
    @Override
    public void loop() {
        if (gamepad1.dpad_up && !gamepad1.dpadUpWasPressed()){
            gain += 0.05;
            pController.setGain(gain);
        }
        else if (gamepad1.dpad_down && !gamepad1.dpadDownWasPressed()){
            gain -= 0.05;
            pController.setGain(gain);
        }
        //---------------------spin revolver---------------------------
        //loopActions
        pController.update();
    }
}
