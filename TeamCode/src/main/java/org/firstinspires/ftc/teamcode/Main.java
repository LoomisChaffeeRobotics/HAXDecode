package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.revolver.ColorTrackAndPointerDesignator;
import org.firstinspires.ftc.teamcode.subsystems.revolver.PointerControl;

@TeleOp
public class Main extends OpMode {
    //public static int pointer = 0;
    public static float gain = 1;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    PointerControl pController = new PointerControl();
    roller rollerControl = new roller();
    @Override
    public void init() {
        pController.init(hardwareMap);
        rollerControl.init();
    }
    @Override
    public void loop() {
        //-------------------gamepad actions--------------------------
        //changing the gain
        if (gamepad1.dpad_up && !gamepad1.dpadUpWasPressed()){
            gain += 0.05;
            pController.setGain(gain);
        }
        else if (gamepad1.dpad_down && !gamepad1.dpadDownWasPressed()){
            gain -= 0.05;
            pController.setGain(gain);
        }
        //intake
        if(gamepad1.a){
            rollerControl.rollerMode = roller.RM.INTAKE;
        }
        else if(gamepad1.b){
            rollerControl.rollerMode = roller.RM.OUTTAKE;
        }
        else{
            rollerControl.rollerMode = roller.RM.IDLE;
        }
        //shooting&revolver
        if (gamepad1.cross){
            pController.curMode = PointerControl.revMode.CONTFIRE;
        }
        else if (gamepad1.left_bumper && !gamepad1.leftBumperWasPressed()){
            pController.tarColor = "green";
            pController.curMode = PointerControl.revMode.FIRECOLOR;
        }
        else if (gamepad1.right_bumper && !gamepad1.rightBumperWasPressed()){
            pController.tarColor = "purple";
            pController.curMode = PointerControl.revMode.FIRECOLOR;
        }
        else if(pController.curMode == PointerControl.revMode.CONTFIRE){
            pController.curMode = PointerControl.revMode.AUTOIN;
        }
        //--------------------------loopActions------------------------
        pController.update();
        pController.updateTelemetry(t2);
        pController.updateTelemetry(telemetry);
        rollerControl.update();
    }
}
