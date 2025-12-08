package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.revolver.ColorTracker;

@TeleOp
@Config
public class colorTesting extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    ColorTracker colorTracker;
    DcMotorEx drumEnc;
    boolean arrived;
    boolean intakingMode = true;
    double FCV = 8192;
    double curPose;

    @Override
    public void init() {
        colorTracker = new ColorTracker();
        drumEnc = hardwareMap.get(DcMotorEx.class, "drumEnc");
        colorTracker.init(hardwareMap);
        colorTracker.setGain((float) 2.5);
    }
    @Override
    public void loop() {
        curPose = drumEnc.getCurrentPosition();
        if (intakingMode) {
            if (Math.abs(curPose - Math.round(curPose/FCV)*FCV) < 100) {
                colorTracker.pointer = 0;
                arrived = true;
            } else if (Math.abs(curPose - (Math.round(curPose/FCV)*FCV) + FCV / 3) < 100) {
                colorTracker.pointer = 1;
                arrived = true;
            } else if (Math.abs(curPose - (Math.round(curPose/FCV)*FCV) - FCV / 3) < 100) {
                colorTracker.pointer = 2;
                arrived = true;
            } else {
                arrived = false;
            }
        }

        colorTracker.arrived = arrived;
        colorTracker.loop();
        colorTracker.updateTelemetry(telemetry);
        colorTracker.updateTelemetry(t2);
    }
}
