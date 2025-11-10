package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;
@Config
@TeleOp
public class pidTune extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    DcMotor revEnc;
    CRServo revSpin;
    FancyPID pid = new FancyPID();
    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0.015;
    public static double iMax = 0;
    public static double iRange = 0;
    public static double errorTol = 0;
    public static double derivTol = 0;
    public static double TARGET = 0;
    double curPos = 0;
    public void updateTelemetry(Telemetry t) {
        t.addData("target", pid.target);
        t.addData("curPos", curPos);
        t.addData("P", pid.PID_P);
        t.addData("I", pid.PID_I);
        t.addData("D", pid.PID_D);
        t.addData("speed", pid.velo);
        t.addData("arrived", pid.arrived);
        t.update();
    }
    @Override
    public void init() {
        pid.init();
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;
        pid.target = TARGET;
        revEnc = hardwareMap.get(DcMotor.class, "Enc");
        revSpin = hardwareMap.get(CRServo.class, "Spin");
        revEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        revEnc.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        pid.target = TARGET;
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;
        curPos = revEnc.getCurrentPosition();
        pid.update(curPos);
        revSpin.setPower(pid.velo);
        updateTelemetry(t2);

    }
}
