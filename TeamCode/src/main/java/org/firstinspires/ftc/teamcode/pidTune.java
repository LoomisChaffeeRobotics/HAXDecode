package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;

public class pidTune extends OpMode {
    DcMotor revEnc;
    CRServo revSpin;
    FancyPID pid = new FancyPID();
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double iMax = 0;
    public static double iRange = 0;
    public static double errorTol = 0;
    public static double derivTol = 0;
    public static double TARGET = 0;
    double curPos = 0;
    void updateTelemetry() {
        telemetry.addData("target", pid.target);
        telemetry.addData("curPos", curPos);
        //telemetry.addData("P", pid.PID_P);
        //telemetry.addData("I", pid.PID_I);
        //telemetry.addData("D", pid.PID_D);
        //telemetry.addData("kP", pid.Kp);
        telemetry.addData("speed", pid.velo);
        telemetry.addData("arrived", pid.arrived);
        telemetry.update();
    }
    @Override
    public void init() {
        pid.init();
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;
        pid.target = 0;
        revEnc = hardwareMap.get(DcMotor.class, "Enc");
        revSpin = hardwareMap.get(CRServo.class, "Spin");
        revEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;
        curPos = revEnc.getCurrentPosition();
        pid.update(curPos);
        revSpin.setPower(pid.velo);
        updateTelemetry();
    }
}
