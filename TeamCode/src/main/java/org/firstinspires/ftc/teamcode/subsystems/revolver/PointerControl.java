package org.firstinspires.ftc.teamcode.subsystems.revolver;

import android.graphics.Color;
import android.graphics.Point;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FancyPID;
import org.firstinspires.ftc.teamcode.Revolver;

public class PointerControl {
    DcMotor revSpin;
    DcMotor trigger;
    FancyPID pid = new FancyPID();
    double curPos = 0;
    double[] slotTarget = {0, 120, -120};

    public enum revMode {
        CONTFIRE,
        AUTOIN,
        FIRECOLOR,
        HP
    }

    private revMode curMode = revMode.AUTOIN;
    // functions
    void pull_trigger(){
        //codeforpulltrigger
    }
    public double findNearest360(double num) {
        return Math.round(num / 360.0) * 360;
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("target", pid.target);
        telemetry.addData("curPos", curPos);
        telemetry.addData("P", pid.PID_P);
        telemetry.addData("I", pid.PID_I);
        telemetry.addData("D", pid.PID_D);
        telemetry.addData("kP", pid.Kp);
        telemetry.addData("speed", pid.velo);
        telemetry.update();
    }
    public void init(HardwareMap hardwareMap) {
        pid.init();
        pid.setCoefficients(1, 0, 0);
        pid.iMax = 0.3;
        pid.iRange = 0.5;
        pid.errorTol = 4;
        pid.dTol = 2;
        pid.target = 0;
        revSpin = hardwareMap.get(DcMotor.class, "Spin");
        trigger = hardwareMap.get(DcMotor.class, "trig");
        revSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop(int pointer) {
        curPos = revSpin.getCurrentPosition();

        //-------------------------------set target---------------------
        pid.target = findNearest360(curPos) + slotTarget[pointer];
        //-----------------------loop actions-------------------------
        pid.update(curPos);
        revSpin.setPower(pid.velo);
    }
}
