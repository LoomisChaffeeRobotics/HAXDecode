package org.firstinspires.ftc.teamcode.subsystems.revolver;

import android.graphics.Color;
import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;

@Config
public class PointerControl{
    ColorTrackAndPointerDesignator colTrack = new ColorTrackAndPointerDesignator();
    DcMotor revEnc;
    CRServo revSpin;
    //DcMotor trigger;
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
    double[] slotTarget = {0, 2371, -2371};
    String tarColor = "white";
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
    public void setGain(float gain){
        colTrack.setGain(gain);
    }

    public double findNearest360(double num) {
        return Math.round(num / 8192.0) * 8192;
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("target", pid.target);
        telemetry.addData("curPos", curPos);
        //telemetry.addData("P", pid.PID_P);
        //telemetry.addData("I", pid.PID_I);
        //telemetry.addData("D", pid.PID_D);
        //telemetry.addData("kP", pid.Kp);
        telemetry.addData("speed", pid.velo);
        telemetry.addData("arrived", pid.arrived);
        colTrack.updateTelemetry(telemetry);
        telemetry.update();
    }

    //----------------------------------------------------------------------------------
    public void init(HardwareMap hardwareMap) {
        colTrack.init(hardwareMap);
        pid.init();
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;
        pid.target = 0;
        revEnc = hardwareMap.get(DcMotor.class, "Enc");
        revSpin = hardwareMap.get(CRServo.class, "Spin");
        //trigger = hardwareMap.get(DcMotor.class, "trig");
        revEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void update() {
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;

        curPos = revEnc.getCurrentPosition();
        //-------------------------------set target---------------------
        //pid.target = findNearest360(curPos) + slotTarget[colTrack.pointer];
        pid.target = TARGET;
        //actions
        if (curMode == revMode.AUTOIN){
            //intake code
            colTrack.pointer = colTrack.findNearestWhite();
        }
        else if (curMode == revMode.CONTFIRE){
            if (pid.arrived) {
                if (colTrack.slotColor[colTrack.pointer].equals("white")) {
                    colTrack.pointer = colTrack.findNearestBall();
                }
                else{
                    pull_trigger();
                }
            }
        }
        else if (curMode == revMode.FIRECOLOR){
            if (colTrack.slotColor[colTrack.pointer].equals(tarColor)){
                pull_trigger();
            }
            else{
                colTrack.pointer = colTrack.findNearestColor(tarColor);
                //waitForPid();
                pull_trigger();
            }
            curMode = revMode.AUTOIN;
        }
        else if (curMode == revMode.HP){
            if (colTrack.emptyAvailble()) {
                colTrack.pointer = (colTrack.findNearestWhite() + 1) % 3;
            }
            else{
                curMode = revMode.AUTOIN;
            }
        }
        //-----------------------loop actions-------------------------
        colTrack.loop(pid.arrived);
        pid.update(curPos);
        revSpin.setPower(pid.velo);
    }
}
