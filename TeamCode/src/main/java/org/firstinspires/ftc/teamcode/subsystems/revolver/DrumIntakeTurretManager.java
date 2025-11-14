package org.firstinspires.ftc.teamcode.subsystems.revolver;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;

@Config
public class DrumIntakeTurretManager {
    ColorTrackAndPointerDesignator colTrack = new ColorTrackAndPointerDesignator();
    DcMotor revEnc;
    CRServo revSpin;
    Servo trigger;
    DcMotor intake;
    FancyPID pid = new FancyPID();
    public static double kP = 0.0004;
    public static double kI = 0;
    public static double kD = 0.0005;
    public static double iMax = 0;
    public static double iRange = 0;
    public static double errorTol = 362;
    public static double derivTol = 0.08;
    public static double TARGET = 0;
    boolean isFiring = false;
    double curPos = 0;
    public boolean testMode = false;
//    public boolean testShoot = false;
    double[] slotTarget = {0, 2700, -2700};
    public String tarColor = "white";
    public enum revMode {
        INTAKING,
        FIRESTANDBY,
        CONTFIRE,
        FIRECOLOR
    }

    public revMode curMode = revMode.INTAKING;
    // functions
    void pull_trigger(){
        //codeforpulltrigger
    }
    public void setGain(float gain){
        colTrack.setGain(gain);
    }
    public void nextSlot() {
        colTrack.pointer = (colTrack.pointer +1) % 3;
    }
    public void lastSlot() {
        colTrack.pointer = Math.abs((colTrack.pointer + 2)) % 3;
    }
    public void toggleManualShoot() {
        if (curMode == revMode.FIRESTANDBY) {
            curMode = revMode.INTAKING;
        } else {
            curMode = revMode.FIRESTANDBY;
        }
    }
    public double optimizeTarg(double targ, double cur) {
        double floor = Math.floor(cur / 8192.0) * 8192 + (targ % 8192);
        double ceiling = Math.ceil(cur / 8192.0) * 8192 + (targ % 8192);
        if (Math.abs(floor-cur) > Math.abs(ceiling - cur)) {
            return ceiling;
        } else {
            return floor;
        }
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("target", pid.target);
        telemetry.addData("curPos", curPos);
        telemetry.addData("speed", pid.velo);
        telemetry.addData("arrived", pid.arrived);
        colTrack.updateTelemetry(telemetry);
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
        trigger = hardwareMap.get(Servo.class, "flicker");
        revEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        revEnc.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void update() {
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;

        curPos = revEnc.getCurrentPosition();
        //-------------------------------set target---------------------
        //actions
        if (!testMode) {
            if (curMode == revMode.INTAKING) {
                //intake code
                isFiring = false;
                colTrack.pointer = colTrack.findNearestWhite();
                pid.target = optimizeTarg(slotTarget[colTrack.pointer], curPos);
            } else {
                pid.target = optimizeTarg(slotTarget[colTrack.pointer] + 180, curPos);
            }

            if (curMode == revMode.CONTFIRE) {
                if (pid.arrived && colTrack.ballAvailble() && !isFiring) {
                    colTrack.pointer = colTrack.findNearestBall();
                    isFiring = true;
                } else if (isFiring && pid.arrived) {
                    isFiring = false;
                    pull_trigger();
                }
            } else if (curMode == revMode.FIRECOLOR) {
                if (!colTrack.colorAvailble(tarColor)) {
                    curMode = revMode.INTAKING;
                } else if (pid.arrived && !isFiring) {
                    colTrack.pointer = colTrack.findNearestColor(tarColor);
                    pid.target = optimizeTarg(slotTarget[colTrack.pointer] + 180, curPos);
                    isFiring = true;
                } else if (isFiring && pid.arrived) {
                    isFiring = false;
                    pull_trigger();
                    curMode = revMode.INTAKING;
                }
            }
        } else {
            if (curMode == revMode.INTAKING) {
                //intake code
                isFiring = false;
                colTrack.pointer = colTrack.findNearestWhite();
                pid.target = optimizeTarg(slotTarget[colTrack.pointer], curPos);
            } else {
                pid.target = optimizeTarg(slotTarget[colTrack.pointer] + 180, curPos);
            }
        }
        //-----------------------loop actions-------------------------
        colTrack.arrived = pid.arrived;
        colTrack.loop();
        pid.update(curPos);
        revSpin.setPower(pid.velo);
    }
}
