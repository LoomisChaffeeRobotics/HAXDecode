package org.firstinspires.ftc.teamcode.subsystems.revolver;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Config
public class DrumIntakeTurretManager {

    ColorTracker colTrack = new ColorTracker();
    DcMotor revEnc;
    CRServo revSpin;
    Servo flicker;
    DcMotor intake;
    Turret turret;
    FancyPID pid = new FancyPID();
    public static double kP = 0.00075;
    public static double kI = 0.000001;
    public static double kD = 0.0032;
    public static double iMax = 0.3;
    public static double iRange = 400;
    public static double errorTol = 50;
    public static double derivTol = 10;
    public static double TARGET = 0;
    public boolean isFiring = false;
    double curPos = 0;
    ElapsedTime fireSequenceTimer = new ElapsedTime();
    public boolean testMode = false;
    double FCV = 8192;
    double[] slotTarget = {0, FCV / 3, -FCV / 3};
    public String tarColor = "white";
    public enum revMode {
        INTAKING,
        FIRESTANDBY,
        CONTFIRE,
        FIRECOLOR
    }

    public revMode curMode = revMode.INTAKING;
    // functions
    void fireSequenceAsync(){
        if (fireSequenceTimer.seconds() < 0.75) {
            flicker.setPosition(0.97);
        } else if (fireSequenceTimer.seconds() > 0.75 && fireSequenceTimer.seconds() < 1.5) {
            flicker.setPosition(0.45);
        } else {
            isFiring = false;
        }
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
        double floor = Math.floor(cur / FCV) * FCV + (targ % FCV);
        double ceiling = Math.ceil(cur / FCV) * FCV + (targ % FCV);
        if (Math.abs(floor-cur) > Math.abs(ceiling - cur)) {
            return ceiling;
        } else {
            return floor;
        }
    }
    public void updateTelemetry(Telemetry t){
        t.addData("target", pid.target);
        t.addData("curPos", curPos);
        t.addData("P", pid.PID_P);
        t.addData("I", pid.PID_I);
        t.addData("D", pid.PID_D);
        t.addData("speed", pid.velo);
        t.addData("arrived", pid.arrived);
        colTrack.updateTelemetry(t);
        t.addData("pointer", colTrack.pointer);
        t.update();
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

        revEnc = hardwareMap.get(DcMotor.class, "drumEnc");
        revSpin = hardwareMap.get(CRServo.class, "drumServo");
        flicker = hardwareMap.get(Servo.class, "flicker");

        revEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        revEnc.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = new Turret(hardwareMap);
    }
    public void fire() {
        isFiring = true;
//        if (fireSequenceTimer.seconds() == 0)
        fireSequenceTimer.reset();
    }
    public void firePurple() {
        tarColor = "purple";
        isFiring = true;
        if (fireSequenceTimer.seconds() == 0) {
            fireSequenceTimer.reset();
        }
    }
    public void fireGreen() {
        tarColor = "green";
        isFiring = true;
        if (fireSequenceTimer.seconds() == 0) {
            fireSequenceTimer.reset();
        }
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


            if (curMode == revMode.INTAKING) {
                //intake code
                isFiring = false;
                if (!isFiring) {
                    flicker.setPosition(0.45);
                }
                colTrack.pointer = colTrack.findNearestWhite();
                pid.target = optimizeTarg(slotTarget[colTrack.pointer], curPos);
            } else {
                pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV / 2, curPos);
                if (isFiring) {
                    fireSequenceAsync();
                } else {
                    flicker.setPosition(0.45);
                }
            }
//        }
        //-----------------------loop actions-------------------------
        colTrack.arrived = pid.arrived;
        colTrack.loop();
        pid.update(curPos);
        revSpin.setPower(pid.velo);
//        try {
//            Thread.sleep(5);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
    }
}
