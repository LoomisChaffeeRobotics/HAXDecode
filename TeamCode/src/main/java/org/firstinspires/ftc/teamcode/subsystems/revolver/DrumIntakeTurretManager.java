package org.firstinspires.ftc.teamcode.subsystems.revolver;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;
import org.firstinspires.ftc.teamcode.practiceArchive.TurretOLD;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Config
public class DrumIntakeTurretManager {
    ColorTracker colTrack = new ColorTracker();
    DcMotor revEnc;
    CRServo revSpin;
    Servo flicker;
    DcMotorEx intake;
    Turret turret;
    FancyPID pid = new FancyPID();
    double flickPosUp = 0.85;
    double flickPosDown = 0.4;
    public static double kP = 0.0002;
    public static double kI = 0.000005;
    public static double kD = 0.0001;
    public static double iMax = 0.3;
    public static double iRange = 0.15;
    public static double errorTol = 100;
    public static double derivTol = 10;
    public boolean isFiring = false;
    public boolean contFiring = false;
    public boolean lastTickArrived = false;
    public boolean isDepressed = false;
    double curPos = 0;
    ElapsedTime fireSequenceTimer = new ElapsedTime();
    public boolean testMode = false;
    double FCV = 8192;
    double[] slotTarget = {0, FCV / 3, -FCV / 3};
    public String flickMode = "off";
    public boolean activateAsync = false;
    public enum revMode {
        INTAKING,
        FIRESTANDBY,
        CONTFIRE,
        FIREPURPLE,
        FIREGREEN,
        FIRESINGLE,
        HPINTAKE,
        FIREIDLE,
        INTAKEIDLE
    }

    public revMode curMode = revMode.INTAKEIDLE;
    public revMode lastMode = revMode.INTAKEIDLE;
    // functions
    void fireSequenceAsync(){
        lastTickArrived = false;
        if (fireSequenceTimer.seconds() < 0.375) {
            flickMode = "flick";
            flicker.setPosition(flickPosUp);
        } else if (fireSequenceTimer.seconds() >= 0.375 && fireSequenceTimer.seconds() < 0.75) {
            flickMode = "retract";
            flicker.setPosition(flickPosDown);
        } else {
            flickMode = "off";
            colTrack.removeFiredBall(colTrack.pointer);
            isFiring = false;
            curMode = revMode.FIRESTANDBY;
        }
    }
    public void setGain(float gain){
        colTrack.setGain(gain);
    }
    public void nextSlot() {
        colTrack.pointer = Math.abs((colTrack.pointer +1)) % 3;
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
        t.addData("timer", fireSequenceTimer.seconds());
        t.addData("flickerMode", flickMode);
        t.addData("asyncisRunning", activateAsync);
        t.addData("fireSequence", isFiring);
        t.addData("curMode", curMode);
        t.addData("lastTickArrived", lastTickArrived);
        turret.updateTelemetry(t);
        t.update();
    }

    //----------------------------------------------------------------------------------
    public void init(HardwareMap hardwareMap, MecanumDrive drive) {
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
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        revEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        revEnc.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret = new Turret();

        turret.init(hardwareMap, drive);
        flicker.setPosition(flickPosDown);
    }
    public void init(HardwareMap hw) {
        this.init(hw, new MecanumDrive(hw, new Pose2d(0,0,0)));
    }
    public void firePurple() {
        curMode = revMode.FIREPURPLE;
    }
    public void fireGreen() {
        curMode = revMode.FIREGREEN;
    }
    public void fireSingle() {curMode = revMode.FIRESINGLE;}
    public void startContFire() {
        curMode = revMode.CONTFIRE;
    }
    public void setCurrentPurple() {
        colTrack.addPurple(colTrack.pointer);
    }
    public void setCurrentGreen() {
        colTrack.addGreen(colTrack.pointer);
    }
    public void contFireAsync() {
        if (!colTrack.ballAvailable()) {
            curMode = revMode.INTAKEIDLE;
            activateAsync = false;
        } else {
            colTrack.pointer = colTrack.findNearestBall();
            pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV / 2, curPos);
            if (lastTickArrived) {
                fireSequenceTimer.reset();
                isFiring = true;
                activateAsync = false;
            } else if (isFiring) {
                activateAsync = true;
                fireSequenceAsync();
            }
            else{
                activateAsync = false;
            }
        }
    }
    public void updateLastTickArrived() {
        if (pid.arrived && !isDepressed) {
            isDepressed = true;
            lastTickArrived = false;
        }
        if (!pid.arrived && isDepressed) {
            isDepressed = false;
            lastTickArrived = true;
        }
    }
    public void update() {
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;

        curPos = revEnc.getCurrentPosition();
        colTrack.loop(pid.arrived, (curMode == revMode.INTAKING));
        //-------------------------------set target---------------------

        if (curMode == revMode.INTAKING) {
            //intake code
            isFiring = false;
            colTrack.pointer = colTrack.findNearestWhite();
            pid.target = optimizeTarg(slotTarget[colTrack.pointer], curPos);
            intake.setVelocity(-1800);
        } else if (curMode == revMode.FIRESTANDBY) {
            colTrack.pointer = colTrack.findNearestBall();
            pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV / 2, curPos);
            turret.mode = Turret.turMode.FIRING;
            intake.setVelocity(0);
        } else if (curMode == revMode.HPINTAKE) {
            // pointer will be changed manually using next slot and last slot
            isFiring = false;
            pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV / 2, curPos);
            turret.mode = Turret.turMode.INTAKING;
        } else if (curMode == revMode.CONTFIRE) {
            contFireAsync();
            turret.mode = Turret.turMode.FIRING;
            intake.setVelocity(0);
        } else if (curMode == revMode.FIREPURPLE) {
            if (colTrack.colorAvailable("purple")) {
                colTrack.pointer = colTrack.findNearestColor("purple");
                pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV / 2, curPos);
                turret.mode = Turret.turMode.FIRING;
                if ((lastTickArrived || (pid.arrived && lastMode != revMode.FIREPURPLE)) && turret.bothMotorsSpunUp) {
                    fireSequenceTimer.reset();
                    isFiring = true;
                } else if (isFiring) {
                    fireSequenceAsync();
                }
            } else {
                curMode = revMode.FIRESTANDBY;
            }
            intake.setVelocity(0);
        } else if (curMode == revMode.FIREGREEN) {
            if (colTrack.colorAvailable("green")) {
                colTrack.pointer = colTrack.findNearestColor("green");
                pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV / 2, curPos);
                turret.mode = Turret.turMode.FIRING;
                if ((lastTickArrived || pid.arrived && lastMode != revMode.FIREGREEN) && turret.bothMotorsSpunUp) {
                    fireSequenceTimer.reset();
                    isFiring = true;

                } else if (isFiring) {
                    fireSequenceAsync();
                }
            } else {
                curMode = revMode.FIRESTANDBY;
            }
            intake.setVelocity(0);
        } else if (curMode == revMode.FIRESINGLE) {
            if (colTrack.ballAvailable()) {
                colTrack.pointer = colTrack.findNearestBall();
                pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV / 2, curPos);
                turret.mode = Turret.turMode.FIRING;
                if (((lastTickArrived || pid.arrived) && lastMode != revMode.FIRESINGLE) && turret.bothMotorsSpunUp) {
                    fireSequenceTimer.reset();
                    isFiring = true;
                } else if (isFiring) {
                    fireSequenceAsync();
                }
            } else {
                curMode = revMode.FIRESTANDBY;
            }
            intake.setVelocity(0);
        } else if (curMode == revMode.FIREIDLE) {
            if (colTrack.ballAvailable()) {
                colTrack.pointer = colTrack.findNearestBall();
            }
            pid.target = optimizeTarg(slotTarget[colTrack.pointer] + FCV/2, curPos);
            turret.mode = Turret.turMode.IDLE;
            intake.setVelocity(0);
        } else if (curMode == revMode.INTAKEIDLE) {
            if (colTrack.emptyAvailable()) {
                colTrack.pointer = colTrack.findNearestColor("white");
            }
            pid.target = optimizeTarg(slotTarget[colTrack.pointer], curPos);
            turret.mode = Turret.turMode.IDLE;
            intake.setVelocity(0);
        }



        //-----------------------loop actions-------------------------
        updateLastTickArrived();
        turret.loop();
        pid.update(curPos);
        revSpin.setPower(pid.velo);
        lastMode = curMode;

    }
    public boolean isFull() {
        return !colTrack.emptyAvailable();
    }
}
