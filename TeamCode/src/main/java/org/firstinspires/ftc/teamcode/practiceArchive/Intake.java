package org.firstinspires.ftc.teamcode.practiceArchive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    DcMotorEx intake;
    double targVel;
//    public static double testPower = 0;
    HardwareMap hw;
    String name;
    public Intake(HardwareMap hardw, String intakeName) {
        hw = hardw;
        name = intakeName;
    }
    public void init() {
        intake = hw.get(DcMotorEx.class, name);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void loop() {
        intake.setVelocity(targVel);
    }
    public void intakeOn() {
        targVel = 1800;
    }
    public void intakeOff() {
        targVel = 0;
    }
    public void intakeOut() {
        targVel = -1000;
    }
    public void setManualIntakePower() {

    }
    public void intakeTele(Telemetry t) {
        t.addData("Intake speed", intake.getVelocity());
    }
}
