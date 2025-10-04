package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeControl {
    DcMotorEx intake;
    double targVel;

    public void init(HardwareMap hw, String intakeName) {
        intake = hw.get(DcMotorEx.class, intakeName);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop() {
        intake.setVelocity(targVel);
    }
    public void intakeOn() {

    }
    public void intakeOff() {

    }
}
