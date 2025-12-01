package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.FancyPID;

public class Turret {
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    double targDist;
    double veloInner;
    double veloOuter;
    double veloRobotX;
    double veloRobotY;
    double rot;
    CRServo spinner;
    FancyPID turPID;
    public Turret(HardwareMap hardwareMap) {
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");

        spinner = hardwareMap.get(CRServo.class, "turret");

        innerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        innerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turPID = new FancyPID();
        turPID.setCoefficients(0.01, 0, 0.09);
    }
    public void off () {
        innerTurret.setVelocity(0);
        outerTurret.setVelocity(0);

    }
    public void shoot (double dist) {
        targDist = dist;
    }
    void updateVelos(double dist, double angVel, double perpVel, double axialVel) {

    }
    void updateServoFeedforwards(double angVel, double perpVel) {

    }
}
