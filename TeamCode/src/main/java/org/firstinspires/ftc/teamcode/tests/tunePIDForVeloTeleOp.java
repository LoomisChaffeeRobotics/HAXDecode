package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelPID;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

@Config
@TeleOp
public class tunePIDForVeloTeleOp extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    public static double kPInnerVelo = 0.000675;
    public static double kIInnerVelo = 0;
    public static double kDInnerVelo = 0;
    public static double kFVelo = 0.0004125;
    public static double kPOuter = 0.00065;
    public static double kIOuter = 0;
    public static double kDOuter = 0;
    public static double kFOuter = 0.0004;
    public static double targVeloInner = 0;
    public static double targVeloOuter = 0;
    DcMotorEx inner;
    DcMotorEx outer;
    double curVeloInner;
    double curVeloOuter;
    static double RPMtoTicksPerSecond = (double) 28 / 60;
    ElapsedTime fireSequenceTimer = new ElapsedTime();
    Servo flicker;
    CRServo drum;
    double flickPosUp = 0.95;
    double flickPosDown = 0.4;
    FancyPID pid = new FancyPID();
    public static double kP = 0.0002;
    public static double kI = 0.00004;
    public static double kD = 0.00015;
    public static double iMax = 0.3;
    public static double iRange = 0.15;
    public static double errorTol = 100;
    public static double derivTol = 10;
    double target = 0;
    DcMotor revEnc;
    double innerCurVel = 0;
    double outerCurVel = 0;
    double filteredInner = 0;
    double filteredOuter = 0;
    FlywheelPID innerPID = new FlywheelPID();
    FlywheelPID outerPID = new FlywheelPID();
    @Override
    public void init() {
        inner = hardwareMap.get(DcMotorEx.class, "innerTurret");
        outer = hardwareMap.get(DcMotorEx.class, "outerTurret");

        revEnc = hardwareMap.get(DcMotor.class, "drumEnc");
        revEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        revEnc.setDirection(DcMotorSimple.Direction.REVERSE);

        drum = hardwareMap.get(CRServo.class, "drumServo");

        pid.init();
        pid.setCoefficients(kP, kI, kD);
        pid.iMax = iMax;
        pid.iRange = iRange;
        pid.errorTol = errorTol;
        pid.dTol = derivTol;
        pid.target = 0;

        inner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        inner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        inner.setVelocityPIDFCoefficients(kPInnerVelo, kIInnerVelo, kDInnerVelo, kFVelo);
        outer.setVelocityPIDFCoefficients(kPOuter, kIOuter, kDOuter, kFOuter);

        flicker = hardwareMap.get(Servo.class, "flicker");
        flicker.setPosition(flickPosDown);

        innerPID.init();
        innerPID.setCoefficients(kPInnerVelo, kIInnerVelo, kDInnerVelo, kFVelo);
        outerPID.init();
        outerPID.setCoefficients(kPOuter, kIOuter, kDOuter, kFOuter);
    }
    @Override
    public void loop() {
        innerCurVel = inner.getVelocity();
        outerCurVel = outer.getVelocity();
//        updateLowerFilter(innerCurVel);
//        updateUpperFilter(outerCurVel);

        innerPID.setCoefficients(kPInnerVelo, kIInnerVelo, kDInnerVelo, kFVelo);
        outerPID.setCoefficients(kPOuter, kIOuter, kDOuter, kFOuter);

        innerPID.update(innerCurVel);
        innerPID.target = targVeloInner * RPMtoTicksPerSecond;
        outerPID.update(outerCurVel);
        outerPID.target = targVeloOuter * RPMtoTicksPerSecond;

        inner.setPower(innerPID.velo);
        outer.setPower(outerPID.velo);

        if (gamepad1.aWasPressed()) {
            fireSequenceTimer.reset();
        }

        if (fireSequenceTimer.seconds() < 0.4) {
//            flickMode = "flick";
            flicker.setPosition(flickPosUp);
        } else if (fireSequenceTimer.seconds() >= 0.4 && fireSequenceTimer.seconds() < 0.75) {
//            flickMode = "retract";
            flicker.setPosition(flickPosDown);
        }

        if (gamepad1.dpadUpWasPressed()) {
            target += (8192/3.0);
        } else if (gamepad1.dpadDownWasPressed()) {
            target -=(8192/3.0);
        }

        pid.target = target;
        pid.update(revEnc.getCurrentPosition());
        drum.setPower(pid.velo);

        t2.addData("Target Velocity Inner", targVeloInner);
        t2.addData("Current Velocity Inner", inner.getVelocity() / RPMtoTicksPerSecond);
        t2.addData("Target Velocity Outer", targVeloOuter);
        t2.addData("Current Velocity Outer", outer.getVelocity() / RPMtoTicksPerSecond);
        t2.update();
    }
}
