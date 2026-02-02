package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class tunePIDForVeloTeleOp extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    public static double kPInnerVelo = 180;
    public static double kIInnerVelo = 0;
    public static double kDInnerVelo = 40;
    public static double kFVelo = 13;
    public static double kPOuter = 90;
    public static double kIOuter = 0;
    public static double kDOuter = 0;
    public static double kFOuter = 13;
    public static double targVeloInner = 0;
    public static double targVeloOuter = 0;
    DcMotorEx inner;
    DcMotorEx outer;
    double curVeloInner;
    double curVeloOuter;
    static double RPMtoTicksPerSecond = (double) 28 / 60;
    @Override
    public void init() {
        inner = hardwareMap.get(DcMotorEx.class, "innerTurret");
        outer = hardwareMap.get(DcMotorEx.class, "outerTurret");

        inner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        inner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        inner.setVelocityPIDFCoefficients(kPInnerVelo, kIInnerVelo, kDInnerVelo, kFVelo);
        outer.setVelocityPIDFCoefficients(kPOuter, kIOuter, kDOuter, kFOuter);
    }

    @Override
    public void loop() {
         inner.setVelocity(targVeloInner * RPMtoTicksPerSecond);
         outer.setVelocity(targVeloOuter * RPMtoTicksPerSecond);
        inner.setVelocityPIDFCoefficients(kPInnerVelo, kIInnerVelo, kDInnerVelo, kFVelo);
        outer.setVelocityPIDFCoefficients(kPOuter, kIOuter, kDOuter, kFOuter);
        curVeloInner = inner.getVelocity() / RPMtoTicksPerSecond;
        curVeloOuter = outer.getVelocity() / RPMtoTicksPerSecond;
        t2.addData("Target Velocity Inner", targVeloInner);
        t2.addData("Current Velocity Inner", curVeloInner);
        t2.addData("Target Velocity Outer", targVeloOuter);
        t2.addData("Current Velocity Outer", curVeloOuter);
        t2.update();
    }
}
