package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class TurretExperiment extends OpMode {
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    Servo flicker;
    FtcDashboard Dash=FtcDashboard.getInstance();
    Telemetry t2=Dash.getTelemetry();
    double inner = 40;
    double outer = 40;
    public static double innerRPM = 0;
    public static double outerRPM = 0;
    boolean prevA = false;
    boolean prevB = false;
    boolean prevX = false;
    boolean prevY = false;

    @Override
    public void init() {
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");

        flicker=hardwareMap.get(Servo.class, "flicker");

        innerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        innerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    @Override
    public void loop() {
        if(gamepad1.a){
            flicker.setPosition(1);
        }
        if(gamepad1.b){
            flicker.setPosition(-1);
        }

        double innerTicks = (innerRPM / 60) * 28 ;
        double outerTicks = (outerRPM / 60) * 28;

        innerTurret.setVelocity(-innerTicks);
        outerTurret.setVelocity(outerTicks);

        double actualInnerTicks = innerTurret.getVelocity();
        double actualInnerRPM = (actualInnerTicks / 28) * 60;
        double actualOuterTicks = outerTurret.getVelocity();
        double actualOuterRPM = (actualOuterTicks / 28) * 60;

        telemetry.addData("innerTurret expected rpm (input)", innerRPM);
        telemetry.addData("innerTurret expected rpm (input)", outerRPM);

        t2.addData("innerTurret expected rpm (input)", innerRPM);
        t2.addData("innerTurret expected rpm (input)", outerRPM);



        telemetry.addData("innerTurret expected rpm (from encoder)", actualInnerRPM);
        telemetry.addData("outerTurret expected rpm (from encoder)", actualOuterRPM);

        t2.addData("innerTurret expected rpm (from encoder)", actualInnerRPM);
        t2.addData("outerTurret expected rpm (from encoder)", actualOuterRPM);

        telemetry.update();
        t2.update();
    }
}
