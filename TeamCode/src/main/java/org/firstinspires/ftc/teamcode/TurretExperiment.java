package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TurretExperiment extends OpMode {
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    FtcDashboard Dash=FtcDashboard.getInstance();
    Telemetry t2=Dash.getTelemetry();
    double inner = 40;
    double outer = 40;
    boolean prevA = false;
    boolean prevB = false;
    boolean prevX = false;
    boolean prevY = false;

    @Override
    public void init() {
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");

        innerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        innerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    @Override
    public void loop() {
        // Gamepad1.A increases the power of inner turret by 0.25 and wraps back to 0 after 1.
        // Gamepad1.B increases the power of outer turret by 0.25 and wraps back to 0 after 1.
        if(gamepad1.a && !prevA){
            inner++;
            prevA=true;
        }
        if(!gamepad1.a){
            prevA=false;
        }
        if(gamepad1.b && !prevB){
            outer++;
            prevB=true;
        }
        if(!gamepad1.b){
            prevB=false;
        }
        if(gamepad1.x && !prevX){
            inner--;
            prevX=true;
        }
        if(!gamepad1.x){
            prevX=false;
        }
        if(gamepad1.y && !prevY){
            outer--;
            prevY=true;
        }
        if(!gamepad1.y){
            prevY=false;
        }

        double innerRPM = (inner%201)*(25);
        double innerTicks = (innerRPM / 60) * 28 ;
        double outerRPM = (outer%101)*(50);
        double outerTicks = (outerRPM / 60) * 28;

        innerTurret.setVelocity(-innerTicks);
        outerTurret.setVelocity(outerTicks);

        double actualInnerTicks = innerTurret.getVelocity();
        double actualInnerRPM = (actualInnerTicks / 28) * 60;
        double actualOuterTicks = outerTurret.getVelocity();
        double actualOuterRPM = (actualOuterTicks / 28) * 60;

        telemetry.addData("innerTurret expected rpm (motor x 2)", innerRPM * 2);
        telemetry.addData("innerTurret Velocity from encoder (rpm)", actualInnerRPM);
        telemetry.addData("innerTurret Velocity after gearbox (x2)", actualInnerRPM * 2);

        telemetry.addData("outerTurret Speed", outerRPM);
        telemetry.addData("outerTurret Velocity from encoder (rpm)", actualOuterRPM);
        telemetry.update();
    }
}
