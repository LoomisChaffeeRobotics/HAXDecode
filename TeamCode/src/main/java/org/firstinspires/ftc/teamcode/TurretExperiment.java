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
    double inner = 0;
    double outer = 0;
    boolean prevA = false;
    boolean prevB = false;

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

        innerTurret.setPower((inner%21)*(0.05));
        outerTurret.setPower((outer%21)*(0.05));

        double tpr = 28.0;
        double innerTps = innerTurret.getVelocity();
        double innerRpm = (innerTps / tpr) * 60.0;

        double outerTps = outerTurret.getVelocity();
        double outerRpm = (outerTps / tpr) * 60.0;


        telemetry.addData("innerTurret Speed", innerTurret.getPower());
        telemetry.addData("innerTurret Velocity from encoder (tps)", innerTps);
        telemetry.addData("innerTurret Velocity from encoder (rpm)", innerRpm);
        telemetry.addData("outerTurret Speed", outerTurret.getPower());
        telemetry.addData("outerTurret Velocity from encoder (tps)", outerTps);
        telemetry.addData("outerTurret Velocity from encoder (rpm)", outerRpm);
        telemetry.update();
    }
}
