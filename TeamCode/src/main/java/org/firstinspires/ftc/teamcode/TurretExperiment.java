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

        innerTurret.setPower((inner%5)*0.25);
        outerTurret.setPower((outer%5)*0.25);

        t2.addData("innerTurret Speed:", innerTurret.getPower());
        t2.addData("outerTurret Speed:", outerTurret.getPower());
        t2.update();
    }
}
