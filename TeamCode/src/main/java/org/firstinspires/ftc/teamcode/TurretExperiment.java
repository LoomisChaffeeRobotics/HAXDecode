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

    @Override
    public void init() {
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");
        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    @Override
    public void loop() {
        if(gamepad1.a) {
            innerTurret.setPower(-0.3);
            outerTurret.setPower(-0.3);
        }
        else if(gamepad1.b) {
            innerTurret.setPower(-0.5);
            outerTurret.setPower(-0.5);
        }
        else if(gamepad1.x) {
            innerTurret.setPower(-1);
            outerTurret.setPower(-1);
        }
        else if(gamepad1.y) {
            innerTurret.setPower(-1);
            outerTurret.setPower(-0.75);
        }
        else if(gamepad1.dpad_down) {
            innerTurret.setPower(-0.75);
            outerTurret.setPower(-1);
        }
        else if (gamepad1.dpad_up) {
            innerTurret.setVelocity(-500);
            outerTurret.setVelocity(-2400);
        }
        else{
            innerTurret.setPower(0);
            outerTurret.setPower(0);
        }
        t2.addData("innervelocity:", innerTurret.getVelocity());
        t2.addData("outervelocity:", outerTurret.getVelocity());
        t2.update();
    }
}
