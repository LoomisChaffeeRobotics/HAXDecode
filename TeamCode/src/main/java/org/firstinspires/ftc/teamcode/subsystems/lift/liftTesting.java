package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp
@Config
public class liftTesting extends OpMode {
    public enum liftMode {
        RISING,
        LOWERING,
        IDLE
    }
    public liftMode mode = liftMode.IDLE;
    DcMotorEx lift;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();

    @Override
    public void init() {
        lift= hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (mode != liftMode.LOWERING && gamepad1.right_trigger > 0.1){
            mode = liftMode.RISING;
            lift.setPower(gamepad1.right_trigger);
        }
        else if (mode != liftMode.RISING && gamepad1.left_trigger > 0.1){
            mode = liftMode.LOWERING;
            lift.setPower((-1 * gamepad1.left_trigger) * 0.6); // Scaled down cause I am scared that the robot will slam down
        }
        else{
            mode = liftMode.IDLE;
            lift.setPower(0);
        }

        t2.addData("motor power",gamepad1.right_trigger);
        t2.update();
    }
}
