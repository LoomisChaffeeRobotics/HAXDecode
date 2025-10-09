package org.firstinspires.ftc.teamcode.practiceArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class Turret extends OpMode {
    DcMotorEx Front;
    DcMotorEx Back;

    boolean CurrenttickX;
    boolean LasttickX;

    boolean CurrenttickY;
    boolean LasttickY;


    double power;

    @Override
    public void init() {
        Front = hardwareMap.get(DcMotorEx.class,"inner_turret" );
        Back = hardwareMap.get(DcMotorEx.class,"outer_turret" );

//        Front.setDirection(DcMotorSimple.Direction.REVERSE);
//        Back.setDirection(DcMotorSimple.Direction.REVERSE);

        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CurrenttickX = false;
        LasttickX = false;
        CurrenttickY = false;
        LasttickY = false;

    }

    @Override
    public void loop() {

        CurrenttickX = gamepad1.x;
        CurrenttickY = gamepad1.y;

        if(!(CurrenttickX && CurrenttickY)){
            if (!LasttickX && CurrenttickX){
                power = Math.min(power+0.1, 1);
            }
            else if (!LasttickY && CurrenttickY){
                power = Math.max(power-0.1, 0);
            }
        }

        Front.setPower(power);
        Back.setPower(power);

        LasttickX = CurrenttickX;
        LasttickY = CurrenttickY;

        telemetry.addData("power", power);
        telemetry.addData("X", CurrenttickX);
        telemetry.addData("Y", CurrenttickY);
        telemetry.update();
    }

}
