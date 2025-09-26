package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Turret extends OpMode {
    DcMotorEx Front;
    DcMotorEx Back;

    boolean Currenttick;
    boolean Lasttick;

    double power;

    @Override
    public void init() {
        Front = hardwareMap.get(DcMotorEx.class,"inner_turret" );
        Back = hardwareMap.get(DcMotorEx.class,"outer_turret" );

        Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Back.setDirection(DcMotorSimple.Direction.REVERSE);

        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Currenttick = false;
        Lasttick = false;

    }

    @Override
    public void loop() {
        if (gamepad1.x){
            Currenttick = true;

        }
        if (!Lasttick & Currenttick){
            power += 0.1;
        }
        if (gamepad1.aWasPressed()){
            Front.setPower(1);
            Back.setPower(1);
        }
        else
        if(gamepad1.b){
            Front.setPower(0.6);
            Back.setPower(1);
        }
        else
        if(gamepad1.a){
            Front.setPower(1);
            Back.setPower(0.6);
        }
        else{
            Front.setPower(0);
            Back.setPower(0);
        }
        Lasttick = Currenttick;

        telemetry.addData("power", power);
        telemetry.update();
    }

}
