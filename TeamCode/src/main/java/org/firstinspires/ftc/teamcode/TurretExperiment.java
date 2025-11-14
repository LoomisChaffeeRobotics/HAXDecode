package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

@TeleOp
@Config
public class TurretExperiment extends OpMode {
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    Servo flicker;
    DrumIntakeTurretManager drum;
    FtcDashboard Dash=FtcDashboard.getInstance();
    Telemetry t2=Dash.getTelemetry();
    double inner = 40;
    double outer = 40;
    public static double innerRPM = 0;
    public static double outerRPM = 0;
    public static double flickPos1 = 0.45;
    public static double flickPos2 = 0.97;
    boolean prevA = false;
    boolean prevB = false;
    boolean prevX = false;
    boolean prevY = false;
    boolean dUpPressed = false;
    boolean dDownPressed = false;

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

        drum = new DrumIntakeTurretManager();
        drum.init(hardwareMap);
        drum.testMode = true;

    }
    @Override
    public void loop() {
        if(gamepad1.dpad_up && !dUpPressed){
            flickPos1 = 0.97;
        }
        if(gamepad1.dpad_down && !dDownPressed){
            flickPos1 = 0.45;
        }


        if (gamepad1.aWasPressed()) {
            drum.lastSlot();
        } else if (gamepad1.bWasPressed()) {
            drum.nextSlot();
        }
        drum.update();
        drum.updateTelemetry(t2);
//        innerRPM = (inner%201)*(25);
        double innerTicks = (innerRPM / 60) * 28 ;
//        outerRPM = (outer%101)*(50);
        double outerTicks = (outerRPM / 60) * 28;

        innerTurret.setVelocity(-innerTicks);
        outerTurret.setVelocity(outerTicks);

        double actualInnerTicks = innerTurret.getVelocity();
        double actualInnerRPM = (actualInnerTicks / 28) * 60;
        double actualOuterTicks = outerTurret.getVelocity();
        double actualOuterRPM = (actualOuterTicks / 28) * 60;

        dUpPressed = gamepad1.dpad_up;
        dDownPressed = gamepad1.dpad_down;

        flicker.setPosition(flickPos1);



        telemetry.addData("innerTurret expected rpm (input)", innerRPM);
        telemetry.addData("innerTurret expected rpm (input)", outerRPM);

        t2.addData("innerTurret expected rpm (input)", innerRPM);
        t2.addData("innerTurret expected rpm (input)", outerRPM);

        telemetry.addData("innerTurret expected rpm (from encoder)", actualInnerRPM);
        telemetry.addData("outerTurret expected rpm (from encoder)", actualOuterRPM);

        t2.addData("innerTurret expected rpm (from encoder)", actualInnerRPM);
        t2.addData("outerTurret expected rpm (from encoder)", actualOuterRPM);

        telemetry.addData("flicker location", flicker.getPosition());
        t2.addData("flicker location", flicker.getPosition());

        telemetry.update();
        t2.update();
    }
}
