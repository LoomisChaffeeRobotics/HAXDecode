package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class TurretExperiment extends OpMode {
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
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
//        if(gamepad1.a && !prevA){
//            inner++;
//            prevA=true;
//        }
//        if(!gamepad1.a){
//            prevA=false;
//        }
//        if(gamepad1.b && !prevB){
//            outer++;
//            prevB=true;
//        }
//        if(!gamepad1.b){
//            prevB=false;
//        }
//        if(gamepad1.x && !prevX){
//            inner--;
//            prevX=true;
//        }
//        if(!gamepad1.x){
//            prevX=false;
//        }
//        if(gamepad1.y && !prevY){
//            outer--;
//            prevY=true;
//        }
//        if(!gamepad1.y){
//            prevY=false;
//        }

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
