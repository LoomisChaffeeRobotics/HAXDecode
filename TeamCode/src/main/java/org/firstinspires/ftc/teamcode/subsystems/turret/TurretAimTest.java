package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.lookUpTable;

import java.util.List;

@TeleOp
public class TurretAimTest extends OpMode {
    lookUpTable<Double> table = new lookUpTable<>();
    limeLight LL;
    CRServo spinner;
    DcMotor turEnc;
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    public static double innerRPM = 0;
    public static double outerRPM = 0;
    public static double thetaDiff = 0;
    public double turretFullLoop = 0;
    public double vG = 0;
    public double curX = 0;
    public double curY = 0;
    public double dGoal = 0;
    public double flightTime = 0;
    public double dPrime = 0;
    double getLimeLightReading(){
        return LL.tx;
    }
    double getGyro(){
        return LL.robotYaw;
    }
    double getTurretAngle(){
        return 360 * (turEnc.getCurrentPosition() / turretFullLoop);
    }
    double getSpeed(){
        return 0.00;
    }
    void updateRoboPos(){

    }
    void updateTelemetry(){
        telemetry.addData("innerTurret rpm", innerRPM);
        telemetry.addData("outerTurret rpm)", outerRPM);
        telemetry.addData("thetaDiff", thetaDiff);
        telemetry.addData("vG", vG);
        telemetry.addData("dGoal", dGoal);
        telemetry.addData("flightTime", flightTime);
        telemetry.addData("dPrime", dPrime);
        telemetry.addData("curX", curX);
        telemetry.addData("curY", curY);
        telemetry.addData("turretAngle", getTurretAngle());
        telemetry.addData("gyro", getGyro());
        telemetry.addData("limelight", getLimeLightReading());
        telemetry.addData("speed", getSpeed());
        telemetry.update();
    }
    @Override
    public void init(){
        LL = new limeLight();
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");
        turEnc = hardwareMap.get(DcMotor.class, "Enc");

        spinner = hardwareMap.get(CRServo.class, "turret");

        innerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        innerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        thetaDiff = Math.abs(getTurretAngle() + getLimeLightReading());
        vG = getSpeed() * Math.cos(thetaDiff);
        dGoal = Math.sqrt(Math.pow(curX, 2) + Math.pow(curY, 2));
        List<Double> result = table.getNumbers(dGoal);
        flightTime = result.get(2);
        dPrime = dGoal - flightTime * vG;
        innerRPM = result.get(0);
        outerRPM = result.get(1);
    }
}
