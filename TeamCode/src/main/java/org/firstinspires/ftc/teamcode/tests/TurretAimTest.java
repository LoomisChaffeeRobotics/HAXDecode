package org.firstinspires.ftc.teamcode.tests;

import android.os.Environment;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.turret.limeLight;
import org.firstinspires.ftc.teamcode.subsystems.turret.lookUpTable;

import java.io.File;
import java.io.FileReader;
import java.util.List;

@TeleOp
public class TurretAimTest extends OpMode {
    lookUpTable<Double> table = new lookUpTable<>();
    limeLight LL;
    CRServo spinner;
    DcMotor turEnc;
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    MecanumDrive drive;
    Pose2d lastPoseEstimate;
    Pose2d goalPose = new Pose2d(-70, -60, 0);
    String logFilePath = String.format("%s/FIRST/lastRunPoseLog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    File dataLog = AppUtil.getInstance().getSettingsFile(logFilePath);
    String fileDataRaw;
    String[] vals;
    public static double innerRPM = 0;
    public static double outerRPM = 0;
    public double thetaDiff = 0;
    public double turretFullLoop = 0;
    public double vGoal = 0;
    public double curX = 0;
    public double curY = 0;
    public double dGoal = 0;
    public double flightTime = 0;
    public double dPrime = 0;
    double getLimeLightX(){
        return LL.tx;
    }
    Pose2d getLimelightPose() { return new Pose2d(LL.botpose.getPosition().x,LL.botpose.getPosition().y,LL.botpose.getOrientation().getYaw()); }
    double getGyro(){
        return LL.robotYaw;
    }
    double getTurretAngle(){
        return 360 * (turEnc.getCurrentPosition() / turretFullLoop);
    }
    double[] getRobotVelo(){
        return new double[]{drive.updatePoseEstimate().linearVel.x, drive.updatePoseEstimate().linearVel.y};
    }
    double getSpeed(){
        double[] velos = getRobotVelo();
        return Math.sqrt(Math.pow(velos[0], 2) + Math.pow(velos[1], 2));
    }
    double getTheta() {
        double[] distVect = new double[] {goalPose.position.x - lastPoseEstimate.position.x, goalPose.position.y - lastPoseEstimate.position.y};
        double[] velVect = getRobotVelo();
        double dotProduct = distVect[0] * velVect[0] + distVect[1] * velVect[1];
        double prodMagDist = Math.sqrt(Math.pow(distVect[0], 2) + Math.pow(distVect[1], 2)) * Math.sqrt(Math.pow(velVect[1],2) + Math.pow(velVect[0],2));
        return Math.acos(dotProduct / (prodMagDist + 0.000001));
    }
    void updateTelemetry(){
        telemetry.addData("innerTurret rpm", innerRPM);
        telemetry.addData("outerTurret rpm)", outerRPM);
        telemetry.addData("thetaDiff", thetaDiff);
        telemetry.addData("vG", vGoal);
        telemetry.addData("dGoal", dGoal);
        telemetry.addData("flightTime", flightTime);
        telemetry.addData("dPrime", dPrime);
        telemetry.addData("curX", curX);
        telemetry.addData("curY", curY);
        telemetry.addData("turretAngle", getTurretAngle());
        telemetry.addData("gyro", getGyro());
        telemetry.addData("limelightX", getLimeLightX());
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

        fileDataRaw = ReadWriteFile.readFile(dataLog);
        vals = fileDataRaw.split(", ");
        lastPoseEstimate = new Pose2d(Double.parseDouble(vals[0]), Double.parseDouble(vals[1]), Double.parseDouble(vals[2]));
        drive = new MecanumDrive(hardwareMap, lastPoseEstimate);
    }

    @Override
    public void loop(){


        thetaDiff = getTheta();
        vGoal = getSpeed() * Math.cos(thetaDiff);
        dGoal = Math.sqrt(Math.pow(curX, 2) + Math.pow(curY, 2));
        List<Double> result = table.getNumbers(dGoal);
        flightTime = result.get(2);
        dPrime = dGoal - flightTime * vGoal;
        innerRPM = result.get(0);
        outerRPM = result.get(1);

        updateTelemetry();
    }
}
