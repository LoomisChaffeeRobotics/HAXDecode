package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.TurretPID;

import java.io.File;
public class Turret {
    double[][] LUT = new double[][]{};
    public double tx;
    public double ty;
    public double ta;
    public static double kPEnc = 0;
    public static double kIEnc = 0;
    public static double kDEnc = 0;
    public static double kFREnc = 0;
    public static double kFVEnc = 0;
    public static double kPTag = 0;
    public static double kITag = 0;
    public static double kDTag = 0;
    public static double kFRTag = 0;
    public static double kFVTag = 0;
    Pose2d botpose;
    Limelight3A limelight;
    CRServo spinner;
    DcMotorEx turEnc;
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    MecanumDrive drive;
    Pose2d lastPoseEstimate;
    Pose2d goalPose = new Pose2d(-70, -60, 0);
    String logFilePath = String.format("%s/FIRST/lastRunPoseLog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    File dataLog = AppUtil.getInstance().getSettingsFile(logFilePath);
    String tableFilePath = String.format("%s/FIRST/LUT.csv", Environment.getExternalStorageDirectory().getAbsolutePath());
    File tableCSV = AppUtil.getInstance().getSettingsFile(tableFilePath);
    String csvDataRaw;
    String fileDataRaw;
    String[] vals;
    TurretPID turPID;
    Telemetry t2;
    FtcDashboard dash;
    public static double innerRPM = 0;
    public static double outerRPM = 0;
    public double thetaDiff = 0;
    static double turretFullLoop = (double) 8192 * 75 / 15;
    static double RPMtoTicksPerSecond = (double) 28 /60;
    public double vGoal = 0;
    public double dGoal = 0;
    public double dGoalEstimate = 0;
    public double flightTime = 0;
    public boolean firing = false;
    double getGyro(){
        return drive.localizer.getPose().heading.toDouble();
    }
    double getTurretAngle() {
        return 2 * Math.PI * (turEnc.getCurrentPosition() / turretFullLoop);
    }
    double[] getRobotVelo(){
        return new double[]{drive.updatePoseEstimate().linearVel.x, drive.updatePoseEstimate().linearVel.y};
    }
    double getSpeed(){
        double[] velos = getRobotVelo();
        return Math.sqrt(Math.pow(velos[0], 2) + Math.pow(velos[1], 2));
    }
    double getTheta() { // angle between robot velocity and vector to goal
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        double[] velVect = getRobotVelo();
        double dotProduct = distVect[0] * velVect[0] + distVect[1] * velVect[1];
        double prodMagDist = Math.sqrt(Math.pow(distVect[0], 2) + Math.pow(distVect[1], 2)) * Math.sqrt(Math.pow(velVect[1],2) + Math.pow(velVect[0],2));
        return Math.acos(dotProduct / (prodMagDist + 0.000001));
    }
    double getAngleToGoal() {
        return Math.atan2(goalPose.position.y - botpose.position.y, goalPose.position.x - botpose.position.x);
    }
    double angleToTicks(double angle) {
        return (angle / (2 * Math.PI)) * turretFullLoop;
    }
    double getOrthogVel() {
        double[] velVect = getRobotVelo();
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        return (velVect[0] * distVect[1] - velVect[1] * distVect[0]) / Math.sqrt(Math.pow(distVect[0], 2) + Math.pow(distVect[1], 2)); // i sure hope copilot did the projection right!
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("innerTurret rpm", innerRPM);
        telemetry.addData("outerTurret rpm)", outerRPM);
        telemetry.addData("thetaDiff", thetaDiff);
        telemetry.addData("vG", vGoal);
        telemetry.addData("dGoal", dGoal);
        telemetry.addData("flightTime", flightTime);
        telemetry.addData("dGoalEstimate", dGoalEstimate);
        telemetry.addData("curX", botpose.position.x);
        telemetry.addData("curY", botpose.position.y);
        telemetry.addData("turretAngle", getTurretAngle());
        telemetry.addData("gyro", getGyro());
        telemetry.addData("limelightX", tx);
        telemetry.addData("speed", getSpeed());
        telemetry.update();
    }
    public void init(HardwareMap hardwareMap, MecanumDrive drive){
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        turret = hardwareMap.get(CRServo.class, "turret");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");
        turEnc = hardwareMap.get(DcMotorEx.class, "Enc");
        spinner = hardwareMap.get(CRServo.class, "turret");

        innerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        innerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turEnc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turEnc.setDirection(DcMotorSimple.Direction.REVERSE);

        fileDataRaw = ReadWriteFile.readFile(dataLog);
        vals = fileDataRaw.split(", ");
        lastPoseEstimate = new Pose2d(Double.parseDouble(vals[0]), Double.parseDouble(vals[1]), Double.parseDouble(vals[2]));
        this.drive = drive;

        csvDataRaw = ReadWriteFile.readFile(tableCSV);
        String[] rows = fileDataRaw.split("\n"); // cut the big string into rows
        for (int i = 0; i < rows.length; i++) {
            String[] entries = rows[i].split(", "); // cut each row into string of number entries
            double[] doubleRow = new double[]{entries.length}; // make a temporary array to hold numbers before they get added
            for (int j = 0; j < entries.length; j++) {
                double curNum = Double.parseDouble(entries[j]); // iterate through the row and turn strings into doubles
                doubleRow[j] = curNum; // add the doubles to temp array
            }
            LUT[i] = doubleRow; // add the full array of doubles in that row to the LUT
        }

        turPID = new TurretPID();
        turPID.init();
    }
    public void loop(){
        LLResult result = limelight.getLatestResult();
        double LLYaw = (getGyro() + getTurretAngle());
        limelight.updateRobotOrientation(LLYaw);

        if (result != null && result.isValid()) { // if LL available, use LL botpose
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                botpose = new Pose2d(botpose_mt2.getPosition().x, botpose_mt2.getPosition().y, botpose_mt2.getOrientation().getYaw());
                drive.localizer.setPose(botpose);
            } else {
                botpose = drive.localizer.getPose();
            }
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)

            // if there's the right tag in sight, update turret PID to focus on tag
            // means you have to change coeffs to tag mode
            if (!result.getFiducialResults().isEmpty() && result.getFiducialResults().get(0).getFiducialId() == 22) {
                turPID.setCoefficients(kPTag, kITag, kDTag, kFRTag, kFVTag);
                turPID.target = 0;
                turPID.update(tx, drive.localizer.update().angVel, getOrthogVel());
            } else {   // if it's the wrong tag, update turret PID based on encoder
                turPID.setCoefficients(kPEnc, kIEnc, kDEnc, kFREnc, kFVEnc);
                turPID.target = angleToTicks(getAngleToGoal());
                turPID.update(turEnc.getCurrentPosition(), drive.localizer.update().angVel, getOrthogVel());
            }
        } else {
            // if there's no tag in sight, update turret PID based on encoder
            botpose = drive.localizer.getPose(); // I worry drive will gain error if SparkFun gets dusty
            turPID.setCoefficients(kPEnc, kIEnc, kDEnc, kFREnc, kFVEnc);
            turPID.target = angleToTicks(getAngleToGoal());
            turPID.update(turEnc.getCurrentPosition(), drive.localizer.update().angVel, getOrthogVel());
        }

        thetaDiff = getTheta();
        vGoal = getSpeed() * Math.cos(thetaDiff);
        dGoal = Math.sqrt(Math.pow(botpose.position.x - goalPose.position.x, 2) + Math.pow(botpose.position.y - goalPose.position.y, 2));
        flightTime = getToF(dGoal);
        dGoalEstimate = dGoal - vGoal * flightTime;

        innerRPM = getLRPM(dGoalEstimate);
        outerRPM = getURPM(dGoalEstimate);

        if (firing) {
            // if there is too much lag, only calc velocities when firing
            innerTurret.setVelocity(innerRPM * RPMtoTicksPerSecond);
            outerTurret.setVelocity(outerRPM * RPMtoTicksPerSecond);
        } else {
            innerTurret.setVelocity(0);
            outerTurret.setVelocity(0);
        }

        spinner.setPower(turPID.out);
        updateTelemetry(t2);
    }
    double getLRPM(double dist) {
        for (int i = 0; i < LUT.length; i++) {
            if (LUT[i][0] < dist && i < LUT.length - 1) {
                double slope = (LUT[i + 1][1] - LUT[i][1]) / (LUT[i + 1][0] - LUT[i][0]);
                return LUT[i][1] + slope * (dist - LUT[i][0]);
            } else if (LUT[i][0] < dist && i == LUT.length - 1) {
                return LUT[i][1];
            }
        }
        return 0;
    }
    double getURPM(double dist) {
        for (int i = 0; i < LUT.length; i++) {
            if (LUT[i][0] < dist && i < LUT.length - 1) {
                double slope = (LUT[i + 1][2] - LUT[i][2]) / (LUT[i + 1][0] - LUT[i][0]);
                return LUT[i][2] + slope * (dist - LUT[i][0]);
            } else if (LUT[i][0] < dist && i == LUT.length - 1) {
                return LUT[i][2];
            }
        }
        return 0;
    }
    double getToF(double dist) {
        for (int i = 0; i < LUT.length; i++) {
            if (LUT[i][0] < dist && i < LUT.length - 1) {
                double slope = (LUT[i + 1][3] - LUT[i][3]) / (LUT[i + 1][0] - LUT[i][0]);
                return LUT[i][3] + slope * (dist - LUT[i][0]);
            } else if (LUT[i][0] < dist && i == LUT.length - 1) {
                return LUT[i][3];
            }
        }
        return 0;
    }
}
