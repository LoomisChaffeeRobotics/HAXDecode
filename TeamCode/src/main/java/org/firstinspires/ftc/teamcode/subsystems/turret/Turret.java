package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Turret {
    double[][] LUT = new double[][]{
            {0.5, 1116.1, 1690.7, 0.6},
            {0.6, 1153.9, 1806.6, 0.7},
            {0.7, 1218.7, 1931.4, 0.8},
            {0.8, 1283.5, 2065.1, 0.8},
            {0.9, 1375.3, 2180.9, 0.9},
            {1.0, 1456.3, 2279.0, 1.0},
            {1.1, 1553.5, 2368.1, 1.1},
            {1.2, 1639.8, 2448.3, 1.1},
            {1.2, 1747.8, 2492.9, 1.1},
            {1.3, 1866.6, 2546.3, 1.2},
            {1.4, 1996.2, 2608.7, 1.2},
            {1.5, 2104.2, 2635.5, 1.3},
            {1.6, 2217.6, 2662.2, 1.3},
            {1.7, 2331.0, 2662.2, 1.3},
            {1.8, 2460.6, 2635.5, 1.4},
            {1.9, 2611.8, 2582.0, 1.4},
            {1.9, 2709.0, 2519.6, 1.4},
            {2.0, 2817.0, 2439.4, 1.4},
            {2.1, 2935.7, 2341.4, 1.4},
            {2.1, 3049.1, 2234.4, 1.4},
            {2.2, 3189.5, 2118.5, 1.4},
            {2.2, 3270.5, 2020.5, 1.4},
            {2.3, 3389.3, 1931.4, 1.4},
            {2.4, 3518.9, 1851.2, 1.4},
            {2.5, 3659.3, 1744.2, 1.4},
            {2.5, 3778.1, 1646.2, 1.3},
            {2.6, 3923.9, 1557.0, 1.3},
            {2.7, 4069.7, 1459.0, 1.3},
            {2.8, 4242.4, 1369.9, 1.4},
            {2.8, 4345.0, 1298.6, 1.3},
            {2.9, 4469.2, 1236.2, 1.3},
            {3.0, 4582.6, 1191.6, 1.4},
            {3.1, 4717.6, 1129.2, 1.4},
            {3.2, 4858.0, 1075.8, 1.4},
            {3.3, 4993.0, 1031.2, 1.4},
            {3.4, 5101.0, 1022.3, 1.4},
            {3.4, 5214.4, 1031.2, 1.4},
            {3.6, 5360.2, 1031.2, 1.4},
            {3.6, 5360.2, 1031.2, 1.4}
    };
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
    public static double offset = 20;
    Pose2d botpose = new Pose2d(0, 0, 0);
    Limelight3A limelight;
    CRServo spinner;
    DcMotorEx turEnc;
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    MecanumDrive drive;
    Pose2d lastPoseEstimate;
    Pose2d goalPose = new Pose2d(0, 0, 0);
    public static double goalPoseX;
    public static double goalPoseY;
    public static double goalPoseH;
    String logFilePath = String.format("%s/FIRST/lastRunPoseLog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    File dataLog = AppUtil.getInstance().getSettingsFile(logFilePath);
    String fileDataRaw;
    TurretPID turPID;
    public double innerRPM = 0;
    public double outerRPM = 0;
    public double thetaDiff = 0;
    static double turretFullLoop = (double) 8192 * 75 / 15;
    static double RPMtoTicksPerSecond = (double) 28 /60;
    public double speed = 0;
    public double vGoal = 0;
    public double dGoal = 0;
    public double turretAngle = 0;
    public double[] robotVelo = new double[2];
    public double angleToGoal = 0;
    public double orthogVelMag = 0;
    public double dGoalEstimate = 0;
    public double flightTime = 0;
    public double innerCurVel;
    public double outerCurVel;
    public enum turMode {
        FIRING,
        INTAKING,
        IDLE
    }
    public turMode mode = turMode.IDLE;
    public boolean bothMotorsSpunUp = false;
    double getGyro(){
        return drive.localizer.getPose().heading.toDouble();
    }
    void updateTurretAngle() {
        turretAngle = 2 * Math.PI * (turEnc.getCurrentPosition() / turretFullLoop);
    }
    void updateRobotVelo(){
        robotVelo = new double[]{drive.updatePoseEstimate().linearVel.x, drive.updatePoseEstimate().linearVel.y};
    }
    void updateSpeed(){
        double velos[] = new double[robotVelo.length];
        for (int i = 0; i < robotVelo.length; i++) {
            velos[i] = robotVelo[i]/39.37;
        }
        speed = Math.sqrt(Math.pow(velos[0], 2) + Math.pow(velos[1], 2));
    }
    void updateTheta() { // angle between robot velocity and vector to goal
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        double[] velVect = robotVelo;
        double dotProduct = distVect[0] * velVect[0] + distVect[1] * velVect[1];
        double prodMagDist = Math.sqrt(Math.pow(distVect[0], 2) + Math.pow(distVect[1], 2)) * Math.sqrt(Math.pow(velVect[1],2) + Math.pow(velVect[0],2));
        thetaDiff = Math.acos(dotProduct / (prodMagDist + 0.000001));
    }
    void updateAngleToGoal() {
        angleToGoal = Math.atan2(goalPose.position.y - botpose.position.y, goalPose.position.x - botpose.position.x);
    }
    double angleToTicks(double angle) {
        return (angle / (2 * Math.PI)) * turretFullLoop;
    }
    void updateOrthogVelMag() {
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        double[] orthogGoalVect = new double[] {-distVect[1], distVect[0]};
        orthogVelMag = (robotVelo[0] * orthogGoalVect[0] + robotVelo[1] * orthogGoalVect[1]) / Math.sqrt(Math.pow(orthogGoalVect[0], 2) + Math.pow(orthogGoalVect[1], 2));
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("innerTurret targ rpm", innerRPM);
        telemetry.addData("outerTurret targ rpm)", outerRPM);
        telemetry.addData("innerTurret cur rpm",  innerCurVel / RPMtoTicksPerSecond);
        telemetry.addData("outerTurret cur rpm)", outerCurVel/ RPMtoTicksPerSecond);
        telemetry.addData("thetaDiff", thetaDiff);
        telemetry.addData("vG", vGoal);
        telemetry.addData("dGoal", dGoal);
        telemetry.addData("flightTime", flightTime);
        telemetry.addData("dGoalEstimate", dGoalEstimate);
        telemetry.addData("curX", botpose.position.x);
        telemetry.addData("curY", botpose.position.y);
        telemetry.addData("turretAngle", turretAngle);
        telemetry.addData("gyro", getGyro());
        telemetry.addData("limelightX", tx);
        telemetry.addData("speed", speed);
        telemetry.update();
    }
    public void init(HardwareMap hardwareMap, MecanumDrive drive){

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        turret = hardwareMap.get(CRServo.class, "turret");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");
        turEnc = hardwareMap.get(DcMotorEx.class, "FL");
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

//        fileDataRaw = ReadWriteFile.readFile(dataLog);
//        vals = fileDataRaw.split(", ");
//        lastPoseEstimate = new Pose2d(Double.parseDouble(vals[0]), Double.parseDouble(vals[1]), Double.parseDouble(vals[2]));
        lastPoseEstimate = new Pose2d(0,0,0);
        this.drive = drive;
        mode = turMode.IDLE;

        turPID = new TurretPID();
        turPID.init();
    }
    public void loop(){
        LLResult result = limelight.getLatestResult();
        double LLYaw = (getGyro() + turretAngle);
        limelight.updateRobotOrientation(LLYaw);
        innerCurVel = innerTurret.getVelocity();
        outerCurVel = outerTurret.getVelocity();

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
                turPID.update(tx, drive.localizer.update().angVel, orthogVelMag);
            } else {   // if it's the wrong tag, update turret PID based on encoder
                turPID.setCoefficients(kPEnc, kIEnc, kDEnc, kFREnc, kFVEnc);
                turPID.target = angleToTicks(angleToGoal);
                turPID.update(turEnc.getCurrentPosition(), drive.localizer.update().angVel, orthogVelMag);
            }
        } else {
            // if there's no tag in sight, update turret PID based on encoder
            botpose = drive.localizer.getPose(); // I worry drive will gain error if SparkFun gets dusty
            turPID.setCoefficients(kPEnc, kIEnc, kDEnc, kFREnc, kFVEnc);
            turPID.target = angleToTicks(angleToGoal);
            turPID.update(turEnc.getCurrentPosition(), drive.localizer.update().angVel, orthogVelMag);
        }
        updateTheta();
        updateAngleToGoal();
        updateOrthogVelMag();
        updateRobotVelo();
        updateSpeed();
        updateTurretAngle();

        // firing stuff?
        vGoal = speed * Math.cos(thetaDiff);
        dGoal = Math.sqrt(Math.pow(botpose.position.x - goalPose.position.x, 2) + Math.pow(botpose.position.y - goalPose.position.y, 2));
        flightTime = getToF(dGoal);
        dGoalEstimate = (dGoal - vGoal * flightTime) + offset;


        innerRPM = getLRPM(dGoalEstimate/39.37);
        outerRPM = getURPM(dGoalEstimate/39.37);

        if (mode == turMode.FIRING) {
            // if there is too much lag, only calc velocities when firing
            innerTurret.setVelocity(innerRPM * RPMtoTicksPerSecond);
            outerTurret.setVelocity(outerRPM * RPMtoTicksPerSecond);
            if (Math.abs(innerRPM - innerCurVel / RPMtoTicksPerSecond) < 100 && Math.abs(outerRPM - outerCurVel / RPMtoTicksPerSecond) < 100) {
                bothMotorsSpunUp = true;
            } else {
                bothMotorsSpunUp = false;
            }
        } else if (mode == turMode.INTAKING) {
            innerTurret.setVelocity(-1500 * RPMtoTicksPerSecond);
            outerTurret.setVelocity(-1500 * RPMtoTicksPerSecond);
            bothMotorsSpunUp = false;
        } else {
            innerTurret.setVelocity(0);
            outerTurret.setVelocity(0);
            bothMotorsSpunUp = false;
        }

        goalPose = new Pose2d(goalPoseX, goalPoseY, goalPoseH);
        spinner.setPower(0.03);
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
            if (LUT[i][0] < dist && i < (LUT.length - 1)) {
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
