package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;
import org.firstinspires.ftc.teamcode.subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

import java.io.File;
import java.util.List;

@Config
public class Turret {
    double[][] LUT1 = new double[][]{
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

    double[][] LUT = new double[][]{
            {0.5206, 1.7931e+03, 969.3000, 0.5200},
            {0.6779, 2.0028e+03, 916.6700, 0.6300},
            {0.8223, 2.2191e+03, 869.3000, 0.7100},
            {0.9786, 2.4588e+03, 825.4400, 0.7800},
            {1.1761, 2.7317e+03, 783.3300, 0.8600},
            {1.3627, 3.0013e+03, 730.7000, 0.9200},
            {1.5521, 3.2809e+03, 662.2800, 0.9700},
            {1.7140, 3.5006e+03, 621.9300, 1.0100},
            {1.8953, 3.7369e+03, 588.6000, 1.0500},
            {2.1189, 4.0065e+03, 558.7700, 1.1000},
            {2.3237, 4.2494e+03, 541.2300, 1.1400},
            {2.4965, 4.4558e+03, 525.4400, 1.1700},
            {2.6907, 4.6888e+03, 509.6500, 1.2000},
            {2.6907, 4.6889e+03, 509.6500, 1.2000},
            {2.9347, 4.9684e+03, 500.8800, 1.2400},
            {2.9347, 4.9684e+03, 500.8800, 1.2400},
    };
    public double tx = 0;
    public double ty;
    public double ta;
    public static double kPEnc = 0.005;
    public static double kIEnc = 0;
    public static double kDEnc = 0.0001;
    public static double kFR = 0.1;
    public static double kFV = 0;
    public static double kPTag = 0.02;
    public static double kITag = 0;
    public static double kDTag = 0.001;
    public static double kPInnerVelo = 180;
    public static double kIInnerVelo = 0;
    public static double kDInnerVelo = 40;
    public static double kFVelo = 13;
    public static double kPOuter = 90;
    public static double kIOuter = 0;
    public static double kDOuter = 0;
    public static double kFOuter = 13;
    public static double outerGain = 0.5;
    public static double innerGain = 0.5;
    double offset = 0;
    Pose2d botpose;
    Servo spinner;
    DcMotorEx turEnc;
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    Pose2d goalPoseBlue = new Pose2d(-68, -53, Math.toRadians(-135));
    Pose2d goalPoseRed = new Pose2d(-68, 53, Math.toRadians(-135));
    Pose2d goalPose = goalPoseBlue;
    String logFilePath = String.format("%s/FIRST/lastInfo.json", Environment.getExternalStorageDirectory().getAbsolutePath());
    File dataLog = AppUtil.getInstance().getSettingsFile(logFilePath);
    TurretPID turPID;
    FancyPID veloPID;
    public double innerRPM = 0;
    public double outerRPM = 0;
    public double veloGoalAngle = 0;
    public double roboRelativeAngleToGoal = 0;
    static double turretFullLoop = (double) 8192 * 75 / 15; // # ticks for loop
    static double RPMtoTicksPerSecond = (double) 28 / 60;
    public double speed = 0;
    public double vGoal = 0;
    public double dGoal = 0;
    public double turretAngle = 0;
    public PoseVelocity2d robotVelo = new PoseVelocity2d(new Vector2d(0,0),0);
//    public double angleToGoal = 0;
    public double orthogVelMag = 0;
    public double dGoalEstimate = 0;
    public double flightTime = 0;
    public double innerCurVel;
    public double outerCurVel;
    double filteredInner = 0;
    double filteredOuter = 0;
    public double turPower = 0;
    double turretCurTicks = 0;
    public enum turMode {
        FIRING,
        INTAKING,
        IDLE
    }
    public turMode mode = turMode.IDLE;
    public boolean bothMotorsSpunUp = false;
    public boolean successfulShot = false;
    public boolean addedOffset = false;
    public boolean usingLL = false;
    double getGyro(){
        return botpose.heading.toDouble();
    }
    void updateTurretAngle() {
        turretAngle = - 2 * Math.PI * (turretCurTicks / turretFullLoop);
    }
    void updateSpeed(){
        speed = Math.sqrt(Math.pow(robotVelo.linearVel.x, 2) + Math.pow(robotVelo.linearVel.y, 2));
    }
    void updateTheta() { // angle between robot velocity and vector to goal
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        double[] velVect = new double[] {robotVelo.linearVel.x, robotVelo.linearVel.y};
        double dotProduct = distVect[0] * velVect[0] + distVect[1] * velVect[1];
        double prodMagDist = Math.sqrt(Math.pow(distVect[0], 2) + Math.pow(distVect[1], 2)) * Math.sqrt(Math.pow(velVect[1],2) + Math.pow(velVect[0],2));
        veloGoalAngle = Math.acos(dotProduct / (prodMagDist + 0.000001));
    }
    public void updateLLState(boolean usingLL){
        this.usingLL = usingLL;
    }
    public void updateAngleToGoal() {
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        double thetaBot = getGyro();
        double thetaGoal = Math.atan2(distVect[1], distVect[0]);
        roboRelativeAngleToGoal = thetaGoal - thetaBot;
    }
    double angleToTicks(double angle) {
        // # ticks = #radians  (turretFullLoop / 2PI)
        return (angle / (2 * Math.PI)) * turretFullLoop;
    }
    void updateOrthogVelMag() {
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        double magVelo = Math.sqrt(Math.pow(robotVelo.linearVel.x, 2) + Math.pow(robotVelo.linearVel.y, 2));
        double dotProduct = distVect[0] * robotVelo.linearVel.x + distVect[1] * robotVelo.linearVel.y;
        double magDist = Math.sqrt(Math.pow(distVect[0], 2) + Math.pow(distVect[1], 2));
        double cosineTheta = dotProduct/(magVelo * magDist);

        double[] projectedVector = new double[] {magVelo * cosineTheta * (distVect[0]/magDist), magVelo * cosineTheta * (distVect[1]/magDist)};
        double[] orthogVelVector = new double[] {robotVelo.linearVel.x - projectedVector[0], robotVelo.linearVel.y - projectedVector[1]};
        orthogVelMag = Math.sqrt(Math.pow(orthogVelVector[0], 2) + Math.pow(orthogVelVector[1], 2)) * Math.signum(dotProduct);
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("innerTurret targ rpm", innerRPM);
        telemetry.addData("outerTurret targ rpm)", outerRPM);
        telemetry.addData("innerTurret cur rpm",  innerCurVel / RPMtoTicksPerSecond);
        telemetry.addData("outerTurret cur rpm)", outerCurVel/ RPMtoTicksPerSecond);
        telemetry.addData("veloGoalAngle", veloGoalAngle);
//        telemetry.addData("angle To Goal (deg)", angleToGoal * 180/Math.PI);
        telemetry.addData("vG", vGoal);
        telemetry.addData("dGoal", dGoal);
        telemetry.addData("flightTime", flightTime);
        telemetry.addData("dGoalEstimate", dGoalEstimate);
        telemetry.addData("curX", botpose.position.x);
        telemetry.addData("curY", botpose.position.y);
//        telemetry.addData("diffX", botpose.position.x - goalPose.position.x);
//        telemetry.addData("diffY", botpose.position.y - goalPose.position.y);
//        telemetry.addData("squareX", (botpose.position.x - goalPose.position.x)*(botpose.position.x - goalPose.position.x));
//        telemetry.addData("squareY", (botpose.position.y - goalPose.position.y)*(botpose.position.y - goalPose.position.y));
//        telemetry.addData("rootDist", Math.sqrt((botpose.position.y - goalPose.position.y)*(botpose.position.y - goalPose.position.y) +(botpose.position.x - goalPose.position.x)*(botpose.position.x - goalPose.position.x)));
        telemetry.addData("turretAngle (deg)", turretAngle * 180/Math.PI);
        telemetry.addData("gyro", getGyro());
        telemetry.addData("limelightX", tx);
        telemetry.addData("turretTarget", turPID.target);
        telemetry.addData("turretTicks", turretCurTicks);
        telemetry.addData("orthogVelMag", orthogVelMag);
        telemetry.addData("turretPower", turPID.out);
        telemetry.addData("angVel", robotVelo.angVel);
        telemetry.addData("speed", speed);
        telemetry.addData("spun up", bothMotorsSpunUp);
        telemetry.addData("offset", offset);
        telemetry.addData("SuccessShot?", successfulShot);
        telemetry.addData("inner pids", innerTurret.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("inner algo", innerTurret.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).algorithm.toString());
        telemetry.addData("outer pids", outerTurret.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
        telemetry.addData("outer algo", outerTurret.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).algorithm.toString());
    }
    public void init(HardwareMap hardwareMap){
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.start(); // This tells Limelight to start looking!
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");
        turEnc = hardwareMap.get(DcMotorEx.class, "FL");
//        spinner = hardwareMap.get(CRServo.class, "turret");
        spinner = hardwareMap.get(Servo.class, "turret");

        innerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        innerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turEnc.setDirection(DcMotorSimple.Direction.REVERSE);

//        fileDataRaw = ReadWriteFile.readFile(dataLog);
//        vals = fileDataRaw.split(", ");
//        lastPoseEstimate = new Pose2d(Double.parseDouble(vals[0]), Double.parseDouble(vals[1]), Double.parseDouble(vals[2]));
//        lastPoseEstimate = new Pose2d(0,0,0);
//        this.drive = drive;
        mode = turMode.IDLE;

        turPID = new TurretPID();
        turPID.init();
        veloPID = new FancyPID();
        veloPID.init();
    }
    public void setBlue(boolean isBlue){
        if (isBlue) {
            goalPose = goalPoseBlue;
        } else {
            goalPose = goalPoseRed;
        }
    }
    public void updateUpperFilter(double velo) {
        filteredOuter = (outerGain * velo) + ((1-outerGain) * filteredOuter);
    }
    public void updateLowerFilter(double velo) {
        filteredInner = (innerGain * velo) + ((1-innerGain) * filteredInner);
    }
    public void loop(Pose2d fusedPose, PoseVelocity2d finalVel){
        turretCurTicks = -turEnc.getCurrentPosition();
        botpose = fusedPose;
        robotVelo = finalVel;
        innerCurVel = innerTurret.getVelocity();
        outerCurVel = outerTurret.getVelocity();
        updateLowerFilter(innerCurVel);
        updateUpperFilter(outerCurVel); // feedback is filtered, output is not (doesn't need to be)
        // we can change the pid coeffs first. if that doesn't work, we use the veloPID
//
        innerTurret.setVelocityPIDFCoefficients(kPInnerVelo, kIInnerVelo, kDInnerVelo, kFVelo);
        outerTurret.setVelocityPIDFCoefficients(kPInnerVelo, kIInnerVelo, kDInnerVelo, kFVelo);
        updateTheta();
//        updateAngleToGoal();
        updateOrthogVelMag();
        updateSpeed();
        updateTurretAngle();
        // put stuff about turPID here using tx, ty, ta's nullness and/or values

        // firing stuff?
        vGoal = speed * Math.cos(veloGoalAngle);
        dGoal = Math.sqrt(((botpose.position.x - goalPose.position.x) * (botpose.position.x - goalPose.position.x)) + ((botpose.position.y - goalPose.position.y)*(botpose.position.y - goalPose.position.y)));
        flightTime = getToF(dGoal/39.37);
        calcOffset(dGoal);
        dGoalEstimate = (dGoal) + offset; // temporarily removed velocity compensation


        innerRPM = getLRPM(dGoalEstimate/39.37);
        outerRPM = getURPM(dGoalEstimate/39.37);

        if (mode == turMode.FIRING) {
            // if there is too much lag, only calc velocities when firing
            innerTurret.setVelocity(innerRPM * RPMtoTicksPerSecond);
            outerTurret.setVelocity(outerRPM * RPMtoTicksPerSecond);
            if (((Math.abs(innerRPM - (innerCurVel / RPMtoTicksPerSecond)) < 200) && ((Math.abs(outerRPM - (outerCurVel / RPMtoTicksPerSecond)) < 200)))) {
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

        if (usingLL) {
            turPID.setCoefficients(kPTag, kITag, kDTag, kFR, kFV);
            turPID.target = 0;
            turPID.update(tx, robotVelo.angVel, orthogVelMag);
        } else {
            turPID.setCoefficients(kPEnc, kIEnc, kDEnc, kFR, kFV);
            turPID.target = angleToTicks(roboRelativeAngleToGoal);
            turPID.update(turretCurTicks, robotVelo.angVel, orthogVelMag);
        }

        spinner.setPosition(0);
    }
    double getLRPM(double dist) {
        for (int i = 0; i < LUT.length; i++) {
            if (LUT[i][0] < dist && i < LUT.length - 1) {
                double slope = (LUT[i + 1][1] - LUT[i][1]) / (LUT[i + 1][0] - LUT[i][0]);
                return LUT[i][1] + slope * (dist - LUT[i][0]);
            } else if (LUT[i][0] < dist && i == LUT.length - 1) {
                return LUT[i][1];
            } else {
                double slope = (LUT[1][1]-LUT[0][1]) / (LUT[1][0] - LUT[0][0]);
                return LUT[0][1] - slope * (LUT[i][0] - dist);
            }
        }
        return 0;
    }
    double getURPM(double dist) {
        for (int i = 0; i < LUT.length; i++) {
            if (LUT[i][0] > dist && i < (LUT.length - 1)) {
                double slope = (LUT[i + 1][2] - LUT[i][2]) / (LUT[i + 1][0] - LUT[i][0]);
                return LUT[i][2] + slope * (dist - LUT[i][0]);
            } else if (LUT[i][0] < dist && i == LUT.length - 1) {
                return LUT[i][2];
            } else {
                double slope = (LUT[1][2]-LUT[0][2]) / (LUT[1][0] - LUT[0][0]);
                return LUT[0][2] - slope * (LUT[i][0] - dist);
            }
        }
        return 0;
    }
    double getToF(double dist) {
        for (int i = 0; i < LUT.length; i++) {
            if (LUT[i][0] > dist && i < (LUT.length - 1)) {
                double slope = (LUT[i + 1][3] - LUT[i][3]) / (LUT[i + 1][0] - LUT[i][0]);
                return LUT[i][3] + slope * (dist - LUT[i][0]);
            } else if (LUT[i][0] < dist && i == LUT.length - 1) {
                return LUT[i][3];
            } else {
                double slope = (LUT[1][3]-LUT[0][3]) / (LUT[1][0] - LUT[0][0]);
                return LUT[0][3] - slope * (LUT[i][0] - dist);
            }
        }
        return 0;
    }
    public void calcOffset(double distanceFinal) {
        if (addedOffset) {
            if (distanceFinal < 24) {
                offset = 12;
            } else if (distanceFinal < 60) {
                offset = 11;
            } else if (distanceFinal < 115) {
                offset = 9;
            } else {
                offset = 9 - 0.0909090909 * (distanceFinal - 115);
            }
        } else {
            if (distanceFinal < 24) {
                offset = 10;
            } else if (distanceFinal < 60) {
                offset = 9;
            } else if (distanceFinal < 115) {
                offset = 9;
            } else {
                offset = 9 - 0.0909090909 * (distanceFinal - 115);
            }
        }
    }
    public void toggleAddedOffset() {
        addedOffset = !addedOffset;
    }
}
