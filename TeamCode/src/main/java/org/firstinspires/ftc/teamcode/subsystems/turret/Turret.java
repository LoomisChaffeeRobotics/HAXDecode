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
import org.firstinspires.ftc.teamcode.subsystems.TurretPID;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

import java.io.File;
import java.util.List;

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
    double offset = 0;
    Pose2d botpose;
    Pose2d LLPose;
    Pose2d drivePose;
    Limelight3A limelight;
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
    public double innerRPM = 0;
    public int curId = 20;
    public int targId = 20;
    public double outerRPM = 0;
    public double thetaDiff = 0;
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
    public double turPower = 0;
    public boolean usingLLForPose = false;
    public boolean LLonCorrectTag = false;
    double turretCurTicks = 0;
    Pose3D botpose_tag;
    LLResultTypes.FiducialResult fIDGetter;
    int atId;
    public enum turMode {
        FIRING,
        INTAKING,
        IDLE
    }
    public turMode mode = turMode.IDLE;
    public boolean bothMotorsSpunUp = false;
    public boolean successfulShot = false;
    double getGyro(){
        return drivePose.heading.toDouble();
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
        thetaDiff = Math.acos(dotProduct / (prodMagDist + 0.000001));
    }
//    void updateAngleToGoal() {
//        // need to first find difference in angle between distance vector and robot pointed vector (not turret)
//        // then need to optimize in case the difference is over 180 and normalize to robot range (-PI to PI)
//        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
//        double[] unitDirVect = new double[] {Math.cos(getGyro()), Math.sin(getGyro())};
//        angleToGoal = Math.acos((distVect[0] * unitDirVect[0] + distVect[1] + unitDirVect[1])/
//                Math.sqrt(Math.pow(distVect[0],2) + Math.pow(unitDirVect[0],2)));
//        //acos gives 0 to PI, need to check for negative angles
//        // this seems wrong
//        if (getGyro() > Math.atan2(distVect[1], distVect[0])) {
//            angleToGoal = -angleToGoal;
//        };
//    }
    double angleToTicks(double angle) {
        // # ticks = #radians  (turretFullLoop / 2PI)
        return (angle / (2 * Math.PI)) * turretFullLoop;
    }
    void updateOrthogVelMag() {
        double[] distVect = new double[] {goalPose.position.x - botpose.position.x, goalPose.position.y - botpose.position.y};
        double[] orthogGoalVect = new double[] {-distVect[1], distVect[0]};
        orthogVelMag = (robotVelo.linearVel.x * orthogGoalVect[0] + robotVelo.linearVel.y * orthogGoalVect[1]) / (Math.sqrt(Math.pow(orthogGoalVect[0], 2) + Math.pow(orthogGoalVect[1], 2)) +0.00000001);
    }
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("innerTurret targ rpm", innerRPM);
        telemetry.addData("outerTurret targ rpm)", outerRPM);
        telemetry.addData("innerTurret cur rpm",  innerCurVel / RPMtoTicksPerSecond);
        telemetry.addData("outerTurret cur rpm)", outerCurVel/ RPMtoTicksPerSecond);
        telemetry.addData("thetaDiff", thetaDiff);
//        telemetry.addData("angle To Goal (deg)", angleToGoal * 180/Math.PI);
        telemetry.addData("vG", vGoal);
        telemetry.addData("dGoal", dGoal);
        telemetry.addData("flightTime", flightTime);
        telemetry.addData("dGoalEstimate", dGoalEstimate);
        telemetry.addData("curX", botpose.position.x);
        telemetry.addData("curY", botpose.position.y);
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
        telemetry.addData("tag?", usingLLForPose);
        telemetry.addData("ATagID", atId);
        telemetry.addData("tagX", tx);
        telemetry.addData("SuccessShot?", successfulShot);
        if (botpose_tag != null) {
            telemetry.addData("botpose tag", botpose_tag);
        }
        telemetry.update();
    }
    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
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
    }
    public void setBlue(boolean isBlue){
        if (isBlue) {
            goalPose = goalPoseBlue;
            targId = 20;
        } else {
            goalPose = goalPoseRed;
            targId = 24;
        }
    }
    public Pose2d updateLL(double robotYaw) {
        // biggest problem with OTOS is that when it's not clean, translation is unreliable
        // OTOS also has built in IMU but has seemed to perhaps be unreliable as well? may need to retune angularScalar
        // perhaps section off the LL processing to this method instead of loop()

        /* this is going to both return a pose from the LL to fuse later and also update class tag data so
        we can use it in loop, the class tag data is purely for shooting PID
        something like if you see the right tag, return a tx ty ta that's not null, otherwise make it null
        and when you're checking in loop check that it's not null then set the PID accordingly
        and if it's null use turret encoders except we don't want to do that yet because we have < 1 week.
        so later, create some PID code in loop

        the loop will look at data for shooting distance AFTER the pose2d returned by this has been FUSED
        with the other data in LocalizationFuser which might be a problem bc it's kind of cyclical
        cyclicality might not be there if not using heading to talk to LL though?

         */
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) { // if LL available, use LL botpose
            // if there's the right tag in sight, update turret PID to focus on tag
            // means you have to change coeffs to tag mode

            if (!result.getFiducialResults().isEmpty()) {
                curId = result.getFiducialResults().get(0).getFiducialId();
                if (curId != 20 && curId != 24) {
                    usingLLForPose = false;
                    LLonCorrectTag = false;
                    return null;
                } else {
                    if (curId == targId) {
                        tx = result.getTx(); // How far left or right the target is (degrees)
                        ty = result.getTy(); // How far up or down the target is (degrees)
                        ta = result.getTa(); // How big the target looks (0%-100% of the image)
                        LLonCorrectTag = true;
                    }
                    botpose_tag = result.getBotpose_MT2();
                    usingLLForPose = true;
                    LLonCorrectTag = false;
                    return new Pose2d(botpose_tag.getPosition().x*39.37, botpose_tag.getPosition().y*39.37, botpose_tag.getOrientation().getYaw(AngleUnit.RADIANS));
                }
            }
        } else {
            usingLLForPose = false;
            LLonCorrectTag = false;
            return null;
        }

        return null;
    }
    public void loop(Pose2d fusedPose, PoseVelocity2d finalVel, double robotYaw){
        turretCurTicks = -turEnc.getCurrentPosition();
        drivePose = fusedPose;
        robotVelo = finalVel;
        innerCurVel = innerTurret.getVelocity();
        outerCurVel = outerTurret.getVelocity();

        LLPose = updateLL(robotYaw);
        if (LLPose != null) {
            botpose = LLPose;
        } else {
            botpose = drivePose;
        }

        updateTheta();
//        updateAngleToGoal();
        updateOrthogVelMag();
        updateSpeed();
        updateTurretAngle();
        // put stuff about turPID here using tx, ty, ta's nullness and/or values

        // firing stuff?
        vGoal = speed * Math.cos(thetaDiff);
        dGoal = Math.sqrt(Math.pow(botpose.position.x - goalPose.position.x, 2) + Math.pow(botpose.position.y - goalPose.position.y, 2));
        flightTime = getToF(dGoal/39.37);
        calcOffset(dGoal);
        dGoalEstimate = (dGoal) + offset; // temporarily removed velocity compensation


        innerRPM = getLRPM(dGoalEstimate/39.37);
        outerRPM = getURPM(dGoalEstimate/39.37);

        if (mode == turMode.FIRING) {
            // if there is too much lag, only calc velocities when firing
            innerTurret.setVelocity(innerRPM * RPMtoTicksPerSecond);
            outerTurret.setVelocity(outerRPM * RPMtoTicksPerSecond);
            if (((Math.abs(innerRPM - (innerCurVel / RPMtoTicksPerSecond)) < 500) && ((Math.abs(outerRPM - (outerCurVel / RPMtoTicksPerSecond)) < 500)))) {
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
        spinner.setPosition(0);
    }
    public Pose2d getBotpose () {
        return LLPose;
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
        if (distanceFinal < 24) {
            offset = 10;
        } else if (distanceFinal < 60) {
            offset = 10;
        } else if (distanceFinal < 115) {
            offset = 10;
        } else {
            offset = 10 - 0.0909090909 * (distanceFinal - 115);
        }
    }
    public String getCurrentMotif(){
        if (atId == 21){
            return "GPP";
        }
        else if (atId == 22){
            return "PGP";
        }
        else if (atId == 23){
            return "PPG";
        }
        return "null";
    }

}
