package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;
public class LocalizationFuser {
    // research "ransack algorithm"
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    double angVelPID;
    FancyPID rotPID = new FancyPID();
    public MecanumDrive drive;
    public Pose2d LLPose = new Pose2d(0,0,0);
    public Pose2d finalPose = new Pose2d(0,0,0);

    public IMU imu;
    public Pose2d PrevPose = new Pose2d(0,0,0);
    public double IMUOffsetRad;
    public boolean usingLLForPose = false;
    Limelight3A limelight;
    Pose2d goalPoseBlue = new Pose2d(-68, -53, Math.toRadians(-135));
    Pose2d goalPoseRed = new Pose2d(-68, 53, Math.toRadians(-135));
    public static double goalX = 0;
    public static double goalY = 0;
    public static double goalH = 0;
    Pose2d goalPose = new Pose2d(goalX, goalY, goalH);
    public int targId = 20;
    public double tagDist = 0;
    public boolean LLonCorrectTag = false;
    public boolean lastUsingLL = false;
    public boolean isFinalPoseNull = false;
    public boolean driveTargeting = false;
    public double tx;
    public double ty;
    public double ta;
    int curId;
    public double curImuYaw = 0;
    double heading;
    double latency = 0;
    double robotAngleToGoal = 0;
    double obeliskID = 0;
    PoseVelocity2d velo;
    public void init(Pose2d startPose, HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu2");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        IMUOffsetRad = startPose.heading.toDouble();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
        limelight.start(); // This tells Limelight to start looking!
        drive = new MecanumDrive(hardwareMap, new Pose2d(66,0,-Math.PI));
    }
    public void start(boolean blue) {
        drive.localizer.setPose(new Pose2d(66, 0, -Math.toRadians(180)));
        if (blue) {
            goalPose = goalPoseBlue;
        } else {
            goalPose = goalPoseRed;
        }
    }
    public void calcGoalAngleDiff(Pose2d bot) {
        double[] distVect = new double[] {goalPose.position.x - bot.position.x, goalPose.position.y - bot.position.y};
        double thetaBot = curImuYaw;
        double thetaGoal = Math.atan2(distVect[1], distVect[0]);
         robotAngleToGoal = thetaGoal - thetaBot;
    }
    public void resetDrive() {
        imu.resetYaw();
        drive.localizer.setPose(new Pose2d(66,0,-Math.PI));
    }
    public void updateLL() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) { // if LL available, use LL botpose
            // if there's the right tag in sight, update turret PID to focus on tag
            // means you have to change coeffs to tag mode
            if (!result.getFiducialResults().isEmpty()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    if (tag.getFiducialId() != 20 && tag.getFiducialId() != 24) {
                        obeliskID = tag.getFiducialId();
                    }
                }
                curId = result.getFiducialResults().get(0).getFiducialId();
                if (curId != 20 && curId != 24) {
                    usingLLForPose = false;
                    LLonCorrectTag = false;
                } else {
                    if (curId == targId) {
                        tx = result.getTx(); // How far left or right the target is (degrees)
                        ty = result.getTy(); // How far up or down the target is (degrees)
                        ta = result.getTa(); // How big the target looks (0%-100% of the image)
                        latency = result.getTargetingLatency();
                        LLonCorrectTag = true; // use this boolean later for turret spinning
                    } else {
                        LLonCorrectTag = false;
                    }
                    Pose3D temp = result.getBotpose_MT2();
                    if (temp.getPosition().x == 0 && temp.getPosition().y == 0 || result.getTargetingLatency() > 35) {
                        usingLLForPose = false;
                    } else {
                        PrevPose = new Pose2d(LLPose.position.x, LLPose.position.y, LLPose.heading.toDouble());
                        LLPose = new Pose2d(temp.getPosition().x*39.37, temp.getPosition().y*39.37, curImuYaw);
                        usingLLForPose = true;
                        lastUsingLL = true;
                    }
                }
            }
        } else {
            usingLLForPose = false;
            LLonCorrectTag = false;
        }
    }
    public double calcLimelightYawRadians(double turretyaw){
        return 0;
    } //Need to fix
    public void loop(double turretYawRad) {
        double imuReadingRad = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw());
        if (Math.signum(imuReadingRad) == -1) {
            IMUOffsetRad = Math.toRadians(180);
        } else {
            IMUOffsetRad = -Math.toRadians(180);
        }
        curImuYaw = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()) + IMUOffsetRad;

        limelight.updateRobotOrientation(Math.toDegrees(curImuYaw));
        updateLL();

        if (usingLLForPose) {
            lastUsingLL = true;
            finalPose = LLPose;
            drive.localizer.setPose(finalPose);
        }
        else {
            if (lastUsingLL) {
                drive.localizer.setPose(PrevPose);
                lastUsingLL = false;
                finalPose = PrevPose;
            }
            else {
                finalPose = new Pose2d(drive.localizer.getPose().position, curImuYaw);
            }
        }

        calcGoalAngleDiff(finalPose);
        if (driveTargeting) {
            rotPID.setCoefficients(kP, kI, kD);
            rotPID.target = robotAngleToGoal;
            rotPID.update(curImuYaw);
            angVelPID = rotPID.velo;
        }

        velo = drive.updatePoseEstimate();
    }
    public double getTagX() {
        return tx;
    }
    public PoseVelocity2d getVelo() {
        return velo;
    }
    public Pose2d getPose() {
        return finalPose;
    }
    public double getYaw() {
        return curImuYaw;
    }
    public void setDrivePowers(PoseVelocity2d powers) {
        if (!driveTargeting) {
            drive.setDrivePowers(powers);
        } else {
            drive.setDrivePowers(new PoseVelocity2d(powers.linearVel, angVelPID));
        }
    }
    public void toggleDriveTargeting() {
        driveTargeting = !driveTargeting;
    }
    public boolean seeingTag() {
        return usingLLForPose;
    }
    public String getCurrentMotif(){
        if (obeliskID == 21){
            return "GPP";
        }
        else if (obeliskID == 22){
            return "PGP";
        }
        else if (obeliskID == 23){
            return "PPG";
        }
        return "null";
    }
    public void setPose(Pose2d pose) {
        drive.localizer.setPose(pose);
        finalPose = pose;
    }
    public void updateTelemetry(Telemetry t) {
        t.addData("Using LLPose?", usingLLForPose);
        t.addData("Right goal tag?", LLonCorrectTag);
        t.addData("fused pose heading", finalPose.heading.toDouble());
        t.addData("imu heading", curImuYaw);
        t.addData("tagX", tx);
        t.addData("latency", latency);
    }
}
