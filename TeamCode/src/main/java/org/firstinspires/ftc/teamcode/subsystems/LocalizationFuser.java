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
    MecanumDrive drive;
    public Pose2d OdoPose;
    public Pose2d LLPose;

    public Pose2d finalPose;
    
    public IMU imu;
    
    public Pose2d PrevPose;
    public double IMUOffsetRad;
    public boolean usingLLForPose = false;

    Limelight3A limelight;

    Pose2d goalPoseBlue = new Pose2d(-68, -53, Math.toRadians(-135));
    Pose2d goalPoseRed = new Pose2d(-68, 53, Math.toRadians(-135));
    Pose2d goalPose = goalPoseBlue;
    Pose3D botpose_tag;
    public int targId = 20;
    public boolean LLonCorrectTag = false;
    public boolean lastUsingLL = false;
    public boolean isFinalPoseNull = false;
    public double tx;
    public double ty;
    public double ta;
    int curId;
    public double curImuYaw = 0;
    double heading;
    PoseVelocity2d velo;
    public void init(Pose2d startPose, HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        IMUOffsetRad = startPose.heading.toDouble();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        drive = new MecanumDrive(hardwareMap, new Pose2d(66,0,-Math.PI));
    }
    public void start(boolean blue) {
        if (blue) {
            drive.localizer.setPose(new Pose2d(66, -17, Math.toRadians(180)));
        } else {
            drive.localizer.setPose(new Pose2d(66, 17, Math.toRadians(180)));
        }
    }
    public void resetDrive() {
        imu.resetYaw();
        drive.localizer.setPose(new Pose2d(66,0,-Math.PI));
    }
    public void updateLL(double robotYawDeg) {
        limelight.updateRobotOrientation(robotYawDeg);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) { // if LL available, use LL botpose
            // if there's the right tag in sight, update turret PID to focus on tag
            // means you have to change coeffs to tag mode
            if (!result.getFiducialResults().isEmpty()) {
                curId = result.getFiducialResults().get(0).getFiducialId();
                if (curId != 20 && curId != 24) {
                    usingLLForPose = false;
                    LLonCorrectTag = false;
                } else {
                    if (curId == targId) {
                        tx = result.getTx(); // How far left or right the target is (degrees)
                        ty = result.getTy(); // How far up or down the target is (degrees)
                        ta = result.getTa(); // How big the target looks (0%-100% of the image)
                        LLonCorrectTag = true; // use this boolean later for turret spinning
                    } else {
                        LLonCorrectTag = false;
                    }
                    Pose3D temp = result.getBotpose_MT2();
                    PrevPose = LLPose;
                    LLPose = new Pose2d(temp.getPosition().x*39.37, temp.getPosition().y*39.37, Math.toRadians(robotYawDeg));
                    usingLLForPose = true;
                    lastUsingLL = true;
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

        /*if (Math.abs(drive.localizer.getPose().heading.toDouble() - curImuYaw) > Math.toRadians(90)) {
            drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position, curImuYaw)); // assume IMU is most reliable
        }*/

        updateLL(Math.toDegrees(curImuYaw));

        if (usingLLForPose) {
            lastUsingLL = true;
            finalPose = LLPose;
        }
        else {
            if (lastUsingLL) {
                drive.localizer.setPose(PrevPose);
                lastUsingLL = false;
                finalPose = PrevPose;
            }
            else {
                finalPose = drive.localizer.getPose();
            }
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
        drive.setDrivePowers(powers);
    }
    public boolean seeingTag() {
        return usingLLForPose;
    }
    public String getCurrentMotif(){
        if (curId == 21){
            return "GPP";
        }
        else if (curId == 22){
            return "PGP";
        }
        else if (curId == 23){
            return "PPG";
        }
        return "null";
    }
    public void updateTelemetry(Telemetry t) {
        t.addData("Using LLPose?", usingLLForPose);
        t.addData("Right goal tag?", LLonCorrectTag);
        t.update();
    }
}
