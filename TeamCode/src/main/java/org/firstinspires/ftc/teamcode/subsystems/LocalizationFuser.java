package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;
public class LocalizationFuser {
    // research "ransack algorithm"
    MecanumDrive drive;
    public Pose2d OdoPose;
    public Pose2d LLPose;

    public Pose2d ReturnPose;
    
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
    public double tx;
    public double ty;
    public double ta;
    int atId;
    public double curImuYaw = 0;

    double heading;
    public void init(Pose2d startPose, IMU initializedIMU) {
        this.imu = initializedIMU;
        IMUOffsetRad = startPose.heading.toDouble();
    }
    public void resetIMU(double resetHeading) {
        imu.resetYaw();
        IMUOffsetRad = resetHeading;
    }

    LLResultTypes.FiducialResult fIDGetter;

    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        turret = hardwareMap.get(CRServo.class, "turret");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    public void updateLL(double robotYaw) {
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) { // if LL available, use LL botpose
            // if there's the right tag in sight, update turret PID to focus on tag
            // means you have to change coeffs to tag mode
            if (!result.getFiducialResults().isEmpty()) {
                int curId = result.getFiducialResults().get(0).getFiducialId();
                if (curId != 20 && curId != 24) {
                    usingLLForPose = false;
                    LLonCorrectTag = false;
                    LLPose = null;
                } else {
                    if (curId == targId) {
                        tx = result.getTx(); // How far left or right the target is (degrees)
                        ty = result.getTy(); // How far up or down the target is (degrees)
                        ta = result.getTa(); // How big the target looks (0%-100% of the image)
                        LLonCorrectTag = true;
                    }
                    Pose3D temp = result.getBotpose_MT2();
                    LLPose = new Pose2d(temp.getPosition().x*39.37, temp.getPosition().y*39.37, Math.toRadians(temp.getOrientation().getYaw()));
                    PrevPose = LLPose;
                    usingLLForPose = true;
                    LLonCorrectTag = false;
                    lastUsingLL = true;
                }
            }
        } else {
            usingLLForPose = false;
            LLonCorrectTag = false;
            LLPose = null;
        }
        LLPose = null;
    }
    public double calcLimelightYawRadians(double turretyaw){
        return 0;
    } //Need to fix

    public Pose2d loop(double turretYaw, Pose2d drPose) {
        curImuYaw = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()) + IMUOffsetRad;
        limelight.updateRobotOrientation(curImuYaw);

        LLResult result = limelight.getLatestResult();
        
        updateLL(curImuYaw);

        if(usingLLForPose){
            return ReturnPose = LLPose;
        }
        else {
            if (lastUsingLL) {
                OdoPose = PrevPose;
                ReturnPose = OdoPose;
                lastUsingLL = false;
            }
            else {
                drPose = drive.localizer.getPose();
                ReturnPose = drPose;
                OdoPose = drPose;
            }
            return ReturnPose;
        }
    }

        //heading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()) + IMUOffsetRad;
        //return new Pose2d(0,0,heading);

}
