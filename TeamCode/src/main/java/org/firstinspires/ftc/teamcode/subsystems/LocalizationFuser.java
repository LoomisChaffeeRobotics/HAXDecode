package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
public class LocalizationFuser {
    // research "ransack algorithm"
    public Pose2d OdoPose;
    public Pose3D LLPose;
    public IMU imu;
    public double IMUOffsetRad;
    public boolean LLisActive = false;

    Limelight3A limelight;

    public double tx;
    public double ty;
    public double ta;
    int atId;
    public double curImuYaw;

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

    public double calcLimelightYawRadians(double turretyaw){
        return 0;
    }
    public Pose2d loop(double turretYaw, Pose2d drPose) {
        curImuYaw = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()) + IMUOffsetRad;
        limelight.updateRobotOrientation(curImuYaw);

        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                LLPose = new Pose3D(botpose_mt2.getPosition(), botpose_mt2.getOrientation());

            }
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)
            fIDGetter = fiducialResults.get(0);
            atId = fIDGetter.getFiducialId();
            heading = calcLimelightYawRadians(tx);
            return new Pose2d(LLPose.getPosition().x, LLPose.getPosition().y, heading);

        } else {
            heading = curImuYaw;
            return drPose;
        }
    }

        //heading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()) + IMUOffsetRad;
        //return new Pose2d(0,0,heading);

}
