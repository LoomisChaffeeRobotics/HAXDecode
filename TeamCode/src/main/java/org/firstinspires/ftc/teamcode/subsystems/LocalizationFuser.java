package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LocalizationFuser {
    public Pose2d OdoPose;
    public Pose3D LLPose;
    public IMU imu;
    public double IMUOffsetRad;
    public void init(Pose2d startPose, IMU initializedIMU) {
        this.imu = initializedIMU;
        IMUOffsetRad = startPose.heading.toDouble();
    }
    public void resetIMU(double resetHeading) {
        imu.resetYaw();
        IMUOffsetRad = resetHeading;
    }
    public Pose2d loop() {
        
        double heading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()) + IMUOffsetRad;
        return new Pose2d(0,0,heading);
    }
}
