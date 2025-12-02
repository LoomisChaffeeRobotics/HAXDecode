package org.firstinspires.ftc.teamcode.subsystems.turret;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

public class limeLight {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    public double tx;
    public double ty;
    public double ta;
//    public double power;
    public Pose3D botpose;
    public double robotYaw;
    Limelight3A limelight;
    CRServo turret;
    FtcDashboard dashboard;
    IMU imu;
    void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
//        turret = hardwareMap.get(CRServo.class, "turret");
        dashboard = FtcDashboard.getInstance();
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }
    void update(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            botpose = result.getBotpose();
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            t2.addData("Target X", tx);
            t2.addData("Target Y", ty);
            t2.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
            telemetry.addData("result", result);
            t2.addData("Limelight", "No Targets");
            t2.addData("result", result);
//            power = 0;
        }
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            t2.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
        }

        robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                t2.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
        telemetry.update();
        t2.update();
//        turret.setPower(power);
    }
}
