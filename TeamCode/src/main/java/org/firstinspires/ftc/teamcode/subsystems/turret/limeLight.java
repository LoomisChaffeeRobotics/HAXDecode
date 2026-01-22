package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class limeLight {
    public double tx;
    public double ty;
    public double ta;
    LLResultTypes.FiducialResult fIDGetter;
    int atId;
    String motifPattern = "GPP";
    public Pose3D botpose;
    Limelight3A limelight;
    public void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        turret = hardwareMap.get(CRServo.class, "turret");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }
    public String getCurrentMotif(){
        if (atId == 21){
            motifPattern = "GPP";
        }
        else if (atId == 22){
            motifPattern = "PGP";
        }
        else if (atId == 23){
            motifPattern = "PPG";
        }
        return motifPattern;
    }
    public void update(double robotYaw, double turretYaw){
        LLResult result = limelight.getLatestResult();
        double LLYaw = calcLimelightYawRadians(robotYaw, turretYaw);
        limelight.updateRobotOrientation(LLYaw);

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                botpose = new Pose3D(botpose_mt2.getPosition(),botpose_mt2.getOrientation());
            }
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)
            fIDGetter = fiducialResults.get(0);
            atId = fIDGetter.getFiducialId();
        }
    }
    double calcLimelightYawRadians(double robot, double turret) {
        return robot + turret;
    }
}
