package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

@Autonomous(group = "comp", preselectTeleOp = "TeleopMode")
@Config
public class AutoRedFarSide extends LinearOpMode {

    Pose2d startPose = new Pose2d(-48,49.5, Math.toRadians(180));
    DrumIntakeTurretManager drum;
    HardwareMap hardwareMap;
    MecanumDrive drive;
    Intake intake;
    public void runOpMode() throws InterruptedException {
        if (opModeInInit()) {
            drum = new DrumIntakeTurretManager();
            intake = new Intake(hardwareMap, "intake");
        }
    }

}
