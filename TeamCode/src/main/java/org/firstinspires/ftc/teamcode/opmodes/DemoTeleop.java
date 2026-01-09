package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.practiceArchive.TurretOLD;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

@TeleOp
@Config
public class DemoTeleop extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    DrumIntakeTurretManager drum;
    Intake intake;
    TurretOLD turretOLD;
    double slowMult = 0.75;
    MecanumDrive drive;
    boolean normLimits;
    Pose2d pose = new Pose2d(0,0,0);
    public static double maxTurLeft = -10000;
    public static double maxTurRight = 10000;
    @Override
    public void init() {
        turretOLD = new TurretOLD(hardwareMap);
        intake = new Intake(hardwareMap, "intake");
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, pose);

        intake.init();
        drum.init(hardwareMap, drive);

        turretOLD.off();
        drum.testMode = true;
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y*slowMult, -gamepad1.left_stick_x*slowMult), -gamepad1.right_stick_x*slowMult));

        if (gamepad1.x) {
            turretOLD.setSimpleVelos(3000,2000);
        } else if (gamepad1.b) {
            turretOLD.off();
        }


        if (gamepad1.right_bumper && turretOLD.getTurPose() > maxTurLeft) {
            turretOLD.setSimpleSpinnerPower(0.5);
        } else if (gamepad1.left_bumper && turretOLD.getTurPose() < maxTurRight ) {
            turretOLD.setSimpleSpinnerPower(-0.5);
        } else {
            turretOLD.setSimpleSpinnerPower(0);
        }

//        if (turret.getTurPose() )

        if (gamepad1.dpad_right) {
            intake.intakeOn();
        } else if (gamepad1.dpad_left) {
            intake.intakeOff();
        }

        if (gamepad1.backWasPressed()) {
            if (slowMult == 0.75) {
                slowMult = 0.375;
            } else {
                slowMult = 0.75;
            }
        } // click options to toggle slowmode

        if (gamepad1.yWasPressed()) {
            drum.toggleManualShoot();
        }

        if (gamepad1.dpadUpWasPressed()) {
            drum.nextSlot();
        } else if (gamepad1.dpadDownWasPressed()) {
            drum.lastSlot();
        }

        if (gamepad1.right_trigger > 0.5 && !drum.isFiring) {
            drum.firePurple();
        }

        intake.loop();
        turretOLD.loop();
        drum.update();
        telemetry.addData("tur pose", turretOLD.getTurPose());
        drum.updateTelemetry(t2);
//        telemetry.addData("within normal limits?");
    }
}
