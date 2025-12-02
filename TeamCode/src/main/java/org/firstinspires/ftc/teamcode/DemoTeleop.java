package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.practiceArchive.DrumColorTracker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

public class DemoTeleop extends OpMode {
    DrumIntakeTurretManager drum;
    Intake intake;
    Turret turret;
    double slowMult;
    MecanumDrive drive;
    Pose2d pose = new Pose2d(0,0,0);
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap, "intake");
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, pose);

        intake.init();
        drum.init(hardwareMap);

        turret.off();
        drum.testMode = true;
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y*slowMult, -gamepad1.left_stick_x*slowMult), -gamepad1.right_stick_x*slowMult));

        if (gamepad1.x) {
            turret.setSimpleVelos(3000,2000);
        } else if (gamepad1.b) {
            turret.off();
        }

        if (gamepad1.right_bumper) {
            turret.setSimpleSpinnerPower(0.5);
        } else if (gamepad1.left_bumper) {
            turret.setSimpleSpinnerPower(-0.5);
        } else {
            turret.setSimpleSpinnerPower(0);
        }

        if (gamepad1.dpad_right) {
            intake.intakeOn();
        } else if (gamepad1.dpad_left) {
            intake.intakeOff();
        }

        if (gamepad1.optionsWasPressed()) {
            if (slowMult == 0.75) {
                slowMult = 0.375;
            } else {
                slowMult = 0.75;
            }
        } // click options to toggle slowmode

        if (gamepad1.dpad_up) {
            drum.toggleManualShoot();
        }

        if (gamepad1.dpadUpWasPressed()) {
            drum.nextSlot();
        } else if (gamepad1.dpadDownWasPressed()) {
            drum.lastSlot();
        }

        if (gamepad1.right_trigger > 0.5) {
            drum.fire();
        }

        intake.loop();
        turret.loop();
        drum.update();
    }
}
