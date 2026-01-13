package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends OpMode {
    MecanumDrive drive;
    DrumIntakeTurretManager drum;
    FtcDashboard dash;
    Telemetry t2;
    double botHeading;
    @Override
    public void init() {
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        drum.init(hardwareMap, drive);
        drum.curMode = DrumIntakeTurretManager.revMode.HPINTAKE;
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Y stick value is reversed
        double x = gamepad1.left_stick_x;
        botHeading = drive.localizer.getPose().heading.real;

        double fieldX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double fieldY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        if (gamepad1.left_trigger > 0.5) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0.3 * fieldY, 0.3 * fieldX),
                    -0.3 * gamepad1.right_stick_x
            ));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(fieldY, fieldX),
                    -gamepad1.right_stick_x
            ));
        }

        if (gamepad1.right_trigger > 0.5) {
            drum.curMode = DrumIntakeTurretManager.revMode.INTAKING;
        } else if (gamepad1.dpadUpWasPressed()) {
            drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.backWasPressed()) {
            if (drum.lastMode == DrumIntakeTurretManager.revMode.INTAKING) {
                drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
            } else {
                drum.curMode = DrumIntakeTurretManager.revMode.FIREIDLE;
            }
        } else if (gamepad2.startWasPressed()) {
            drum.curMode = DrumIntakeTurretManager.revMode.HPINTAKE;
        }

        if (drum.curMode == DrumIntakeTurretManager.revMode.FIRESTANDBY) {
            if (gamepad1.leftBumperWasPressed()) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIREPURPLE;
            } else if (gamepad1.rightBumperWasPressed()) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIREGREEN;
            } else if (gamepad1.yWasPressed()) {
                drum.curMode = DrumIntakeTurretManager.revMode.CONTFIRE;
            } else if (gamepad1.aWasPressed()) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIRESINGLE;
            }
        }

        if (drum.curMode == DrumIntakeTurretManager.revMode.HPINTAKE) {
            if (gamepad2.xWasPressed()) {
                drum.setCurrentPurple();
            } else if (gamepad1.bWasPressed()) {
                drum.setCurrentGreen();
            }
            if (gamepad2.rightBumperWasPressed()) {
                drum.nextSlot();
            } else if (gamepad2.leftBumperWasPressed()) {
                drum.lastSlot();
            }
        }

        if (drum.curMode == DrumIntakeTurretManager.revMode.INTAKING) {
            if (gamepad2.xWasPressed()) {
                drum.setCurrentPurple();
            } else if (gamepad1.bWasPressed()) {
                drum.setCurrentGreen();
            }
            if (gamepad2.rightBumperWasPressed()) {
                drum.nextSlot();
            } else if (gamepad2.leftBumperWasPressed()) {
                drum.lastSlot();
            }
        }

        drum.update();
        drum.updateTelemetry(t2);
    }
}
