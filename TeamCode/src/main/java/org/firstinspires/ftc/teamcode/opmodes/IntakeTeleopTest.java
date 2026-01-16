package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class IntakeTeleopTest extends OpMode {
    MecanumDrive drive;
    DrumIntakeTurretManager drum;
    FtcDashboard dash;
    Telemetry t2;
    double botHeading;
    IMU imu;
    double lastTriggerVal;
    @Override
    public void init() {
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.PI/2));
        drum.init(hardwareMap);
        drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
        double gamepady = -gamepad1.left_stick_x;
        double gamepadx = -gamepad1.left_stick_y;
//        botHeading = drive.localizer.getPose().heading.toDouble();

        double fieldX = gamepady;
        double fieldY = gamepadx;

        if (gamepad1.left_trigger > 0.5) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0.3 * gamepadx, 0.3 * gamepady),
                    -0.3 * gamepad1.right_stick_x
            ));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepadx,
                            gamepady
                    ),
                    -gamepad1.right_stick_x
            ));
        }
        // else {if (curMode == INTAKING) do intake idle, else if everything}
        if (gamepad1.right_trigger >= 0.5 && lastTriggerVal < 0.5) {
            drum.curMode = DrumIntakeTurretManager.revMode.INTAKING;
        } else if (gamepad1.right_trigger < 0.5 && drum.curMode == DrumIntakeTurretManager.revMode.INTAKING) {
            drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
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

        if (drum.curMode == DrumIntakeTurretManager.revMode.INTAKEIDLE && drum.isFull()) {
            drum.curMode = DrumIntakeTurretManager.revMode.FIREIDLE;
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
        }

        if (drum.curMode == DrumIntakeTurretManager.revMode.INTAKING) {
            if (gamepad2.xWasPressed()) {
                drum.setCurrentPurple();
            } else if (gamepad1.bWasPressed()) {
                drum.setCurrentGreen();
            }
        }
        if (gamepad2.rightBumperWasPressed()) {
            drum.nextSlot();
        } else if (gamepad2.leftBumperWasPressed()) {
            drum.lastSlot();
        }
        t2.addData("botheading", botHeading);
        lastTriggerVal = gamepad1.right_trigger;

        drive.updatePoseEstimate();
        drum.update(drive.localizer.getPose(), new PoseVelocity2d(new Vector2d(0,0),0));
        drive.localizer.setPose(drum.getNewPoseFromTurret());
        drum.updateTelemetry(t2);
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
