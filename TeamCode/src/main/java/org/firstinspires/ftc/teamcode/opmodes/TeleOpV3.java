package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
//import org.json.simple.parser.JSONParser;

import java.io.File;
import java.io.FileReader;
import java.util.Arrays;
import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOpV3 extends OpMode {
    MecanumDrive drive;
    DrumIntakeTurretManager drum;
    FtcDashboard dash;
    Telemetry t2;
    double botHeading;
    IMU imu;
    double lastTriggerVal;
    String[] colorsString = {"white", "white", "white"};
    int initPointer = 0;
    boolean blue;
    DrumIntakeTurretManager.revMode tempMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
    @Override
    public void init() {
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, new Pose2d(66,0,-Math.PI));
        drum.init(hardwareMap, DrumIntakeTurretManager.revMode.FIREIDLE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
        // file read
    }
    @Override
    public void init_loop() {
        if (gamepad2.x) {
            drum.setBlue(true);
            blue = true;
        } else if (gamepad2.b) {
            drum.setBlue(false);
            blue = false;
        }

        if (gamepad2.dpadDownWasPressed()) {
            initPointer = Math.abs((initPointer + 2)) % 3;
        } else if (gamepad2.dpadUpWasPressed()) {
            initPointer = Math.abs((initPointer + 1)) % 3;
        }

        if (gamepad2.leftBumperWasPressed()) {
            colorsString[initPointer] = "purple";
        } else if (gamepad2.rightBumperWasPressed()) {
            colorsString[initPointer] = "green";
        } else if (gamepad2.aWasPressed()) {
            colorsString[initPointer] = "white";
        }
        if (gamepad2.yWasPressed()) {
            if (tempMode == DrumIntakeTurretManager.revMode.INTAKEIDLE) {
                tempMode = DrumIntakeTurretManager.revMode.FIREIDLE;
            } else {
                tempMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
            }
        }
        telemetry.addData("isBlue?", blue);
        telemetry.addData("Mode", tempMode);
        telemetry.addData("colors", Arrays.toString(colorsString));
        telemetry.update();
    }
    @Override
    public void start() {
        drum.setStartingColors(colorsString);
        drum.setBlue(blue);
        if (tempMode == DrumIntakeTurretManager.revMode.INTAKEIDLE) {
            drum.resetDrumEnc();
        }
        drum.curMode = tempMode;
        if (blue) {
            drive.localizer.setPose(new Pose2d(66, -17, Math.toRadians(180)));
        } else {
            drive.localizer.setPose(new Pose2d(66, 17, Math.toRadians(180)));
        }
        imu.resetYaw();
    }
    @Override
    public void loop() {
        double gamepady = -gamepad1.left_stick_x;
        double gamepadx = -gamepad1.left_stick_y;
        botHeading = drive.localizer.getPose().heading.toDouble();

        double fieldX = gamepadx * Math.sin(-botHeading) - gamepady * Math.cos(-botHeading);
        double fieldY = gamepadx * Math.cos(-botHeading) + gamepady * Math.sin(-botHeading);

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

        if (gamepad1.right_trigger >= 0.5 && lastTriggerVal < 0.5) {
            drum.curMode = DrumIntakeTurretManager.revMode.INTAKING;
        } else if (gamepad1.right_trigger < 0.5 && drum.curMode == DrumIntakeTurretManager.revMode.INTAKING) {
            drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
        } else if (gamepad1.dpadUpWasPressed()) {
            drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.aWasPressed()) {
            if (drum.lastMode == DrumIntakeTurretManager.revMode.INTAKING) {
                drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
            } else {
                drum.curMode = DrumIntakeTurretManager.revMode.FIREIDLE;
            }
        } else if (gamepad2.yWasPressed()) {
            drum.curMode = DrumIntakeTurretManager.revMode.HPINTAKE;
        }

        if (drum.curMode == DrumIntakeTurretManager.revMode.INTAKEIDLE && drum.isFull()) {
            drum.curMode = DrumIntakeTurretManager.revMode.FIREIDLE;
        }

        if (drum.curMode == DrumIntakeTurretManager.revMode.FIRESTANDBY) {
            if (gamepad1.leftBumperWasPressed()) {
                drum.firePurple();
            } else if (gamepad1.rightBumperWasPressed()) {
                drum.fireGreen();
            } else if (gamepad1.yWasPressed()) {
                drum.startContFire();
            } else if (gamepad1.aWasPressed()) {
                drum.fireSingle();
            }
        }


        if (gamepad2.dpadUpWasPressed()) {
            drum.nextSlot();
        } else if (gamepad1.dpadDownWasPressed()) {
            drum.lastSlot();
        }


        if (gamepad1.startWasPressed()) {
            drive.localizer.setPose(new Pose2d(66,0,-Math.PI));
            imu.resetYaw();
        }
        if (gamepad1.xWasPressed()) {
            drum.curMode = DrumIntakeTurretManager.revMode.SIMPLEFIRE;
        }
        if (gamepad2.xWasPressed()) {
            drum.setBlue(!blue);
            blue = !blue;
        }

        if (gamepad2.rightBumperWasPressed()) {
            drum.setCurrentGreen();
        } else if (gamepad2.leftBumperWasPressed()) {
            drum.setCurrentPurple();
        }
        t2.addData("botheading", botHeading);
        lastTriggerVal = gamepad1.right_trigger;

        PoseVelocity2d vel = drive.updatePoseEstimate();
        drum.update(drive.localizer.getPose(), vel, imu.getRobotYawPitchRollAngles().getYaw() + 180); // degrees
        if (drum.getNewPoseFromTurret() != null) {
            drive.localizer.setPose(drum.getNewPoseFromTurret());
        }
        drum.updateTelemetry(t2);

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.addData("colors", Arrays.toString(drum.getColors()));
        t2.addData("IMU yaw", imu.getRobotYawPitchRollAngles().getYaw() + 180);
        t2.update();
        telemetry.addData("mode", drum.curMode.toString());

        if (drum.seeingTag()) {
            telemetry.addLine("!------------TAG SEEN----------!");
        }

        telemetry.update();
    }
    }
