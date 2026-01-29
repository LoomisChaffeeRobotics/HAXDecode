package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Environment;
import android.util.JsonReader;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.google.gson.JsonParser;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.*;
//import org.json.simple.parser.JSONParser;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Objects;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleopV2 extends OpMode {
    MecanumDrive drive;
    DrumIntakeTurretManager drum;
    FtcDashboard dash;
    Telemetry t2;
    double botHeading;
    IMU imu;
    double lastTriggerVal;
    JSONArray jsonArray;
    JSONObject jsonObject;
    JSONArray pose;
    JSONArray colors;
    String alliance;
    String mode;
    String[] colorsString;
    Pose2d startPose;
    String filelogPath = String.format("/%s/FIRST/lastInfo.json", Environment.getExternalStorageDirectory().getAbsolutePath());
    File dataLog = AppUtil.getInstance().getSettingsFile(filelogPath);
    int initPointer = 0;
    @Override
    public void init() {
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.PI/2));
        drum.init(hardwareMap, DrumIntakeTurretManager.revMode.FIREIDLE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN)));
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
        // file read
        try {
            //JSONParser parser = new JSONParser();
            //jsonArray = (JSONArray) parser.parse(new FileReader(dataLog));
            jsonObject = jsonArray.getJSONObject(0);
        } catch (Exception e) {
            telemetry.addData("error", e.toString());
            telemetry.addLine("Failed to read file");
            telemetry.update();
        }
        // then turn jsons into usable data
        try {
            pose = jsonObject.getJSONArray("pose");
            colors = jsonObject.getJSONArray("colors");
            alliance = jsonObject.get("alliance").toString();
            mode = jsonObject.get("mode").toString();
            colorsString = new String[] {
                    colors.getString(0),
                    colors.getString(1),
                    colors.getString(2)
            };
            startPose = new Pose2d(
                    pose.getDouble(0),
                    pose.getDouble(1),
                    pose.getDouble(2)
            );
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
    }
    @Override
    public void init_loop() {
        if (gamepad1.x) {
            alliance = "blue";
        } else if (gamepad1.b) {
            alliance = "red";
        }

        if (gamepad1.dpadDownWasPressed()) {
            initPointer = Math.abs((initPointer + 2)) % 3;
        } else if (gamepad1.dpadUpWasPressed()) {
            initPointer = Math.abs((initPointer + 1)) % 3;
        }

        if (gamepad1.leftBumperWasPressed()) {
            colorsString[initPointer] = "purple";
        } else if (gamepad1.rightBumperWasPressed()) {
            colorsString[initPointer] = "green";
        } else if (gamepad1.aWasPressed()) {
            colorsString[initPointer] = "white";
        }

        if (gamepad1.yWasPressed()) {
            if (Objects.equals(mode, "INTAKEIDLE") || Objects.equals(mode, "INTAKING")) {
                mode = "FIREIDLE";
            } else {
                mode = "INTAKEIDLE";
            }
        }

        telemetry.addData("alliance", alliance);
        telemetry.addData("mode", mode);
        telemetry.addData("colors", colors);
        telemetry.addData("pose", startPose);
        telemetry.update();
    }
    @Override
    public void start() {
        drum.setBlue(Objects.equals(alliance, "blue"));
        drum.setStartingColors(colorsString);
        drive.localizer.setPose(startPose);
        drum.curMode = DrumIntakeTurretManager.revMode.valueOf(mode);
    }
    @Override
    public void loop() {
        double gamepady = -gamepad1.left_stick_x;
        double gamepadx = -gamepad1.left_stick_y;
        botHeading = drive.localizer.getPose().heading.toDouble();

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
                drum.firePurple();
            } else if (gamepad1.rightBumperWasPressed()) {
                drum.fireGreen();
            } else if (gamepad1.yWasPressed()) {
                drum.startContFire();
            } else if (gamepad1.aWasPressed()) {
                drum.fireSingle();
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

        if (gamepad1.startWasPressed()) {
            drive.localizer.setPose(new Pose2d(66,0,-Math.PI));
        }

        if (gamepad2.rightBumperWasPressed()) {
            drum.nextSlot();
        } else if (gamepad2.leftBumperWasPressed()) {
            drum.lastSlot();
        }
        t2.addData("botheading", botHeading);
        lastTriggerVal = gamepad1.right_trigger;

        drive.updatePoseEstimate();
//        drum.update(drive.localizer.getPose(), new PoseVelocity2d(new Vector2d(0,0),0), imu.getRobotYawPitchRollAngles().getYaw());

//        drive.localizer.setPose(drum.getNewPoseFromTurret());
        drum.updateTelemetry(t2);

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    @Override
    public void stop() {
        Pose2d pose = drive.localizer.getPose();
        StringBuilder b = new StringBuilder();
        String[] colors = drum.getColors();
        // pose
        b.append("{\n" + "    \"pose\": {\n" + "\"x\": ")
                .append(pose.position.x).append(",\n")
                .append("\"y\": ")
                .append(pose.position.y)
                .append(",\n")
                .append("\"heading\": ")
                .append(pose.heading.toDouble())
                .append("\n },\n");
        // colors
        b.append("\"colors\": ")
                .append(Arrays.toString(colors))
                .append(",\n");
        // alliance
        b.append("\"alliance\": \"")
                .append(alliance)
                .append("\",\n")
                .append("}");
        // mode
        b.append("\"mode\": \"")
                .append(drum.curMode.name())
                .append("\"\n");
        String string = b.toString();

        // update dataLog with the endpoint information for next run
        ReadWriteFile.writeFile(dataLog, string);
    }
}
