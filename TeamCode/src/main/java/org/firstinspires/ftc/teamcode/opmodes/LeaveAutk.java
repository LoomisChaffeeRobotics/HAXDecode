package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.practiceArchive.TurretOLD;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

import java.util.Arrays;

@Autonomous
@Config
public class LeaveAutk extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    DrumIntakeTurretManager drum;
    Intake intake;
    TurretOLD turretOLD;
    MecanumDrive drive;
    boolean normLimits;
    public static double maxTurLeft = -10000;
    public static double maxTurRight = 10000;
    Pose2d target = new Pose2d(-66, -15, Math.toRadians(-160));
    String[] colorsString = {"white", "white", "white"};
    int initPointer = 0;
    boolean blue;
    public enum Mode {
        START,
        FIRING,
        DRIVING,
        STOP
    }
    DrumIntakeTurretManager.revMode tempMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
    Mode curMode = Mode.START;
    Mode lastMode = Mode.START;
    Action action;
    @Override
    public void init() {
        turretOLD = new TurretOLD(hardwareMap);
        intake = new Intake(hardwareMap, "intake");
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, new Pose2d(-66, -7, Math.toRadians(-160)));

        intake.init();
        drum.init(hardwareMap);

        turretOLD.off();
        drum.testMode = true;

    }
    @Override
    public void init_loop() {
        if (gamepad1.x) {
            drum.setBlue(true);
            blue = true;
        } else if (gamepad1.b) {
            drum.setBlue(false);
            blue = false;
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
        curMode = Mode.FIRING;
        action = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(target.position)
                .build();
        if (blue) {
            drive.localizer.setPose(new Pose2d(66, -7, Math.toRadians(-160)));
             target = new Pose2d(-66, -15, Math.toRadians(-160));
        } else {
            drive.localizer.setPose(new Pose2d(66, 7, Math.toRadians(160)));
             target = new Pose2d(-66, 15, Math.toRadians(160));
        }
    }

    @Override
    public void loop() {
        drum.update(drive.localizer.getPose(), drive.updatePoseEstimate());
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
        if (curMode == Mode.FIRING) {
            if (lastMode != curMode) {
                drum.startContFire();
            }

            if (drum.curMode != DrumIntakeTurretManager.revMode.CONTFIRE) {
                curMode = Mode.DRIVING;
            }
        } else if (curMode == Mode.DRIVING) {
            Actions.runBlocking(action);
            if (drive.localizer.getPose().position.x < 20) {
                curMode = Mode.STOP;
            }
        } else if (curMode == Mode.STOP) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        }

//        drum.update(drive.localizer.getPose(), drive.localizer.update());
        telemetry.addData("tur pose", turretOLD.getTurPose());
        drum.updateTelemetry(telemetry);
        drum.updateTelemetry(t2);
        lastMode = curMode;

//        telemetry.addData("within normal limits?");
    }
}