package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationFuser;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

import java.util.Arrays;
import java.util.Objects;

@Autonomous
@Config
public class Red3BallAuto extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    public static double intArt1 = -34;
    public static double intArtOffset = -34;
    public static double artGap = 3.5;
    public static double curArtNum = 1;
    public static Pose2d START_POSE  = new Pose2d(46,46, Math.toRadians(-140)); //Need to tune these values
    public static Pose2d ATAG_POSE = new Pose2d(20,20, Math.toRadians(-165));
    public static Pose2d SHOOT_POSE  = new Pose2d(20, 20, Math.toRadians(135));
    public static Pose2d END_POSE = new Pose2d(40, 20, Math.toRadians(90));
//    public static Pose2d INTAKE1_POSE = new Pose2d(12, intArtOffset, Math.toRadians(-90));
//    public static Pose2d INTAKE2_POSE = new Pose2d(12, intArtOffset, Math.toRadians(-90));

    public static double GOAL_X = -68;
    public static double GOAL_Y = -53;
    public static double GOAL_H = Math.toRadians(-135);
    public static double INTAKE_SECONDS = 1.0;
    LocalizationFuser fuser;
    DrumIntakeTurretManager drum;
    Action strafeOut = null;
    Action goAprilTagPose = null;
    Action goEndPose = null;
    Action spinUp = null;
    Action shoot = null;
    Action runLL = null;
    int initPointer = 0;
    String motifString = "null";
    String[] colorsString = {"green", "purple", "purple"};
    String alliance;
    @Override
    public void init() {
        fuser = new LocalizationFuser();
        fuser.init(START_POSE, hardwareMap);
        drum = new DrumIntakeTurretManager();
        drum.init(hardwareMap, DrumIntakeTurretManager.revMode.INTAKEIDLE);
        drum.setBlue(true);
    }
    @Override
    public void init_loop () {

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
        telemetry.addData("colors", Arrays.toString(colorsString));
        telemetry.addData("pointer", initPointer);
        telemetry.update();
    }
    @Override
    public void start() {
        drum.setStartingColors(colorsString);
        drum.resetDrumEnc();
        fuser.start(true);
        fuser.setPose(START_POSE);

        runLL = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telmetryPacket) {
                motifString = fuser.getCurrentMotif();
                fuser.loop(0);
                drum.update(fuser.getPose(), fuser.getVelo());
                t2.addData("Drum mode", drum.curMode);
                t2.addLine("motif: " + motifString);
                t2.addData("Spun", drum.shooterSpunUp());
                fuser.updateTelemetry(t2);
                drum.updateTelemetry(t2);
                t2.update();
                if (Objects.equals(motifString, "null")) {
                    return true;
                } else {
                    return false;
                }
            }
        };
        spinUp = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
                fuser.loop(0);
                drum.update(fuser.getPose(), fuser.getVelo());
                t2.addData("Drum mode", drum.curMode);
                t2.addLine("motif: " + motifString);
                t2.addData("Spun", drum.shooterSpunUp());
                fuser.updateTelemetry(t2);
                drum.updateTelemetry(t2);
                t2.update();
                if (drum.shooterSpunUp()) {
                    return false;
                } else {
                    return true;
                }
            }
        };
        shoot = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                int numberEmpty = 0;
                // fire based on motif
                fuser.loop(0);
                drum.update(fuser.getPose(), fuser.getVelo());
                t2.addData("Drum mode", drum.curMode);
                t2.addLine("motif: " + motifString);
                t2.addData("Spun", drum.shooterSpunUp());
                fuser.updateTelemetry(t2);
                drum.updateTelemetry(t2);
                t2.update();

                for (String color : drum.getColors()) {
                    if (Objects.equals(color, "white")) {
                        numberEmpty++;
                    }
                }

                if (numberEmpty == 0) { // haven't started firing yet
                    if (Objects.equals(motifString.split("")[0], "P")) {
                        drum.firePurple();
                    } else if (Objects.equals(motifString.split("")[0], "G")) {
                        drum.fireGreen();
                    } else {
                        drum.fireSingle();
                    }
                    return true;
                } else if (numberEmpty == 1) {
                    if (Objects.equals(motifString.split("")[1], "P")) {
                        drum.firePurple();
                    } else if (Objects.equals(motifString.split("")[1], "G")) {
                        drum.fireGreen();
                    } else {
                        drum.fireSingle();
                    }
                    return true;
                } else if (numberEmpty == 2) {
                    if (Objects.equals(motifString.split("")[2], "P")) {
                        drum.firePurple();
                    } else if (Objects.equals(motifString.split("")[2], "G")) {
                        drum.fireGreen();
                    } else {
                        drum.fireSingle();
                    }
                    return true;
                } else {
                    drum.curMode = DrumIntakeTurretManager.revMode.FIREIDLE;
                    return false;
                }
            }
        };
        goAprilTagPose = fuser.drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(ATAG_POSE.position, ATAG_POSE.heading)
                .afterDisp(10, runLL)
                .stopAndAdd(new InstantFunction() {
                    @Override
                    public void run() {
                        fuser.setPose(ATAG_POSE);
                    }
                })
                .build();
        strafeOut = fuser.drive.actionBuilder(ATAG_POSE)
                .turn(Math.toRadians(60))
                .stopAndAdd(new InstantFunction() {
                    @Override
                    public void run() {
                        fuser.setPose(SHOOT_POSE);
                    }
                })
                .build();
        goEndPose = fuser.drive.actionBuilder(SHOOT_POSE)
                .strafeTo(END_POSE.position)
                .stopAndAdd(new InstantFunction() {
                    @Override
                    public void run() {
                        fuser.setPose(END_POSE);
                    }
                })
                .build();
    }
    @Override
    public void loop () {

        Actions.runBlocking(new SequentialAction(
                goAprilTagPose,
                strafeOut,
                shoot,
                goEndPose
        ));

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), fuser.getPose());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        drum.updateTelemetry(t2);

    }

}