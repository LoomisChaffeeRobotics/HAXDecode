package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationFuser;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;
import java.util.Objects;

@Autonomous
@Config
public class Blue3BallAuto extends OpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    enum State {
        DRIVE_TO_SHOT,
        SPINUP_AND_AIM,
        INTAKE,
        FIRE_GREEN,
        FIRE_PURPLE,
        DRIVE_TO_END,
        DONE
    }
    public static double intArt1 = -34;
    public static double intArtOffset = -34;
    public static double artGap = 3.5;
    public static double curArtNum = 1;
    public static Pose2d START_POSE  = new Pose2d(-48,-48, Math.toRadians(140)); //Need to tune these values
    public static Pose2d ATAG_POSE = new Pose2d(-30,-30, Math.toRadians(165));
    public static Pose2d SHOOT_POSE  = new Pose2d(-30, -30, Math.toRadians(-135));
    public static Pose2d END_POSE = new Pose2d(-12, -34, Math.toRadians(-90));
    public static Pose2d INTAKE1_POSE = new Pose2d(-12, intArtOffset, Math.toRadians(-90));
    public static Pose2d INTAKE2_POSE = new Pose2d(12, intArtOffset, Math.toRadians(-90));

    public static double GOAL_X = -68;
    public static double GOAL_Y = -53;
    public static double GOAL_H = Math.toRadians(-135);
    public static double INTAKE_SECONDS = 1.0;
    LocalizationFuser fuser;
    DrumIntakeTurretManager drum;
    State lastState = State.DRIVE_TO_SHOT;
    State state = State.DRIVE_TO_SHOT;
    State next_state = state.SPINUP_AND_AIM;
    String motif = "WWW";
    int motifIndex = 0;
    int cycle = 0;
    int inArtNum = 0;
    Pose2d driveTarget = SHOOT_POSE;
    PoseVelocity2d velo;
    Action strafeOut = null;
    Action goAprilTagPose = null;
    Action goEndPose = null;
    Action spinUp = null;
    Action shoot = null;
    Action runLL = null;
    boolean startedShotSequence = false;
    int initPointer = 0;
    String motifString;
    String[] colorsString = {"green", "purple", "purple"};
    String alliance;
    File dataLog = AppUtil.getInstance().getSettingsFile("/%s/FIRST/lastInfo.json");
    private void go(State next) {
        state = next;
    }

    private boolean shotFinished() {
        return (!drum.isFiring && drum.curMode == DrumIntakeTurretManager.revMode.FIRESTANDBY);
    }

    private void loadMotifAndResetShots() {
        //motif = somehow get motif
        if (!(motif.equals("GPP") || motif.equals("PGP") || motif.equals("PPG"))) {
            motif = "GPP";
        }
        motifIndex = 0;
    }

    private void requestNextShotState() {
        if(motifIndex>=3){
            state = State.DONE;
        }
        char c = motif.charAt(motifIndex);
        if (c == 'G') go(State.FIRE_GREEN);
        else go(State.FIRE_PURPLE);
    }
    @Override
    public void init() {
        fuser = new LocalizationFuser();
        fuser.init(START_POSE, hardwareMap);
        drum = new DrumIntakeTurretManager();
        drum.init(hardwareMap, DrumIntakeTurretManager.revMode.FIREIDLE);

        fuser.setPose(START_POSE);

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
        drum.setBlue(Objects.equals(alliance, "blue"));
        drum.setStartingColors(colorsString);
        loadMotifAndResetShots();


        runLL = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telmetryPacket) {
                motifString = fuser.getCurrentMotif();
                fuser.loop(0);
                drum.update(fuser.getPose(), fuser.getVelo());
                t2.addData("State", state);
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
                t2.addData("State", state);
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
                // fire based on motif
                fuser.loop(0);
                drum.update(fuser.getPose(), fuser.getVelo());
                t2.addData("State", state);
                t2.addData("Drum mode", drum.curMode);
                t2.addLine("motif: " + motifString);
                t2.addData("Spun", drum.shooterSpunUp());
                fuser.updateTelemetry(t2);
                drum.updateTelemetry(t2);
                t2.update();
                if (!startedShotSequence) {
                    drum.curMode = DrumIntakeTurretManager.revMode.CONTFIRE;
                    startedShotSequence = true;
                    return true;
                } else {
                    if (drum.isEmpty()) {
                        return false;
                    } else {
                        return true;
                    }
                }
            }
        };
        goAprilTagPose = fuser.drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(ATAG_POSE.position, ATAG_POSE.heading)
                .afterDisp(10, runLL)
                .build();
        strafeOut = fuser.drive.actionBuilder(ATAG_POSE)
                .turn(Math.toRadians(60))
                .build();
        goEndPose = fuser.drive.actionBuilder(SHOOT_POSE)
                .splineToLinearHeading(END_POSE, Math.toRadians(-90))
                .build();


    }
    @Override
    public void loop () {

        Actions.runBlocking(new SequentialAction(
                goAprilTagPose,
                strafeOut
        ));

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), fuser.getPose());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        drum.updateTelemetry(t2);

        lastState = state;
    }

}