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
    public static Pose2d SHOOT_POSE  = new Pose2d(-30, -30, Math.toRadians(-135));
    public static Pose2d END_POSE = new Pose2d(-12, -34, Math.toRadians(-90));
    public static Pose2d INTAKE1_POSE = new Pose2d(-12, intArtOffset, Math.toRadians(-90));
    public static Pose2d INTAKE2_POSE = new Pose2d(12, intArtOffset, Math.toRadians(-90));

    public static double GOAL_X = -68;
    public static double GOAL_Y = -53;
    public static double GOAL_H = Math.toRadians(-135);

    public static double INTAKE_SECONDS = 1.0;

    MecanumDrive drive;
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
    Action goEndPose = null;
    Action spinUp = null;
    Action shoot = null;
    boolean startedShotSequence = false;
    int initPointer = 0;
    String[] colorsString = {"white", "white", "white"};
    JSONObject jsonObject;
    JSONArray pose;
    JSONArray colors;
    String alliance;
    String mode;
    Pose2d startPose;
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
        drive = new MecanumDrive(hardwareMap, START_POSE);
        drum = new DrumIntakeTurretManager();
        drum.init(hardwareMap, DrumIntakeTurretManager.revMode.FIREIDLE);

        drive.localizer.setPose(START_POSE);

        try (InputStream is = new FileInputStream(dataLog)) {
            InputStreamReader reader = new InputStreamReader(is, "UTF-8");
            jsonObject = new JSONObject(reader.toString());
        } catch (Exception e) {
            telemetry.addData("error", e.toString());
            telemetry.update();
        }

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
        drive.localizer.setPose(START_POSE);


        strafeOut = drive.actionBuilder(START_POSE)
                .splineToLinearHeading(new Pose2d(-24, -24,Math.toRadians(220)), Math.toRadians(135))
                .build();
        goEndPose = drive.actionBuilder(SHOOT_POSE)
                .splineToLinearHeading(END_POSE, Math.toRadians(-90))
                .build();


        spinUp = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
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

    }
    @Override
    public void loop () {
        if (state != State.DONE) {
            drive.updatePoseEstimate();
            drum.update(drive.localizer.getPose(), drive.updatePoseEstimate());
        }

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(strafeOut,
                spinUp),
                shoot,
                goEndPose
        ));


//        if (state == State.DRIVE_TO_SHOT) {
//            Actions.runBlocking(strafeOut);
//            if ((Math.abs(drive.localizer.getPose().position.x + 24) < 2) && (Math.abs(drive.localizer.getPose().position.y + 24) < 2)) {
//                state = State.SPINUP_AND_AIM;
//            }
//
//        } else if (state == State.SPINUP_AND_AIM) {
//
//            drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
//            boolean ready = drum.shooterSpunUp();
//            if (ready) {
//                requestNextShotState();
//            }
//        }
//        else if (state == State.FIRE_PURPLE) {
//            if (state != lastState) {
//                drum.firePurple();
//            }
//
//            if (shotFinished()) {
//                motifIndex++;
//                go(State.SPINUP_AND_AIM);
//            }
//        }
//        else if (state == State.FIRE_GREEN) {
//            if (state != lastState) {
//                drum.fireGreen();
//            }
//
//            if (shotFinished()) {
//                motifIndex++;
//                go(State.SPINUP_AND_AIM);
//            }
//        }
//        else if (state == State.DONE) {
//            drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
//            Actions.runBlocking(goEndPose);
//        }
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.localizer.getPose());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        t2.addData("State", state);
        t2.addData("Drum mode", drum.curMode);
        t2.addData("Spun", drum.shooterSpunUp());
        drum.updateTelemetry(t2);
        t2.update();
        lastState = state;
    }

}