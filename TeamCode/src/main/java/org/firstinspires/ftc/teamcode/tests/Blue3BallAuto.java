package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous
@Config
public class Blue3BallAuto extends OpMode {
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
    public static Pose2d START_POSE  = new Pose2d(-40, -40, Math.toRadians(120)); //Need to tune these values
    public static Pose2d SHOOT_POSE  = new Pose2d(-30, -30, Math.toRadians(-135));
    public static Pose2d INTAKE1_POSE = new Pose2d(-12, intArtOffset, Math.toRadians(-90));
    public static Pose2d INTAKE2_POSE = new Pose2d(12, intArtOffset, Math.toRadians(-90));

    public static double GOAL_X = -68;
    public static double GOAL_Y = -53;
    public static double GOAL_H = Math.toRadians(-135);

    public static double INTAKE_SECONDS = 1.0;

    MecanumDrive drive;
    DrumIntakeTurretManager drum;
    State state = State.DRIVE_TO_SHOT;
    State next_state = state.SPINUP_AND_AIM;

    String motif = "WWW";
    int motifIndex = 0;
    int cycle = 0;
    int inArtNum = 0;

    Pose2d driveTarget = SHOOT_POSE;
    Action currentAction = null;

    private void go(State next) {
        state = next;
    }

    private boolean shotFinished() {
        return (!drum.isFiring && drum.curMode == DrumIntakeTurretManager.revMode.FIRESTANDBY);
    }

    private void loadMotifAndResetShots() {
        //motif = drum.readMotif(); // Somehow read motif
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
        drum.init(hardwareMap);
        drum.init(hardwareMap);

        Turret.goalPoseX = GOAL_X;
        Turret.goalPoseY = GOAL_Y;
        Turret.goalPoseH = GOAL_H;
    }
    @Override
    public void init_loop () {
        telemetry.update();
        loadMotifAndResetShots();
        Pose2d cur = drive.localizer.getPose();
        currentAction = drive.actionBuilder(cur)
                .strafeTo(new Vector2d(-24, -24))
                .turn(Math.toRadians(80))
                .waitSeconds(3)
                .build();
    }
    @Override
    public void loop () {
        if (state != State.DONE) {
            drive.updatePoseEstimate();
            drum.update(drive.localizer.getPose(), drive.updatePoseEstimate());
             if (state == State.SPINUP_AND_AIM) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
                boolean ready = drum.shooterSpunUp();
                if (ready) {
                    requestNextShotState();
                }
            }
            else if (state == State.FIRE_PURPLE) {
                drum.firePurple();
                if (shotFinished()) {
                    motifIndex++;
                    go(State.SPINUP_AND_AIM);
                }
            }
            else if (state == State.FIRE_GREEN) {
                drum.fireGreen();
                if (shotFinished()) {
                    motifIndex++;
                    go(State.SPINUP_AND_AIM);
                }
            }
            else if (state == State.DONE) {
                drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
            }
        }
    }
}