package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
@Autonomous
@Config
public class justinAuto extends LinearOpMode {

    enum State {
        SPINUP_AND_AIM,
        INTAKE,
        FIRE_GREEN,
        FIRE_PURPLE,
        DRIVE_TO_END,
        DONE
    }
    public static Pose2d START_POSE  = new Pose2d(-48, -49.5, Math.toRadians(-36));
    public static Pose2d SHOOT_POSE  = new Pose2d(-23.8, -22.7, Math.toRadians(-135));

    public static Pose2d INTAKE1_POSE = new Pose2d(35, -30, Math.toRadians(-90));
    public static Pose2d INTAKE2_POSE = new Pose2d(12, -30, Math.toRadians(-90));
    public static Pose2d INTAKE3_POSE = new Pose2d(-11, -30, Math.toRadians(-90));
    public static Pose2d INTAKE1_FINAL = new Pose2d(35, -50, Math.toRadians(-90));
    public static Pose2d INTAKE2_FINAL = new Pose2d(12, -50, Math.toRadians(-90));
    public static Pose2d INTAKE3_FINAL = new Pose2d(-11, -50, Math.toRadians(-90));


    public static double GOAL_X = -68;
    public static double GOAL_Y = -53;
    public static double GOAL_H = -135;


    MecanumDrive drive;
    DrumIntakeTurretManager drum;
    State state = State.SPINUP_AND_AIM;
    State next_state = State.SPINUP_AND_AIM;

    String motif = "WWW";
    int motifIndex = 0;
    int cycle = 0;

    private boolean already_shot = false;

    Pose2d driveTarget = SHOOT_POSE;
    Pose2d finalTarget = SHOOT_POSE;
    Action currentAction = null;

    private void go(State next) {
        state = next;
    }

    private void startDriveToShoot(Pose2d target, State after) {
        driveTarget = target;
        next_state = after;

        Pose2d cur = drive.localizer.getPose();

        currentAction = drive.actionBuilder(cur)
                .splineToLinearHeading(driveTarget, Math.toRadians(135))
                .build();

        Actions.runBlocking(currentAction);
        currentAction = null;
        go(next_state);
    }

    private void startDriveToIntake(Pose2d target, Pose2d finalPose, State after) {
        driveTarget = target;
        finalTarget = finalPose;
        next_state = after;
        Pose2d cur = drive.localizer.getPose();
        drum.curMode = DrumIntakeTurretManager.revMode.INTAKING;

        currentAction = drive.actionBuilder(cur)
            .splineToSplineHeading(driveTarget, Math.toRadians(-90))
            .splineToLinearHeading(finalTarget, Math.toRadians(-90))
            .build();

        Actions.runBlocking(currentAction);
        currentAction = null;
        drum.curMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
        go(next_state);
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
        if (motifIndex >= 3) {
            handleEndOfMotif();
            return;
        }

        char c = motif.charAt(motifIndex);
        if (c == 'G') go(State.FIRE_GREEN);
        else go(State.FIRE_PURPLE);
    }

    private void handleEndOfMotif() {
        motifIndex = 0;
        if (cycle == 0) {
            cycle = 1;
            startDriveToIntake(INTAKE1_POSE, INTAKE1_FINAL, State.INTAKE);
        }
        else if (cycle == 1) {
            cycle = 2;
            startDriveToIntake(INTAKE2_POSE, INTAKE2_FINAL, State.INTAKE);
        }
        else if (cycle == 2) {
            cycle = 3;
            startDriveToIntake(INTAKE3_POSE, INTAKE3_FINAL, State.INTAKE);
        }
        else {
            go(State.DONE);
        }
    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, START_POSE);
        drum = new DrumIntakeTurretManager();
        drum.init(hardwareMap, drive);

        Turret.goalPoseX = GOAL_X;
        Turret.goalPoseY = GOAL_Y;
        Turret.goalPoseH = GOAL_H;

        telemetry.update();
        loadMotifAndResetShots();

        waitForStart();
        if (isStopRequested()) return;

        startDriveToShoot(SHOOT_POSE, State.SPINUP_AND_AIM);

        while(opModeIsActive() && state != State.DONE) {
            drive.updatePoseEstimate();
            drum.update();

             if (state == State.SPINUP_AND_AIM) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
                boolean ready = drum.shooterSpunUp();
                if (ready) {
                    requestNextShotState();
                }
            }
            else if (state == State.FIRE_PURPLE) {
                if(!already_shot) {
                    drum.firePurple();
                    already_shot = true;
                }
                if (shotFinished()) {
                    already_shot = false;
                    motifIndex++;
                    go(State.SPINUP_AND_AIM);
                }
            }
            else if (state == State.FIRE_GREEN) {
                 if(!already_shot) {
                     drum.fireGreen();
                     already_shot = true;
                 }
                if (shotFinished()) {
                    already_shot = false;
                    motifIndex++;
                    go(State.SPINUP_AND_AIM);
                }
            }
            else if (state == State.INTAKE) {
                startDriveToShoot(SHOOT_POSE, State.SPINUP_AND_AIM);
            }
            else if (state == State.DONE) {
                return;
            }
        }
    }
}