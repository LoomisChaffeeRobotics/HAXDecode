package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.limeLight;

@Autonomous
@Config
public class justinAuto extends LinearOpMode {
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
    public static double GOAL_H = 0;

    public static double INTAKE_SECONDS = 1.0;

    MecanumDrive drive;
    DrumIntakeTurretManager drum;
    limeLight LL;

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

    private void startDriveTo(Pose2d target, State after) {
        driveTarget = target;
        next_state = after;

        Pose2d cur = drive.localizer.getPose();

        currentAction = drive.actionBuilder(cur)
                .splineToLinearHeading(target, target.heading.toDouble())
                .build();

        go(next_state);
    }
    private boolean driveFinished() {
        //return true once roadrunner says drive is finished
        return true;
    }
    private boolean shotFinished() {
        return (!drum.isFiring && drum.curMode == DrumIntakeTurretManager.revMode.FIRESTANDBY);
    }

    private void loadMotifAndResetShots() {
        motif = LL.getCurrentMotif(); // Somehow read motif
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
    private void waitForDrum(){
        while (!drum.isArrived()) {
            drum.update();
            telemetry.update();
            drive.updatePoseEstimate();
        }
    }

    private void handleEndOfMotif() {
        motifIndex = 0;
        if (cycle == 0) {
            cycle = 1;
            startDriveTo(INTAKE1_POSE, State.INTAKE);
        } else if (cycle == 1) {
            cycle = 2;
            startDriveTo(INTAKE2_POSE, State.INTAKE);
        } else {
            go(State.DONE);
        }
    }
    private void intake3Art(int cycle) {

        if (cycle == 1){
            for (curArtNum = 0; curArtNum < 3; curArtNum++) {
                intArtOffset = intArt1 + curArtNum * artGap;
                startDriveTo(INTAKE1_POSE, State.INTAKE);
                waitForDrum();
            }

        }
        else if (cycle == 2){
            for (curArtNum = 0; curArtNum < 3; curArtNum++) {
                intArtOffset = intArt1 + curArtNum * artGap;
                startDriveTo(INTAKE2_POSE, State.INTAKE);
                waitForDrum();
            }
        }
    }
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, START_POSE);
        drum = new DrumIntakeTurretManager();
        LL = new limeLight();
        drum.init(hardwareMap, drive);

        Turret.goalPoseX = GOAL_X;
        Turret.goalPoseY = GOAL_Y;
        Turret.goalPoseH = GOAL_H;

        telemetry.update();
        loadMotifAndResetShots();
        startDriveTo(SHOOT_POSE, State.SPINUP_AND_AIM);

        while(opModeIsActive() && state != State.DONE) {
            drive.updatePoseEstimate();
            drum.update();

            if (state == State.DRIVE_TO_SHOT) {
                if (driveFinished()) {
                    go(next_state);
                }
            }
            else if (state == State.SPINUP_AND_AIM) {
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
            else if (state == State.INTAKE) {

                drum.curMode = DrumIntakeTurretManager.revMode.INTAKING;

                //Write Intake Auto
                if (driveFinished()) {
                    intake3Art(cycle);
                }

                startDriveTo(SHOOT_POSE, State.SPINUP_AND_AIM);
            }
            else if (state == State.DONE) {
                return;
            }
        }
    }
}