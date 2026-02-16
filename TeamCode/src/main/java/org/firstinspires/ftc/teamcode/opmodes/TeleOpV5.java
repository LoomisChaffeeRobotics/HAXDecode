package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FancyPID;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationFuser;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
@Disabled
public class TeleOpV5 extends OpMode {
    DrumIntakeTurretManager drum;
    FtcDashboard dash;
    LocalizationFuser fuser;
    Telemetry t2;
    double botHeading;
    double lastTriggerVal;
    String[] colorsString = {"green", "purple", "purple"};
    int initPointer = 0;
    boolean blue;
    double offset = 0;
    Pose2d fusedPose = new Pose2d(0,0,0);
    int fcMultiplier = 1;
    FancyPID rotPID = new FancyPID();
    DrumIntakeTurretManager.revMode tempMode = DrumIntakeTurretManager.revMode.INTAKEIDLE;
    @Override
    public void init() {
        drum = new DrumIntakeTurretManager();
        drum.init(hardwareMap, DrumIntakeTurretManager.revMode.FIREIDLE);
        fuser = new LocalizationFuser();
        fuser.init(new Pose2d(66,0,-Math.PI), hardwareMap);
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
        fuser.start(blue);
        fuser.resetDrive();
        if (blue) {
            fcMultiplier*=-1;
        }
    }
    @Override
    public void loop() {
        double gamepady = -gamepad1.left_stick_y;
        double gamepadx = -gamepad1.left_stick_x;

        if(!blue){
            gamepadx *= -1;
            gamepady *= -1;
        }

        double controlAngle =
        botHeading = fcMultiplier * fuser.getYaw();

        fusedPose = fuser.getPose();
        fuser.loop(0); // eventually use a turret yaw getter before looping fuser once turret spinning

        double fieldX = gamepadx * Math.cos(botHeading) + gamepady * Math.sin(botHeading);
        double fieldY = -gamepadx * Math.sin(botHeading) + gamepady * Math.cos(botHeading);
        if (gamepad1.left_trigger > 0.5) {
            fuser.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0.3 * fieldX, 0.3 * -fieldY),
                    -0.3 * gamepad1.right_stick_x
            ));
        } else {
            fuser.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(fieldX,-fieldY),
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
        } else if (gamepad2.dpadDownWasPressed()) {
            drum.lastSlot();
        }


        if (gamepad1.startWasPressed()) {
            fuser.resetDrive();
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

        drum.update(fuser.getPose(), fuser.getVelo(), fuser.getTagX()); // degrees
        drum.updateTelemetry(t2);
        fuser.updateTelemetry(t2);

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), fusedPose);
        packet.fieldOverlay().setStroke("#FF51B5");
//        Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(fusedPose.position, botHeading));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.addData("colors", Arrays.toString(drum.getColors()));
        t2.update();
        telemetry.addData("mode", drum.curMode.toString());

        if (fuser.seeingTag()) {
            telemetry.addLine("!------------TAG SEEN----------!");
        }

        telemetry.update();
    }
}
