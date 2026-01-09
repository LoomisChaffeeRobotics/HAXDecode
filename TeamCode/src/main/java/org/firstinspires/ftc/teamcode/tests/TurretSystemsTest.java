package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp
@Config
public class TurretSystemsTest extends OpMode {
    DrumIntakeTurretManager drum;
    FtcDashboard dash;
    Telemetry t2;
    MecanumDrive drive;

    @Override
    public void init() {
        drum = new DrumIntakeTurretManager();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        drum.init(hardwareMap, drive);
        drum.curMode = DrumIntakeTurretManager.revMode.HPINTAKE;
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            if (drum.curMode == DrumIntakeTurretManager.revMode.HPINTAKE) {
                drum.curMode = DrumIntakeTurretManager.revMode.FIRESTANDBY;
            } else {
                drum.curMode = DrumIntakeTurretManager.revMode.HPINTAKE;
            }
        }
        if (drum.curMode == DrumIntakeTurretManager.revMode.FIRESTANDBY) {
            if (gamepad1.right_trigger>.5) {
                drum.fireSingle();
            }
        }

        if (drum.curMode == DrumIntakeTurretManager.revMode.HPINTAKE) {
            if (gamepad1.dpadLeftWasPressed()) {
                drum.setCurrentPurple();
            } else if (gamepad1.dpadRightWasPressed()) {
                drum.setCurrentGreen();
            }
            if (gamepad1.rightBumperWasPressed()) {
                drum.nextSlot();
            } else if (gamepad1.leftBumperWasPressed()) {
                drum.lastSlot();
            }
        }
        drum.update();
        drum.updateTelemetry(t2);
    }
}
