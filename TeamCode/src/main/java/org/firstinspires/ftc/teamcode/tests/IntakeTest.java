package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;

@TeleOp
@Config
public class IntakeTest extends OpMode {
    Intake intake;
    DrumIntakeTurretManager drum;
    public static double testPower;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry t2 = dash.getTelemetry();
    MecanumDrive drive;
    Rotation2d rotation = new Rotation2d(0,0);
    @Override
    public void init() {
        intake = new Intake(hardwareMap, "intake");
        drum = new DrumIntakeTurretManager();
        drum.init(hardwareMap);
        intake.init();
        drum.testMode = true;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

    }

    @Override
    public void loop() {
        rotation = drive.localizer.getPose().heading;
        if (gamepad1.dpad_up){
            intake.intakeOn();
        }
        else if (gamepad1.dpad_down) {
            intake.intakeOut();
        }
        else {
            intake.intakeOff();
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ), gamepad1.right_stick_x));

        intake.loop();
        if (gamepad1.aWasPressed()) {
            drum.nextSlot();
        }
        if (gamepad1.bWasPressed()) {
            drum.lastSlot();
        }
        if (gamepad1.xWasPressed()) {
            drum.toggleManualShoot();
        }


        drum.update();
        drum.updateTelemetry(t2);
        intake.intakeTele(telemetry);
        intake.intakeTele(t2);
        t2.addData("X Vel", drive.localizer.update().linearVel.x);
        t2.addData("Y Vel", drive.localizer.update().linearVel.y);
        t2.addData("Ang Vel", drive.localizer.update().angVel);
        t2.update();
        telemetry.update();
    }
}
