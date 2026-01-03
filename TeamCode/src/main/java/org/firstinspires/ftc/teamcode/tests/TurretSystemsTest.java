package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

public class TurretSystemsTest extends OpMode {
    Turret turret;
    @Override
    public void init() {
        turret = new Turret();
        turret.init(hardwareMap, new MecanumDrive(hardwareMap, new Pose2d(0,-7,-Math.PI/2)));
    }

    @Override
    public void loop() {
        turret.loop();
    }
}
