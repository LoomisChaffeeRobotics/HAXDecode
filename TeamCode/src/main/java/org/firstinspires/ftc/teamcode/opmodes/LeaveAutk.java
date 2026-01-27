package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Config
public class LeaveAutk extends OpMode {
    MecanumDrive drive;
    ElapsedTime moveTime = new ElapsedTime();
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }
    @Override
    public void start() {
        moveTime.reset();
    }

    @Override
    public void loop() {
       if (moveTime.seconds() < 0.8) {
           drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.5, 0), 0));
       } else {
           drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
       }
    }
}