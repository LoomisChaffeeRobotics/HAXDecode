package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 0, Math.PI))
                .setTangent(Math.PI * 1.1)
                .splineToLinearHeading(new Pose2d(35, 25, Math.PI / 2), Math.PI * 0.5)
                .lineToYSplineHeading(50, Math.PI * 0.5)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(50, 0, Math.PI * 0.8), -Math.PI * 0.3)

                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(12, 32, Math.PI * 0.5), Math.PI * 0.5)
                .lineToYSplineHeading(50, Math.PI * 0.5)
                .setTangent(-Math.PI * 0.5)
                .splineToLinearHeading(new Pose2d(0, 0, Math.PI * 0.76), -Math.PI * 0.4)

                .setTangent(Math.PI * 0.75)
                .splineToLinearHeading(new Pose2d(-12, 32, Math.PI * 0.5), Math.PI * 0.5)
                //.setTangent(Math.PI * 0.5)
                .lineToYSplineHeading(50, Math.PI * 0.5)
                .setTangent(-Math.PI * 0.5)
                .splineToLinearHeading(new Pose2d(0, 0, Math.PI * 0.76), -Math.PI * 0.4)
                .setTangent(-Math.PI * 0.2)
                .splineToLinearHeading(new Pose2d(36, 32, Math.PI), Math.PI * 0.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}