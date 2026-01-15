package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, 49.5, Math.toRadians(-144)))

                        .setTangent(Math.toRadians(-70))
                .splineToSplineHeading(new Pose2d(-34, 17, Math.toRadians(90)), Math.toRadians(-45))
                //.setTangent(Math.toRadians(0))
                .strafeTo(new Vector2d(-11, 25))
                .strafeTo(new Vector2d(-11,50))
                .setTangent(Math.toRadians(-90))
                //get ready for fire
                .splineToLinearHeading(new Pose2d(-23.8, 22.7, Math.toRadians(135)), Math.toRadians(-135))

                /*
           .splineToSplineHeading(new Pose2d(35, 30, Math.toRadians(90)), Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(35, 50), Math.toRadians(90))//slow
               .setTangent(Math.toRadians(-90))
             //get ready for fire
              .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(135)), Math.toRadians(225))
               //fire fire fire
              .setTangent(Math.toRadians(45))
             .splineToSplineHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(90))//slow
              .setTangent(Math.toRadians(-90))
             //get ready for fire
              .splineToLinearHeading(new Pose2d(-11, 11, Math.toRadians(135)), Math.toRadians(225))
             //fire fire fire
               .setTangent(Math.toRadians(90))
             .splineToSplineHeading(new Pose2d(-11, 30, Math.toRadians(90)), Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(-11, 50), Math.toRadians(90))//slow
                .setTangent(Math.toRadians(-90))
              .splineToLinearHeading(new Pose2d(-11, 11, Math.toRadians(135)), Math.toRadians(-90))
                //to box
                .setTangent(Math.toRadians(30))
                .splineToLinearHeading(new Pose2d(36, 32, Math.toRadians(180)), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(36, 36),Math.PI)

                 */
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}