package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueSpecimenMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16, 63, Math.toRadians(-90)))
                .strafeTo(new Vector2d(0,36))
                        .lineToY(40)
                        .strafeTo(new Vector2d(-35,40))
                        .strafeTo(new Vector2d(-35,30))
                        .splineToConstantHeading(new Vector2d(-50,10), Math.toRadians(90))
                        .strafeTo(new Vector2d(-50,60))
                .splineToLinearHeading(new Pose2d(-55,10, Math.toRadians(90)), Math.toRadians(180))

                        .strafeTo(new Vector2d(-55,60))
                .splineToConstantHeading(new Vector2d(-60,10), Math.toRadians(180))
                .strafeTo(new Vector2d(-60,60))
                        .strafeTo(new Vector2d(-60,50))
                        .splineToConstantHeading(new Vector2d(3,40), Math.toRadians(-90))
                        .strafeTo(new Vector2d(3,34))
                        .strafeTo(new Vector2d(3,40))
                .splineToConstantHeading(new Vector2d(-30,55), Math.toRadians(90))
                        .strafeTo(new Vector2d(-30,50))
                        .splineToConstantHeading(new Vector2d(6,40), Math.toRadians(-90))
                        .strafeTo(new Vector2d(6,34))
                        .strafeTo(new Vector2d(6,38))
                        .splineToConstantHeading(new Vector2d(-30,55), Math.toRadians(90))
                .strafeTo(new Vector2d(-30,50))
                .splineToConstantHeading(new Vector2d(9,40), Math.toRadians(-90))
                        .strafeTo(new Vector2d(9,34))
                .strafeTo(new Vector2d(9,38))
                .splineToConstantHeading(new Vector2d(-30,55), Math.toRadians(90))
                .strafeTo(new Vector2d(-30,50))
                .splineToConstantHeading(new Vector2d(11,40), Math.toRadians(-90))
                .strafeTo(new Vector2d(11,34))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}