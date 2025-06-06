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
                .strafeTo(new Vector2d(3,36))
                .lineToY(38)
                //.strafeTo(new Vector2d(-40,60))
                .splineToLinearHeading(new Pose2d(-32,30, Math.toRadians(250)), Math.toRadians(-90))

                .strafeToLinearHeading(new Vector2d(-40,50), Math.toRadians(150))

                .strafeToLinearHeading(new Vector2d(-46, 30), Math.toRadians(250))

                .strafeToLinearHeading(new Vector2d(-46,50), Math.toRadians(150))
                .strafeToLinearHeading(new Vector2d(-54, 30), Math.toRadians(250))

                .strafeToLinearHeading(new Vector2d(-54,50), Math.toRadians(150))
                .strafeToLinearHeading(new Vector2d(-48,54), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(7,42), Math.toRadians(-90))
                        .strafeTo(new Vector2d(7,36))
                        .strafeTo(new Vector2d(7,38))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-40,45), Math.toRadians(90))
                        .strafeTo(new Vector2d(-40,54))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}