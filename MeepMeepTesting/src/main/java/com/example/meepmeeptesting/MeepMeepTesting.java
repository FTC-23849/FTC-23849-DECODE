package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(580);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(/*new Pose2d(63.5, -13.6, Math.toRadians(90))*/new Pose2d(-54.5, -45, Math.toRadians(225)))
//                .strafeToLinearHeading(new Vector2d(52, -15), Math.toRadians(15))
//                .strafeToLinearHeading(new Vector2d(45, -64), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(62, -64), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(52, -15), Math.toRadians(15))
//                .strafeToLinearHeading(new Vector2d(57, -30), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(-23, -15), Math.toRadians(230))
                .strafeToLinearHeading(new Vector2d(-12, -25), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-12, -53), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-23, -15), Math.toRadians(230))

                .strafeToLinearHeading(new Vector2d(13, -25), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(13, -63), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(13, -52), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-23, -15), Math.toRadians(230))

                .strafeToLinearHeading(new Vector2d(35, -25), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(35, -63), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-23, -15), Math.toRadians(230))

                .strafeToLinearHeading(new Vector2d(-22, -58), Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}