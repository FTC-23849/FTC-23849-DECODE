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

        myBot.runAction(myBot.getDrive().actionBuilder(/*new Pose2d(63, -14.5, Math.toRadians(270))*/ /*new Pose2d(-54.5, -45, Math.toRadians(225))*/ new Pose2d(0, 0, Math.toRadians(270)))

//                .setTangent(-Math.PI/2)
//                .splineToConstantHeading(new Vector2d(-48, -48), (-Math.PI))

                .strafeToSplineHeading(new Vector2d(-12, -15), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-12, -53))
                .strafeToConstantHeading(new Vector2d(-12, -15))


                .setTangent(0)
                .splineToConstantHeading(new Vector2d(13, -63), (-Math.PI/2))


                .strafeToLinearHeading(new Vector2d(13, -50), Math.toRadians(270))

                //.setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(2, -50, Math.toRadians(270)), -Math.PI/2)

                .strafeToLinearHeading(new Vector2d(2, -52), Math.toRadians(270))


                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(-12, -15), (Math.PI))


                .setTangent(0)
                .splineToLinearHeading(new Pose2d(8, -56, Math.toRadians(225)), (-Math.PI/2))
                .strafeToConstantHeading(new Vector2d(15, -60))
                .setTangent(Math.PI/2)
                .splineTo(new Vector2d(-12, -15), (Math.PI))

                .setTangent(0)
                .splineToSplineHeading(new Pose2d(35.5, -35, Math.toRadians(270)), (-Math.PI/2))
                .strafeToConstantHeading(new Vector2d(35.5, -63))
                .strafeToSplineHeading(new Vector2d(-12, -15), Math.toRadians(315))

                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(60, -63, Math.toRadians(0)), (0))
                .strafeToSplineHeading(new Vector2d(-12, -15), Math.toRadians(315))

//                .strafeToLinearHeading(new Vector2d(13, -25), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(13, -63), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(13, -52), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-12, -15), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(35, -25), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(35, -63), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-12, -15), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(-22, -58), Math.toRadians(270))





//                    .strafeToLinearHeading(new Vector2d(36, -28), Math.toRadians(270))
//                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(270))
//                    .strafeToLinearHeading(new Vector2d(62, -15), Math.toRadians(270))
//
////                                .strafeToLinearHeading()
////                                .strafeToLinearHeading()
//                    .strafeToLinearHeading(new Vector2d(62, -59), Math.toRadians(270))
//
//                    .strafeToLinearHeading(new Vector2d(62, -50), Math.toRadians(270))
//                    .strafeToLinearHeading(new Vector2d(62, -59), Math.toRadians(270))
//
//                    .strafeToLinearHeading(new Vector2d(62, -15), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(62, -59), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(62, -50), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(62, -59), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(62, -15), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(62, -59), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(62, -50), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(62, -59), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(62, -15), Math.toRadians(270))
//
//                    .strafeToLinearHeading(new Vector2d(57, -30), Math.toRadians(270))




//                .strafeToLinearHeading(new Vector2d(-12, -15), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-12, -25), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-12, -53), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-12, -15), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(13, -25), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(13, -63), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(13, -52), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-12, -15), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(35, -25), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(35, -63), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-12, -15), Math.toRadians(270))
//
//                .strafeToLinearHeading(new Vector2d(-22, -58), Math.toRadians(270))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}