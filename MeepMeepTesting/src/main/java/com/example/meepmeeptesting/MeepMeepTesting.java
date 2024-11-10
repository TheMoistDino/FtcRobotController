package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.65613859451274, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .build();

        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.65613859451274, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .build();

        leftBot.runAction(leftBot.getDrive().actionBuilder(new Pose2d(-35, -61.5, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-57,-57))
                .waitSeconds(1)

                .lineToXLinearHeading(-48, Math.toRadians(-90))
                .strafeTo(new Vector2d(-48,-44), new TranslationalVelConstraint(10))
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(1.5)

                .strafeTo(new Vector2d(-57,-57))
                .waitSeconds(1)

                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-58,-57))
                .strafeTo(new Vector2d(-58,-44), new TranslationalVelConstraint(10))
                .waitSeconds(3)

                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-57,-57))
                .waitSeconds(1)

                .strafeTo(new Vector2d(-52,-52))
                .splineToSplineHeading(new Pose2d(-36,-12, 0), 0, new TranslationalVelConstraint(17.5))
                .strafeTo(new Vector2d(-23.5,-12), new TranslationalVelConstraint(10.0))
                .build());

        rightBot.runAction(rightBot.getDrive().actionBuilder(new Pose2d(12, -61.5, Math.toRadians(-90)))
                .strafeTo(new Vector2d(36,-36))
                .waitSeconds(0.2)
                .lineToY(-12)
                .strafeTo(new Vector2d(46.5,-12))
                .strafeTo(new Vector2d(46.5,-59))
                .lineToY(-12)
                .strafeTo(new Vector2d(56.5,-12))
                .strafeTo(new Vector2d(56.5,-59))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(leftBot)
                .addEntity(rightBot)
                .start();
    }
}