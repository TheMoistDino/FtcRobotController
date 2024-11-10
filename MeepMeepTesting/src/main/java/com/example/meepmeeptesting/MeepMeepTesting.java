package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.entity.AxesEntity;
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

        Pose2d firstPose = new Pose2d(-35, -61.5, Math.toRadians(-90));
        Pose2d secondPose = new Pose2d(-57,-57, Math.toRadians(-135));
        Pose2d thirdPose = new Pose2d(-48,-44, Math.toRadians(-90));
        Pose2d fourthPose = new Pose2d(-52,-52, Math.toRadians(-135));
        Pose2d fifthPose = secondPose;
        Pose2d sixthPose = new Pose2d(-58, -44, Math.toRadians(-90));

        TrajectoryActionBuilder step1 = leftBot.getDrive().actionBuilder(firstPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-57,-57));

        TrajectoryActionBuilder step2 = leftBot.getDrive().actionBuilder(secondPose)
                .lineToXLinearHeading(-48, Math.toRadians(-90))
                .strafeTo(new Vector2d(-48,-44));

        TrajectoryActionBuilder step3 = leftBot.getDrive().actionBuilder(thirdPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(2.5);

        TrajectoryActionBuilder step4 = leftBot.getDrive().actionBuilder(fourthPose)
                .strafeTo(new Vector2d(-57, -57));

        TrajectoryActionBuilder step5 = leftBot.getDrive().actionBuilder(fifthPose)
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-58,-57))
                .strafeTo(new Vector2d(-58,-44));

        TrajectoryActionBuilder step6 = leftBot.getDrive().actionBuilder(sixthPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(2.5)
                .strafeTo(new Vector2d(-57,-57));

        Action trajectoryStep1 = step1.build();
        Action trajectoryStep2 = step2.build();
        Action trajectoryStep3 = step3.build();
        Action trajectoryStep4 = step4.build();
        Action trajectoryStep5 = step5.build();
        Action trajectoryStep6 = step6.build();

        Action trajectoryActionFinal = step6.fresh()
                .strafeTo(new Vector2d(-52,-52))
                .splineToSplineHeading(new Pose2d(-36,-12, 0), 0, new TranslationalVelConstraint(25.0))
                .strafeTo(new Vector2d(-23.5,-12), new TranslationalVelConstraint(17.5))
                .build();

        Action left3 = new SequentialAction(
                trajectoryStep1, new SleepAction(1),
                trajectoryStep2,
                new SleepAction(4),
                trajectoryStep3, trajectoryStep4, new SleepAction(1),
                trajectoryStep5,
                new SleepAction(4),
                trajectoryStep6, new SleepAction(1),
                trajectoryActionFinal
        );

        TrajectoryActionBuilder ascentPark = leftBot.getDrive().actionBuilder(secondPose)
                .strafeToLinearHeading(new Vector2d(-37,-38),0)
                .strafeTo(new Vector2d(-36,-12))
                .strafeTo(new Vector2d(-23.5,-12), new TranslationalVelConstraint(10.0));

        Action left1 = new SequentialAction(
                trajectoryStep1,
                new SleepAction(1),
                ascentPark.build()
        );

        TrajectoryActionBuilder push2 = leftBot.getDrive().actionBuilder(secondPose)
                .strafeToLinearHeading(new Vector2d(-37,-38),Math.toRadians(-90))
                .strafeTo(new Vector2d(-37,-12))
                .strafeTo(new Vector2d(-46, -12))
                .strafeTo(new Vector2d(-46,-60))
                .strafeTo(new Vector2d(-55,-60))
                .strafeTo(new Vector2d(-46, -36))
                .strafeTo(new Vector2d(-46, -12))
                .strafeTo(new Vector2d(-56, -12))
                .strafeTo(new Vector2d(-56, -60))
                .strafeTo(new Vector2d(-56, -48))
                .strafeToLinearHeading(new Vector2d(-56, -12),0)
                .strafeTo(new Vector2d(-23.5,-12),new TranslationalVelConstraint(15.0));

        Action leftPush2 = new SequentialAction(
                trajectoryStep1,
                new SleepAction(1),
                push2.build()
        );

        leftBot.runAction(
                leftPush2
        );




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
                .setBackgroundAlpha(1.0f)
                .addEntity(leftBot)
                .addEntity(rightBot)
                .removeEntity(MeepMeep.getDEFAULT_AXES_ENTITY())
                .removeEntity(MeepMeep.getDEFAULT_COMPASS_ENTITY())
                .start();
    }
}