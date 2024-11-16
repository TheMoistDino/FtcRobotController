package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(580,30);

        RoadRunnerBotEntity leftRedBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.65613859451274, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .build();

        RoadRunnerBotEntity rightRedBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.65613859451274, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .build();

        RoadRunnerBotEntity leftBlueBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.65613859451274, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .build();

        RoadRunnerBotEntity rightBlueBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.65613859451274, 52.48180821614297, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 16.34)
                .build();

        Pose2d firstPose = new Pose2d(-35, -61.5, Math.toRadians(-90));
        Pose2d secondPose = new Pose2d(-57,-57, Math.toRadians(-135));
        Pose2d thirdPose = new Pose2d(-48,-44, Math.toRadians(-90));
        Pose2d fourthPose = new Pose2d(-52,-52, Math.toRadians(-135));
        Pose2d fifthPose = secondPose;
        Pose2d sixthPose = new Pose2d(-58, -44, Math.toRadians(-90));

        TrajectoryActionBuilder step1 = leftRedBot.getDrive().actionBuilder(firstPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-57,-57));

        TrajectoryActionBuilder step2 = leftRedBot.getDrive().actionBuilder(secondPose)
                .lineToXLinearHeading(-48, Math.toRadians(-90))
                .strafeTo(new Vector2d(-48,-44));

        TrajectoryActionBuilder step3 = leftRedBot.getDrive().actionBuilder(thirdPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(2.5);

        TrajectoryActionBuilder step4 = leftRedBot.getDrive().actionBuilder(fourthPose)
                .strafeTo(new Vector2d(-57, -57));

        TrajectoryActionBuilder step5 = leftRedBot.getDrive().actionBuilder(fifthPose)
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-58,-57))
                .strafeTo(new Vector2d(-58,-44));

        TrajectoryActionBuilder step6 = leftRedBot.getDrive().actionBuilder(sixthPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(2.5)
                .strafeTo(new Vector2d(-57,-57))
                .waitSeconds(1);

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
                trajectoryStep6,
                trajectoryActionFinal
        );

        TrajectoryActionBuilder ascentPark = leftRedBot.getDrive().actionBuilder(secondPose)
                .strafeToLinearHeading(new Vector2d(-37,-38),0)
                .strafeTo(new Vector2d(-36,-12))
                .strafeTo(new Vector2d(-23.5,-12), new TranslationalVelConstraint(10.0));

        TrajectoryActionBuilder push2 = leftRedBot.getDrive().actionBuilder(secondPose)
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

        Pose2d firstBluePose = new Pose2d(35, 61.5, Math.toRadians(90));
        Pose2d secondBluePose = new Pose2d(57,57, Math.toRadians(45));

        TrajectoryActionBuilder blueStep1 = leftRedBot.getDrive().actionBuilder(firstBluePose)
                .strafeToLinearHeading(new Vector2d(52,52), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(57,57));

        TrajectoryActionBuilder bluePush2 = leftBlueBot.getDrive().actionBuilder(secondBluePose)
                .strafeToLinearHeading(new Vector2d(37,38),Math.toRadians(90))
                .strafeTo(new Vector2d(37,12))
                .strafeTo(new Vector2d(46, 12))
                .strafeTo(new Vector2d(46,60))
                .strafeTo(new Vector2d(55,60))
                .strafeTo(new Vector2d(46, 36))
                .strafeTo(new Vector2d(46, 12))
                .strafeTo(new Vector2d(56, 12))
                .strafeTo(new Vector2d(56, 60))
                .strafeTo(new Vector2d(56, 48))
                .strafeToLinearHeading(new Vector2d(56, 12),Math.toRadians(180))
                .strafeTo(new Vector2d(23.5,12),new TranslationalVelConstraint(15.0));

        Action leftBluePush2 = new SequentialAction(
                blueStep1.build(),
                new SleepAction(1),
                bluePush2.build());

        leftRedBot.runAction(
                left3
        );

        rightRedBot.runAction(rightRedBot.getDrive().actionBuilder(new Pose2d(12, -61.5, Math.toRadians(-90)))
                .strafeTo(new Vector2d(36,-36))
                .waitSeconds(0.2)
                .lineToY(-12)
                .strafeTo(new Vector2d(46.5,-12))
                .strafeTo(new Vector2d(46.5,-59))
                .lineToY(-12)
                .strafeTo(new Vector2d(56.5,-12))
                .strafeTo(new Vector2d(56.5,-59))
                .build());

        leftBlueBot.runAction(leftBluePush2);

        rightBlueBot.runAction(rightBlueBot.getDrive().actionBuilder(new Pose2d(-12, 61.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(-36,36))
                .waitSeconds(0.2)
                .lineToY(12)
                .strafeTo(new Vector2d(-46.5,12))
                .strafeTo(new Vector2d(-46.5,59))
                .lineToY(12)
                .strafeTo(new Vector2d(-56.5,12))
                .strafeTo(new Vector2d(-56.5,59))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(leftRedBot)
                .addEntity(rightRedBot)
                .addEntity(leftBlueBot)
                .addEntity(rightBlueBot)
                .removeEntity(MeepMeep.getDEFAULT_AXES_ENTITY())
                .removeEntity(MeepMeep.getDEFAULT_COMPASS_ENTITY())
                .start();
    }
}