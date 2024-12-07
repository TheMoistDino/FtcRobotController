package org.firstinspires.ftc.teamcode.autonomous.oldauto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Disabled
@Autonomous(name = "Meet 1: Left Baskets", group = "Auto", preselectTeleOp = "Meet 1 TeleOp")
public class AutoMeetOneL extends LinearOpMode
{
    // Variables used for Method Calling
    MotorControl motorControl;
    ServoControl servoControl;
    /////////////////////////

    public class Lift
    {
        public class HighBasket implements Action
        {
            private boolean initialized = motorControl.isLiftRunning;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (!initialized)
                {
                    motorControl.LiftToPosition(MotorControl.LiftHeight.high_basket, 3);
                    initialized = motorControl.isLiftRunning;
                }
                else
                {
                    return false;
                }
                return false;
            }
        }
        public Action highBasket()
        {
            return new HighBasket();
        }

        public class LowBasket implements Action
        {
            private boolean initialized = motorControl.isLiftRunning;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (!initialized)
                {
                    motorControl.LiftToPosition(MotorControl.LiftHeight.low_basket, 3);
                    initialized = motorControl.isLiftRunning;
                }
                else
                {
                    return false;
                }
                return false;
            }
        }
        public Action lowBasket()
        {
            return new LowBasket();
        }

        public class Zero implements Action
        {
            private boolean initialized = motorControl.isLiftRunning;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (!initialized)
                {
                    motorControl.LiftToPosition(MotorControl.LiftHeight.zero, 3);
                    initialized = motorControl.isLiftRunning;
                }
                else
                {
                    return false;
                }
                return false;
            }
        }
        public Action zero()
        {
            return new Zero();
        }
    }

    public class Arm
    {
        public class Forward implements Action {
            private boolean initialized = motorControl.isArmRunning;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    motorControl.ArmToPosition(400, 2);
                    initialized = motorControl.isArmRunning;
                } else {
                    return false;
                }
                return false;
            }
        }
        public Action forward() {
            return new Forward();
        }

        public class Backward implements Action {
            private boolean initialized = motorControl.isArmRunning;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    motorControl.ArmToPosition(0, 2);
                    initialized = motorControl.isArmRunning;
                } else {
                    return false;
                }
                return false;
            }
        }
        public Action backward() {
            return new Backward();
        }
    }

    public class Claw
    {
        public class Grab implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servoControl.Grab();
                return false;
            }
        }
        public Action grab()
        {
            return new Grab();
        }
    }

    public class Bucket
    {
        public class Dump implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servoControl.Dump();
                sleep(500);
                return false;
            }
        }
        public Action dump()
        {
            return new Dump();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize other control objects
        motorControl = new MotorControl(hardwareMap, telemetry);
        servoControl = new ServoControl(hardwareMap, telemetry);
        /////////////////////////

        // Initialize Road Runner

        Lift lift = new Lift();
        Arm arm = new Arm();
        Claw claw = new Claw();
        Bucket bucket = new Bucket();

        Pose2d firstPose = new Pose2d(-35, -61.5, Math.toRadians(-90));
        Pose2d secondPose = new Pose2d(-57,-57, Math.toRadians(-135));
        Pose2d thirdPose = new Pose2d(-48,-44, Math.toRadians(-90));
        Pose2d fourthPose = new Pose2d(-52,-52, Math.toRadians(-135));
        Pose2d fifthPose = secondPose;
        Pose2d sixthPose = new Pose2d(-58, -44, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, firstPose);

        TrajectoryActionBuilder step1 = drive.actionBuilder(firstPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-57,-57));

        TrajectoryActionBuilder step2 = drive.actionBuilder(secondPose)
                .lineToXLinearHeading(-48, Math.toRadians(-90))
                .strafeTo(new Vector2d(-48,-44));

        TrajectoryActionBuilder step3 = drive.actionBuilder(thirdPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(2);

        TrajectoryActionBuilder step4 = drive.actionBuilder(fourthPose)
                .strafeTo(new Vector2d(-57, -57));

        TrajectoryActionBuilder step5 = drive.actionBuilder(fifthPose)
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-58,-57))
                .strafeTo(new Vector2d(-58,-44));

        TrajectoryActionBuilder step6 = drive.actionBuilder(sixthPose)
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-57,-57))
                .waitSeconds(0.5);

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

        Action scoreFirst = new SequentialAction(
                                new ParallelAction(
                                    trajectoryStep1,
                                    lift.highBasket()),
                                bucket.dump());
        Action setupSecond = new ParallelAction(
                                trajectoryStep2,
                                bucket.dump(),
                                lift.zero(),
                                arm.forward());
        Action intakeSecond = new SequentialAction(
                                new SleepAction(0.2),
                                claw.grab(),
                                new SleepAction(0.2),
                                arm.backward(),
                                new SleepAction(0.2),
                                claw.grab());
        Action scoreSecond = new SequentialAction(
                                new ParallelAction(
                                    trajectoryStep3,
                                    lift.highBasket()),
                                trajectoryStep4,
                                bucket.dump());
        Action setupThird = new SequentialAction(
                                new ParallelAction(
                                    trajectoryStep5,
                                    bucket.dump(),
                                    arm.forward()),
                                lift.zero());
        Action intakeThird = new SequentialAction(
                                claw.grab(),
                                new SleepAction(0.2),
                                arm.backward(),
                                new SleepAction(0.2),
                                claw.grab());
        Action scoreThird = new SequentialAction(
                                new ParallelAction(
                                    trajectoryStep6,
                                    lift.highBasket()),
                                bucket.dump());
        Action ascentPark = new ParallelAction(
                                trajectoryActionFinal,
                                bucket.dump(),
                                lift.lowBasket());

        // Initialize control functions
        servoControl.StartServos();

        telemetry.addData("robot ready","");
        telemetry.update();


        // Wait for play button to be pressed
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        scoreFirst,
                        setupSecond,
                        intakeSecond,
                        scoreSecond,
                        setupThird,
                        intakeThird,
                        scoreThird,
                        ascentPark
                )
        );

        // Stop all servos
        servoControl.StopServos();

        telemetry.addData("autonomous","stopped");
        telemetry.update();
    }
}
