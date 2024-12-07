package org.firstinspires.ftc.teamcode.autonomous.oldauto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
@Autonomous(name = "Meet 1: Left Push", group = "Auto", preselectTeleOp = "Meet 1 TeleOp")
public class AutoMeetOneLPush extends LinearOpMode
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

        Action trajectoryStep1 = step1.build();
        Action trajectoryStep2 = step2.build();

        Action scoreFirst = new SequentialAction(
                                new ParallelAction(
                                    trajectoryStep1,
                                    lift.highBasket()),
                                bucket.dump());
        Action push = new ParallelAction(
                trajectoryStep2,
                lift.lowBasket()
        );

        // Initialize control functions
        servoControl.StartServos();

        telemetry.addData("robot ready","");
        telemetry.update();


        // Wait for play button to be pressed
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        scoreFirst,
                        push
                )
        );

        // Stop all servos
        servoControl.StopServos();

        telemetry.addData("autonomous","stopped");
        telemetry.update();
    }
}
