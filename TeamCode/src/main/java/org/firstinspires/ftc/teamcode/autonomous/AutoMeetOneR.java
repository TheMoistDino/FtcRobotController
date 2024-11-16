package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Disabled
@Autonomous(name = "Meet 1: Right", group = "Auto", preselectTeleOp = "Meet 1 TeleOp")
public class AutoMeetOneR extends LinearOpMode
{
    // Variables used for Method Calling
    HolonomicDrive holonomicDrive;
    MotorControl motorControl;
    ServoControl servoControl;
    /////////////////////////

    @Override
    public void runOpMode() throws InterruptedException
    {
        // For Holonomic Drive
        holonomicDrive = new HolonomicDrive(hardwareMap, telemetry);

        // Initialize other control objects
        motorControl = new MotorControl(hardwareMap, telemetry);
        servoControl = new ServoControl(hardwareMap, telemetry);
        /////////////////////////

        // Initialize Road Runner

        Pose2d initialPose = new Pose2d(-35, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder push = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(36,-36))
                .waitSeconds(0.2)
                .lineToY(-12)
                .strafeTo(new Vector2d(46.5,-12))
                .strafeTo(new Vector2d(46.5,-59))
                .lineToY(-12)
                .strafeTo(new Vector2d(56.5,-12))
                .strafeTo(new Vector2d(56.5,-59));

        // Initialize control functions
        holonomicDrive.InitAuto();
        servoControl.StartServos();

        telemetry.addData("robot ready","");
        telemetry.update();

        // Wait for play button to be pressed
        waitForStart();

        Actions.runBlocking(push.build());

        // Stop all servos
        servoControl.StopServos();

        telemetry.addData("autonomous","stopped");
        telemetry.update();
    }
}
