package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

// TODO: Transfer RR movement from MeepMeepTesting.java to this class


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

        // Initialize control functions
        holonomicDrive.InitAuto();
        servoControl.StartServos();

        telemetry.addData("robot ready","");
        telemetry.update();


        // Wait for play button to be pressed
        waitForStart();



        // Stop all servos
        servoControl.StopServos();

        telemetry.addData("autonomous","stopped");
        telemetry.update();
    }
}
