package org.firstinspires.ftc.teamcode.autonomous.oldauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;

@Disabled
@Autonomous(name = "Meet 1: Simple", group = "Auto")
public class AutoMeetZero extends LinearOpMode
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
        ////////////////////

        motorControl = new MotorControl(hardwareMap, telemetry);
        servoControl = new ServoControl(hardwareMap, telemetry);

        holonomicDrive.InitAuto();
        servoControl.StartServos();

        telemetry.addData("robot ready","");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        motorControl.LiftToPosition(MotorControl.LiftHeight.high_basket, 3);
        //sleep(3000);
        holonomicDrive.TimerStraight(-0.2, 3);
        servoControl.GrabOuttake();
        sleep(500);
        servoControl.GrabOuttake();
        holonomicDrive.TimerStraight(0.2, 2);
        motorControl.LiftToPosition(MotorControl.LiftHeight.zero, 3);
        //sleep(3000);
        holonomicDrive.TimerStraight(0.5, 3.0);
    }
}
