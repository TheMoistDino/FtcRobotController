package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;

@Autonomous(name = "Meet 0 Auto", group = "TeleOp")
public class AutoMeetZero extends LinearOpMode
{
    // Variables used for Method Calling
    HolonomicDrive holonomicDrive;
    /////////////////////////

    @Override
    public void runOpMode() throws InterruptedException
    {
        // For Holonomic Drive
        holonomicDrive = new HolonomicDrive(hardwareMap, telemetry);
        ////////////////////

        holonomicDrive.InitAuto();

        telemetry.addData("robot ready","");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        holonomicDrive.ForwardDrive(24,0.8,15);
    }
}
