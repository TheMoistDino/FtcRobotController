package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;

@Autonomous(name = "Meet 2: Right", group = "Auto", preselectTeleOp = "Meet 2 TeleOp")
public class AutoMeetTwoR extends LinearOpMode
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

        holonomicDrive.TimerStraight(0.3, 3.0);
    }
}
