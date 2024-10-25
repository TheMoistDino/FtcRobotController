package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;

@TeleOp(name = "Meet 0 TeleOp", group = "TeleOp")
public class TeleOpMeetZero extends LinearOpMode
{
    // Variables for Method Calling
    HolonomicDrive holonomicDrive;
    //TankDrive tankDrive;
    ServoControl servoControl;
    MotorControl motorControl;
    //////////////////////

    double DRIVETRAIN_SPEED = 1.0;
    double LIFT_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // For Holonomic Drive
        holonomicDrive = new HolonomicDrive(hardwareMap, telemetry);
        //////////////////////

        // For Tank Drive
        //tankDrive = new TankDrive(hardwareMap, telemetry);
        //////////////////////

        // For Servo Control (Claws)
        servoControl = new ServoControl(hardwareMap, telemetry);
        //////////////////////

        // For Motor Control (Lift)
        motorControl = new MotorControl(hardwareMap, telemetry);
        //////////////////////


        telemetry.addData("robot ready","");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        while(opModeIsActive())
        {
            // For Holonomic Drive
            holonomicDrive.ActiveDriveRO(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED);
            //////////////////////

            // For Tank Drive
            //tankDrive.ActiveDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);
            //////////////////////

            // Button to slow down driving
            DRIVETRAIN_SPEED = gamepad2.right_bumper ? 0.5 : 1.0;

            // Button to slow down lift
            LIFT_SPEED = gamepad2.left_bumper ? 0.5 : 1.0;

            // Button to control the claw servo

            if(gamepad1.right_bumper)
            {
                servoControl.Grab();
            }

            if(gamepad1.left_bumper)
            {
                servoControl.Dump();
            }

            // Buttons to move lift up/down
            if(gamepad1.right_trigger != 0)
            {
                motorControl.MoveLift(MotorControl.LiftDirection.up, LIFT_SPEED);
            }
            else if(gamepad1.left_trigger != 0)
            {
                motorControl.MoveLift(MotorControl.LiftDirection.down, LIFT_SPEED);
            }
            else if((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
            {
                // Brake lift
                motorControl.LockLift();
            }

            // Buttons to move arm up/down
            if(gamepad1.dpad_up)
            {
                motorControl.MoveArm(MotorControl.ArmDirection.up, LIFT_SPEED);
            }
            else if(gamepad1.dpad_down)
            {
                motorControl.MoveArm(MotorControl.ArmDirection.down, LIFT_SPEED);
            }
            else if(!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                // Brake arm
                motorControl.LockArm();
            }

            telemetry.update();
        }
    }
}
