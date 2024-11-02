package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Meet 1 TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode
{
    // Variables for Method Calling
    HolonomicDrive holonomicDrive;
    //TankDrive tankDrive;
    ServoControl servoControl;
    MotorControl motorControl;
    //////////////////////

    double DRIVETRAIN_SPEED_MULTIPLIER = 1.0;
    double LIFT_SPEED_MULTIPLIER = 1.0;

    // Initialize a variable to store the current driving mode
    boolean isFieldOriented = false;

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
                // Call the appropriate driving method based on the current mode
            if (isFieldOriented)
            {
                // Call your field-oriented driving method
                holonomicDrive.ActiveDriveFO
                        (gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED_MULTIPLIER);
            }
            else
            {
                // Call your robot-oriented driving method
                holonomicDrive.ActiveDriveRO
                        (gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED_MULTIPLIER);
            }
            //////////////////////

            // For Tank Drive
            //tankDrive.ActiveDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);
            //////////////////////

            // Button to slow down driving
            DRIVETRAIN_SPEED_MULTIPLIER = gamepad2.right_bumper ? 0.5 : 1.0;

            // Button to slow down lift
            LIFT_SPEED_MULTIPLIER = gamepad2.left_bumper ? 0.5 : 1.0;

            // Button to control the claw servo
            if(gamepad1.right_bumper)
            {
                servoControl.Grab();
            }

            // Button to control the bucket servo
            if(gamepad1.left_bumper)
            {
                servoControl.Dump();
            }

            // Determine the desired lift direction based on trigger input
            MotorControl.LiftDirection liftDirection = null;
            if (gamepad1.right_trigger != 0 && motorControl.currentLiftPos < motorControl.maxLiftPos)
            {
                liftDirection = MotorControl.LiftDirection.up;
            }
            else if (gamepad1.left_trigger != 0 && motorControl.currentLiftPos > motorControl.minLiftPos)
            {
                liftDirection = MotorControl.LiftDirection.down;
            }

            // Control the lift based on the determined direction
            if (liftDirection == MotorControl.LiftDirection.up)
            {
                motorControl.MoveLift(liftDirection, LIFT_SPEED_MULTIPLIER);
            }
            else if (liftDirection == MotorControl.LiftDirection.down)
            {
                motorControl.MoveLift(liftDirection, LIFT_SPEED_MULTIPLIER);
            }
            else
            {
                motorControl.LockLift(); // Brake lift when no direction is specified
            }

            // If the lift is beyond the set min or max positions, return the lift to the nearest limit
            if (motorControl.currentLiftPos < motorControl.minLiftPos || motorControl.currentLiftPos > motorControl.maxLiftPos)
            {
                motorControl.StopAndReturnLift();
            }

            // Buttons to move arm up/down
            if(gamepad1.dpad_up)
            {
                motorControl.MoveArm(MotorControl.ArmDirection.up, LIFT_SPEED_MULTIPLIER);
            }
            else if(gamepad1.dpad_down)
            {
                motorControl.MoveArm(MotorControl.ArmDirection.down, LIFT_SPEED_MULTIPLIER);
            }
            else if(!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                // Brake arm
                motorControl.LockArm();
            }

            if(gamepad2.start)
            {
                // Toggle the driving mode
                isFieldOriented = !isFieldOriented;

                // Provide feedback to the user (optional)
                telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
                telemetry.update();

                // Add a small delay to avoid rapid toggling
                sleep(200);
            }

            telemetry.update();
        }
    }
}
