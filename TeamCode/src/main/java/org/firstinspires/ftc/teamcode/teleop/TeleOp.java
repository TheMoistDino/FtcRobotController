package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    boolean driveSlow = false;
    double LIFT_SPEED_MULTIPLIER = 1.0;
    boolean liftSlow = false;
    double ARM_SPEED_MULTIPLIER = 1.0;
    boolean armSlow = false;

    boolean isFieldOriented = false; // Initialize a variable to store the current driving mode
    boolean liftDebug = false; // Initialize a variable to store the current lift debug mode

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

        // For gamepad state storage
        Gamepad currentGamepad1 = new Gamepad();   // This is to prevent rapid
        Gamepad currentGamepad2 = new Gamepad();   // toggling of gamepad inputs.
        Gamepad previousGamepad1 = new Gamepad();  // Holding down a button no
        Gamepad previousGamepad2 = new Gamepad();  // longer repeats inputs.
        //////////////////////

        telemetry.addData("Robot Status","Ready");
        telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        // Start servos after the play button is pressed to avoid movement between AUTO and TELEOP periods
        servoControl.StartServos();

        while(opModeIsActive())
        {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // For Holonomic Drive
                // Call the appropriate driving method based on the current mode
            if (isFieldOriented)
            {
                // Call field-oriented driving method
                holonomicDrive.ActiveDriveFO
                        (gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED_MULTIPLIER);
            }
            else
            {
                // Call robot-oriented driving method
                holonomicDrive.ActiveDriveRO
                        (gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED_MULTIPLIER);
            }
            //////////////////////

            // For Tank Drive
            //tankDrive.ActiveDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, telemetry);
            //////////////////////

            // Button to control the claw servo
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
            {
                servoControl.Grab();
            }

            // Button to control the bucket servo
            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
            {
                servoControl.Dump();
            }

            // Determine the desired lift direction based on trigger input
            if (gamepad1.right_trigger != 0 && (liftDebug || motorControl.currentLiftPos < motorControl.maxLiftPos))
            {
                motorControl.MoveLift(MotorControl.LiftDirection.up, LIFT_SPEED_MULTIPLIER);
            }
            else if (gamepad1.left_trigger != 0 && (liftDebug || motorControl.currentLiftPos > motorControl.minLiftPos))
            {
                motorControl.MoveLift(MotorControl.LiftDirection.down, LIFT_SPEED_MULTIPLIER);
            }
            else
            {
                motorControl.LockLift(); // Brake lift when no direction is specified
            }

            // If the lift is beyond the set min or max positions, return the lift to the nearest limit
            if ((motorControl.currentLiftPos < motorControl.minLiftPos || motorControl.currentLiftPos > motorControl.maxLiftPos) && (!liftDebug))
            {
                motorControl.StopAndReturnLift();
            }

            // Buttons to move arm up/down
            if(gamepad1.dpad_up)
            {
                motorControl.ArmControl(MotorControl.ArmDirection.forward, ARM_SPEED_MULTIPLIER);
            }
            else if(gamepad1.dpad_down)
            {
                motorControl.ArmControl(MotorControl.ArmDirection.backward, ARM_SPEED_MULTIPLIER);
            }
            // Buttons to setup arm to enter submersible
            else if((!gamepad1.dpad_up) && (!gamepad1.dpad_down))
            {
                motorControl.ArmControl(MotorControl.ArmDirection.setup, ARM_SPEED_MULTIPLIER);
            }


            if(currentGamepad2.start && !previousGamepad2.start)
            {
                // Toggle the driving mode
                isFieldOriented = !isFieldOriented;
            }

            if(currentGamepad2.b && !previousGamepad2.b)
            {
                // Toggle liftDebug
                liftDebug = !liftDebug;
            }

            if(currentGamepad2.back && !previousGamepad2.back)
            {
                // Reset lift position
                motorControl.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Button to slow driving
            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper)
            {
                driveSlow = !driveSlow;
            }

            // Button to slow lift
            if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
            {
                liftSlow = !liftSlow;
            }

            // Button to slow arm
            if(currentGamepad2.a && !previousGamepad2.a)
            {
                armSlow = !armSlow;
            }

            // Toggle to slow down driving
            DRIVETRAIN_SPEED_MULTIPLIER = driveSlow ? 0.4 : 0.8;
            // Toggle to slow down lift
            LIFT_SPEED_MULTIPLIER = liftSlow ? 0.5 : 1.0;
            // Toggle to slow down arm
            ARM_SPEED_MULTIPLIER = armSlow ? 0.5 : 1.0;

            telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
            telemetry.addData("Driving Speed", driveSlow ? "50%" : "100%");
            telemetry.addData("Lift Position", motorControl.lift.getCurrentPosition());
            telemetry.addData("Lift Debug Mode", liftDebug ? "On" : "Off");
            telemetry.addData("Lift Speed", liftSlow ? "50%" : "100%");
            telemetry.addData("Arm Position", motorControl.arm.getCurrentPosition());
            telemetry.addData("Arm Speed", armSlow ? "50%" : "100%");

            telemetry.update();
        }
    }
}
