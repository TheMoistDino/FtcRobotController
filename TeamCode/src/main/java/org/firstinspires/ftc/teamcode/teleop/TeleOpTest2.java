package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.a;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.b;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.back;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.dpad_left;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.dpad_right;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.left_bumper;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.right_bumper;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.GamepadInput.start;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.InputType.onButtonHold;
import static org.firstinspires.ftc.teamcode.control.GamepadEx.InputType.onPress;
import static org.firstinspires.ftc.teamcode.control.MotorControl.ArmDirection.backward;
import static org.firstinspires.ftc.teamcode.control.MotorControl.ArmDirection.forward;
import static org.firstinspires.ftc.teamcode.control.MotorControl.LiftDirection.down;
import static org.firstinspires.ftc.teamcode.control.MotorControl.LiftDirection.up;
import static org.firstinspires.ftc.teamcode.control.MotorControl.LiftHeight.high_basket;
import static org.firstinspires.ftc.teamcode.control.MotorControl.LiftHeight.zero;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.control.GamepadEx;
import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test", group = "TeleOp")
public class TeleOpTest2 extends LinearOpMode
{
    // Variables for Method Calling
    HolonomicDrive drive;
    //TankDrive tankDrive;
    ServoControl servo;
    MotorControl motor;
    GamepadEx gamepadEx1, gamepadEx2;
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
        drive = new HolonomicDrive(hardwareMap, telemetry);
        //////////////////////

        // For Tank Drive
        //tankDrive = new TankDrive(hardwareMap, telemetry);
        //////////////////////

        // For Servo Control (Claws)
        servo = new ServoControl(hardwareMap, telemetry);
        //////////////////////

        // For Motor Control (Lift)
        motor = new MotorControl(hardwareMap, telemetry);
        //////////////////////

        // For gamepad state storage
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        //////////////////////

        // Initialize the map with lambda expressions for each action
        // Toggles
        gamepadEx1.addAction(start, onPress, () -> isFieldOriented = !isFieldOriented);
        gamepadEx1.addAction(left_bumper, onPress, () -> driveSlow = !driveSlow);
        gamepadEx2.addAction(start, onPress, () -> liftSlow = !liftSlow);
        gamepadEx2.addAction(b, onPress, () -> liftDebug = !liftDebug);
        gamepadEx2.addAction(back, onPress, () -> {
            MotorControl.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorControl.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });

        // Presets
        gamepadEx2.addAction(dpad_right, onPress, () -> {
            motor.LiftToPosition(high_basket, 4.5);
        });
        gamepadEx2.addAction(dpad_left, onPress, () -> {
            motor.LiftToPosition(zero, 4.5);
        });

        // Servo Control
        gamepadEx2.addAction(right_bumper, onPress, () -> {
            servo.GrabOuttake();
        });
        gamepadEx2.addAction(left_bumper, onPress, () -> {
            servo.GrabIntake();
        });
        // Motor Control
        // Buttons to move arm forward/backward
        gamepadEx2.addAction(a, onButtonHold, () -> {
            motor.ArmControl(forward, ARM_SPEED_MULTIPLIER);
        });
        gamepadEx2.addAction(dpad_left, onButtonHold, () -> {
            motor.ArmControl(backward, ARM_SPEED_MULTIPLIER);
        });
        // Buttons to move lift up/down
        gamepadEx2.addAction(right_bumper, onButtonHold, (liftDebug || motor.currentLiftPos < motor.maxLiftPos), () -> {
            motor.MoveLift(up, LIFT_SPEED_MULTIPLIER);
        });
        gamepadEx2.addAction(left_bumper, onButtonHold, (liftDebug || motor.currentLiftPos > motor.minLiftPos), () -> {
            motor.MoveLift(down, LIFT_SPEED_MULTIPLIER);
        });
        //////////////////////

        telemetry.addData("Robot Status","Ready");
        telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        // Start servos after the play button is pressed to avoid movement between AUTO and TELEOP periods
        servo.StartServos();

        while(opModeIsActive())
        {
            // Gamepad controls are updated & executed
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);

            // For Holonomic Drive
            drive.ActiveDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED_MULTIPLIER, isFieldOriented);
            //////////////////////

            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
            {
                motor.LockLift(); // Brake lift when no direction is specified
            }

            // If the lift is beyond the set min or max positions, return the lift to the nearest limit
            if ((motor.currentLiftPos < motor.minLiftPos || motor.currentLiftPos > motor.maxLiftPos) && (!liftDebug))
            {
                motor.StopAndReturnLift();
            }

            ////////////////////////////////////
            // Toggle to slow down driving
            DRIVETRAIN_SPEED_MULTIPLIER = driveSlow ? 0.4 : 0.8;
            // Toggle to slow down lift
            LIFT_SPEED_MULTIPLIER = liftSlow ? 0.5 : 1.0;
            // Toggle to slow down arm
            ARM_SPEED_MULTIPLIER = armSlow ? 0.5 : 1.0;

            telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
            telemetry.addData("Driving Speed", driveSlow ? "50%" : "100%");
            telemetry.addData("Lift Position", MotorControl.liftLeft.getCurrentPosition());
            telemetry.addData("Lift Debug Mode", liftDebug ? "On" : "Off");
            telemetry.addData("Lift Speed", liftSlow ? "50%" : "100%");
            telemetry.addData("Arm Position", MotorControl.outtakeArm.getCurrentPosition());
            telemetry.addData("Arm Speed", armSlow ? "50%" : "100%");

            telemetry.update();
        }
    }
}
