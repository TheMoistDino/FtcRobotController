package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class MotorControl
{
    ///// Create Motor Variables
    public static DcMotor lift, arm;
    /////

    ///// Name of Control Motors on Driver Hub
    private static final String liftName = "lift",
                                armName = "arm";
    /////

    ///// Create and Define Motion Variables
    // Lift Variables
    double liftAccel = 0.5;
    double liftPower = 0.0;
    double max_liftPower = 1.0;
    public int currentLiftPos;
    public int minLiftPos = 0;
    public int maxLiftPos = 11200;
    public enum LiftDirection {up, down}
    public enum LiftHeight {high_basket, low_basket, high_chamber, low_chamber}
    int high_basket_pos = 9000, low_basket_pos = 5000,
        high_chamber_pos = 7000, low_chamber_pos = 3000;

    // Initialize the map
    Map<LiftHeight, Integer> liftPositions = new HashMap<>();

    // Arm Variables
    double armAccel = 0.5;
    double armPower = 0.0;
    double max_armPower;
    public int currentArmPos;
    public enum ArmDirection {forward, backward}
    /////

    ///// Create PIDF Variables
    private PIDController pidController;
    private static final double[] armPIDF = {0,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f
    private static final double[] liftPIDF = {0.006,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f
    /////

    ///// Create and Define Timer Variables to let the motors have time to run to position
    private final ElapsedTime runtime = new ElapsedTime();
    /////

    ///// Extra variables
    static Telemetry telemetry;
    /////

    // This method is used to initialize the motors of the robot
    public MotorControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Motor Objects
        MotorControl.lift = hardwareMap.get(DcMotor.class, liftName);
        MotorControl.arm  = hardwareMap.get(DcMotor.class, armName);
        // Instantiate Telemetry
        MotorControl.telemetry = telemetry;
        // Initialize the Map for liftPositions
        liftPositions.put(LiftHeight.high_basket, high_basket_pos);
        liftPositions.put(LiftHeight.low_basket, low_basket_pos);
        liftPositions.put(LiftHeight.high_chamber, high_chamber_pos);
        liftPositions.put(LiftHeight.low_chamber, low_chamber_pos);

        // If the joysticks aren't touched, the robot won't move (set to BRAKE)
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Motor Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Display Message on Screen
        telemetry.addData("initializing", "motors");
    }

    // This method is used to lock the position of the lift
    public void LockLift()
    {
        liftPower = 0;
        currentLiftPos = lift.getCurrentPosition();
        lift.setTargetPosition(currentLiftPos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(max_liftPower);
    }

    public void StopAndReturnLift()
    {
        // Calculate the distance to each limit
        int distanceToMin = Math.abs(currentLiftPos - minLiftPos);
        int distanceToMax = Math.abs(currentLiftPos - maxLiftPos);

        // Determine the closest limit
        int targetPosition = (distanceToMin < distanceToMax) ? (minLiftPos + 50) : (maxLiftPos - 50);

        // Move the lift to the closest limit
        lift.setTargetPosition(targetPosition);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(max_liftPower); // Assuming you have a max_liftPower variable
    }

    // This method is used to lock the position of the arm
    public void LockArm()
    {
        armPower = 0;
        currentArmPos = arm.getCurrentPosition();
        arm.setTargetPosition(currentArmPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(max_armPower);
    }
    public void LockArmPID()
    {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController.setPID(armPIDF[0],armPIDF[1],armPIDF[2]);
        currentArmPos = arm.getCurrentPosition();

        double pid = pidController.calculate(currentArmPos, currentArmPos);
        double ff = currentArmPos * armPIDF[3];
        double power = pid + ff;

        arm.setPower(power);

        telemetry.addData("arm pos", currentArmPos);
    }

    // This method is used to move the lift
    public void MoveLift(LiftDirection liftDirection, double LIFT_SPEED)
    {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (liftDirection)
        {
            case up:
                liftPower += liftAccel * (max_liftPower - liftPower);
                break;
            case down:
                liftPower -= liftAccel * (max_liftPower - liftPower);
                break;
        }

        lift.setPower(liftPower * LIFT_SPEED);

        currentLiftPos = lift.getCurrentPosition();
    }

    // This method is used to move the arm
    public void MoveArm(ArmDirection armDirection, double LIFT_SPEED)
    {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (armDirection)
        {
            case forward:
                max_armPower = 0.3;
                armPower += armAccel * (max_armPower - armPower);
                break;
            case backward:
                max_armPower = 0.4;
                armPower -= armAccel * (max_armPower - armPower);
                break;
        }

        arm.setPower(armPower * LIFT_SPEED);

        currentArmPos = arm.getCurrentPosition();
    }

    // This method is used to make the lift go to a specified position
    public void LiftToPosition(LiftHeight liftHeight, double timeoutSeconds)
    {
        int target;

        // Get the target position
        target = liftPositions.getOrDefault(liftHeight, 0);

        // Initialize the lift motor to the correct mode (to maximize power)
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Resets timer
        runtime.reset();

        // Gets the position of the lift motor
        currentLiftPos = lift.getCurrentPosition();

        while ((runtime.seconds() < timeoutSeconds) && (lift.isBusy()))
        {
            // Sets controller PID to the variables in the array
            pidController.setPID(liftPIDF[0],liftPIDF[1],liftPIDF[2]);

            // Gets position of lift motor
            currentLiftPos = lift.getCurrentPosition();

            // Calculates the power of the lift motor
            double liftPID = pidController.calculate(currentLiftPos, target);
            double ff = currentLiftPos * liftPIDF[3];

            // Sets the power of the lift motor
            double power = liftPID + ff;
            lift.setPower(power);

            // Update current state to telemetry
            telemetry.addData("currently running","");
            telemetry.update();
        }

        // After the lift runs to position, it stops
        lift.setPower(0);
        telemetry.addData("final position", lift.getCurrentPosition());
        telemetry.update();
    }

    // This method is used to make the arm go to a specified position
    public void ArmToPosition(int target, double timeoutSeconds)
    {
        // Initialize the arm motor to the correct mode (to maximize power)
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Resets timer
        runtime.reset();

        // Gets the position of the lift motor
        currentArmPos = arm.getCurrentPosition();

        while ((runtime.seconds() < timeoutSeconds) && (arm.isBusy()))
        {
            // Sets controller PID to the variables in the array
            pidController.setPID(armPIDF[0],armPIDF[1],armPIDF[2]);

            // Gets position of lift motor
            currentArmPos = arm.getCurrentPosition();

            // Calculates the power of the lift motor
            double armPID = pidController.calculate(currentArmPos, target);
            double ff = currentArmPos * armPIDF[3];

            // Sets the power of the lift motor
            double power = armPID + ff;
            arm.setPower(power);

            // Update current state to telemetry
            telemetry.addData("currently running","");
            telemetry.update();
        }

        // After the lift runs to position, it stops
        arm.setPower(0);
        telemetry.addData("final position", arm.getCurrentPosition());
        telemetry.update();
    }
}
