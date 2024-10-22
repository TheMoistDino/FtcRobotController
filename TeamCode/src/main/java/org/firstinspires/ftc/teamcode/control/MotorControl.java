package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    double liftAccel = 0.5;
    double liftPower = 0.0;
    double max_liftPower = 1.0;
    double armAccel = 0.5;
    double armPower = 0.0;
    double max_armPower = 0.5;
    int currentLiftPos;
    int currentArmPos;
    public enum LiftDirection {up, down}
    public enum ArmDirection {up, down}
    /////

    ///// Create PIDF Variables
    private PIDController pidController;
    private static final double[] armPIDF = {0,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f
    private static final double[] liftPIDF = {0,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f
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
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(max_liftPower);
    }

    public void LockLiftPID()
    {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController.setPID(liftPIDF[0],liftPIDF[1],liftPIDF[2]);
        currentLiftPos = lift.getCurrentPosition();

        double pid = pidController.calculate(currentLiftPos, currentLiftPos);
        double ff = currentLiftPos * liftPIDF[3];

        double power = pid + ff;

        lift.setPower(power);

        telemetry.addData("lift pos", currentLiftPos);
    }

    // This method is used to lock the position of the arm
    public void LockArm()
    {
        armPower = 0;
        currentArmPos = arm.getCurrentPosition();
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

        if(liftDirection == LiftDirection.up)
        {
            liftPower += liftAccel * (max_liftPower - liftPower);
        }
        else
        {
            liftPower -= liftAccel * (max_liftPower - liftPower);
        }

        lift.setPower(liftPower * LIFT_SPEED);

        currentLiftPos = lift.getCurrentPosition();
    }

    // This method is used to move the arm
    public void MoveArm(ArmDirection armDirection, double LIFT_SPEED)
    {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(armDirection == ArmDirection.up)
        {
            armPower += armAccel * (max_armPower - armPower);
        }
        else
        {
            armPower -= armAccel * (max_armPower - armPower);
        }

        arm.setPower(armPower * LIFT_SPEED);

        currentArmPos = arm.getCurrentPosition();
    }
}
