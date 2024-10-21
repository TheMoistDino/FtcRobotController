package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HolonomicDrive
{
    ///// Create Motor Variables
    static DcMotor leftFront, leftBack, rightFront, rightBack;
    /////

    ///// Name of Drive Motors on Driver Hub
    private static final String leftFrontName  = "leftFront",
                                leftBackName   = "leftBack",
                                rightFrontName = "rightFront",
                                rightBackName  = "rightBack";
    /////

    ///// Create Motion Variables
    double accel = 0.5; // Determine how fast the robot should go to full speed

    // Actual motor power
    double  leftFront_power  = 0,
            leftBack_power   = 0,
            rightFront_power = 0,
            rightBack_power  = 0;

    // Intended motor power
    double  target_leftFront_power  = 0,
            target_leftBack_power   = 0,
            target_rightFront_power = 0,
            target_rightBack_power  = 0;
    /////

    ///// Create Input Variables
    double x, y, turn;
    /////

    ///// Physical to Digital Variables
    //Get the motor's ppr and multiply it by 4 to get counts per motor rev
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    //If you have no drive gear reduction leave at 1
    static final double DRIVE_GEAR_REDUCTION = 1;
    //Input the diameter measure in inches
    static final double WHEEL_DIAMETER_INCHES = (10.4) / (2.54);
    //How many times the motor goes per a inch
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private PIDController pidController;
    private static double[] drivePIDF = {0,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f

    ///// Create and Define Timer Variables to let the motors have time to run to position
    private ElapsedTime runtime = new ElapsedTime();
    /////


    ///// Extra variables
    static Telemetry telemetry;
    /////


    // This method is used to initialize the motors/drivetrain of the robot to holonomic/mecanum drive
    public HolonomicDrive(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Motor Objects
        HolonomicDrive.leftFront  = hardwareMap.get(DcMotor.class, leftFrontName);
        HolonomicDrive.leftBack   = hardwareMap.get(DcMotor.class, leftBackName);
        HolonomicDrive.rightFront = hardwareMap.get(DcMotor.class, rightFrontName);
        HolonomicDrive.rightBack  = hardwareMap.get(DcMotor.class, rightBackName);

        // If the joysticks aren't touched, the robot won't move (set to BRAKE)
        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Instantiate Telemetry
        MotorControl.telemetry = telemetry;

        // Display Message on Screen
        telemetry.addData("initializing", "motors");

        // Reverse Motor Directions for Positive Values
        leftBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("reversing", "motors");
    }


    // This method is used to set the drive motor's powers in TeleOp
    public void ActiveDrive(double leftStickX, double leftStickY, double rightStickX, double DRIVETRAIN_SPEED)
    {
        double max; // Limit motor's powers to 100%

        // Cool vector math to calculate power to wheels
        y    = -leftStickY;
        x    = leftStickX;
        turn = rightStickX;

        // Combine variables to find power and set the intended power
        target_leftFront_power  = y + x + turn;
        target_leftBack_power   = y - x + turn;
        target_rightFront_power = y - x - turn;
        target_rightBack_power  = y + x - turn;

        max = Math.max(Math.abs(target_leftFront_power), Math.abs(target_rightFront_power));
        max = Math.max(max, Math.abs(target_leftBack_power));
        max = Math.max(max, Math.abs(target_rightBack_power));

        if (max > 1.0) // Limit motor's powers to 100%
        {
            target_leftFront_power  /= max;
            target_rightFront_power /= max;
            target_leftBack_power   /= max;
            target_rightBack_power  /= max;
        }

        // Apply acceleration to the motor powers
        leftFront_power  += accel * (target_leftFront_power  - leftFront_power);
        leftBack_power   += accel * (target_leftBack_power   - leftBack_power);
        rightFront_power += accel * (target_rightFront_power - rightFront_power);
        rightBack_power  += accel * (target_rightBack_power  - rightBack_power);

        // Set motor powers to desired values
        leftFront .setPower(leftFront_power  * DRIVETRAIN_SPEED);
        leftBack  .setPower(leftBack_power   * DRIVETRAIN_SPEED);
        rightFront.setPower(rightFront_power * DRIVETRAIN_SPEED);
        rightBack .setPower(rightBack_power  * DRIVETRAIN_SPEED);

        // Display motor power
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFront_power, rightFront_power);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBack_power, rightBack_power);
    }

    // This method is used to drive the robot forward some number of inches forward in Auto
    public void ForwardDrive(double inchesForward, double maxPower, double timeoutS)
    {
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tells the motors the target position
        leftFront.setTargetPosition((leftFront.getCurrentPosition() + (int) (-inchesForward * COUNTS_PER_INCH)));
        leftBack.setTargetPosition((leftBack.getCurrentPosition() + (int) (-inchesForward * COUNTS_PER_INCH)));
        rightFront.setTargetPosition((rightFront.getCurrentPosition() + (int) (-inchesForward * COUNTS_PER_INCH)));
        rightBack.setTargetPosition((rightBack.getCurrentPosition() + (int) (-inchesForward * COUNTS_PER_INCH)));

        //Sets the mode of the motors to run to the positions
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Resets timer
        runtime.reset();

        //Sets power to the motors
        leftFront .setPower(maxPower);
        leftBack  .setPower(maxPower);
        rightFront.setPower(maxPower);
        rightBack .setPower(maxPower);

        //Puts the current position of the motors and puts it onto the telemetry
        while ((runtime.seconds() < timeoutS) &&  (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("currently running","");
            telemetry.update();
        }
        //After it runs to the position it stops
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        //Sets mode back to run without the encoder
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ForwardDrivePID(double inchesForward, double timeoutS)
    {
        // Resets timer
        runtime.reset();

        // Gets positions of drive motors
        int lfPos = leftFront .getCurrentPosition(), lbPos = leftBack .getCurrentPosition(),
            rfPos = rightFront.getCurrentPosition(), rbPos = rightBack.getCurrentPosition();

        // Sets the target positions for each drive motor
        int lfTarget = lfPos + (int) (-inchesForward * COUNTS_PER_INCH),
            lbTarget = lbPos + (int) (-inchesForward * COUNTS_PER_INCH),
            rfTarget = rfPos + (int) (-inchesForward * COUNTS_PER_INCH),
            rbTarget = rbPos + (int) (-inchesForward * COUNTS_PER_INCH);

        while ((runtime.seconds() < timeoutS) &&  (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            // Sets controller PID to the variables set
            pidController.setPID(drivePIDF[0],drivePIDF[1],drivePIDF[2]);

            // Gets positions of drive motors
            lfPos = leftFront .getCurrentPosition();
            lbPos = leftBack  .getCurrentPosition();
            rfPos = rightFront.getCurrentPosition();
            rbPos = rightBack .getCurrentPosition();

            // Calculates how much power to input based on the position to run to the desired target
            double lfPID = pidController.calculate(lfPos, lfTarget),
                   lbPID = pidController.calculate(lbPos, lbTarget),
                   rfPID = pidController.calculate(rfPos, rfTarget),
                   rbPID = pidController.calculate(rbPos, rbTarget);

            double lfFF = lfTarget * drivePIDF[3],
                   lbFF = lbTarget * drivePIDF[3],
                   rfFF = rfTarget * drivePIDF[3],
                   rbFF = rbTarget * drivePIDF[3];

            double lfPower = lfPID + lfFF,
                   lbPower = lbPID + lbFF,
                   rfPower = rfPID + rfFF,
                   rbPower = rbPID + rbFF;

            //Sets power to the motors
            leftFront .setPower(lfPower);
            leftBack  .setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack .setPower(rbPower);

            telemetry.addData("currently running","");
            telemetry.update();
        }

        //After it runs to the position it stops
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addData("finished driving","");
        telemetry.update();
    }

    // This method is used to initialize the drive motors' modes
    public void InitAuto()
    {
        //Stops and resets the encoder on the motors
        leftFront .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets mode to run without the encoder (to maximize motor efficiency/power)
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("drive motors' mode set","");
    }
}
