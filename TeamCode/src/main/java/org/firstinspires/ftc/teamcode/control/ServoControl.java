package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoControl
{
    ///// Create Servo Variables
    static ServoImplEx claw, bucket;
    static ServoImplEx slide;
    /////

    ///// Name of Servos on Driver Hub
    private static final String clawName = "claw",
                                bucketName = "bucket";
    private static final String slideName = "slide";
    /////

    ///// Create and Define Motion Variables
    static boolean isGrab, isDumped;
    static final double closeClawPos = 0.34, // Change to closed claw position
                        openClawPos  = 0.0, // Change to open claw position
                        notDumpedPos = 0.9, // Change to not dumped position
                        dumpedPos = 0.0; // Change to dumped position
    /////

    ///// Create and Define Timer Variables to let the servos have time to run to position
    private final ElapsedTime runtime = new ElapsedTime();
    double timeout = 250;
    /////


    ///// Extra variables
    static Telemetry telemetry;
    /////


    // This method is used to initialize the servos of the robot
    public ServoControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Servo Objects
        ServoControl.claw = hardwareMap.get(ServoImplEx.class, clawName);
        ServoControl.bucket = hardwareMap.get(ServoImplEx.class, bucketName);

        // ServoControl.slide = hardwareMap.get(ServoImplEx.class, slideName);

        // Increase max range of slide servo
        // slide.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Disable servo power by default
        claw.setPwmDisable();
        bucket.setPwmDisable();
        // slide.setPwmDisable();

        // Instantiate Telemetry
        ServoControl.telemetry = telemetry;

        // Display Message on Screen
        telemetry.addData("servos", "initializing");
    }

    // This method is used to "turn on" the servos (help prevent movement between AUTO and TELEOP periods)
    public void StartServos()
    {
        // Enable servo power
        claw.setPwmEnable();
        bucket.setPwmEnable();
        // slide.setPwmEnable();

        // Start the claw and bucket in initial positions
        isGrab = false;
        isDumped = false;
        claw.setPosition(openClawPos);
        bucket.setPosition(notDumpedPos);

        // Display Message on Screen
        telemetry.addData("servos", "started");
    }

    // This method is used to "turn off" the servos (help prevent movement between AUTO and TELEOP periods)
    public void StopServos()
    {
        // Disable servo power
        claw.setPwmDisable();
        bucket.setPwmDisable();
        // slide.setPwmDisable();

        // Display Message on Screen
        telemetry.addData("stopping", "servos");
    }

    // This method is used to open/close the claw servo
    public void Grab()
    {
        // Restart timer
        runtime.reset();

        // Open/Close the Claw
        isGrab = !isGrab;
        claw.setPosition(isGrab ? openClawPos : closeClawPos);

        // Give time for the servo to run to position
        while(runtime.milliseconds() < timeout)
        {
            telemetry.addData("claw running for:", runtime.milliseconds());
            telemetry.update();
        }
        telemetry.addData("isGrab", isGrab);
        telemetry.update();
    }

    // This method is used to dump the bucket servo
    public void Dump()
    {
        // Restart timer
        runtime.reset();

        // Open/Close the Claw
        isDumped = !isDumped;
        bucket.setPosition(isDumped ? dumpedPos : notDumpedPos);

        // Give time for the servo to run to position
        while(runtime.milliseconds() < timeout)
        {
            telemetry.addData("bucket running for:", runtime.milliseconds());
            telemetry.update();
        }
        telemetry.addData("isDumped", isDumped);
        telemetry.update();
    }
}
