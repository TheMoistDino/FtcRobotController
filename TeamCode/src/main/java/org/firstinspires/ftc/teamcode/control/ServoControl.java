package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoControl
{
    ///// Create Servo Variables
    static Servo claw, bucket;
    /////

    ///// Name of Servos on Driver Hub
    private static final String clawName = "claw",
                                bucketName = "bucket";
    /////

    ///// Create and Define Motion Variables
    static boolean isGrab, isDumped;
    static final double closeClawPos = 0.55, // Change to closed claw position
                        openClawPos  = 0.1, // Change to open claw position
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
        ServoControl.claw = hardwareMap.get(Servo.class, clawName);
        ServoControl.bucket = hardwareMap.get(Servo.class, bucketName);
        // Instantiate Telemetry
        ServoControl.telemetry = telemetry;

        ///// Start the claw in the "open" position
        isGrab = false;
        isDumped = false;
        claw.setPosition(openClawPos);
        bucket.setPosition(notDumpedPos);

        // Display Message on Screen
        telemetry.addData("initializing", "servos");
    }

    // This method is used to open/close the claw servo
    public void Grab()
    {
        // Restart timer
        runtime.reset();

        // Open/Close the Claw
        claw.setPosition(isGrab ? openClawPos : closeClawPos);
        isGrab = !isGrab;

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
        bucket.setPosition(isDumped ? dumpedPos : notDumpedPos);
        isDumped = !isDumped;

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
