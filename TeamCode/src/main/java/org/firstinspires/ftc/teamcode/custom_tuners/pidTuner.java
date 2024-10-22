package org.firstinspires.ftc.teamcode.custom_tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PID Tuner", group = "Test")
public class pidTuner extends OpMode {
    private PIDController controller;
    // Tune p first, then d, then i (start small)
    public static double kP = 0, kI = 0, kD = 0, kF = 0;

    public static int target = 0;

    DcMotorEx armMotor;
    @Override
    public void init() {
        controller = new PIDController(kP,kI,kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
    }

    @Override
    public void loop() {
        // Sets controller pid to variables: p, i, and d
        controller.setPID(kP,kI,kD);

        // Gets positions of motor
        int armPos = armMotor.getCurrentPosition();

        // Calculates how much to go based on the position to run to the target
        double pid = controller.calculate(armPos, target);

        // Calculates Feed Forward so the robot adjusts against resisting forces
        double ff = target * kF;

        // Calculates power for motor
        double power = pid + ff;

        // Sets motor powers
        armMotor.setPower(power);

        // Adds data to the telemetry/driver hub
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
