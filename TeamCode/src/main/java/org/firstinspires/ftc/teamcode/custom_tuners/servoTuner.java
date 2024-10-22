package org.firstinspires.ftc.teamcode.custom_tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Tuner", group = "Test")
public class servoTuner extends OpMode {
    public static double target_claw = 0.5;
    public static double target_bucket = 1.0;

    Servo claw;
    Servo bucket;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
    }

    @Override
    public void loop() {
        // Sets servo position
        claw.setPosition(target_claw);
        bucket.setPosition(target_bucket);

        // Adds data to the telemetry/driver hub
        telemetry.addData("target_claw", target_claw);
        telemetry.addData("target_bucket", target_bucket);
        telemetry.update();
    }
}
