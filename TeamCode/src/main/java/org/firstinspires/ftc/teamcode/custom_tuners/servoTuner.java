package org.firstinspires.ftc.teamcode.custom_tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Servo Tuner", group = "Test")
public class servoTuner extends OpMode {
    public static double target_claw = 0.0;
    public static double target_bucket = 0.0;

    ServoImplEx claw;
    ServoImplEx bucket;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        claw = hardwareMap.get(ServoImplEx.class, "claw");
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");
    
        claw.setPwmRange(new PwmControl.PwmRange(700,2200));
        bucket.setPwmRange(new PwmControl.PwmRange(500,2500));
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
