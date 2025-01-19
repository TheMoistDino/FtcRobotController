package org.firstinspires.ftc.teamcode.custom_tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Servo Tuner", group = "Test")
public class servoTuner extends OpMode {
    public static double target_roll = 0.0;

    ServoImplEx roll;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        roll = hardwareMap.get(ServoImplEx.class, "roll");
    
        roll.setPwmRange(new PwmControl.PwmRange(500,2500));
    }

    @Override
    public void loop() {
        // Sets servo position
        roll.setPosition(target_roll);

        // Adds data to the telemetry/driver hub
        telemetry.addData("target_roll", target_roll);
        telemetry.update();
    }
}
