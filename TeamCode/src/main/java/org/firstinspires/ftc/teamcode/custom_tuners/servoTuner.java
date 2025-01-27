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
    public static double target_clawIntake = 0.0;

    ServoImplEx clawOuttake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        clawOuttake = hardwareMap.get(ServoImplEx.class, "clawIntake");
    
        clawOuttake.setPwmRange(new PwmControl.PwmRange(500,2500));
    }

    @Override
    public void loop() {
        // Sets servo position
        clawOuttake.setPosition(target_clawIntake);

        // Adds data to the telemetry/driver hub
        telemetry.addData("target_clawIntake", target_clawIntake);
        telemetry.update();
    }
}
