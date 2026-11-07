package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Calibration", group = "Test")
public class ServoCalibration extends OpMode {

    private Servo servo;
    private double position = 0.5; // safe-ish starting guess, in the middle of the range

    private static final double COARSE_STEP = 0.05;
    private static final double FINE_STEP = 0.005;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo"); // change to your servo's config name
    }

    @Override
    public void loop() {
        // Coarse adjust
        if (gamepad1.dpadUpWasPressed())   position += COARSE_STEP;
        if (gamepad1.dpadDownWasPressed()) position -= COARSE_STEP;

        // Fine adjust
        if (gamepad1.dpadRightWasPressed()) position += FINE_STEP;
        if (gamepad1.dpadLeftWasPressed())  position -= FINE_STEP;

        // Jump to the extremes
        if (gamepad1.aWasPressed()) position = 0.0;
        if (gamepad1.bWasPressed()) position = 1.0;

        position = Math.max(0.0, Math.min(1.0, position));
        servo.setPosition(position);

        telemetry.addData("Servo position", "%.3f", position);
        telemetry.addLine("D-pad up/down: +/- 0.05 | left/right: +/- 0.005");
        telemetry.addLine("A: go to 0.0 | B: go to 1.0");
        telemetry.update();
    }
}