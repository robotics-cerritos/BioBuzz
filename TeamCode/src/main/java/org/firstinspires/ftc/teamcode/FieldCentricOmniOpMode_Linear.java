/*
 *   Copyright (c) 2026 Alan Smith / Team Code
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Field Centric Drive", group = "TeleOp")
public class FieldCentricOmniOpMode_Linear extends LinearOpMode {

    // Create an instance of the sensor
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        // 1. Initialize Drive Motors
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Reverse left motors (adjust based on your robot's physical wiring)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior to brake for crisp stops
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Initialize and configure the goBILDA Pinpoint computer
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 3. Always update the Pinpoint at the start of the loop to pull fresh sensor data
            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

            // Press A to reset full position and heading back to 0, 0, 0
            if (gamepad1.a) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }

            // Press B to reset ONLY the heading/orientation to 0 while keeping current X and Y positions
            if (gamepad1.b) {
                pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH,
                        pose2D.getX(DistanceUnit.INCH),
                        pose2D.getY(DistanceUnit.INCH),
                        AngleUnit.DEGREES,
                        0
                ));
            }

            // Read gamepad inputs (Invert Y stick because gamepad Y is negative-up)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Get the current robot heading in radians from the Pose2D object
            double botHeading = pose2D.getHeading(AngleUnit.RADIANS);

            // Rotate the movement direction vector by the bot's heading for field-centric math
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator ensures motor power values never exceed 1.0 while maintaining ratios
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower  = (rotY + rotX - rx) / denominator;
            double backLeftPower   = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower  = (rotY + rotX + rx) / denominator;

            // Apply calculated power to motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry feedback for debugging tracking and field orientation
            telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Device Status", pinpoint.getDeviceStatus());
            telemetry.update();
        }
    }

    public void configurePinpoint() {
        // Set odometry pod offsets matching your build
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // Set the kind of pods used by your robot
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set the counting directions for the pods
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Recalibrate IMU and reset position to 0,0,0
        pinpoint.resetPosAndIMU();
    }
}