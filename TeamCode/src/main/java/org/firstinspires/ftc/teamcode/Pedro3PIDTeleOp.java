package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants; // your tuned Constants class

@TeleOp(name = "Field Centric TeleOp (Pedro 3)", group = "TeleOp")
public class Pedro3PIDTeleOp extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        // Starting pose if you didn't run an auto first.
        // Heading here defines which way "forward" is for field-centric driving.
        follower.setPose(new Pose(0, 0, 0));


    }

    @Override
    public void start() {
        // OPTIONAL: if you pass the pose from auto -> TeleOp with an OpModeStorage class
        // (see Pedro docs), uncomment these two lines:
        // follower.setPose(OpModeStorage.autonomousEndPose);
        // follower.update();
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );

        follower.manual(powers);

        if (gamepad1.startWasPressed()) {
            Pose current = follower.pose();
            follower.setPose(new Pose(current.x(), current.y(), Math.toRadians(0)));
        }

        // Must be called every loop
        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Slow mode", gamepad1.left_bumper);
        telemetry.update();
    }
}