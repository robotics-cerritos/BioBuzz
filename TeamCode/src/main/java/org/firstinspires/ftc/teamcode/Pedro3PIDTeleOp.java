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
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Field Centric TeleOp (Pedro 3)", group = "TeleOp")
public class Pedro3PIDTeleOp extends OpMode {

    private Follower follower;

    private DcMotorEx launcher;
    private DcMotorEx intake;

    private boolean launcherOn = false;
    private boolean intakeOn = false;

    // Tune these as needed (range -1.0 to 1.0)
    private static final double LAUNCHER_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;

    private Servo servo;
    private boolean servoOpen = false;

    // Replace these with the values you read from the calibration OpMode
    private static final double SERVO_START_POS = 0.20;  // closed / start position
    private static final double SERVO_OPEN_POS  = 0.65;  // open position

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        // Starting pose if you didn't run an auto first.
        // Heading here defines which way "forward" is for field-centric driving.
        follower.setPose(new Pose(0, 0, 0));

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Flip either of these if the mechanism spins the wrong way:
        // launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo = hardwareMap.get(Servo.class, "servo"); // your config name
        servo.setPosition(SERVO_START_POS);
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

        // A toggles launcher
        if (gamepad1.aWasPressed()) {
            launcherOn = !launcherOn;
        }
        launcher.setPower(launcherOn ? LAUNCHER_POWER : 0);

        // Y toggles intake
        if (gamepad1.yWasPressed()) {
            intakeOn = !intakeOn;
        }
        intake.setPower(intakeOn ? INTAKE_POWER : 0);

        // B toggles servo open/closed
        if (gamepad1.bWasPressed()) {
            servoOpen = !servoOpen;
        }
        servo.setPosition(servoOpen ? SERVO_OPEN_POS : SERVO_START_POS);

        // Must be called every loop
        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Servo", servoOpen ? "OPEN" : "CLOSED");
        telemetry.update();
    }
}