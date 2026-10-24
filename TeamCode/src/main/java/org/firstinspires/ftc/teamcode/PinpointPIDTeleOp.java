package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Field-centric mecanum TeleOp using the goBILDA Pinpoint odometry computer.
 *
 * Features
 *  - Field-centric driving (heading from Pinpoint)
 *  - PID heading hold: when you release the turn stick, the robot locks its heading
 *  - PID drive-to-pose: hold A to drive to TARGET_X / TARGET_Y / TARGET_H
 *  - Live PID tuning on gamepad2 (see controls below)
 *
 * Requires GoBildaPinpointDriver.java in the same package (copy it from the SDK
 * samples folder or goBILDA's GitHub), and a hardware-config device named "pinpoint"
 * of type "goBILDA Pinpoint Odometry Computer".
 *
 * Pinpoint coordinate convention: +X = forward, +Y = left, heading CCW-positive.
 *
 * Controls
 *  gamepad1: left stick = move, right stick X = turn, A (hold) = drive to target,
 *            B = set current position as target, OPTIONS = reset pose/heading to 0
 *  gamepad2: X = switch tuning between HEADING and TRANSLATION PID
 *            D-pad up/down = kP +/-10%, D-pad right/left = kD +/-10%
 *            Y/A = kI up/down (small steps)
 */
@TeleOp(name = "Pinpoint PID TeleOp", group = "Main")
public class PinpointPIDTeleOp extends LinearOpMode {

    // ---------------- Hardware names (change to match your config) ----------------
    static final String LF = "front_left_drive", LB = "back_left_drive", RF = "front_right_drive", RB = "back_right_drive";
    static final String PINPOINT = "pinpoint";

    // ---------------- Pod offsets (MILLIMETERS) ----------------
    // X pod (the one that measures FORWARD motion): how far LEFT of the robot's
    // center of rotation it sits (left = positive, right = negative).
    static final double X_POD_OFFSET_MM = -110.0;
    // Y pod (the one that measures SIDEWAYS motion): how far FORWARD of the robot's
    // center of rotation it sits (forward = positive, behind = negative).
    static final double Y_POD_OFFSET_MM = -75.0;

    // ---------------- PID gains (starting points, tune these!) ----------------
    // Heading PID: error in radians -> turn power
    static final PID headingPID = new PID(1.2, 0.0, 0.08);
    // Translation PID (shared by X and Y): error in inches -> drive power
    static final PID xPID = new PID(0.04, 0.0, 0.005);
    static final PID yPID = new PID(0.04, 0.0, 0.005);

    // ---------------- Drive-to-pose target (field frame, inches / radians) ----------------
    static double TARGET_X = 24.0;
    static double TARGET_Y = 0.0;
    static double TARGET_H = 0.0;

    static final double MAX_AUTO_POWER = 0.7;
    static final double STICK_DEADBAND = 0.05;
    static final double TURN_RELEASE_SETTLE_S = 0.25; // wait for spin to stop before locking heading

    private DcMotorEx lf, lb, rf, rb;
    private GoBildaPinpointDriver pinpoint;

    private double headingTarget = 0;
    private final ElapsedTime sinceTurn = new ElapsedTime();
    private boolean wasAutoDriving = false;

    // tuning state
    private boolean tuningHeading = true;
    private boolean pX, pUp, pDown, pLeft, pRight, pY, pA, pB;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotorEx.class, LF);
        lb = hardwareMap.get(DcMotorEx.class, LB);
        rf = hardwareMap.get(DcMotorEx.class, RF);
        rb = hardwareMap.get(DcMotorEx.class, RB);

        // Typical goBILDA mecanum: reverse the left side. Flip if your robot drives wrong.
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{lf, lb, rf, rb}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // odometry handles position
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Verify these by pushing the robot (see tuning guide): forward => X up, left => Y up.
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // robot must be STILL during this (~0.25 s)

        telemetry.addLine("Keep the robot still, then press START");
        telemetry.update();
        waitForStart();

        pinpoint.update();
        headingTarget = heading();
        sinceTurn.reset();

        while (opModeIsActive()) {
            pinpoint.update(); // call exactly once per loop

            double x = pos(true), y = pos(false), h = heading();

            // ---- Inputs ----
            double stickFwd = -gamepad1.left_stick_y;   // field forward (away from driver)
            double stickRight = gamepad1.left_stick_x;
            double stickTurn = -gamepad1.right_stick_x; // CCW positive to match Pinpoint

            if (gamepad1.options) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
                headingTarget = 0;
            }
            if (gamepad1.b && !pB) { TARGET_X = x; TARGET_Y = y; TARGET_H = h; }
            pB = gamepad1.b;

            double fwd, left, turn;

            if (gamepad1.a) {
                // ================= PID DRIVE-TO-POSE =================
                if (!wasAutoDriving) { xPID.reset(); yPID.reset(); headingPID.reset(); }
                wasAutoDriving = true;

                double errX = TARGET_X - x;                 // field frame
                double errY = TARGET_Y - y;
                double errH = wrap(TARGET_H - h);

                double fieldFwd = xPID.calculate(errX);
                double fieldLeft = yPID.calculate(errY);
                // rotate field-frame command into robot frame
                fwd = fieldFwd * Math.cos(h) + fieldLeft * Math.sin(h);
                left = -fieldFwd * Math.sin(h) + fieldLeft * Math.cos(h);
                turn = headingPID.calculate(errH);

                double mag = Math.hypot(fwd, left);
                if (mag > MAX_AUTO_POWER) { fwd *= MAX_AUTO_POWER / mag; left *= MAX_AUTO_POWER / mag; }
                turn = clamp(turn, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                headingTarget = h; // so heading hold picks up from here when A is released
            } else {
                // ================= FIELD-CENTRIC DRIVE + HEADING HOLD =================
                if (wasAutoDriving) { headingPID.reset(); wasAutoDriving = false; }

                double fieldFwd = stickFwd;
                double fieldLeft = -stickRight;
                fwd = fieldFwd * Math.cos(h) + fieldLeft * Math.sin(h);
                left = -fieldFwd * Math.sin(h) + fieldLeft * Math.cos(h);

                if (Math.abs(stickTurn) > STICK_DEADBAND) {
                    turn = stickTurn;
                    headingTarget = h;
                    sinceTurn.reset();
                    headingPID.reset();
                } else if (sinceTurn.seconds() < TURN_RELEASE_SETTLE_S) {
                    turn = 0;
                    headingTarget = h; // keep following while the robot coasts to a stop
                    headingPID.reset();
                } else {
                    turn = headingPID.calculate(wrap(headingTarget - h));
                }
            }

            setDrive(fwd, left, turn);
            handleTuning();

            telemetry.addData("Pose (in, in, deg)", "%.1f, %.1f, %.1f", x, y, Math.toDegrees(h));
            telemetry.addData("Target", "%.1f, %.1f, %.1f", TARGET_X, TARGET_Y, Math.toDegrees(TARGET_H));
            telemetry.addData("Heading target (deg)", "%.1f", Math.toDegrees(headingTarget));
            telemetry.addData("Tuning", tuningHeading ? "HEADING" : "TRANSLATION");
            telemetry.addData("Heading PID", "P=%.4f I=%.4f D=%.4f", headingPID.kP, headingPID.kI, headingPID.kD);
            telemetry.addData("Trans PID", "P=%.4f I=%.4f D=%.4f", xPID.kP, xPID.kI, xPID.kD);
            telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
            telemetry.addData("Pinpoint freq (Hz)", "%.0f", pinpoint.getFrequency());
            telemetry.update();
        }
        setDrive(0, 0, 0);
    }

    // ---------------- Drive helpers ----------------

    /** fwd: +forward, left: +left strafe, turn: +CCW (all robot-frame, -1..1). */
    private void setDrive(double fwd, double left, double turn) {
        double lfP = fwd - left - turn;
        double lbP = fwd + left - turn;
        double rfP = fwd + left + turn;
        double rbP = fwd - left + turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lfP), Math.abs(lbP)),
                Math.max(Math.abs(rfP), Math.abs(rbP))));
        lf.setPower(lfP / max);
        lb.setPower(lbP / max);
        rf.setPower(rfP / max);
        rb.setPower(rbP / max);
    }

    private double pos(boolean isX) {
        Pose2D p = pinpoint.getPosition();
        return isX ? p.getX(DistanceUnit.INCH) : p.getY(DistanceUnit.INCH);
    }

    private double heading() {
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ---------------- Live tuning (gamepad2) ----------------

    private void handleTuning() {
        boolean tune = gamepad1.left_bumper;            // hold LB to tune
        boolean x  = tune && gamepad1.x;
        boolean up = tune && gamepad1.dpad_up,   down  = tune && gamepad1.dpad_down;
        boolean lt = tune && gamepad1.dpad_left, rt    = tune && gamepad1.dpad_right;
        boolean y  = tune && gamepad1.y;

        if (x && !pX) tuningHeading = !tuningHeading;
        if (up && !pUp) applyP(1.1);
        if (down && !pDown) applyP(1 / 1.1);
        if (rt && !pRight) applyD(1.1);
        if (lt && !pLeft) applyD(1 / 1.1);
        if (y && !pY) applyI(gamepad1.right_bumper ? -1 : +1);   // LB+Y = kI up, LB+RB+Y = kI down

        pX = x; pUp = up; pDown = down; pLeft = lt; pRight = rt; pY = y;
    }

    private void applyP(double f) {
        if (tuningHeading) headingPID.kP *= f;
        else { xPID.kP *= f; yPID.kP *= f; }
    }

    private void applyD(double f) {
        if (tuningHeading) headingPID.kD = Math.max(headingPID.kD, 1e-4) * f;
        else {
            xPID.kD = Math.max(xPID.kD, 1e-4) * f;
            yPID.kD = Math.max(yPID.kD, 1e-4) * f;
        }
    }

    private void applyI(int dir) {
        double step = tuningHeading ? 0.02 : 0.001;
        if (tuningHeading) headingPID.kI = Math.max(0, headingPID.kI + dir * step);
        else {
            xPID.kI = Math.max(0, xPID.kI + dir * step);
            yPID.kI = Math.max(0, yPID.kI + dir * step);
        }
    }

    // ---------------- PID controller ----------------

    static class PID {
        double kP, kI, kD;
        double integralLimit = 0.3; // max power contribution from I term
        private double integral, lastError, filteredD;
        private boolean first = true;
        private final ElapsedTime timer = new ElapsedTime();

        PID(double kP, double kI, double kD) { this.kP = kP; this.kI = kI; this.kD = kD; }

        void reset() { integral = 0; filteredD = 0; first = true; }

        double calculate(double error) {
            double dt = timer.seconds();
            timer.reset();
            if (first || dt <= 0 || dt > 0.5) { // first run or stale timer
                first = false;
                lastError = error;
                return kP * error;
            }
            if (kI != 0) {
                integral += error * dt;
                double lim = integralLimit / kI;
                integral = Math.max(-lim, Math.min(lim, integral));
            }
            double rawD = (error - lastError) / dt;
            filteredD = 0.7 * filteredD + 0.3 * rawD; // light low-pass to tame noise
            lastError = error;
            return kP * error + kI * integral + kD * filteredD;
        }
    }
}