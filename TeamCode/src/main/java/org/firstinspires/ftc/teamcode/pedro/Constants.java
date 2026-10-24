package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(-4.276854747862328);
        c.yPodOffset.set(-1.9429089703897793);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.26857470705511255);
                Controller secondaryTranslationalForward = Controller.proportional(0.09923117780852103);
                Controller primaryTranslationalLateral = Controller.proportional(0.2861255921663608);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10571576088883997);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01812840093076196));
                c.brake.set(Controller.proportionalFeedforward(0.015409140791147665));

                c.headingFeedback.set(Controller.proportional(2.787759261659463));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.042533302988365974, 0.013325286663727787));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1046600805216333, 0.1308024102114489));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0023170873672470073, 0.0017793971760140677));

                c.maxAchievableForwardVelocity.set(57.945608297765375);
                c.maxAchievableStrafeVelocity.set(48.57701625792447);
                c.naturalForwardDeceleration.set(29.802802528134617);
                c.naturalStrafeDeceleration.set(34.209718203991216);
            }
    );
}