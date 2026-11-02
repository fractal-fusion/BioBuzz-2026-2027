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
        c.frontLeftName.set("frontleft");
        c.frontRightName.set("frontright");
        c.backLeftName.set("backleft");
        c.backRightName.set("backright");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.824024831216167);
        c.yPodOffset.set(0.3666650967335138);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.37566643732548494);
                Controller secondaryTranslationalForward = Controller.proportional(0.138798710599689);
                Controller primaryTranslationalLateral = Controller.proportional(0.5618103344689344);
                Controller secondaryTranslationalLateral = Controller.proportional(0.20757390673765708);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010016997454367038));
                c.brake.set(Controller.proportionalFeedforward(0.008514447836211983));

                c.headingFeedback.set(Controller.proportional(7.637445589138914));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.08589276362158528, 0.005614021788791535));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1947668909184146, 0.09921467902372769));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010717926444742593, 0.0018971943617741508));

                c.maxAchievableForwardVelocity.set(79.52047331629802);
                c.maxAchievableStrafeVelocity.set(68.23306598994971);
                c.naturalForwardDeceleration.set(24.062561866437544);
                c.naturalStrafeDeceleration.set(60.63865872761951);
            }
    );

}