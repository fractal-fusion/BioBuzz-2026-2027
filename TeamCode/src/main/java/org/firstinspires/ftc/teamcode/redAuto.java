package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@Autonomous(name = "\uD83D\uDD34 redAuto")
public class redAuto extends LinearOpMode {
    //define poses
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public static double FIRST_FLOWER_Y = 47.6335;
    public static double FIRST_FLOWER_X = 5.9;

    public static double FIRST_REV_UP_WAIT_TIME_MS = 1850;
    public static double REV_UP_WAIT_TIME_MS = 500;
    public static double SHOOTING_WAIT_TIME_MS = 500;

    private final Pose start = poseFactory.of(56, 11.26, 90);
    private final Pose goToIntakeFirstFlowerStart = poseFactory.of(56, 11.26, 90);
    private final Pose goToIntakeFirstFlower = poseFactory.of(17.2658, FIRST_FLOWER_Y, 0);
    private final Pose goToIntakeFirstFlowerControl1 = poseFactory.of(12.8992, 29.3762, 0);
    private final Pose intakeFirstFlower = poseFactory.of(FIRST_FLOWER_X, FIRST_FLOWER_Y, 0);
    private final Pose backUpFromFirstFlower = poseFactory.of(17.5657, 46.493, 0);
    private final Pose goToShootFirstFlowerStart = poseFactory.of(17.5657, 46.493, 0);

    private final Pose goToShootFirstFlower = poseFactory.of(52.2425, 124.9408, 270);
    private final Pose goToShootFirstFlowerControl1 = poseFactory.of(28.4159, 53.9977, 0);
    private final Pose goToShootFirstFlowerControl2 = poseFactory.of(12.8901, 95.9933, 0);
    private final Pose intakeSecondFlower = poseFactory.of(49.1517, 130.1241, 270);
    private final Pose intakeSecondFlowerControl1 = poseFactory.of(46.8084, 126.3793, 0);
    private final Pose goToShootSecondFlowerStart = poseFactory.of(49.1517, 130.1241, 270);
    private final Pose goToShootSecondFlower = poseFactory.of(50.2807, 126.966, 280);
    private final Pose park = poseFactory.of(13.232, 116.4468, 177.7608);
    private final Pose parkControl1 = poseFactory.of(52.0519, 114.9289, 0);

    //initialize paths from poses

    public Path goToIntakeFirstFlower() {
        return Paths.curve(goToIntakeFirstFlowerStart, goToIntakeFirstFlowerControl1, goToIntakeFirstFlower).linear(goToIntakeFirstFlowerStart, goToIntakeFirstFlower);
    }

    public Path intakeFirstFlower() {
        return Paths.line(goToIntakeFirstFlower, intakeFirstFlower).reverseTangent();
    }

    public Path backUpFromFirstFlower() {
        return Paths.line(intakeFirstFlower, backUpFromFirstFlower).tangent();
    }

    public Path goToShootFirstFlower() {
        return Paths.curve(goToShootFirstFlowerStart, goToShootFirstFlowerControl1, goToShootFirstFlowerControl2, goToShootFirstFlower).linear(goToShootFirstFlowerStart, goToShootFirstFlower);
    }

    public Path intakeSecondFlower() {
        return Paths.curve(goToShootFirstFlower, intakeSecondFlowerControl1, intakeSecondFlower).constant(intakeSecondFlower);
    }

    public Path goToShootSecondFlower() {
        return Paths.line(goToShootSecondFlowerStart, goToShootSecondFlower).linear(goToShootSecondFlowerStart, goToShootSecondFlower);
    }

    public Path park() {
        return Paths.curve(goToShootSecondFlower, parkControl1, park).tangent();
    }

    @Override
    public void runOpMode() {
        //Since the scheduler is static, we need to reset it before each OpMode
        //so commands don't carry over from one OpMode to the next
        Scheduler.reset();

        // Initialize hardware
        Shooter shooter = new Shooter(this);
        Intake intake = new Intake(this);

        //initialize follower
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);


        Command autoSequence = sequential(
            //shoot preloads
            shooter.setRPM(1581),
            shooter.setHood(0.2955),
            shooter.openGate(),
            waitMs(FIRST_REV_UP_WAIT_TIME_MS),
            intake.turnOnIntakeSlow(),
            waitMs(SHOOTING_WAIT_TIME_MS),
            shooter.closeGate(),

            //go to intake first flower
            parallel(follow(follower, goToIntakeFirstFlower()),
                        intake.putDownRamp()),
            waitMs(500),
            follow(follower, intakeFirstFlower()),

            //turn on intake and wait for balls to be intaked
            intake.turnOnIntake(),
            waitMs(1000),

            //go to shoot first flower
            follow(follower, backUpFromFirstFlower()),
            follow(follower, goToShootFirstFlower()),

            //shoot first flower
            shooter.setRPM(1581),
            shooter.setHood(0.26),
            shooter.openGate(),
            waitMs(REV_UP_WAIT_TIME_MS),
            intake.turnOnIntakeSlow(),
            waitMs(SHOOTING_WAIT_TIME_MS),
            shooter.closeGate()

        );
        waitForStart();

        // Schedule the sequence when the OpMode starts
        schedule(autoSequence);
        while (opModeIsActive()) {
            //update the follower
            follower.update();

            // Run the scheduler each loop
            Scheduler.execute();
        }
    }
}
