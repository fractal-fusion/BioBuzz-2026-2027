package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@Autonomous(name = "\uD83D\uDD35 blueAuto")
public class blueAuto extends LinearOpMode {
    //define poses
    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
    public static double FIRST_FLOWER_Y = 48.6335;
    public static double FIRST_FLOWER_X = 6.4 - 0.3;
    public static double SECOND_FLOWER_Y = 131.5;
    public static double SECOND_FLOWER_X = 42.2;
    public static double SHIMMY_OFFSET = 1;

    public static double SECOND_SHOOT_Y = 131.5;
    public static double SECOND_SHOOT_X = 55.72;

    public static double SHOOT_SECOND_FLOWER_ANGLE = 260;

    public static double FIRST_REV_UP_WAIT_TIME_MS = 3000;
    public static double SECOND_REV_UP_WAIT_TIME_MS = 1500;
    public static double THIRD_REV_UP_WAIT_TIME_MS = 3000;
    public static double TIP_WAIT_TIME_MS = 2500;
    public static double SHOOTING_WAIT_TIME_MS = 750;
    public static double SHOOTING_WAIT_TIME_MS_2 = 750;

    public static double FIRST_FLOWER_WAIT_TIME_MS = 1000;
    public static double SECOND_FLOWER_WAIT_TIME_MS = 1500;

    private final Pose start = poseFactory.of(56, 11.26, 90);
    private final Pose goToIntakeFirstFlowerStart = poseFactory.of(56, 11.26, 90);
    private final Pose goToIntakeFirstFlower = poseFactory.of(17.2658, FIRST_FLOWER_Y, 0);
    private final Pose goToIntakeFirstFlowerControl1 = poseFactory.of(12.8992, 29.3762, 0);
    private final Pose intakeFirstFlower = poseFactory.of(FIRST_FLOWER_X, FIRST_FLOWER_Y, 0);
    private final Pose shimmeyBackFirstFlower = poseFactory.of(FIRST_FLOWER_X + SHIMMY_OFFSET, FIRST_FLOWER_Y, 0);
    private final Pose shimmeyForwardFirstFlower = poseFactory.of(FIRST_FLOWER_X, FIRST_FLOWER_Y, 0);
    private final Pose backUpFromFirstFlower = poseFactory.of(17.5657, 46.493, 0);
    private final Pose goToShootFirstFlowerStart = poseFactory.of(17.5657, 46.493, 0);

    private final Pose goToShootFirstFlower = poseFactory.of(SECOND_SHOOT_X, SECOND_SHOOT_Y, SHOOT_SECOND_FLOWER_ANGLE);
    private final Pose goToShootFirstFlowerControl1 = poseFactory.of(28.4159, 53.9977, 0);
    private final Pose goToShootFirstFlowerControl2 = poseFactory.of(10.4061, 95.461, 0);
//    private final Pose backUpToShootFirstFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y, SECOND_FLOWER_ANGLE);
    private final Pose intakeSecondFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y, 270);
    private final Pose shimmyBackSecondFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y - SHIMMY_OFFSET, 270);
    private final Pose shimmyForwardSecondFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y, 270);
    private final Pose intakeSecondFlowerControl1 = poseFactory.of(48.2351, 111.8873, 0);
    private final Pose goToShootSecondFlowerStart = poseFactory.of(49.1517, 130.1241, 270);
    private final Pose goToShootSecondFlower = poseFactory.of(SECOND_SHOOT_X, SECOND_SHOOT_Y, SHOOT_SECOND_FLOWER_ANGLE);
    private final Pose goToShootSecondFlowerControl1 = poseFactory.of(54.0167, 119.8707, 0);
    private final Pose park = poseFactory.of(12.232, 118.4468, 177.7608);
    private final Pose parkControl1 = poseFactory.of(52.0519, 114.9289, 0);

    //initialize paths from poses

    public Path goToIntakeFirstFlower() {
        return Paths.curve(goToIntakeFirstFlowerStart, goToIntakeFirstFlowerControl1, goToIntakeFirstFlower).linear(goToIntakeFirstFlowerStart, goToIntakeFirstFlower);
    }

    public Path intakeFirstFlower() {
        return Paths.line(goToIntakeFirstFlower, intakeFirstFlower).reverseTangent();
    }

    public Path shimmeyBackFirstFlower() {
        return Paths.line(intakeFirstFlower, shimmeyBackFirstFlower).constant(shimmeyBackFirstFlower);
    }

    public Path shimmeyForwardFirstFlower() {
        return Paths.line(shimmeyBackFirstFlower, shimmeyForwardFirstFlower).constant(shimmeyForwardFirstFlower);
    }

    public Path backUpFromFirstFlower() {
        return Paths.line(intakeFirstFlower, backUpFromFirstFlower).tangent();
    }

    public Path goToShootFirstFlower() {
        return Paths.curve(goToShootFirstFlowerStart, goToShootFirstFlowerControl1, goToShootFirstFlowerControl2, goToShootFirstFlower).linear(goToShootFirstFlowerStart, goToShootFirstFlower);
    }

//    public Path backUpToShootFirstFlower() {
//        return Paths.line(goToShootFirstFlower, backUpToShootFirstFlower).constant(backUpToShootFirstFlower);
//    }
    public Path intakeSecondFlower() {
        return Paths.curve(goToShootFirstFlower, intakeSecondFlowerControl1, intakeSecondFlower).constant(intakeSecondFlower);
    }
    public Path shimmeyBackSecondFlower() {
        return Paths.line(intakeSecondFlower, shimmyBackSecondFlower).constant(shimmyBackSecondFlower);
    }

    public Path shimmeyForwardSecondFlower() {
        return Paths.line(shimmyBackSecondFlower, shimmyForwardSecondFlower).constant(shimmyForwardSecondFlower);
    }

    public Path goToShootSecondFlower() {
        return Paths.curve(goToShootSecondFlowerStart, goToShootSecondFlowerControl1, goToShootSecondFlower).linear(goToShootSecondFlowerStart, goToShootSecondFlower);
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


        Command autoSequence =
            parallel(
                TeleopPoseManager.initializeTeleop(TeleopPoseManager.Team.BLUE, follower.pose()),
                sequential(
                //shoot preloads
                shooter.setRPM(1618),
                shooter.setHood(0.285),
                shooter.openGate(),
                waitMs(FIRST_REV_UP_WAIT_TIME_MS),
                intake.turnOnIntakeSlow(),
                waitMs(SHOOTING_WAIT_TIME_MS),
                shooter.closeGate(),

                //go to intake first flower
                parallel(follow(follower, goToIntakeFirstFlower()),
                        sequential(
//                                waitMs(2000),
                                intake.putDownRampHalfway())
                ),
                intake.putDownRamp(),
                waitMs(1000),
                follow(follower, intakeFirstFlower()),
                //turn on intake and wait for balls to be intaked
                intake.turnOnIntake(),
                waitMs(FIRST_FLOWER_WAIT_TIME_MS),
                //shimmey
                follow(follower, shimmeyBackFirstFlower()),
                follow(follower, shimmeyForwardFirstFlower()),


                //go to shoot first flower
                follow(follower, backUpFromFirstFlower()),
                follow(follower, goToShootFirstFlower()),
                intake.turnOffIntake(),
    //            follow(follower, backUpToShootFirstFlower()),
                shooter.openGate(),

                //shoot first flower
                shooter.setRPM(1603),
                shooter.setHood(0.27),
                waitMs(TIP_WAIT_TIME_MS),
                waitMs(SECOND_REV_UP_WAIT_TIME_MS),
                intake.turnOnIntakeSlow(),
                waitMs(SHOOTING_WAIT_TIME_MS_2),
                shooter.closeGate(),

                //go to intake second flower
                follow(follower, intakeSecondFlower()),
                //turn on intake and wait to intake balls
                intake.turnOnIntake(),
                waitMs(SECOND_FLOWER_WAIT_TIME_MS),
                //shimmey
                follow(follower, shimmeyBackSecondFlower()),
                follow(follower, shimmeyForwardSecondFlower()),


                //go to shoot second flower
                follow(follower, goToShootSecondFlower()),

                //shoot second flower
                shooter.setRPM(1603),
                shooter.setHood(0.27),
                shooter.openGate(),
                waitMs(THIRD_REV_UP_WAIT_TIME_MS),
                intake.turnOnIntakeSlow(),
                waitMs(SHOOTING_WAIT_TIME_MS),
                shooter.closeGate(),

                //goes to park
                follow(follower, park())
            )
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
