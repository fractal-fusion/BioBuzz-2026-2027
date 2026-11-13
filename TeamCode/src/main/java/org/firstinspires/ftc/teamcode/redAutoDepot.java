package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.*;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@Autonomous(name = "\uD83D\uDD34 redAutoDepot")
public class redAutoDepot extends LinearOpMode {
    //define poses
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public static double FIRST_FLOWER_Y = 48.6335;
    public static double FIRST_FLOWER_X = 5.5;
    public static double SECOND_FLOWER_Y = 131.5;
    public static double SECOND_FLOWER_X = 42.2;
    public static double SHIMMY_OFFSET_BACKWARD = 1;
    public static double SHIMMY_OFFSET_FORWARD_FIRST_FLOWER = 1.1;
    public static double SHIMMY_OFFSET_FORWARD_SECOND_FLOWER = 0.9;

    public static double DEPOT_TIMEOUT_TIME_MS = 2500;


    public static double SECOND_SHOOT_Y = 131.5;
    public static double SECOND_SHOOT_X = 52.72;

    public static double SHOOT_SECOND_FLOWER_ANGLE = 261;
    public static double FIRST_REV_UP_WAIT_TIME_MS = 1500;
    public static double SECOND_REV_UP_WAIT_TIME_MS = 500;
    public static double THIRD_REV_UP_WAIT_TIME_MS = 500;
    public static double FOURTH_REV_UP_WAIT_TIME_MS = 500;
    public static double TIP_WAIT_TIME_MS = 1000;
    public static double SHOOTING_WAIT_TIME_MS = 1250;
    public static double SHOOTING_WAIT_TIME_MS_2 = 1250;

    public static double FIRST_FLOWER_WAIT_TIME_MS = 1500;
    public static double SECOND_FLOWER_WAIT_TIME_MS = 1500;
    public static double FIRST_FLOWER_DROP_RAMP_WAIT_TIME_MS = 300;

    private final Pose start = poseFactory.of(55, 11.26, 90);
    private final Pose intakeDepot = poseFactory.of(0, 9.0866, 90);
    private final Pose intakeDepotControl1 = poseFactory.of(0, 28.3869, 0);
    private final Pose goToShootDepot = poseFactory.of(SECOND_SHOOT_X, SECOND_SHOOT_Y, SHOOT_SECOND_FLOWER_ANGLE);
    private final Pose goToShootDepotControl1 = poseFactory.of(27.683, 21.1185, 0);
    private final Pose goToShootDepotControl2 = poseFactory.of(0, 139.11, 0);
    private final Pose goToShootDepotControl3 = poseFactory.of(56.9043, 116.0123, 0);
    private final Pose goToShootDepotSegment1Start = poseFactory.of(59.98, 133.67, 0);
    private final Pose goToShootDepotSegment1End = poseFactory.of(59.98, 133.67, SHOOT_SECOND_FLOWER_ANGLE);
    private final Pose goToIntakeFirstFlowerStart = poseFactory.of(56, 11.26, 90);
    private final Pose goToIntakeFirstFlowerControl1 = poseFactory.of(7.36, 97.56, 0);
    private final Pose goToIntakeFirstFlowerSegment1Start = poseFactory.of(15.7542, 48.6489, SHOOT_SECOND_FLOWER_ANGLE);
    private final Pose goToIntakeFirstFlowerSegment1End = poseFactory.of(15.7542, 48.6489, 0);
    private final Pose goToIntakeFirstFlower = poseFactory.of(17.2658, FIRST_FLOWER_Y, 0);
    private final Pose intakeFirstFlower = poseFactory.of(FIRST_FLOWER_X, FIRST_FLOWER_Y, 0);
    private final Pose shimmeyBackFirstFlower = poseFactory.of(FIRST_FLOWER_X + SHIMMY_OFFSET_BACKWARD, FIRST_FLOWER_Y, 0);
    private final Pose shimmeyForwardFirstFlower = poseFactory.of(FIRST_FLOWER_X - SHIMMY_OFFSET_FORWARD_FIRST_FLOWER, FIRST_FLOWER_Y, 0);
    private final Pose backUpFromFirstFlower = poseFactory.of(17.5657, 46.493, 0);
    private final Pose goToShootFirstFlowerStart = poseFactory.of(17.5657, 46.493, 0);

    private final Pose goToShootFirstFlower = poseFactory.of(SECOND_SHOOT_X, SECOND_SHOOT_Y, SHOOT_SECOND_FLOWER_ANGLE);
    private final Pose goToShootFirstFlowerControl1 = poseFactory.of(28.4159, 53.9977, 0);
    private final Pose goToShootFirstFlowerControl2 = poseFactory.of(0, 90.47, 0);
    //    private final Pose backUpToShootFirstFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y, SECOND_FLOWER_ANGLE);
    private final Pose intakeSecondFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y, 270);
    private final Pose shimmeyBackSecondFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y - SHIMMY_OFFSET_BACKWARD, 270);
    private final Pose shimmeyForwardSecondFlower = poseFactory.of(SECOND_FLOWER_X, SECOND_FLOWER_Y + SHIMMY_OFFSET_FORWARD_SECOND_FLOWER, 270);
    private final Pose intakeSecondFlowerControl1 = poseFactory.of(48.2351, 111.8873, 0);
    private final Pose goToShootSecondFlowerStart = poseFactory.of(49.1517, 130.1241, 270);
    private final Pose goToShootSecondFlower = poseFactory.of(SECOND_SHOOT_X, SECOND_SHOOT_Y, SHOOT_SECOND_FLOWER_ANGLE);
    private final Pose goToShootSecondFlowerControl1 = poseFactory.of(54.0167, 119.8707, 0);
    private final Pose park = poseFactory.of(10.232, 118.4468, 177.7608);
    private final Pose parkControl1 = poseFactory.of(52.0519, 114.9289, 0);

    //initialize paths from poses

    public Path goToIntakeDepot() {
        return Paths.curve(start, intakeDepotControl1, intakeDepot).constant(intakeDepot);
    }

    public Path goToShootDepot() {
        return Paths.curve(intakeDepot, goToShootDepotControl1, goToShootDepotControl2, goToShootDepotControl3, goToShootDepot).heading(Interpolator.piecewise().until(1, Interpolator.linear(goToShootDepotSegment1Start, goToShootDepotSegment1End)));
    }

    public Path goToIntakeFirstFlower() {
        return Paths.curve(goToShootDepot, goToIntakeFirstFlowerControl1, goToIntakeFirstFlower).heading(Interpolator.piecewise().until(1, Interpolator.linear(goToIntakeFirstFlowerSegment1Start, goToIntakeFirstFlowerSegment1End)));
    }

    public Path intakeFirstFlower() {
        return Paths.line(goToIntakeFirstFlower, intakeFirstFlower).reverseTangent();
    }

    public Path shimmeyBackFirstFlower() {
        return Paths.line(shimmeyForwardFirstFlower, shimmeyBackFirstFlower).constant(shimmeyBackFirstFlower).with(
                Constants.foresightConfig.maxPathSpeed.at(0.9)
        );
    }

    public Path shimmeyForwardFirstFlower() {
        return Paths.line(shimmeyBackFirstFlower, shimmeyForwardFirstFlower).constant(shimmeyForwardFirstFlower);
    }

    public Path pushForwardFirstFlower() {
        return Paths.line(intakeFirstFlower, shimmeyForwardFirstFlower).constant(shimmeyForwardFirstFlower);
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
        return Paths.line(shimmeyForwardSecondFlower, shimmeyBackSecondFlower).constant(shimmeyBackSecondFlower).with(
                Constants.foresightConfig.maxPathSpeed.at(0.9)
        );
    }

    public Path shimmeyForwardSecondFlower() {
        return Paths.line(shimmeyBackSecondFlower, shimmeyForwardSecondFlower).constant(shimmeyForwardSecondFlower);
    }

    public Path pushForwardSecondFlower() {
        return Paths.line(intakeSecondFlower, shimmeyForwardSecondFlower).constant(shimmeyForwardSecondFlower);
    }

    public Path goToShootSecondFlower() {
        return Paths.curve(goToShootSecondFlowerStart, goToShootSecondFlowerControl1, goToShootSecondFlower).linear(goToShootSecondFlowerStart, goToShootSecondFlower)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.9));
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
                        TeleopPoseManager.initializeTeleop(TeleopPoseManager.Team.RED, follower.pose()),
                        sequential(
                                //shoot preloads
                                shooter.setRPM(1550), //TODO: change arc
                                shooter.setHood(0.265),
                                shooter.openGate(),
                                waitMs(FIRST_REV_UP_WAIT_TIME_MS),
                                intake.turnOnIntakeSlow(),
                                waitMs(SHOOTING_WAIT_TIME_MS),
                                shooter.closeGate(),

                                //go to intake depot and turn on intake in the middle of the path
                                parallel(
                                        race(follow(follower, goToIntakeDepot()),
                                                waitMs(DEPOT_TIMEOUT_TIME_MS)
                                        ),
                                        sequential(
                                                waitMs(DEPOT_TIMEOUT_TIME_MS/3),
                                                intake.turnOnIntake())

                                ),

                                //go to shoot depot
                                parallel(
                                        follow(follower, goToShootDepot()),
                                        shooter.setRPM(1570),
                                        shooter.setHood(0.265)
                                ),
                                intake.turnOffIntake(),
                                //shoot depot
                                shooter.openGate(),
                                waitMs(TIP_WAIT_TIME_MS),
                                waitMs(SECOND_REV_UP_WAIT_TIME_MS),
                                intake.turnOnIntakeSlow(),
                                waitMs(SHOOTING_WAIT_TIME_MS_2),
                                shooter.closeGate(),

                                //go to intake first flower
                                parallel(follow(follower, goToIntakeFirstFlower()),
                                        sequential(
//                                waitMs(2000),
                                                intake.putDownRampHalfway())
                                ),
                                intake.putDownRamp(),
                                waitMs(FIRST_FLOWER_DROP_RAMP_WAIT_TIME_MS),
                                follow(follower, intakeFirstFlower()),
                                //turn on intake and wait for balls to be intaked
                                intake.turnOnIntake(),
                                race(
                                        follow(follower, pushForwardFirstFlower()),
                                        waitMs(FIRST_FLOWER_WAIT_TIME_MS)),
                                //shimmey
                                follow(follower,shimmeyBackFirstFlower()),
                                race(
                                    follow(follower, shimmeyForwardFirstFlower()),
                                    waitMs(250)),


                                //go to shoot first flower
                                follow(follower, backUpFromFirstFlower()),
                                follow(follower, goToShootFirstFlower()),
                                intake.turnOffIntake(),
                                //            follow(follower, backUpToShootFirstFlower()),

                                //shoot first flower
                                shooter.setRPM(1570),
                                shooter.setHood(0.265),
                                shooter.openGate(),
                                waitMs(THIRD_REV_UP_WAIT_TIME_MS),
                                intake.turnOnIntakeSlow(),
                                waitMs(SHOOTING_WAIT_TIME_MS_2),
                                shooter.closeGate(),

                                //go to intake second flower
                                follow(follower, intakeSecondFlower()),
                                //turn on intake and wait to intake balls
                                intake.turnOnIntake(),
                                //shimmey
//                                follow(follower, shimmeyBackSecondFlower()),
                                race(
                                        follow(follower, pushForwardSecondFlower()),
                                        waitMs(SECOND_FLOWER_WAIT_TIME_MS)),
                                follow(follower,shimmeyBackSecondFlower()),
                                race(
                                        follow(follower, shimmeyForwardSecondFlower()),
                                        waitMs(250)),



                                //go to shoot second flower
                                follow(follower, goToShootSecondFlower()),
                                intake.turnOffIntake(),

                                //shoot second flower
                                shooter.setRPM(1570),
                                shooter.setHood(0.265),
                                shooter.openGate(),
                                waitMs(FOURTH_REV_UP_WAIT_TIME_MS),
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
