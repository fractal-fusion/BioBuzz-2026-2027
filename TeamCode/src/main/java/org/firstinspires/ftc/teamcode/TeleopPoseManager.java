package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class TeleopPoseManager {
    public enum Team {
        NONE,
        BLUE,
        RED
    }
    private static final PoseFactory poseFactory = PoseFactory.degrees();
    public static Pose currentPose = new Pose(0,0,0);

    //real coordinates for pedro calculation
    public static double RED_UPWARDS_GOAL_X = 57.75;
    public static double RED_UPWARDS_GOAL_Y = 85.75;

    public static Pose RED_UPWARDS_GOAL_POSITION = poseFactory.of(RED_UPWARDS_GOAL_X, RED_UPWARDS_GOAL_Y, 0);
    public static Pose RED_DOWNWARDS_GOAL_POSITION = new Pose(57.8, 144);


    //define where team-specific poses are stored
    public static Pose currentUpwardsGoalPose = GOAL_POSITION; //start on red in case teleop is started without first starting auto
    public static Pose currentDownwardsGoalPose =
    //initalize and store the team-specific poses into the correct storage variables, as well as store the current pose
    public static void initializeTeleopPoses(Team team, Pose currentpose){
        if (team == Team.BLUE) {
            currentGoalPose = GOAL_POSITION;
        }
        else if (team == Team.RED) {
            currentGoalPose = GOAL_POSITION;
        }

        currentPose = currentpose;
    }
}
