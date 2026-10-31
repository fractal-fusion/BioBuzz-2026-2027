package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class TeleopPoseManager {
    public enum Team {
        NONE,
        BLUE,
        RED
    }
    public static PoseFactory poseFactory = PoseFactory.degrees();
    public static Pose currentPose = new Pose(0,0,0);

    //real coordinates for pedro calculation
    public static double RED_UPWARDS_GOAL_X = 57.75;
    public static double RED_UPWARDS_GOAL_Y = 85.75;

    public static Pose RED_UPWARDS_GOAL_POSITION = poseFactory.of(RED_UPWARDS_GOAL_X, RED_UPWARDS_GOAL_Y, 0);
    public static Pose RED_DOWNWARDS_GOAL_POSITION = poseFactory.mirrorY(70.75).of(RED_UPWARDS_GOAL_X, RED_UPWARDS_GOAL_Y, 0);
    public static Pose BLUE_UPWARDS_GOAL_POSITION = poseFactory.mirrorX(70.75).of(RED_UPWARDS_GOAL_X, RED_UPWARDS_GOAL_Y, 0);
    public static Pose BLUE_DOWNWARDS_GOAL_POSITION = poseFactory.mirrorX(70.75).mirrorY(70.75).of(RED_UPWARDS_GOAL_X, RED_UPWARDS_GOAL_Y, 0);



    //define where team-specific poses are stored
    //start on red in case teleop is started without first starting auto
    public static Pose currentUpwardsGoalPose = RED_UPWARDS_GOAL_POSITION;

    public static Pose currentDownwardsGoalPose = RED_DOWNWARDS_GOAL_POSITION;

    //initalize and store the team-specific poses into the correct storage variables, as well as store the current pose
    public static void initializeTeleopPoses(Team team, Pose currentpose){

        if (team == Team.BLUE) {
            currentUpwardsGoalPose = BLUE_UPWARDS_GOAL_POSITION;
            currentDownwardsGoalPose = BLUE_DOWNWARDS_GOAL_POSITION;
        }
        else if (team == Team.RED) {
            currentUpwardsGoalPose = RED_UPWARDS_GOAL_POSITION;
            currentDownwardsGoalPose = RED_DOWNWARDS_GOAL_POSITION;
        }

        currentPose = currentpose;
    }
}
