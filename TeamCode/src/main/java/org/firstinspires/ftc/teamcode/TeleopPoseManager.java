package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class TeleopPoseManager {
    //initialize posefactory
    public static PoseFactory poseFactory = PoseFactory.degrees();

    //relocalization pose constants
    public static double BOT_WIDTH = 16.5;
    public static double BOT_HEIGHT = 16.75;
    public static double FIELD_WIDTH = 141.5;
    public static double FIELD_HEIGHT = 141.5;

    //naming scheme is from the perspective of the driver

    //robot width and height is dependent on the rotation, so need to flip when calculating field-relative coordinates
    //when the robot is facing 90 degrees to the driver but 0 degrees to the field

    public static Pose RED_BOTTOM_LEFT_POSE = poseFactory.of(BOT_HEIGHT/2, FIELD_HEIGHT - BOT_WIDTH/2, 0);
    public static Pose RED_BOTTOM_RIGHT_POSE = poseFactory.of(BOT_HEIGHT/2, BOT_WIDTH/2, 0);
    public static Pose RED_TOP_LEFT_POSE = poseFactory.of(FIELD_WIDTH - BOT_HEIGHT/2, FIELD_HEIGHT - BOT_WIDTH/2, 0);
    public static Pose RED_TOP_RIGHT_POSE = poseFactory.of(FIELD_WIDTH - BOT_HEIGHT/2, BOT_WIDTH/2, 0);

    //TODO: find these
    public static Pose BLUE_BOTTOM_LEFT_POSE = poseFactory.of(BOT_HEIGHT/2, FIELD_HEIGHT - BOT_WIDTH/2, 180);
    public static Pose BLUE_BOTTOM_RIGHT_POSE = poseFactory.of(BOT_HEIGHT/2, BOT_WIDTH/2, 180);
    public static Pose BLUE_TOP_LEFT_POSE = poseFactory.of(FIELD_WIDTH - BOT_HEIGHT/2, FIELD_HEIGHT - BOT_WIDTH/2, 180);
    public static Pose BLUE__TOP_RIGHT_POSE = poseFactory.of(FIELD_WIDTH - BOT_HEIGHT/2, BOT_WIDTH/2, 180);


    public enum Team {
        NONE,
        BLUE,
        RED
    }
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
    public static Pose matchUpwardsGoalPose = RED_UPWARDS_GOAL_POSITION;

    public static Pose matchDownwardsGoalPose = RED_DOWNWARDS_GOAL_POSITION;

    //initalize and store the team-specific poses into the correct storage variables, as well as store the current pose
    public static void initializeTeleopPoses(Team team, Pose currentpose){

        if (team == Team.BLUE) {
            matchUpwardsGoalPose = BLUE_UPWARDS_GOAL_POSITION;
            matchDownwardsGoalPose = BLUE_DOWNWARDS_GOAL_POSITION;
        }
        else if (team == Team.RED) {
            matchUpwardsGoalPose = RED_UPWARDS_GOAL_POSITION;
            matchDownwardsGoalPose = RED_DOWNWARDS_GOAL_POSITION;
        }

        currentPose = currentpose;
    }
}
