package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.field.FieldManager;

public class Drawing {

    public static final double ROBOT_RADIUS = 9;

    private static final FieldManager panelsField =
            PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );

    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * Prepares Panels Field for Pedro Pathing offsets.
     */
    public static void init() {
        panelsField.setOffsets(
                PanelsField.INSTANCE.getPresets().getPEDRO_PATHING()
        );
    }

    /**
     * Draws the current path and robot pose.
     *
     * @param follower Pedro 3 Follower instance.
     */
    public static void drawDebug(Follower follower) {
        Path currentPath = follower.currentPath();

        if (currentPath != null) {
            drawPath(currentPath, robotLook);

            Pose closestPoint = follower.closestPose();

            if (closestPoint != null) {
                drawRobot(closestPoint, robotLook);
            }
        }

        drawRobot(follower.pose(), historyLook);

        sendPacket();
    }

    /**
     * Draws a robot at the specified pose.
     * The heading is represented by a line.
     *
     * @param pose  The pose to draw.
     * @param style The drawing style.
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null
                || Double.isNaN(pose.x())
                || Double.isNaN(pose.y())
                || Double.isNaN(pose.heading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.x(), pose.y());
        panelsField.circle(ROBOT_RADIUS);

        double heading = pose.heading();

        double x1 = pose.x()
                + Math.cos(heading) * ROBOT_RADIUS / 2.0;

        double y1 = pose.y()
                + Math.sin(heading) * ROBOT_RADIUS / 2.0;

        double x2 = pose.x()
                + Math.cos(heading) * ROBOT_RADIUS;

        double y2 = pose.y()
                + Math.sin(heading) * ROBOT_RADIUS;

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * Draws a robot using the default robot style.
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * Draws a Path with a specified look.
     *
     * Samples points along the path's curve and draws line segments
     * between consecutive points.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        if (path == null) {
            return;
        }

        // Number of line segments used to approximate the curve.
        final int SAMPLES = 100;

        panelsField.setStyle(style);

        Pose start = path.get(0.0);

        if (Double.isNaN(start.x()) || Double.isNaN(start.y())) {
            return;
        }

        panelsField.moveCursor(start.x(), start.y());

        for (int i = 1; i <= SAMPLES; i++) {
            double t = (double) i / SAMPLES;

            Pose point = path.get(t);

            if (Double.isNaN(point.x()) || Double.isNaN(point.y())) {
                continue;
            }

            panelsField.line(point.x(), point.y());
        }
    }

    /**
     * Draws the current packet to Panels Field.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}