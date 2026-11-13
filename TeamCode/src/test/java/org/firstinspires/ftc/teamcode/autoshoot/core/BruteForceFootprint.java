package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.List;
import java.util.Locale;

/**
 * A deliberately simple, slow "does the robot's rectangle hit anything?" checker for the tests. It shares NO code with
 * {@link RouteSweep}: it walks the route in tiny steps (a few hundredths of an inch and a fraction of a degree), builds
 * the rotated rectangle's four corners with plain trigonometry, and asks three plain questions about each step:
 * "is a corner of the robot inside the box?", "is a corner of the box inside the robot?", and "do an edge of the robot
 * and an edge of the box cross?". If any answer is yes, the two shapes overlap. For the walls it only asks "is a corner
 * outside the field?".
 *
 * <p>Why it is separate: the production check is clever (it proves a whole piece of route safe from two samples). A
 * test that used the same cleverness could share its mistakes. This one is obviously right and only too slow to run on
 * the robot, so the tests use it as the referee.
 *
 * <p>The robot's heading along a route still comes from {@link HeadingProfile}, because that class is the one rule
 * the real drive follows (and has its own test). Everything else here is independent.
 *
 * <p>Words used: <b>tol</b> (tolerance) is how many inches of overlap are forgiven, to absorb rounding; the route
 * checker forgives a few hundredths of an inch the same way.
 */
final class BruteForceFootprint {
    private BruteForceFootprint() {
    }

    /**
     * The four corners (x, y) of the robot rectangle at a pose, counter-clockwise from the front-left corner. The
     * rectangle is NOT centered on the pose (the robot's center of rotation): it reaches {@code front} ahead, {@code rear}
     * behind, {@code left} to the left and {@code right} to the right of it.
     */
    static double[][] corners(Pose2 pose, RobotShape robot) {
        double front = robot.frontIn();
        double rear = robot.rearIn();
        double left = robot.leftIn();
        double right = robot.rightIn();
        double[][] local = {{front, left}, {-rear, left}, {-rear, -right}, {front, -right}};
        double c = Math.cos(pose.heading);
        double s = Math.sin(pose.heading);
        double[][] out = new double[4][2];
        for (int i = 0; i < 4; i++) {
            out[i][0] = pose.x + local[i][0] * c - local[i][1] * s;
            out[i][1] = pose.y + local[i][0] * s + local[i][1] * c;
        }
        return out;
    }

    /** How far the robot's corners stick out of the field box at a pose, inches (0 if it is inside). */
    static double wallPoke(Pose2 pose, RobotShape robot, Rect field) {
        double poke = 0.0;
        for (double[] p : corners(pose, robot)) {
            poke = Math.max(poke, field.minX - p[0]);
            poke = Math.max(poke, p[0] - field.maxX);
            poke = Math.max(poke, field.minY - p[1]);
            poke = Math.max(poke, p[1] - field.maxY);
        }
        return poke;
    }

    /**
     * True if the robot at a pose overlaps the box by more than the tolerance. The box is shrunk by the tolerance on
     * each side first (but never by more than a quarter of its size, so a very thin box is not shrunk away).
     */
    static boolean overlaps(Pose2 pose, RobotShape shape, Rect box, double tol) {
        // Quick answer for the common case: the box is farther from the robot's center than the robot's farthest corner.
        double dx = Math.max(Math.max(box.minX - pose.x, pose.x - box.maxX), 0.0);
        double dy = Math.max(Math.max(box.minY - pose.y, pose.y - box.maxY), 0.0);
        if (Math.hypot(dx, dy) > Math.hypot(Math.max(shape.frontIn(), shape.rearIn()), Math.max(shape.leftIn(), shape.rightIn())) + 1e-9) {
            return false;
        }
        double tx = Math.min(tol, 0.25 * (box.maxX - box.minX));
        double ty = Math.min(tol, 0.25 * (box.maxY - box.minY));
        double x0 = box.minX + tx;
        double x1 = box.maxX - tx;
        double y0 = box.minY + ty;
        double y1 = box.maxY - ty;
        double[][] robot = corners(pose, shape);
        double[][] rect = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
        // 1. A corner of the robot strictly inside the box.
        for (double[] p : robot) {
            if (p[0] > x0 && p[0] < x1 && p[1] > y0 && p[1] < y1) {
                return true;
            }
        }
        // 2. A corner of the box, or the box's middle, strictly inside the robot.
        for (double[] p : rect) {
            if (insideConvex(robot, p[0], p[1])) {
                return true;
            }
        }
        if (insideConvex(robot, 0.5 * (x0 + x1), 0.5 * (y0 + y1))) {
            return true;
        }
        // 3. An edge of the robot crossing an edge of the box, or the robot's middle inside the box.
        if (pose.x > x0 && pose.x < x1 && pose.y > y0 && pose.y < y1) {
            return true;
        }
        for (int i = 0; i < 4; i++) {
            double[] a = robot[i];
            double[] b = robot[(i + 1) % 4];
            for (int j = 0; j < 4; j++) {
                if (cross(a, b, rect[j], rect[(j + 1) % 4])) {
                    return true;
                }
            }
        }
        return false;
    }

    /** True if the point is strictly inside the convex counter-clockwise polygon. */
    private static boolean insideConvex(double[][] poly, double x, double y) {
        for (int i = 0; i < poly.length; i++) {
            double[] a = poly[i];
            double[] b = poly[(i + 1) % poly.length];
            double side = (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]);
            if (side <= 1e-12) {
                return false;
            }
        }
        return true;
    }

    /** True if segment a-b properly crosses segment c-d (they cross at a point inside both). */
    private static boolean cross(double[] a, double[] b, double[] c, double[] d) {
        double d1 = orient(c, d, a);
        double d2 = orient(c, d, b);
        double d3 = orient(a, b, c);
        double d4 = orient(a, b, d);
        return d1 * d2 < 0 && d3 * d4 < 0;
    }

    private static double orient(double[] p, double[] q, double[] r) {
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    }

    /**
     * Walks the route in tiny steps and returns a sentence describing the first place the robot overlaps one of the
     * boxes by more than {@code tol} or pokes more than {@code tol} out of the field; null if it never does.
     *
     * @param profile    the route and its headings
     * @param field      the field box (walls)
     * @param boxes      the HIVE frame and the FLOWER plates (the real ones, with no margin added)
     * @param tol        overlap forgiven, inches
     */
    static String firstClip(HeadingProfile profile, RobotShape robot, Rect field, List<Rect> boxes,
                            double tol) {
        return firstClip(profile, robot, field, boxes, tol, 0.02, 0.1);
    }

    /** Same, with the step set explicitly: one look every {@code stepIn} inches and every {@code stepDeg} degrees. */
    static String firstClip(HeadingProfile profile, RobotShape robot, Rect field, List<Rect> boxes,
                            double tol, double stepIn, double stepDeg) {
        if (profile.isEmpty()) {
            return null;
        }
        for (int leg = 0; leg < profile.vertexCount() - 1; leg++) {
            double sA = profile.distanceAtVertex(leg);
            double sB = profile.distanceAtVertex(leg + 1);
            double turnDeg = Math.abs(Math.toDegrees(profile.headingAtVertex(leg + 1) - profile.headingAtVertex(leg)));
            // The step in inches or in degrees, whichever gives more steps.
            int n = (int) Math.max(1.0, Math.max(Math.ceil((sB - sA) / stepIn), Math.ceil(turnDeg / stepDeg)));
            for (int k = 0; k <= n; k++) {
                double s = sA + (sB - sA) * k / n;
                Pose2 pose = profile.poseAt(s);
                double poke = wallPoke(pose, robot, field);
                if (poke > tol) {
                    return String.format(Locale.ROOT, "pokes %.2f in through a wall at %s (%.2f in along the route)",
                            poke, pose, s);
                }
                for (Rect b : boxes) {
                    if (overlaps(pose, robot, b, tol)) {
                        return String.format(Locale.ROOT, "overlaps %s at %s (%.2f in along the route)", b, pose, s);
                    }
                }
            }
        }
        return null;
    }
}
