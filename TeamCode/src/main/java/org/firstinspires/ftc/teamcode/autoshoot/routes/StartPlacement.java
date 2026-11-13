package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

/**
 * Where to put the robot before an AUTO, in plain words a student can follow with a tape measure.
 *
 * <p>A start pose is a point (the robot's center of rotation, which is inside the robot and not in its middle) and a heading. Nobody can measure a point
 * inside a robot, and the pose point is not what the referee checks. So the words are about the robot's EDGES and CORNERS, the parts you can touch with a tape:
 * which side of the robot touches which wall, and how far from another wall the nearest edge is. For example, for the team's old start at the audience wall:
 * <pre>
 *   Front on the audience wall. Right edge 51.4 in from the red wall. Facing the audience wall.
 * </pre>
 * <b>Left and right are the robot's own</b>, the way you see them standing behind the robot looking where it faces (a robot facing the audience wall has its
 * left side toward the blue wall). The edge named is the one nearest the wall the distance is measured from, so there is only one right answer to measure.
 *
 * <p>Three kinds of start, three kinds of words:
 * <ul>
 *   <li><b>Straight at a wall</b> (within {@code Auto.PLACEMENT_ALIGNED_TOLERANCE_DEG} of 0, 90, 180 or -90 degrees) and touching ONE wall: which side is on
 *       the wall, the distance of the nearest edge from the own side wall (from the audience wall for a start on the side wall), and which wall it faces.</li>
 *   <li><b>Straight, in a corner</b> (touching two walls): which side is on each wall, and which way it faces. Nothing to measure: the two walls place it.</li>
 *   <li><b>Turned</b> (any other heading): the corner that touches each wall and how far along the wall that corner is from the other wall, then how many
 *       degrees the robot is turned from straight at the nearest wall, clockwise or counter-clockwise as seen from above (as in the route diagrams, with the
 *       audience wall at the bottom).</li>
 * </ul>
 * A start that touches no wall is not a legal start (G304.C); the words say how far it is from the nearest wall.
 *
 * <p>The numbers come from the real outline of the robot ({@link Footprint}), so they change with the measured center of rotation.
 */
public final class StartPlacement {
    /** An angle this close to a whole number of degrees is said as the whole number (35.02 is "35"). */
    private static final double WHOLE_DEGREE_TOLERANCE = 0.05;

    private StartPlacement() {
    }

    /** The words for the start of a route, in the route's own alliance frame. */
    public static String placementText(AutoRoute route) {
        return placementText(route.startPose(), route.alliance());
    }

    /** The words for a start (in {@code alliance}'s own frame). */
    public static String placementText(Pose2 start, Alliance alliance) {
        double size = BioBuzzConstants.Field.SIZE_IN;
        Footprint fp = Footprint.of(start);
        Rect b = fp.bounds();
        List<Wall> touched = new ArrayList<Wall>();
        for (Wall wall : Wall.values()) {
            if (wall.gapIn(b, size) <= BioBuzzConstants.Auto.START_WALL_CONTACT_TOLERANCE_IN) {
                touched.add(wall);
            }
        }
        if (touched.isEmpty()) {
            return notTouching(b, size);
        }
        double quarter = Math.PI / 2.0;
        double turns = start.heading / quarter;
        double faceRad = Math.rint(turns) * quarter;
        double offDeg = Math.toDegrees((turns - Math.rint(turns)) * quarter);
        Wall faced = wallInDirection(faceRad);
        boolean straight = Math.abs(offDeg) <= BioBuzzConstants.Auto.PLACEMENT_ALIGNED_TOLERANCE_DEG;
        Wall own = alliance == Alliance.RED ? Wall.RED : Wall.BLUE;

        StringBuilder sb = new StringBuilder();
        if (straight && touched.size() == 1) {
            Wall wall = touched.get(0);
            sb.append(sideOn(start, wall)).append(" on the ").append(wall.plainName()).append(". ");
            Wall reference = wall.runsAlongX() ? own : Wall.AUDIENCE;
            sb.append(edgeNamed(start, reference)).append(" edge ").append(in(reference.gapIn(b, size))).append(" from the ").append(reference.plainName()).append(". ");
        } else if (straight) {
            for (int i = 0; i < touched.size(); i++) {
                sb.append(i == 0 ? sideOn(start, touched.get(i)) : lower(sideOn(start, touched.get(i)))).append(" on the ").append(touched.get(i).plainName());
                sb.append(i < touched.size() - 1 ? " and " : ". ");
            }
        } else {
            for (Wall wall : touched) {
                Point2 corner = cornerOn(fp, wall);
                Wall reference = wall.runsAlongX() ? own : Wall.AUDIENCE;
                sb.append(cornerName(fp, corner)).append(" corner on the ").append(wall.plainName()).append(", ")
                        .append(in(reference.gapIn(new Rect(corner.x, corner.x, corner.y, corner.y), size))).append(" from the ").append(reference.plainName()).append(". ");
            }
            sb.append("Turned ").append(degrees(Math.abs(offDeg))).append(" degrees ").append(offDeg > 0.0 ? "counter-clockwise" : "clockwise")
                    .append(" (seen from above, as in the diagram) from facing the ").append(faced.plainName()).append(".");
            return sb.toString();
        }
        sb.append("Facing the ").append(faced.plainName()).append(".");
        return sb.toString();
    }

    // ---- helpers ------------------------------------------------------------------------------------------------------------------

    private static String notTouching(Rect b, double size) {
        Wall nearest = Wall.RED;
        double gap = Double.POSITIVE_INFINITY;
        for (Wall wall : Wall.values()) {
            if (wall.gapIn(b, size) < gap) {
                gap = wall.gapIn(b, size);
                nearest = wall;
            }
        }
        return "This is not a legal start: the robot touches no wall (rule G304.C). Its nearest edge is " + in(gap) + " from the " + nearest.plainName() + ".";
    }

    /** The wall in the direction {@code rad} (a straight heading): the wall a robot facing that way has in front of it. */
    private static Wall wallInDirection(double rad) {
        Wall best = Wall.RED;
        double bestError = Double.POSITIVE_INFINITY;
        for (Wall wall : Wall.values()) {
            double e = Math.abs(Angles.error(rad, wall.outwardRad()));
            if (e < bestError) {
                bestError = e;
                best = wall;
            }
        }
        return best;
    }

    /**
     * Which of the robot's four sides faces {@code wall}: 0 front, 1 its left, 2 the back, 3 its right (counter-clockwise from the front, seen from above),
     * for a robot that is straight or nearly so.
     */
    private static int sideIndex(Pose2 start, Wall wall) {
        double e = Angles.error(start.heading, wall.outwardRad());          // positive: the wall is to the robot's left
        int quarters = (int) Math.rint(e / (Math.PI / 2.0));
        return ((quarters % 4) + 4) % 4;
    }

    /** "Front", "Left side", "Back" or "Right side": the part of the robot that is on the wall. */
    private static String sideOn(Pose2 start, Wall wall) {
        switch (sideIndex(start, wall)) {
            case 0:
                return "Front";
            case 1:
                return "Left side";
            case 2:
                return "Back";
            default:
                return "Right side";
        }
    }

    /** "Front", "Left", "Back" or "Right": the robot's edge that is nearest {@code reference}, to measure from. */
    private static String edgeNamed(Pose2 start, Wall reference) {
        switch (sideIndex(start, reference)) {
            case 0:
                return "Front";
            case 1:
                return "Left";
            case 2:
                return "Back";
            default:
                return "Right";
        }
    }

    /** The corner of the footprint that is nearest {@code wall}. */
    private static Point2 cornerOn(Footprint fp, Wall wall) {
        Point2 best = fp.corner(0);
        double bestGap = Double.POSITIVE_INFINITY;
        double size = BioBuzzConstants.Field.SIZE_IN;
        for (int i = 0; i < 4; i++) {
            Point2 c = fp.corner(i);
            double gap = wall.gapIn(new Rect(c.x, c.x, c.y, c.y), size);
            if (gap < bestGap) {
                bestGap = gap;
                best = c;
            }
        }
        return best;
    }

    /** "Front-left", "Back-left", "Back-right" or "Front-right": which corner of the robot the point is (it is one of the footprint's four corners). */
    private static String cornerName(Footprint fp, Point2 corner) {
        String[] names = {"Front-left", "Back-left", "Back-right", "Front-right"};
        int best = 0;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 4; i++) {
            double d = Math.hypot(fp.corner(i).x - corner.x, fp.corner(i).y - corner.y);
            if (d < bestDistance) {
                bestDistance = d;
                best = i;
            }
        }
        return names[best];
    }

    private static String lower(String text) {
        return text.substring(0, 1).toLowerCase(Locale.ROOT) + text.substring(1);
    }

    private static String in(double v) {
        return String.format(Locale.ROOT, "%.1f in", v);
    }

    /** An angle in degrees as a person says it: whole degrees if it is whole, else one decimal ("35", "12.5"). */
    private static String degrees(double v) {
        return Math.abs(v - Math.rint(v)) < WHOLE_DEGREE_TOLERANCE ? String.format(Locale.ROOT, "%.0f", v) : String.format(Locale.ROOT, "%.1f", v);
    }
}
