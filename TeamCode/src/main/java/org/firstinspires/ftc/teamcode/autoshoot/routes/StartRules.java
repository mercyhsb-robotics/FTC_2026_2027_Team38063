package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotShape;

/**
 * Rule G304 (where a robot may be set up on the field before an AUTO) as ONE piece of code. The route validator's start check, the search for the
 * best place to start ({@link StartPoseSearch}), the "legal start" bands of the diagrams and the TeleOp start pose check in {@code ConstantsCheck} all
 * call it, so they can never disagree about what is legal.
 *
 * <p>What G304 asks of the robot's PLACE (the other conditions, F, G and H, are about the robot's size, the POLLEN it holds and standing still,
 * not about where it is; the 18 in cube of F is checked once for the robot as a whole by {@code ConstantsCheck}):
 * <ul>
 *   <li><b>A</b> the whole robot is inside the field (it may touch a wall but not overhang it: the tolerance is
 *       {@code Auto.FIELD_BOUNDS_TOLERANCE_IN}) and inside its own alliance's half of the field.</li>
 *   <li><b>B</b> it is not attached to the HIVE frame.</li>
 *   <li><b>C</b> it TOUCHES a perimeter wall (within {@code Auto.START_WALL_CONTACT_TOLERANCE_IN}).</li>
 *   <li><b>D</b> it does not touch a FLOWER. The wall bracket that holds a FLOWER sticks out a little past the plate, at a height only the tallest part of
 *       the robot reaches: standing over it is a WARNING here, and {@link #isLegalAndClear} (what the search uses) refuses it.</li>
 *   <li><b>E</b> it is not in the LOADING ZONE.</li>
 * </ul>
 * There are no start lines, no required heading and no required distance from the HIVE: the place is free, which is why the search is worth running.
 *
 * <p>The robot is judged by its real outline, which is NOT centered on the pose point (see {@code RobotShape}): every test below uses
 * {@link Footprint#of(Pose2)}.
 */
public final class StartRules {
    private static final double EPS = 1e-9;
    private static final int ERROR_BIT = 1;
    private static final int WARN_BIT = 2;

    private StartRules() {
    }

    /** The parts of G304 that depend on where the robot stands. */
    public enum Part {
        /** A: the robot overhangs a field wall. */
        FIELD("G304.A"),
        /** A: the robot reaches over the center line into the other alliance's half. */
        OWN_SIDE("G304.A"),
        /** C: the robot touches no perimeter wall. */
        TOUCH_WALL("G304.C"),
        /** E: the robot is in its own LOADING ZONE. */
        LOADING_ZONE("G304.E"),
        /** D: the robot touches a FLOWER plate. */
        FLOWER("G304.D"),
        /** D (a WARNING): the robot is over the wall bracket of a FLOWER, which sticks out past the plate. */
        FLOWER_BRACKET("G304.D"),
        /** B: the robot is attached to the HIVE frame. */
        HIVE("G304.B");

        /** The manual's name of the rule, for example {@code "G304.C"}. */
        public final String rule;

        Part(String rule) {
            this.rule = rule;
        }
    }

    /** One thing wrong with a start: which part of G304, whether it is an ERROR (false: a WARN), and the words for people. */
    public static final class Finding {
        public final Part part;
        public final boolean error;
        public final String message;

        Finding(Part part, boolean error, String message) {
            this.part = part;
            this.error = error;
            this.message = message;
        }

        @Override
        public String toString() {
            return (error ? "ERROR " : "WARN ") + part.rule + ": " + message;
        }
    }

    /**
     * Every finding for a robot set up at {@code start} (in the frame of {@code field}'s alliance), in a fixed order: A, C, E, D, B. Empty when the start is
     * legal in every way. The messages are plain words and start with {@code "start footprint at (x, y, heading)"}.
     */
    public static List<Finding> check(AllianceField field, Pose2 start) {
        List<Finding> out = new ArrayList<Finding>();
        scan(field, start, out);
        return Collections.unmodifiableList(out);
    }

    /** True if the start has no ERROR: a WARN (the wall bracket) does not make it illegal. */
    public static boolean isLegal(AllianceField field, Pose2 start) {
        return (scan(field, start, null) & ERROR_BIT) == 0;
    }

    /**
     * True if the start has no finding at all, not even a WARN. This is the test the search for a start uses: a robot over the bracket of a FLOWER
     * might well be judged to touch the FLOWER, so the search never proposes one.
     */
    public static boolean isLegalAndClear(AllianceField field, Pose2 start) {
        return scan(field, start, null) == 0;
    }

    /**
     * Runs every test once. With a {@code sink} the findings are written into it (with their messages); with {@code null} only the result is
     * worked out, which is what the search does hundreds of thousands of times. Returns a mask of {@link #ERROR_BIT} and {@link #WARN_BIT}.
     */
    private static int scan(AllianceField f, Pose2 start, List<Finding> sink) {
        int found = 0;
        Footprint fp = Footprint.of(start);
        double tol = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
        String at = sink == null ? "" : "start footprint at " + pose(start);

        double outside = fp.overshootOutside(f.fieldRect());
        if (outside > tol) {
            found |= ERROR_BIT;
            if (sink != null) {
                sink.add(new Finding(Part.FIELD, true, at + " sticks out of the field by " + in(outside) + " (G304.A)" + wallHint(start, f.fieldRect())));
            }
        }
        double past = 0.0;
        for (int i = 0; i < 4; i++) {
            past = Math.max(past, f.pastCenterLine(fp.corner(i).x));
        }
        if (past > tol) {
            found |= ERROR_BIT;
            if (sink != null) {
                sink.add(new Finding(Part.OWN_SIDE, true, at + " reaches " + in(past)
                        + " into the opponent half; the robot must start fully inside its own columns (G304.A)"));
            }
        }
        Rect b = fp.bounds();
        double size = f.fieldRect().maxX;
        double gap = Math.min(Math.min(b.minX, b.minY), Math.min(size - b.maxX, size - b.maxY));
        if (gap > BioBuzzConstants.Auto.START_WALL_CONTACT_TOLERANCE_IN) {
            found |= ERROR_BIT;
            if (sink != null) {
                sink.add(new Finding(Part.TOUCH_WALL, true, at + " is " + in(gap) + " away from the nearest wall; it must touch a"
                        + " perimeter wall (G304.C, tolerance " + in(BioBuzzConstants.Auto.START_WALL_CONTACT_TOLERANCE_IN) + ")"));
            }
        }
        if (fp.overlaps(f.ownLoadingZone())) {
            found |= ERROR_BIT;
            if (sink != null) {
                sink.add(new Finding(Part.LOADING_ZONE, true, at + " overlaps the own LOADING ZONE " + f.ownLoadingZone()
                        + "; no robot may start there (G304.E)"));
            }
        }
        for (int i = 0; i < f.flowerPlates().size(); i++) {
            if (fp.overlaps(f.flowerPlates().get(i))) {
                found |= ERROR_BIT;
                if (sink != null) {
                    sink.add(new Finding(Part.FLOWER, true, at + " touches FLOWER " + (i + 1) + "; a start may not contact a FLOWER (G304.D)"));
                }
            } else if (BioBuzzConstants.Robot.HEIGHT_IN > BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN && fp.overlaps(f.flowerBracket(i))) {
                // The bracket is longer than the plate, so a robot can stand clear of the plate and still be over the bracket.
                found |= WARN_BIT;
                if (sink != null) {
                    sink.add(new Finding(Part.FLOWER_BRACKET, false, at + " is over the wall bracket of FLOWER " + (i + 1) + " (it sticks out "
                            + in(BioBuzzConstants.Field.FLOWER_BRACKET_PROTRUSION_IN) + " from the wall, " + in(BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN)
                            + " along it, at " + in(BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN) + " to " + in(BioBuzzConstants.Field.FLOWER_BRACKET_TOP_Z_IN)
                            + " above the floor): any part of the robot taller than " + in(BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN)
                            + " there would be touching the FLOWER (G304.D). Slide the start along the wall"));
                }
            }
        }
        if (fp.overlaps(f.hiveFrame())) {
            found |= ERROR_BIT;
            if (sink != null) {
                sink.add(new Finding(Part.HIVE, true, at + " overlaps the HIVE frame " + f.hiveFrame()
                        + "; a start may not be attached to a FIELD element (G304.B)"));
            }
        }
        return found;
    }

    /**
     * For a start that pokes through a wall: which wall it is, how far the pose point (the center of rotation) is from it,
     * and how far it has to be for the robot's edge that faces the wall to just touch it. The robot is not centered on its
     * pose point, so the answer depends on which way it faces: the front edge is {@code Robot.FRONT_EXTENT_IN} from the
     * pose point, the back edge {@code REAR_EXTENT_IN}, the sides {@code LEFT_EXTENT_IN} and {@code RIGHT_EXTENT_IN}.
     * Empty if no wall is poked through.
     */
    private static String wallHint(Pose2 start, Rect field) {
        double[] direction = {Math.PI, 0.0, -Math.PI / 2.0, Math.PI / 2.0};       // toward the wall at x = min, x = max, y = min, y = max
        double[] distance = {start.x - field.minX, field.maxX - start.x, start.y - field.minY, field.maxY - start.y};
        double[] wallAt = {field.minX, field.maxX, field.minY, field.maxY};
        String[] axis = {"x", "x", "y", "y"};
        RobotShape shape = RobotShape.fromConstants();
        int worst = -1;
        double worstPoke = EPS;
        for (int i = 0; i < 4; i++) {
            double poke = shape.reachTowardIn(start.heading, direction[i]) - distance[i];
            if (poke > worstPoke) {
                worstPoke = poke;
                worst = i;
            }
        }
        if (worst < 0) {
            return "";
        }
        double needed = shape.reachTowardIn(start.heading, direction[worst]);
        return ": its center of rotation is " + in(distance[worst]) + " from the wall at " + axis[worst] + " = "
                + String.format(Locale.ROOT, "%.1f", wallAt[worst])
                + " but the robot's edge that faces that wall reaches " + in(needed) + " from the center (front " + in(shape.frontIn())
                + ", rear " + in(shape.rearIn()) + ", sides " + in(shape.leftIn())
                + "); place the real robot with that edge ON the wall, its center " + in(needed) + " out";
    }

    private static String pose(Pose2 p) {
        return String.format(Locale.ROOT, "(%.1f, %.1f, %.0f deg)", p.x, p.y, p.headingDegrees());
    }

    private static String in(double v) {
        return String.format(Locale.ROOT, "%.1f in", v);
    }
}
