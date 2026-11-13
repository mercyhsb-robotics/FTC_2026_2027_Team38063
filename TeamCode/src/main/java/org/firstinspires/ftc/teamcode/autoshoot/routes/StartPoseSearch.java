package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

/**
 * Where may the robot be set up before an AUTO, and which of those places is best?
 *
 * <p><b>Why it matters.</b> Rule G304 (see {@link StartRules}) has no start lines and no required heading: the robot may stand anywhere on its own
 * half of the field as long as it TOUCHES a perimeter wall and stays clear of the HIVE, the FLOWERS and the LOADING ZONE. The start is therefore a
 * free choice, and a good one removes whole legs of driving: for example a robot that starts already standing where it would shoot from needs no
 * drive at all before its first volley.
 *
 * <p><b>The legal starts.</b> {@link #legalStarts} lists every legal place on the own walls: a position every {@code Auto.START_SEARCH_STEP_IN} (0.25 in)
 * along each wall, for every heading on a grid of {@code Auto.START_SEARCH_HEADING_STEP_DEG} (5 degrees), the robot pushed against the wall exactly (its
 * footprint touches it, never overlaps it), and kept only if {@link StartRules} finds nothing at all wrong with it. A rotated robot touches the wall with
 * a corner, and the pose point (the robot's center of rotation, which is NOT the middle of the robot) is a different distance from the wall at every
 * heading, so the list is worked out with the real outline: {@link Footprint}.
 *
 * <p><b>The legal region for a picture.</b> {@link #legalIntervals} says the same thing as stretches of wall: for one heading, from where to where along
 * each own wall the robot may stand. The route diagrams tint them.
 *
 * <p>Pure Java. Everything is in the frame of the alliance asked for (the blue lists are the half-turn of the red ones).
 */
public final class StartPoseSearch {
    private static final double EPS = 1e-9;
    /** How many times a boundary between a legal and an illegal place is cut in half: far finer than any measurement of a real field. */
    private static final int BISECTIONS = 40;

    private StartPoseSearch() {
    }

    /** From where to where along one wall (inches along it: x for the audience and rear walls, y for the red and blue walls) a robot may stand. */
    public static final class Interval {
        public final Wall wall;
        public final double fromIn;
        public final double toIn;

        Interval(Wall wall, double fromIn, double toIn) {
            this.wall = wall;
            this.fromIn = fromIn;
            this.toIn = toIn;
        }

        /** How long the stretch is, in inches. */
        public double lengthIn() {
            return toIn - fromIn;
        }

        @Override
        public String toString() {
            return String.format(Locale.ROOT, "%s %.2f to %.2f in", wall.plainName(), fromIn, toIn);
        }
    }

    // ---- the legal starts ---------------------------------------------------------------------------------------------

    /**
     * Every legal start of the alliance at the standard resolution ({@code Auto.START_SEARCH_STEP_IN} along the walls, {@code Auto.START_SEARCH_HEADING_STEP_DEG}
     * of heading). See {@link #legalStarts(Alliance, double, double)}.
     */
    public static List<Pose2> legalStarts(Alliance alliance) {
        return legalStarts(alliance, BioBuzzConstants.Auto.START_SEARCH_STEP_IN, BioBuzzConstants.Auto.START_SEARCH_HEADING_STEP_DEG);
    }

    /**
     * Every legal start of the alliance: for each own wall, each heading of the grid and each position along the wall, the robot pushed against the wall
     * (its footprint exactly touching it), kept if {@link StartRules#isLegalAndClear} says it is legal. The positions are the multiples of {@code stepIn}
     * plus the two ends of the stretch where the robot still fits inside the field, so the corner starts (touching two walls) are always there. The
     * multiples are counted from the red wall and the audience wall for red, and from the blue wall and the rear wall for blue, so a blue start is exactly
     * the half turn of a red one. The headings are {@code headingStepDeg} apart from -180 (exclusive) to 180; the grid contains 0 and plus or minus 90 only if the step divides 90.
     * A pose that touches two walls is listed once. The poses are in the alliance's own frame.
     */
    public static List<Pose2> legalStarts(Alliance alliance, double stepIn, double headingStepDeg) {
        if (!(stepIn > 0.0) || !(headingStepDeg > 0.0)) {
            throw new IllegalArgumentException("the steps must be positive: " + stepIn + " in, " + headingStepDeg + " degrees");
        }
        FieldModel model = new FieldModel();
        AllianceField field = new AllianceField(model, alliance);
        double size = model.size();
        List<Pose2> out = new ArrayList<Pose2>();
        Set<String> seen = new HashSet<String>();
        for (double heading : headingGrid(headingStepDeg)) {
            for (Wall wall : Wall.own(alliance)) {
                double[] range = alongRange(wall, heading, size);
                for (double along : positions(range[0], range[1], stepIn, alliance == Alliance.BLUE, size)) {
                    Pose2 p = seatedAt(wall, along, heading, size);
                    if (StartRules.isLegalAndClear(field, p) && seen.add(key(p))) {
                        out.add(p);
                    }
                }
            }
        }
        return out;
    }

    /**
     * The robot pushed against {@code wall}: the pose, {@code alongIn} inches along the wall and facing {@code headingRad}, whose footprint touches the
     * wall exactly. The pose point is a different distance from the wall at every heading, because the robot is not centered on it: facing the audience
     * wall front-first the pose is 9.77 in from it, facing away 8.03 in, sideways 8.90 in, and a robot turned 45 degrees needs about 13 in.
     */
    public static Pose2 seatedAt(Wall wall, double alongIn, double headingRad, double fieldSize) {
        Pose2 loose = wall.runsAlongX() ? new Pose2(alongIn, 0.0, headingRad) : new Pose2(0.0, alongIn, headingRad);
        return wall.seated(loose, Footprint.of(loose).bounds(), fieldSize);
    }

    /** The same as {@link #seatedAt(Wall, double, double, double)} for the real field and a heading in degrees. */
    public static Pose2 seatedAt(Wall wall, double alongIn, double headingDeg) {
        return seatedAt(wall, alongIn, Math.toRadians(headingDeg), BioBuzzConstants.Field.SIZE_IN);
    }

    // ---- the legal region, as stretches of wall -------------------------------------------------------------------------

    /**
     * The legal region for one heading, as stretches of the own walls: where along each wall the robot, facing {@code headingDeg} and pushed against that
     * wall, may stand. Worked out every {@code Auto.START_SEARCH_STEP_IN} and then each end of a stretch is found exactly (cut in half until it is
     * known to a hair). Stretches are in the order of {@link Wall#own}, and each wall's stretches run from the small end to the large end.
     */
    public static List<Interval> legalIntervals(Alliance alliance, double headingDeg) {
        return legalIntervals(alliance, headingDeg, BioBuzzConstants.Auto.START_SEARCH_STEP_IN);
    }

    /** As {@link #legalIntervals(Alliance, double)} with the positions looked at {@code stepIn} apart. A stretch narrower than that can be missed. */
    public static List<Interval> legalIntervals(Alliance alliance, double headingDeg, double stepIn) {
        if (!(stepIn > 0.0)) {
            throw new IllegalArgumentException("the step must be positive: " + stepIn + " in");
        }
        FieldModel model = new FieldModel();
        AllianceField field = new AllianceField(model, alliance);
        double size = model.size();
        double heading = Math.toRadians(headingDeg);
        List<Interval> out = new ArrayList<Interval>();
        for (Wall wall : Wall.own(alliance)) {
            double[] range = alongRange(wall, heading, size);
            List<Double> at = positions(range[0], range[1], stepIn, alliance == Alliance.BLUE, size);
            boolean[] legal = new boolean[at.size()];
            for (int i = 0; i < legal.length; i++) {
                legal[i] = StartRules.isLegalAndClear(field, seatedAt(wall, at.get(i), heading, size));
            }
            int i = 0;
            while (i < legal.length) {
                if (!legal[i]) {
                    i++;
                    continue;
                }
                int j = i;
                while (j + 1 < legal.length && legal[j + 1]) {
                    j++;
                }
                double from = i == 0 ? at.get(0) : boundary(field, wall, heading, size, at.get(i - 1), at.get(i));
                double to = j == legal.length - 1 ? at.get(j) : boundary(field, wall, heading, size, at.get(j + 1), at.get(j));
                out.add(new Interval(wall, from, to));
                i = j + 1;
            }
        }
        return Collections.unmodifiableList(out);
    }

    /**
     * The legal region whatever the heading: the stretches of each own wall where a robot facing SOME heading of the grid
     * ({@code Auto.START_SEARCH_HEADING_STEP_DEG}) may stand. It is what the walls look like to a start that is free to turn: the wider picture of
     * {@link #legalIntervals(Alliance, double)}, which is for one heading.
     */
    public static List<Interval> legalIntervalsAnyHeading(Alliance alliance) {
        List<Interval> all = new ArrayList<Interval>();
        for (double heading : headingGrid(BioBuzzConstants.Auto.START_SEARCH_HEADING_STEP_DEG)) {
            all.addAll(legalIntervals(alliance, Math.toDegrees(heading)));
        }
        List<Interval> merged = new ArrayList<Interval>();
        for (Wall wall : Wall.own(alliance)) {
            List<Interval> mine = new ArrayList<Interval>();
            for (Interval in : all) {
                if (in.wall == wall) {
                    mine.add(in);
                }
            }
            Collections.sort(mine, new java.util.Comparator<Interval>() {
                @Override
                public int compare(Interval a, Interval b) {
                    return Double.compare(a.fromIn, b.fromIn);
                }
            });
            Interval run = null;
            for (Interval in : mine) {
                if (run != null && in.fromIn <= run.toIn + EPS) {
                    run = new Interval(wall, run.fromIn, Math.max(run.toIn, in.toIn));
                } else {
                    if (run != null) {
                        merged.add(run);
                    }
                    run = in;
                }
            }
            if (run != null) {
                merged.add(run);
            }
        }
        return Collections.unmodifiableList(merged);
    }

    // ---- helpers ----------------------------------------------------------------------------------------------------------

    /** The headings of the grid, in radians: {@code stepDeg} apart, from -180 (exclusive) to 180 (inclusive). */
    static List<Double> headingGrid(double stepDeg) {
        double step = Math.toRadians(stepDeg);
        List<Double> out = new ArrayList<Double>();
        for (int k = 1; -Math.PI + k * step <= Math.PI + EPS; k++) {
            out.add(Math.min(Math.PI, -Math.PI + k * step));       // the last one is exactly a half turn, not a hair past it
        }
        return out;
    }

    /**
     * The first and last position along {@code wall} at which a robot facing {@code heading} and pushed against the wall still fits inside the field:
     * the two ends are where it touches the neighbouring walls (the corners).
     */
    private static double[] alongRange(Wall wall, double heading, double size) {
        Rect box = Footprint.of(new Pose2(0.0, 0.0, heading)).bounds();
        return wall.runsAlongX() ? new double[] {-box.minX, size - box.maxX} : new double[] {-box.minY, size - box.maxY};
    }

    /**
     * The positions to look at between {@code lo} and {@code hi}, in order: both ends and every multiple of {@code step} between them. For the blue
     * alliance ({@code fromFarEnd}) the multiples are counted from the far end of the field instead, so that a blue start is exactly the half turn of a
     * red one (the field is not a whole number of steps across, and a blue student measures from the blue wall, as a red one does from the red wall).
     */
    private static List<Double> positions(double lo, double hi, double step, boolean fromFarEnd, double size) {
        List<Double> out = new ArrayList<Double>();
        out.add(lo);
        if (fromFarEnd) {
            // multiples of the step counted from the far end: size - k * step, for the k that keep the place strictly between the two ends
            for (long k = (long) Math.floor((size - hi) / step) + 1; size - k * step > lo + EPS; k++) {
                out.add(size - k * step);
            }
        } else {
            for (long k = (long) Math.floor(lo / step) + 1; k * step < hi - EPS; k++) {
                if (k * step > lo + EPS) {
                    out.add(k * step);
                }
            }
        }
        if (hi > lo + EPS) {
            out.add(hi);
        }
        Collections.sort(out);
        return out;
    }

    /** Between an illegal and a legal position along a wall, the place where it changes: found by cutting the gap in half until it is a hair wide. */
    private static double boundary(AllianceField field, Wall wall, double heading, double size, double illegal, double legal) {
        double bad = illegal;
        double good = legal;
        for (int i = 0; i < BISECTIONS; i++) {
            double mid = (bad + good) / 2.0;
            if (StartRules.isLegalAndClear(field, seatedAt(wall, mid, heading, size))) {
                good = mid;
            } else {
                bad = mid;
            }
        }
        return good;
    }

    /** A text key that is equal for equal poses, so a pose that touches two walls is listed once. */
    private static String key(Pose2 p) {
        return Double.doubleToLongBits(p.x) + "/" + Double.doubleToLongBits(p.y) + "/" + Double.doubleToLongBits(p.heading);
    }
}
