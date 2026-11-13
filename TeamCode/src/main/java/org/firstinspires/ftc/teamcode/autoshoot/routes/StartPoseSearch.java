package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
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
     * wall front-first the pose is 9.77 in from it, facing away 8.03 in, sideways 8.81 in, and a robot turned 45 degrees needs about 13 in.
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
            Collections.sort(mine, new Comparator<Interval>() {
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

    // ---- the best start for a route --------------------------------------------------------------------------------------------

    /** How {@link #bestStart} searches: how fine the grid is, how big the placement margin is, and which robot's timing it plans for. */
    public static final class Settings {
        public final double stepIn;
        public final double headingStepDeg;
        public final double marginIn;
        public final double marginDeg;
        public final TimingModel timing;
        /** How many starts the search checks with the full validator before it gives up ({@code Auto.START_SEARCH_MAX_VALIDATIONS} unless changed). */
        public final int maxValidations;

        private Settings(double stepIn, double headingStepDeg, double marginIn, double marginDeg, TimingModel timing, int maxValidations) {
            this.stepIn = stepIn;
            this.headingStepDeg = headingStepDeg;
            this.marginIn = marginIn;
            this.marginDeg = marginDeg;
            this.timing = timing;
            this.maxValidations = maxValidations;
        }

        /** The grid, the placement margin and the timing of {@link BioBuzzConstants}. */
        public static Settings standard() {
            return new Settings(BioBuzzConstants.Auto.START_SEARCH_STEP_IN, BioBuzzConstants.Auto.START_SEARCH_HEADING_STEP_DEG,
                    BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN, BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_DEG, new TimingModel(),
                    BioBuzzConstants.Auto.START_SEARCH_MAX_VALIDATIONS);
        }

        /** The same with another grid (a coarser one makes the search quicker, for a test or a first look). */
        public Settings withResolution(double newStepIn, double newHeadingStepDeg) {
            return new Settings(newStepIn, newHeadingStepDeg, marginIn, marginDeg, timing, maxValidations);
        }

        /** The same with another placement margin (0 and 0 asks for nothing: the best start if the robot could be placed exactly). */
        public Settings withMargin(double newMarginIn, double newMarginDeg) {
            return new Settings(stepIn, headingStepDeg, newMarginIn, newMarginDeg, timing, maxValidations);
        }

        /** The same, giving up after this many starts have been checked with the full validator. */
        public Settings withMaxValidations(int newMax) {
            return new Settings(stepIn, headingStepDeg, marginIn, marginDeg, timing, newMax);
        }

        /** The same for a robot with other timing and wiggle numbers. */
        public Settings withTiming(TimingModel newTiming) {
            return new Settings(stepIn, headingStepDeg, marginIn, marginDeg, newTiming, maxValidations);
        }
    }

    /** The answer of {@link #bestStart}: the best start, what it saves, and how hard the search had to look. */
    public static final class Best {
        /** True if some legal start gave a route that validates and keeps the placement margin. */
        public final boolean found;
        /** The best start (null if none was found), and the route built from it. */
        public final Pose2 start;
        public final AutoRoute route;
        /** The time estimate of that route, seconds (NaN if none). */
        public final double seconds;
        /** The standard start that gives the fastest valid route (null if no standard start gives a valid route), its route and its time (NaN). */
        public final Pose2 standardStart;
        public final AutoRoute standardRoute;
        public final double standardSeconds;
        /** How many legal starts there are on the grid, how many of them keep the placement margin, and how many were checked with the full validator. */
        public final int legalStarts;
        public final int keepingTheMargin;
        public final int validated;
        /** What the placement check says about the best start (null if none was found). */
        public final StartRobustness.Report placement;
        /** Empty when a start was found; else the plain reason why not. */
        public final String reason;

        Best(boolean found, Pose2 start, AutoRoute route, double seconds, Pose2 standardStart, AutoRoute standardRoute, double standardSeconds, int legalStarts,
             int keepingTheMargin, int validated, StartRobustness.Report placement, String reason) {
            this.found = found;
            this.start = start;
            this.route = route;
            this.seconds = seconds;
            this.standardStart = standardStart;
            this.standardRoute = standardRoute;
            this.standardSeconds = standardSeconds;
            this.legalStarts = legalStarts;
            this.keepingTheMargin = keepingTheMargin;
            this.validated = validated;
            this.placement = placement;
            this.reason = reason;
        }

        /** How many seconds the best start saves over the standard start (NaN if there is no best start or no valid standard start). */
        public double secondsSaved() {
            return standardSeconds - seconds;
        }
    }

    /** {@link #bestStart(RouteTemplate, List, Settings)} with the standard settings. */
    public static Best bestStart(RouteTemplate template, List<Pose2> standardStarts) {
        return bestStart(template, standardStarts, Settings.standard());
    }

    /**
     * The legal start (red frame) that gives the fastest route: every legal start of the grid that keeps the placement margin is tried with the template, the
     * routes are ordered by the time the {@link TimingModel} estimates, and the first one that the full {@link RouteValidator} accepts (no ERROR) and whose
     * placement check passes ({@link StartRobustness}: the same route from every start that is {@code marginIn} off along the wall and {@code marginDeg} turned)
     * is the answer. Among starts that are equally fast the one that is easier to place wins (a robot straight at a wall before a turned one).
     *
     * <p>It also reports how many seconds that saves over the standard start: the fastest of {@code standardStarts} (for example tiles B1 and C1, front on the
     * audience wall) whose route validates.
     *
     * <p>Safety of the answer: the legal starts come from {@link StartRules} (the validator's own G304 check), the route is checked by the validator, and the
     * margin is checked by moving the start; nothing here is a rule of its own.
     */
    public static Best bestStart(RouteTemplate template, List<Pose2> standardStarts, Settings settings) {
        FieldModel model = new FieldModel();
        AllianceField field = new AllianceField(model, Alliance.RED);
        RouteValidator validator = new RouteValidator(model, settings.timing);

        // the standard start: the fastest of the ones a team would use without this search whose route is valid
        Pose2 standardStart = null;
        AutoRoute standardRoute = null;
        double standardSeconds = Double.NaN;
        for (Pose2 p : standardStarts) {
            AutoRoute r = template.build(p);
            double sec = RouteAnalysis.of(r, model, settings.timing).totalSeconds();
            if (!hasError(validator, r) && (standardStart == null || sec < standardSeconds)) {
                standardStart = p;
                standardRoute = r;
                standardSeconds = sec;
            }
        }

        // every legal start that keeps its margin, with the time of its route
        List<Pose2> legal = legalStarts(Alliance.RED, settings.stepIn, settings.headingStepDeg);
        List<Candidate> candidates = new ArrayList<Candidate>();
        for (Pose2 p : legal) {
            if (StartRobustness.startsStayLegal(field, p, settings.marginIn, settings.marginDeg)) {
                candidates.add(new Candidate(p, RouteAnalysis.of(template.build(p), model, settings.timing).totalSeconds()));
            }
        }
        final double tie = BioBuzzConstants.Auto.START_SEARCH_TIE_SEC;
        Collections.sort(candidates, new Comparator<Candidate>() {
            @Override
            public int compare(Candidate a, Candidate b) {
                int byTime = tie > 0.0 ? Long.compare(Math.round(a.seconds / tie), Math.round(b.seconds / tie)) : Double.compare(a.seconds, b.seconds);
                if (byTime != 0) {
                    return byTime;
                }
                int byAlignment = Double.compare(a.misalignmentRad(), b.misalignmentRad());
                if (byAlignment != 0) {
                    return byAlignment;
                }
                int byX = Double.compare(a.start.x, b.start.x);
                if (byX != 0) {
                    return byX;
                }
                int byY = Double.compare(a.start.y, b.start.y);
                return byY != 0 ? byY : Double.compare(a.start.heading, b.start.heading);
            }
        });

        // A standard start that keeps the margin is a start nobody has to improve on: a start that is slower than it saves nothing, so the search can stop there.
        StartRobustness.Report standardPlacement = null;
        if (standardStart != null) {
            standardPlacement = StartRobustness.check(template, standardStart, Alliance.RED, settings.timing, settings.marginIn, settings.marginDeg);
        }
        boolean standardIsSafe = standardPlacement != null && standardPlacement.placementSafe();

        // the fastest first, until one is valid and keeps its margin
        int validated = 0;
        String firstProblem = "";
        for (Candidate c : candidates) {
            if (validated >= settings.maxValidations) {
                break;
            }
            if (standardIsSafe && c.seconds > standardSeconds + BioBuzzConstants.Auto.START_SEARCH_TIE_SEC) {
                // nothing faster than the standard start is both valid and placeable: the standard start is the best there is
                return new Best(true, standardStart, standardRoute, standardSeconds, standardStart, standardRoute, standardSeconds, legal.size(), candidates.size(),
                        validated, standardPlacement, "");
            }
            validated++;
            AutoRoute route = template.build(c.start);
            String error = firstError(validator, route);
            if (!error.isEmpty()) {
                firstProblem = firstProblem.isEmpty() ? "the start " + c.start + " gives " + error : firstProblem;
                continue;
            }
            if (!StartRobustness.passes(template, c.start, Alliance.RED, settings.timing, settings.marginIn, settings.marginDeg)) {
                if (firstProblem.isEmpty()) {
                    // say what went wrong for the fastest start that failed, once (the full report is slow, so only for this one)
                    StartRobustness.Report why = StartRobustness.check(template, c.start, Alliance.RED, settings.timing, settings.marginIn, settings.marginDeg);
                    firstProblem = "the start " + c.start + " fails when placed " + settings.marginIn + " in or " + settings.marginDeg + " degrees off: " + why.newErrors();
                }
                continue;
            }
            StartRobustness.Report placement = StartRobustness.check(template, c.start, Alliance.RED, settings.timing, settings.marginIn, settings.marginDeg);
            return new Best(true, c.start, route, c.seconds, standardStart, standardRoute, standardSeconds, legal.size(), candidates.size(), validated, placement, "");
        }
        String reason = legal.isEmpty() ? "no legal start on the grid"
                : candidates.isEmpty() ? "none of the " + legal.size() + " legal starts keeps the placement margin of " + settings.marginIn + " in and " + settings.marginDeg
                        + " degrees (the gap between the FLOWERS, the LOADING ZONE and the center line is too narrow for it)"
                : "none of the " + validated + " fastest starts gives a route the validator accepts with the margin; the first: " + firstProblem;
        return new Best(false, null, null, Double.NaN, standardStart, standardRoute, standardSeconds, legal.size(), candidates.size(), validated, null, reason);
    }

    /** A legal start with the time of its route, for sorting. */
    private static final class Candidate {
        final Pose2 start;
        final double seconds;

        Candidate(Pose2 start, double seconds) {
            this.start = start;
            this.seconds = seconds;
        }

        /** How far the heading is from the nearest of the four ways straight at a wall, radians: a robot that is straight is the easiest to place. */
        double misalignmentRad() {
            double quarter = Math.PI / 2.0;
            double r = Math.abs(start.heading % quarter);
            return Math.min(r, quarter - r);
        }
    }

    private static boolean hasError(RouteValidator validator, AutoRoute route) {
        return validator.firstError(route) != null;
    }

    /** The first ERROR the validator finds in the route, as text; empty if there is none. */
    private static String firstError(RouteValidator validator, AutoRoute route) {
        Issue i = validator.firstError(route);
        return i == null ? "" : i.toString();
    }

    /**
     * The start on {@code wall}, facing {@code headingDeg}, that is closest to {@code alongIn} (inches along the wall) and still legal and clear with the
     * placement margin: the answer to "I want to start about here: where exactly may I?". Null if no place of that wall keeps the margin facing that way.
     *
     * @throws IllegalArgumentException if the wall is not one of the alliance's own walls
     */
    public static Pose2 nearestSafeStart(Alliance alliance, Wall wall, double headingDeg, double alongIn, double marginIn, double marginDeg) {
        if (!Wall.own(alliance).contains(wall)) {
            throw new IllegalArgumentException("the " + wall.plainName() + " is not a wall of the " + alliance + " half, no start can touch it");
        }
        FieldModel model = new FieldModel();
        AllianceField field = new AllianceField(model, alliance);
        double size = model.size();
        double heading = Math.toRadians(headingDeg);
        double[] range = alongRange(wall, heading, size);
        Pose2 best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (double along : positions(range[0], range[1], BioBuzzConstants.Auto.START_SEARCH_STEP_IN, alliance == Alliance.BLUE, size)) {
            Pose2 p = seatedAt(wall, along, heading, size);
            if (StartRobustness.startsStayLegal(field, p, marginIn, marginDeg) && Math.abs(along - alongIn) < bestDistance) {
                bestDistance = Math.abs(along - alongIn);
                best = p;
            }
        }
        return best;
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
