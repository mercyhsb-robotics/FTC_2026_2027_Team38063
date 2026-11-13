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
 * "What if the robot is not placed exactly?" A person sets the robot down by hand: it is an inch off along the wall and a few degrees turned. This class
 * moves a route's start by that much (the placement margin, {@code Auto.START_PLACEMENT_MARGIN_IN} and {@code START_PLACEMENT_MARGIN_DEG}) in every
 * combination, runs the route validator on each, and reports every ERROR it finds. An ERROR is a finding: it means the route is set up too close to the
 * edge of what is legal or what is safe, and the placement margin is too small.
 *
 * <p><b>How a start is moved.</b> The robot is set against a wall, and the wall decides where it ends up: a hand that pushes it toward the wall stops
 * at the wall, and a robot that is 1 in away from the wall is not a legal start (G304.C). So the part of the move that goes into or out of a wall is
 * taken away again (the robot is "pushed back against the wall") and what is left is the part along the wall, and the turn. A turn is about the robot's
 * contact with the wall, so it also moves the pose point. A start that touches two walls (a corner) can only turn.
 *
 * <p><b>What is held fixed.</b> Every step after the start stays where the route's author put it ({@link RouteTemplate#keeping}); a template may let
 * the rest change with the start. The robot's odometry is told the route's start pose, so a robot that really stands 1 in off would carry that error
 * through the route; this check does not add it up along the route. It asks the question that can be answered before the robot drives: is the route
 * still legal, and does its first leg still clear everything, from a start that is a little off? The time spread says how much the estimate moves.
 */
public final class StartRobustness {
    private StartRobustness() {
    }

    /** One way the robot may be set down away from the route's start. */
    public static final class Move {
        /** How far the hand moved it in x and y before the wall stopped it, inches. */
        public final double dxIn;
        public final double dyIn;
        /** How far the hand turned it, degrees (counter-clockwise positive). */
        public final double dHeadingDeg;
        /** Where the robot ends up: pushed back against the wall it touched. */
        public final Pose2 start;

        Move(double dxIn, double dyIn, double dHeadingDeg, Pose2 start) {
            this.dxIn = dxIn;
            this.dyIn = dyIn;
            this.dHeadingDeg = dHeadingDeg;
            this.start = start;
        }

        /** For example {@code "x +1.0 in, y -1.0 in, heading +3.0 deg"}. */
        public String describe() {
            return String.format(Locale.ROOT, "x %+.1f in, y %+.1f in, heading %+.1f deg", dxIn, dyIn, dHeadingDeg);
        }
    }

    /** What the validator said about the route when the robot was set down one way. */
    public static final class Case {
        public final Move move;
        /** The validator's ERRORs for the route started there (empty: fine). */
        public final List<Issue> errors;
        /** The time estimate for the route started there, seconds. */
        public final double seconds;

        Case(Move move, List<Issue> errors, double seconds) {
            this.move = move;
            this.errors = Collections.unmodifiableList(errors);
            this.seconds = seconds;
        }

        public boolean failed() {
            return !errors.isEmpty();
        }
    }

    /** The result for one start. */
    public static final class Report {
        /** The start that was moved. */
        public final Pose2 start;
        /** The ERRORs of the route from the start itself, before anything is moved. */
        public final List<Issue> nominalErrors;
        public final double nominalSeconds;
        /** One case for every different place the robot can end up (the start itself is not among them). */
        public final List<Case> cases;

        Report(Pose2 start, List<Issue> nominalErrors, double nominalSeconds, List<Case> cases) {
            this.start = start;
            this.nominalErrors = Collections.unmodifiableList(nominalErrors);
            this.nominalSeconds = nominalSeconds;
            this.cases = Collections.unmodifiableList(cases);
        }

        /** The cases in which the validator found an ERROR. */
        public List<Case> failedCases() {
            List<Case> out = new ArrayList<Case>();
            for (Case c : cases) {
                if (c.failed()) {
                    out.add(c);
                }
            }
            return out;
        }

        /** True if no way of setting the robot down gave an ERROR, and the start itself is clean: the route has the placement margin. */
        public boolean placementSafe() {
            return nominalErrors.isEmpty() && failedCases().isEmpty();
        }

        /**
         * The ERRORs that appear only because the robot was moved: those of a case whose rule and step the start itself does not have. These are the ones
         * that say "the margin is too small"; an ERROR the start already has is not caused by the placement.
         */
        public List<Issue> newErrors() {
            Set<String> known = new HashSet<String>();
            for (Issue i : nominalErrors) {
                known.add(i.rule + "/" + i.stepIndex);
            }
            List<Issue> out = new ArrayList<Issue>();
            Set<String> seen = new HashSet<String>();
            for (Case c : cases) {
                for (Issue i : c.errors) {
                    String k = i.rule + "/" + i.stepIndex;
                    if (!known.contains(k) && seen.add(k)) {
                        out.add(i);
                    }
                }
            }
            return out;
        }

        public double minSeconds() {
            double m = nominalSeconds;
            for (Case c : cases) {
                m = Math.min(m, c.seconds);
            }
            return m;
        }

        public double maxSeconds() {
            double m = nominalSeconds;
            for (Case c : cases) {
                m = Math.max(m, c.seconds);
            }
            return m;
        }

        /** How much the time estimate moves between the best and the worst way of setting the robot down, seconds. */
        public double spreadSeconds() {
            return maxSeconds() - minSeconds();
        }

        /** One line for people: how many placements were tried, how many failed, and the time spread. */
        public String summary() {
            return String.format(Locale.ROOT, "%d placements tried, %d with an ERROR (%d new), time %.1f to %.1f s (spread %.2f s)",
                    cases.size(), failedCases().size(), newErrors().size(), minSeconds(), maxSeconds(), spreadSeconds());
        }
    }

    // ---- how a start is moved -----------------------------------------------------------------------------------------

    /**
     * Every different place the robot can end up when it is set down {@code marginIn} off in x and in y (either way, or not at all) and {@code marginDeg}
     * turned (either way, or not at all), pushed back against the walls the start touches. The start itself and places that come to the same pose
     * are left out.
     */
    public static List<Move> moves(AllianceField field, Pose2 start, double marginIn, double marginDeg) {
        double size = field.fieldRect().maxX;
        Rect home = Footprint.of(start).bounds();
        List<Wall> touched = new ArrayList<Wall>();
        for (Wall wall : Wall.values()) {
            if (wall.gapIn(home, size) <= BioBuzzConstants.Auto.START_WALL_CONTACT_TOLERANCE_IN) {
                touched.add(wall);
            }
        }
        List<Move> out = new ArrayList<Move>();
        Set<String> seen = new HashSet<String>();
        seen.add(key(start));
        for (int ix = -1; ix <= 1; ix++) {
            for (int iy = -1; iy <= 1; iy++) {
                for (int ih = -1; ih <= 1; ih++) {
                    if (ix == 0 && iy == 0 && ih == 0) {
                        continue;
                    }
                    Pose2 moved = new Pose2(start.x + ix * marginIn, start.y + iy * marginIn, start.heading + Math.toRadians(ih * marginDeg));
                    for (Wall wall : touched) {
                        moved = wall.seated(moved, Footprint.of(moved).bounds(), size);
                    }
                    if (seen.add(key(moved))) {
                        out.add(new Move(ix * marginIn, iy * marginIn, ih * marginDeg, moved));
                    }
                }
            }
        }
        return Collections.unmodifiableList(out);
    }

    /** {@link #moves} with the placement margin of {@code BioBuzzConstants.Auto}. */
    public static List<Move> moves(AllianceField field, Pose2 start) {
        return moves(field, start, BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN, BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_DEG);
    }

    /**
     * The quick half of the placement check, without any route: True if the start, and the start moved every way of {@link #moves}, is legal and clear
     * by {@link StartRules}. A start next to a FLOWER, the LOADING ZONE or the center line that fails this is too close to it to be placed by hand.
     */
    public static boolean startsStayLegal(AllianceField field, Pose2 start, double marginIn, double marginDeg) {
        if (!StartRules.isLegalAndClear(field, start)) {
            return false;
        }
        for (Move m : moves(field, start, marginIn, marginDeg)) {
            if (!StartRules.isLegalAndClear(field, m.start)) {
                return false;
            }
        }
        return true;
    }

    // ---- the check --------------------------------------------------------------------------------------------------------

    /** The check for a finished route, with the placement margin and the default timing and validator. */
    public static Report check(AutoRoute route) {
        return check(route, new TimingModel());
    }

    /** As {@link #check(AutoRoute)} for a robot with other timing and wiggle numbers. */
    public static Report check(AutoRoute route, TimingModel timing) {
        return check(RouteTemplate.keeping(route), route.startPose(), route.alliance(), timing, BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN,
                BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_DEG);
    }

    /**
     * The check for a template started at {@code start} (in the frame of {@code alliance}): the route is built and validated for the start itself and for
     * each way the robot can be set down off it, and the ERRORs and the times are collected.
     */
    public static Report check(RouteTemplate template, Pose2 start, Alliance alliance, TimingModel timing, double marginIn, double marginDeg) {
        FieldModel model = new FieldModel();
        RouteValidator validator = new RouteValidator(model, timing);
        AllianceField field = new AllianceField(model, alliance);
        AutoRoute home = template.build(start);
        List<Case> cases = new ArrayList<Case>();
        for (Move move : moves(field, start, marginIn, marginDeg)) {
            AutoRoute route = template.build(move.start);
            cases.add(new Case(move, errorsOf(validator, route), RouteAnalysis.of(route, model, timing).totalSeconds()));
        }
        return new Report(start, errorsOf(validator, home), RouteAnalysis.of(home, model, timing).totalSeconds(), cases);
    }

    /**
     * The quick yes or no of {@link #check}: True if the route is valid from the start and from every way the robot can be set down off it. It stops at the first
     * ERROR it meets, so it is much faster than {@link #check} for a start that fails; the search for the best start asks it for every candidate. Use {@link #check}
     * for the report.
     */
    public static boolean passes(RouteTemplate template, Pose2 start, Alliance alliance, TimingModel timing, double marginIn, double marginDeg) {
        FieldModel model = new FieldModel();
        RouteValidator validator = new RouteValidator(model, timing);
        if (validator.firstError(template.build(start)) != null) {
            return false;
        }
        for (Move move : moves(new AllianceField(model, alliance), start, marginIn, marginDeg)) {
            if (validator.firstError(template.build(move.start)) != null) {
                return false;
            }
        }
        return true;
    }

    private static List<Issue> errorsOf(RouteValidator validator, AutoRoute route) {
        List<Issue> out = new ArrayList<Issue>();
        for (Issue i : validator.validate(route)) {
            if (i.isError()) {
                out.add(i);
            }
        }
        return out;
    }

    private static String key(Pose2 p) {
        return String.format(Locale.ROOT, "%.6f/%.6f/%.6f/%.6f", p.x, p.y, Math.cos(p.heading), Math.sin(p.heading));
    }
}
