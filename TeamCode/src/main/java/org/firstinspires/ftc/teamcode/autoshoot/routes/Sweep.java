package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * Checks a rule against EVERY pose of a leg, not just the sampled ones, and can prove it did not miss anything.
 *
 * <p><b>The problem.</b> A computer cannot test infinitely many poses, so the validator tests a list of samples:
 * one every {@code PATH_SAMPLE_STEP_IN} inches along the path and one every {@code TURN_SAMPLE_STEP_DEG} degrees of
 * turning. A thin obstacle (a post, the edge of a wall) can sit in the strip of floor the robot sweeps over between
 * two samples without being inside the robot at either of them. The corners are the danger: the robot's corner is
 * 13.16 in from its center of rotation, so a 5 degree turn moves it 1.15 in.
 *
 * <p><b>The guarantee.</b> Call a pose's <i>depth</i> how far the robot is into the thing it must stay out of
 * (a wall, the HIVE keep-out, a FLOWER plate); a NEGATIVE depth means "clear by that many inches", and the rule is
 * broken when the depth is above 0. Take two neighbouring samples A and B and any pose Q on the route between them.
 * <ol>
 *   <li>Every point of the robot is within {@code R} of its rotation center, where {@code R} is
 *       {@link Footprint#circumscribedRadiusIn()} (the farthest corner, 13.16 in). Sliding the center by {@code d} moves every
 *       point by {@code d}; turning the robot by an angle {@code t} (radians) moves a point at radius {@code <= R} along an
 *       arc of {@code <= R * t}. So no point of the robot, corners included, moves more than
 *       <br>{@code travel(A, Q) = d(A, Q) + R * t(A, Q)}<br> between A and Q.</li>
 *   <li>The sampler cuts the path until neighbouring samples are at most {@code PATH_SAMPLE_STEP_IN} apart and turn at
 *       most {@code TURN_SAMPLE_STEP_DEG} (see {@link DrivePath#sampleArcLengths(double, double)}), so for neighbours
 *       {@code travel(A, B) <= T = PATH_SAMPLE_STEP_IN + R * TURN_SAMPLE_STEP_DEG(in radians)}, which is
 *       {@link #maxTravelBetweenSamples()}: 1 + 13.16 * 0.0873 = 2.15 in. (While turning in place only the second term
 *       counts; while driving straight with a fixed heading only the first.)</li>
 *   <li>The depth is <b>1-Lipschitz</b>: if every point of the robot moves by at most {@code m}, the depth changes by at
 *       most {@code m}. It is true for every depth used here, because each is a plain distance (the signed gap between
 *       two shapes, or how far the worst corner is past a wall) and a distance cannot change faster than its end points
 *       move. So {@code depth(Q) <= depth(A) + travel(A, Q)} and {@code depth(Q) <= depth(B) + travel(Q, B)}.</li>
 *   <li>The smaller of two numbers is at most their average, and {@code travel(A, Q) + travel(Q, B) <= travel(A, B)}
 *       (the robot moves one way between two samples; the sampler guarantees the heading does not swing out and back),
 *       so <br>{@code depth(Q) <= (depth(A) + depth(B) + travel(A, B)) / 2}.</li>
 * </ol>
 * In words: <b>grow the obstacle by half the largest distance a corner can travel between samples</b>; if neither sample
 * reaches the grown obstacle, nothing between them can touch the real one. That is the whole proof, and it is
 * what the code below does: when the bound is above 0 the gap is <i>not</i> proven clear, so it is cut in half and each
 * half is tested the same way with its own, smaller travel. The cutting continues only around trouble and stops at
 * {@code SWEEP_RESOLUTION_IN} (1/1000 in). The answer is therefore the same as sampling infinitely densely: what is
 * reported is what the robot really does, to within that resolution, while a leg that is far from every obstacle costs
 * just its coarse samples.
 *
 * <p>Both ends of every leg are always tested (they are the first and last coarse samples), so a robot that merely
 * ends its leg in the wrong place is never missed.
 */
final class Sweep {
    /** Numeric guards, not tuning values. */
    private static final double TINY = 1e-9;
    /** Findings of the later phase of a leg (the drive) always sort after those of the earlier one (the turn). */
    private static final double PHASE_SPAN = 1e6;

    private Sweep() {
    }

    /** One continuous movement of the robot, described by a parameter {@code u}. */
    interface Motion {
        /** The robot pose at parameter u (arc length in inches for a drive, radians turned for a turn in place). */
        Pose2 poseAt(double u);

        /** The most any point of the robot moves between parameters u0 and u1, in inches. */
        double travel(double u0, double u1);

        /** The distance along the path at parameter u, for reports (0 for a turn in place). */
        double arc(double u);
    }

    /** How deep the robot is into the thing a rule keeps it out of: above 0 = rule broken, below 0 = clear by that much. */
    interface Depth {
        double at(Pose2 pose);
    }

    /** Everything found while sweeping one rule over a leg. */
    static final class Result {
        /** Number of tested poses that break the rule (coarse samples and the poses the cutting added). */
        int count;
        /** Distance along the leg of the first and of the last broken pose, in inches. */
        double firstArc = Double.POSITIVE_INFINITY;
        double lastArc = Double.NEGATIVE_INFINITY;
        /** The first broken pose along the leg (null if none). */
        Pose2 firstPose;
        /** The largest depth found (negative infinity if nothing broke the rule). */
        double worst = Double.NEGATIVE_INFINITY;
        /**
         * True if a gap was cut {@code SWEEP_MAX_CUTS} times and still could not be proven clear: the robot's heading JUMPS there (see
         * {@link #refine}). Nothing was found in it, so it is not counted as a collision, but the leg cannot be followed: the validator reports it.
         */
        boolean unprovable;
        /** The first gap that could not be proven (valid only if {@link #unprovable}): the distance along the leg, and the pose. */
        double unprovableArc;
        Pose2 unprovablePose;

        private double firstKey = Double.POSITIVE_INFINITY;

        void add(double order, double arc, Pose2 pose, double depth) {   // order: sorts findings along the leg
            count++;
            if (order < firstKey) {
                firstKey = order;
                firstArc = arc;
                firstPose = pose;
            }
            lastArc = Math.max(lastArc, arc);
            worst = Math.max(worst, depth);
        }

        boolean broken() {
            return count > 0;
        }
    }

    /**
     * The most any point of the robot can move between two neighbouring samples:
     * {@code PATH_SAMPLE_STEP_IN + circumscribedRadius * TURN_SAMPLE_STEP_DEG}, in inches (derived, see the class comment).
     */
    static double maxTravelBetweenSamples() {
        return BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN
                + Footprint.circumscribedRadiusIn() * Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG);
    }

    /**
     * How far an obstacle is grown for the first, cheap test: half of {@link #maxTravelBetweenSamples()}. A sample
     * at least this far from an obstacle proves the poses next to it are clear too.
     */
    static double obstacleGrowthIn() {
        return maxTravelBetweenSamples() / 2.0;
    }

    // ---- sweeping --------------------------------------------------------------------------------------

    /**
     * Sweeps a rule over the whole of one leg: first the turn in place before it starts (if the robot has to
     * turn), then the drive, including both end poses.
     */
    static Result run(DrivePath leg, Depth depth) {
        Result total = new Result();
        double turn = Math.abs(leg.preTurnSignedRadians());
        if (turn > 0.0) {
            // The first pose of the turn is where the previous step left the robot: that step already reported it.
            run(new TurnMotion(leg), turnParameters(turn), depth, false, 0, total);
        }
        run(new LegMotion(leg), leg.sampleArcLengths(BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN,
                Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG)), depth, true, 1, total);
        return total;
    }

    /** The coarse parameters of a turn: 0, then every TURN_SAMPLE_STEP_DEG, then the full turn. */
    static List<Double> turnParameters(double totalRadians) {
        List<Double> out = new ArrayList<Double>();
        double step = Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG);
        int n = (int) Math.ceil(totalRadians / step - TINY);
        for (int i = 0; i <= n; i++) {
            out.add(Math.min(totalRadians, i * step));
        }
        return out;
    }

    /**
     * Sweeps a rule along one motion. {@code coarse} are the parameters of the samples (ascending, first and last
     * included); {@code reportFirst} says whether a broken first sample is reported (the first pose of a turn belongs
     * to the previous step); {@code phase} (0 for a turn, 1 for the drive after it) orders findings along the leg.
     * Findings are added to {@code into}.
     */
    static void run(Motion motion, List<Double> coarse, Depth depth, boolean reportFirst, int phase, Result into) {
        int n = coarse.size();
        double[] d = new double[n];
        for (int i = 0; i < n; i++) {
            Pose2 pose = motion.poseAt(coarse.get(i));
            d[i] = depth.at(pose);
            if (d[i] > 0.0 && (i > 0 || reportFirst)) {
                into.add(phase * PHASE_SPAN + coarse.get(i), motion.arc(coarse.get(i)), pose, d[i]);
            }
        }
        for (int i = 1; i < n; i++) {
            refine(motion, depth, coarse.get(i - 1), d[i - 1], coarse.get(i), d[i], phase, into, 0);
        }
    }

    /**
     * The proof step: between parameters ua and ub (depths da and db, already known), either show that no pose can
     * break the rule, or cut the gap in half and look at both halves.
     */
    private static void refine(Motion motion, Depth depth, double ua, double da, double ub, double db, int phase,
                               Result into, int cuts) {
        if (into.unprovable) {
            return;                                                 // the leg is already known to be one nothing can follow: no more cutting
        }
        double bound = (da + db + motion.travel(ua, ub)) / 2.0;    // no pose between can be deeper than this
        double known = Math.max(Math.max(da, db), 0.0);            // the deepest the two ends already show
        if (bound <= known + BioBuzzConstants.Auto.SWEEP_RESOLUTION_IN) {
            return;                                                 // proven: nothing worth reporting hides in here
        }
        if (cuts >= BioBuzzConstants.Auto.SWEEP_MAX_CUTS) {
            // A smooth motion is proven long before this. A gap that stays unproven however small it gets means the travel does not shrink with the gap:
            // the heading JUMPS inside it (a cusp, or a path through the point the robot must face). Stop, and say so, instead of cutting for ever.
            if (!into.unprovable) {
                into.unprovable = true;
                into.unprovableArc = motion.arc((ua + ub) / 2.0);
                into.unprovablePose = motion.poseAt((ua + ub) / 2.0);
            }
            return;
        }
        double um = (ua + ub) / 2.0;
        Pose2 pose = motion.poseAt(um);
        double dm = depth.at(pose);
        if (dm > 0.0) {
            into.add(phase * PHASE_SPAN + um, motion.arc(um), pose, dm);
        }
        refine(motion, depth, ua, da, um, dm, phase, into, cuts + 1);
        refine(motion, depth, um, dm, ub, db, phase, into, cuts + 1);
    }

    // ---- the two kinds of motion of a leg -----------------------------------------------------------------

    /** Turning in place at the start point of a leg; the parameter is the angle turned so far, in radians. */
    private static final class TurnMotion implements Motion {
        private final DrivePath leg;
        private final double radius = Footprint.circumscribedRadiusIn();

        TurnMotion(DrivePath leg) {
            this.leg = leg;
        }

        @Override
        public Pose2 poseAt(double u) {
            return leg.turnPoseAt(u);
        }

        @Override
        public double travel(double u0, double u1) {
            return radius * Math.abs(u1 - u0);       // the center stays put: only the corners move
        }

        @Override
        public double arc(double u) {
            return 0.0;
        }
    }

    /** Driving along a leg; the parameter is the distance along the path, in inches. */
    private static final class LegMotion implements Motion {
        private final DrivePath leg;
        private final double radius = Footprint.circumscribedRadiusIn();

        LegMotion(DrivePath leg) {
            this.leg = leg;
        }

        @Override
        public Pose2 poseAt(double u) {
            return leg.poseAt(u);
        }

        @Override
        public double travel(double u0, double u1) {
            double middle = (u0 + u1) / 2.0;
            double h0 = leg.headingAt(u0);
            double hm = leg.headingAt(middle);
            double h1 = leg.headingAt(u1);
            // the turning between the ends, counted through the middle so that a swing out and back is not hidden
            double turned = Math.max(Math.abs(Angles.error(h0, h1)), Math.abs(Angles.error(h0, hm)) + Math.abs(Angles.error(hm, h1)));
            return Math.abs(u1 - u0) + radius * turned;
        }

        @Override
        public double arc(double u) {
            return u;
        }
    }
}
