package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * The ONE function that decides which flat wheel does a FLOWER pick-up: {@link #choose}. The validator, the diagram renderer, the timing model and
 * (later) the robot's route runner all get their answer from here (through {@code RouteAnalysis}, which calls it once per step), so they can never
 * disagree about the wheel, the poses or the time.
 *
 * <p><b>Why a choice.</b> Either flat wheel can do the wiggle: the robot stands off center with the wheel beside the ball. The two options are mirror images
 * of each other along the wall, so which is better depends on where the robot comes from and where it goes next: coming from the left, standing off
 * to the left saves driving across the FLOWER. A route does not have to say; {@link IntakeWheel#AUTO}, the default, picks, and LEFT or RIGHT forces it.
 *
 * <p><b>How it chooses.</b> For each wheel it plans the whole pick-up ({@link FlowerPress}) and then
 * <ol>
 *   <li>keeps it only if it is LEGAL: the wheel can reach the ball, and no rule is broken at any pose of the move to the line-up pose, the approach,
 *       the swing (the chassis keeps its clearance off the plate, the wheel stays short of the stop line, nothing leaves the field), the retreat, or by
 *       the HIVE keep-out and the center line (G402). This uses the same {@link LegSweeps} the validator uses;</li>
 *   <li>among the legal ones prefers the cheapest: <b>cost = the move from where the robot is to the line-up pose + the pick-up itself (approach, wiggle,
 *       retreat) + the move from the retreat pose to the first pose of the next step</b> (the one step after it is looked at, see {@link Lookahead});</li>
 *   <li>on a tie (the same time to within a millionth of a second) prefers the one with more room (the smallest gap to a wall, the HIVE keep-out or the
 *       center line over the whole pick-up), and then the LEFT wheel, so the answer never depends on luck.</li>
 * </ol>
 * If neither wheel is legal it still returns the cheapest one, marked not legal, so that the validator can report exactly what is wrong with it.
 *
 * <p>Pure Java (no FTC or Pedro imports), so a PC can run it, and so the runner on the robot can call the same function with the robot's real pose.
 */
public final class WheelChoice {
    /** Two costs closer than this (seconds) are the same cost: a rounding guard, not a tuning value. */
    private static final double TIE_SEC = 1e-6;
    /** Two clearances closer than this (inches) are the same clearance. */
    private static final double TIE_IN = 1e-6;
    /** Positions closer than this (inches) are the same place. */
    private static final double SAME_SPOT_IN = 1e-6;
    /** Headings closer than this (radians) are the same heading. */
    private static final double SAME_HEADING_RAD = 1e-9;

    private WheelChoice() {
    }

    // ---------------------------------------------------------------------------------------------------------------
    // what happens after the pick-up
    // ---------------------------------------------------------------------------------------------------------------

    /**
     * What the robot does right after the pick-up, as far as the cost of the wheel choice is concerned: the leg from the retreat pose to the first pose of
     * the next step. Only that next step is looked at, not the whole rest of the route.
     */
    public static final class Lookahead {
        private enum Kind { NONE, DRIVE, POSE, FLOWER }

        private final Kind kind;
        private final Step.Drive drive;
        private final Pose2 pose;
        private final Step.Intake flower;

        private Lookahead(Kind kind, Step.Drive drive, Pose2 pose, Step.Intake flower) {
            this.kind = kind;
            this.drive = drive;
            this.pose = pose;
            this.flower = flower;
        }

        /** There is no next step that moves the robot: the retreat pose costs nothing more. */
        public static Lookahead none() {
            return new Lookahead(Kind.NONE, null, null, null);
        }

        /** The next step is a DRIVE: its path starts at the retreat pose, so its length and turning depend on the wheel chosen. */
        public static Lookahead drive(Step.Drive next) {
            return new Lookahead(Kind.DRIVE, next, null, null);
        }

        /** The next step moves the robot to this pose (a SHOOT, a PARK or a GARDEN or floor INTAKE). */
        public static Lookahead pose(Pose2 next) {
            return new Lookahead(Kind.POSE, null, next, null);
        }

        /**
         * The next step is another FLOWER pick-up: the robot goes to its line-up pose, and when that step may choose its wheel the nearer one is used
         * (no deeper look-ahead: it would only be a guess about a guess).
         */
        public static Lookahead flower(Step.Intake next) {
            return new Lookahead(Kind.FLOWER, null, null, next);
        }

        /** Seconds the leg from {@code retreat} to the next step's first pose takes. */
        double seconds(Pose2 retreat, AllianceField field, TimingModel timing) {
            switch (kind) {
                case DRIVE:
                    return timing.legSeconds(DrivePath.of(retreat, drive), drive.speedFraction());
                case POSE:
                    return timing.moveSeconds(retreat, pose);
                case FLOWER: {
                    int f = field.nearestFlower(flower.target());
                    if (f < 0) {
                        return 0.0;
                    }
                    double best = Double.POSITIVE_INFINITY;
                    for (IntakeWheel w : wheelsToTry(flower.wheel())) {
                        FlowerPress next = FlowerPress.plan(field, timing.flowerSpec(), f, w, flower.wheel(), flower.expectedCount(),
                                flower.retreatLengthIn());
                        best = Math.min(best, timing.moveSeconds(retreat, next.lineUp));
                    }
                    return best;
                }
                default:
                    return 0.0;
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // the answer
    // ---------------------------------------------------------------------------------------------------------------

    /** One wheel considered: its pick-up, whether it is legal and what it would cost. */
    public static final class Option {
        /** LEFT or RIGHT. */
        public final IntakeWheel wheel;
        /** The whole pick-up planned for this wheel. */
        public final FlowerPress press;
        /** True if the wheel reaches the ball and no rule is broken anywhere on the move, approach, swing or retreat. */
        public final boolean legal;
        /** If not legal: the first thing wrong, in plain words. Empty when legal. */
        public final String problem;
        /** Seconds from the robot's pose to the line-up pose (0 if the pose is not known or the robot is already there). */
        public final double moveInSec;
        /** Seconds of the pick-up itself: the slow approach, the wiggle and the slow retreat. */
        public final double pickupSec;
        /** Seconds from the retreat pose to the first pose of the next step. */
        public final double moveOutSec;
        /** The smallest gap to a wall, the HIVE keep-out or the center line over the whole pick-up, in inches (bigger is roomier). */
        public final double clearanceIn;

        Option(IntakeWheel wheel, FlowerPress press, boolean legal, String problem, double moveInSec, double pickupSec, double moveOutSec,
               double clearanceIn) {
            this.wheel = wheel;
            this.press = press;
            this.legal = legal;
            this.problem = problem;
            this.moveInSec = moveInSec;
            this.pickupSec = pickupSec;
            this.moveOutSec = moveOutSec;
            this.clearanceIn = clearanceIn;
        }

        /** The cost the choice compares: the move in, the pick-up and the move out, in seconds. */
        public double costSec() {
            return moveInSec + pickupSec + moveOutSec;
        }
    }

    /** The result of {@link #choose}: every option looked at and the one picked. */
    public static final class Choice {
        /** What the route asked for: AUTO, LEFT or RIGHT. */
        public final IntakeWheel requested;
        private final List<Option> options;
        private final Option chosen;

        Choice(IntakeWheel requested, List<Option> options, Option chosen) {
            this.requested = requested;
            this.options = Collections.unmodifiableList(options);
            this.chosen = chosen;
        }

        /** The options considered, LEFT first: both wheels for AUTO, the one wheel that was asked for otherwise. */
        public List<Option> options() {
            return options;
        }

        /** The option picked (the best legal one; if none is legal, the cheapest, which {@link Option#legal} says is not legal). */
        public Option chosen() {
            return chosen;
        }

        /** The pick-up of the chosen wheel. */
        public FlowerPress press() {
            return chosen.press;
        }

        /** The wheel picked: LEFT or RIGHT. */
        public IntakeWheel wheel() {
            return chosen.wheel;
        }

        /** True if at least one option is legal. When this is false the route cannot pick up from this FLOWER with the numbers it has. */
        public boolean anyLegal() {
            for (Option o : options) {
                if (o.legal) {
                    return true;
                }
            }
            return false;
        }

        /** The other legal option, or null if there is none (only one wheel was asked for, or the other is not legal). */
        public Option otherLegal() {
            for (Option o : options) {
                if (o != chosen && o.legal) {
                    return o;
                }
            }
            return null;
        }

        /** The option that was not picked, or null if only one wheel was considered. */
        public Option other() {
            for (Option o : options) {
                if (o != chosen) {
                    return o;
                }
            }
            return null;
        }

        /**
         * How many seconds cheaper the chosen wheel is than the other legal one, or NaN if there is no other legal wheel to compare with. Zero or
         * tiny when the two cost the same.
         */
        public double savedSec() {
            Option o = otherLegal();
            return o == null || !chosen.legal ? Double.NaN : o.costSec() - chosen.costSec();
        }

        /**
         * Why this wheel, in plain words for the gallery: "chosen automatically: 1.4 s shorter than the RIGHT wheel", "the only legal wheel (...)",
         * "asked for by the route", or why neither wheel is legal.
         */
        public String reason() {
            if (!requested.isAuto()) {
                return "the route asked for the " + chosen.wheel + " wheel" + (chosen.legal ? "" : ", but it is not legal: " + chosen.problem);
            }
            if (!anyLegal()) {
                // the two wheels are mirror images, so usually they fail for the same reason: say it once
                boolean same = true;
                for (Option o : options) {
                    same &= o.problem.replace(o.wheel.name(), "flat").equals(options.get(0).problem.replace(options.get(0).wheel.name(), "flat"));
                }
                if (same) {
                    return "neither wheel is legal: " + options.get(0).problem.replace(options.get(0).wheel.name(), "flat");
                }
                StringBuilder sb = new StringBuilder("neither wheel is legal");
                for (Option o : options) {
                    sb.append("; ").append(o.wheel).append(": ").append(o.problem);
                }
                return sb.toString();
            }
            Option o = other();
            if (o == null) {
                return "chosen automatically";
            }
            if (!o.legal) {
                return "the " + chosen.wheel + " wheel is the only legal one (the " + o.wheel + " wheel: " + o.problem + ")";
            }
            double saved = savedSec();
            if (saved > TIE_SEC) {
                return "chosen automatically: " + String.format(Locale.ROOT, "%.1f", saved) + " s shorter than the " + o.wheel + " wheel";
            }
            if (chosen.clearanceIn > o.clearanceIn + TIE_IN) {
                return "chosen automatically: the same time as the " + o.wheel + " wheel but more room around the robot";
            }
            return "chosen automatically: the " + o.wheel + " wheel is just as good, " + chosen.wheel + " wins a tie";
        }
    }

    /** The wheels a request stands for: LEFT then RIGHT for AUTO, the one wheel otherwise. */
    static IntakeWheel[] wheelsToTry(IntakeWheel requested) {
        if (requested == null || requested.isAuto()) {
            return IntakeWheel.physical();
        }
        return new IntakeWheel[] {requested};
    }

    // ---------------------------------------------------------------------------------------------------------------
    // choosing
    // ---------------------------------------------------------------------------------------------------------------

    /**
     * Chooses the wheel for the FLOWER pick-up {@code step} at FLOWER {@code flower}, for a robot that is at {@code from} (null: not known, so the move to the
     * line-up pose costs nothing and is not checked) and whose next step is {@code next}. See the class comment for the rules.
     *
     * @param field  the field of the alliance the route is for (its sides and keep-outs)
     * @param timing the timing numbers (and the wiggle numbers, {@link TimingModel#flowerSpec()})
     */
    public static Choice choose(AllianceField field, TimingModel timing, Step.Intake step, int flower, Pose2 from, Lookahead next) {
        IntakeWheel requested = step.wheel() == null ? IntakeWheel.AUTO : step.wheel();
        List<Option> options = new ArrayList<Option>();
        for (IntakeWheel w : wheelsToTry(requested)) {
            options.add(evaluate(field, timing, step, flower, w, requested, from, next));
        }
        Option best = null;
        for (Option o : options) {
            if (best == null || better(o, best)) {
                best = o;
            }
        }
        return new Choice(requested, options, best);
    }

    /** True if option a beats option b: legal beats not legal, then cheaper, then roomier; an exact tie keeps b (the one tried first, the LEFT wheel). */
    private static boolean better(Option a, Option b) {
        if (a.legal != b.legal) {
            return a.legal;
        }
        if (Math.abs(a.costSec() - b.costSec()) > TIE_SEC) {
            return a.costSec() < b.costSec();
        }
        if (Math.abs(a.clearanceIn - b.clearanceIn) > TIE_IN) {
            return a.clearanceIn > b.clearanceIn;
        }
        return false;
    }

    private static Option evaluate(AllianceField field, TimingModel timing, Step.Intake step, int flower, IntakeWheel wheel, IntakeWheel requested,
                                   Pose2 from, Lookahead next) {
        FlowerPress press = FlowerPress.plan(field, timing.flowerSpec(), flower, wheel, requested, step.expectedCount(), step.retreatLengthIn());
        String problem = "";
        if (!press.reaches()) {
            problem = "the wheel cannot reach the POLLEN: " + press.solution.shortfallText();
        }
        // every leg of the pick-up, with the plate of THIS FLOWER judged by the pick-up rules; the move in is the whole robot against every plate
        List<LegSweeps> sweeps = new ArrayList<LegSweeps>();
        DrivePath moveIn = from == null ? null : moveBetween(from, press.lineUp);
        if (moveIn != null) {
            sweeps.add(LegSweeps.run(field, moveIn, null, false));
        }
        sweeps.add(LegSweeps.run(field, press.approach, press, false));
        sweeps.add(LegSweeps.run(field, press.swing, press, false));
        sweeps.add(LegSweeps.run(field, press.retreatPath, press, false));
        for (LegSweeps s : sweeps) {
            if (problem.isEmpty() && s.broken()) {
                problem = s.firstProblem();
            }
        }
        double moveInSec = from == null ? 0.0 : timing.moveSeconds(from, press.lineUp);
        return new Option(wheel, press, problem.isEmpty(), problem, moveInSec, timing.flowerSeconds(press),
                next.seconds(press.retreat, field, timing), clearance(field, press));
    }

    /** The straight move from one pose to another, or null if the robot is already there (same spot, same heading). */
    static DrivePath moveBetween(Pose2 from, Pose2 to) {
        boolean sameSpot = from.distanceTo(to) < SAME_SPOT_IN;
        boolean sameHeading = Math.abs(Angles.error(from.heading, to.heading)) < SAME_HEADING_RAD;
        return sameSpot && sameHeading ? null : DrivePath.straight(from, to);
    }

    /**
     * How much room the pick-up leaves around the robot: the smallest gap, over the poses of the approach, the swing and the retreat, between the whole
     * footprint and the nearest of a wall, the HIVE keep-out and the center line (G402). Bigger is roomier; below 0 would be a collision.
     */
    private static double clearance(AllianceField field, FlowerPress press) {
        double least = Double.POSITIVE_INFINITY;
        List<Pose2> poses = new ArrayList<Pose2>();
        poses.addAll(press.approach.sample(BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN));
        poses.addAll(press.swing.turnInPlaceSamples(Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG)));
        poses.addAll(press.retreatPath.sample(BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN));
        for (Pose2 p : poses) {
            Footprint fp = Footprint.of(p);
            least = Math.min(least, -fp.signedOvershoot(field.fieldRect()));
            least = Math.min(least, fp.signedGap(field.hiveKeepOut()));
            least = Math.min(least, -field.pastCenterLine(p.x));
        }
        return least;
    }
}
