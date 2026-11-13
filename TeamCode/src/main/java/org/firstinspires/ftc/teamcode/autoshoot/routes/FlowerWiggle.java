package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

/**
 * The geometry of the FLOWER WIGGLE, worked out from the robot and the FLOWER so that a route author never computes it by hand.
 *
 * <h3>What the robot does</h3>
 * The bottom POLLEN of a FLOWER sits in a ring hole and has to be rolled up over the ring's lip. The two flat intake wheels are 12.2 in
 * apart, so ONE of them does it, with a small oscillation:
 * <ol>
 *   <li><b>Approach.</b> The robot drives straight at the wall (heading perpendicular to it) standing OFF CENTER, so that the pressing wheel
 *       is beside the ball: its middle is {@code Auto.FLOWER_WHEEL_START_OFFSET_IN} from the ball's middle along the wall.</li>
 *   <li><b>Push.</b> It turns on the spot about its pose point (its center of rotation) toward the ball. The wheel is far from that point, so
 *       a few degrees swing it sideways, and a little forward, into the ball until the middle of the wheel is
 *       {@code Auto.FLOWER_WHEEL_PRESS_OFFSET_IN} from the middle of the ball (the wheel overlaps it). The spinning wheel and the push of the whole
 *       robot roll the ball over the lip.</li>
 *   <li><b>Release.</b> It turns back the same angle, never past straight, so the next ball can drop into the hole. Then it repeats, and at the end
 *       backs straight out.</li>
 * </ol>
 *
 * <h3>What is derived</h3>
 * <ul>
 *   <li><b>The turn angle.</b> The wheel's middle is {@code lever arm} = {@code hypot(forward distance of the wheel, its sideways offset)}
 *       (10.4 in) from the pose point, so as the robot turns it travels on a circle of that radius. The angle is where that circle first
 *       gets within the press offset of the ball's middle (the meeting point of two circles, solved exactly). Nobody types it in.</li>
 *   <li><b>The standoff.</b> How far from the wall the pose point stands. The turn swings the chassis corner that is nearest the plate toward it, so
 *       the robot must stand farther out than a straight press would need; the standoff is the SMALLEST distance for which the chassis keeps its
 *       clearance off the plate at EVERY pose of the swing, the pressing wheel never goes past the POLLEN's center line, and nothing goes through
 *       the wall. It is found with the same {@link PressRules} the validator uses, so the validator accepts it by construction.</li>
 *   <li><b>Whether the wheel can still reach.</b> Standing farther out costs reach. If the wheel cannot get within the press offset of the ball at any
 *       standoff that keeps the chassis clear (or would have to turn more than {@code Auto.FLOWER_WIGGLE_MAX_TURN_DEG}), the pick-up does not work:
 *       the {@link Solution} says so and says by how many inches the wheels would have to stick out farther ({@link Solution#missingReachIn}).</li>
 * </ul>
 *
 * <h3>The frame the maths is done in</h3>
 * The four FLOWERS are the same plate on four walls, so everything is derived once in a "wall frame": the wall is the line x = 0, x is the
 * distance from the wall (field side positive), y is the distance along the wall from the ball's middle line, and a heading of 180 degrees
 * means "facing the wall". {@link FlowerPress} turns that solution into field poses for any FLOWER. The result is cached per (wheel, numbers).
 *
 * <p>Pure Java (no FTC or Pedro imports), so a PC can run it.
 */
public final class FlowerWiggle {
    /** How finely the search for the standoff steps outward from the wall, in inches (the exact edge is then found by halving). */
    private static final double SCAN_STEP_IN = 0.05;
    /** The pose the derivation returns is this much (inches) clearer of every rule than the bare minimum, so the validator's own rounding never objects. */
    private static final double DERIVE_MARGIN_IN = 1e-6;
    /** Halvings used to home in on the edge of the legal standoffs: 2^-50 of a scan step is far below anything that matters. */
    private static final int HALVINGS = 50;
    /** Halvings used to home in on the missing wheel reach: a range of a few inches halved 20 times is known to within a hundred-thousandth of an inch. */
    private static final int REACH_HALVINGS = 20;
    /** How many equal steps the quick first look at a swing divides it into (so it looks at the start, the middle and the end). */
    private static final int QUICK_POSES = 2;
    /** The most solutions kept in the cache: a run uses a handful, so this only stops a test that tries thousands of numbers from growing it forever. */
    private static final int CACHE_LIMIT = 256;
    private static final Map<Key, Solution> CACHE = new HashMap<Key, Solution>();

    private FlowerWiggle() {
    }

    // ---------------------------------------------------------------------------------------------------------------
    // the numbers the geometry depends on
    // ---------------------------------------------------------------------------------------------------------------

    /**
     * Every number of the wiggle that is an estimate (an ADD or TUNE value), gathered in one object so that a test, or a "what if the wheels were
     * longer?" question, can change one without touching the constants. {@link #fromConstants()} is the real robot. Immutable: the
     * {@code with...} methods return a changed copy.
     */
    public static final class Spec {
        /** How far (inches) the flat wheels stick out in front of the chassis frame: {@code Robot.INTAKE_REACH_IN}. */
        public final double intakeReachIn;
        /** The gap (inches) the chassis keeps from the FLOWER plate during the whole swing: {@code Auto.FLOWER_CHASSIS_CLEARANCE_IN}. */
        public final double chassisClearanceIn;
        /** The distance (inches) between the middle of the wheel and the middle of the ball when the wiggle starts: {@code Auto.FLOWER_WHEEL_START_OFFSET_IN}. */
        public final double startOffsetIn;
        /** The distance (inches) between the middle of the wheel and the middle of the ball at the end of the push: {@code Auto.FLOWER_WHEEL_PRESS_OFFSET_IN}. */
        public final double pressOffsetIn;
        /** True if the ball starts between the wheel and the robot's center line: {@code Auto.FLOWER_WHEEL_BALL_INBOARD}. */
        public final boolean ballInboard;
        /** The biggest turn (degrees) the push may use: {@code Auto.FLOWER_WIGGLE_MAX_TURN_DEG}. */
        public final double maxTurnDeg;
        /** Push-and-release cycles for one POLLEN: {@code Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN}. */
        public final int cyclesPerPollen;
        /** The most cycles one pick-up may plan: {@code Auto.FLOWER_WIGGLE_MAX_CYCLES}. */
        public final int maxCycles;

        private Spec(double intakeReachIn, double chassisClearanceIn, double startOffsetIn, double pressOffsetIn, boolean ballInboard,
                     double maxTurnDeg, int cyclesPerPollen, int maxCycles) {
            this.intakeReachIn = intakeReachIn;
            this.chassisClearanceIn = chassisClearanceIn;
            this.startOffsetIn = startOffsetIn;
            this.pressOffsetIn = pressOffsetIn;
            this.ballInboard = ballInboard;
            this.maxTurnDeg = maxTurnDeg;
            this.cyclesPerPollen = cyclesPerPollen;
            this.maxCycles = maxCycles;
        }

        /** The numbers of {@link BioBuzzConstants} (the robot as it is believed to be). */
        public static Spec fromConstants() {
            return new Spec(BioBuzzConstants.Robot.INTAKE_REACH_IN, BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN,
                    BioBuzzConstants.Auto.FLOWER_WHEEL_START_OFFSET_IN, BioBuzzConstants.Auto.FLOWER_WHEEL_PRESS_OFFSET_IN,
                    BioBuzzConstants.Auto.FLOWER_WHEEL_BALL_INBOARD, BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_TURN_DEG,
                    BioBuzzConstants.Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN, BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_CYCLES);
        }

        public Spec withIntakeReach(double inches) {
            return new Spec(inches, chassisClearanceIn, startOffsetIn, pressOffsetIn, ballInboard, maxTurnDeg, cyclesPerPollen, maxCycles);
        }

        public Spec withChassisClearance(double inches) {
            return new Spec(intakeReachIn, inches, startOffsetIn, pressOffsetIn, ballInboard, maxTurnDeg, cyclesPerPollen, maxCycles);
        }

        public Spec withStartOffset(double inches) {
            return new Spec(intakeReachIn, chassisClearanceIn, inches, pressOffsetIn, ballInboard, maxTurnDeg, cyclesPerPollen, maxCycles);
        }

        public Spec withPressOffset(double inches) {
            return new Spec(intakeReachIn, chassisClearanceIn, startOffsetIn, inches, ballInboard, maxTurnDeg, cyclesPerPollen, maxCycles);
        }

        public Spec withBallInboard(boolean inboard) {
            return new Spec(intakeReachIn, chassisClearanceIn, startOffsetIn, pressOffsetIn, inboard, maxTurnDeg, cyclesPerPollen, maxCycles);
        }

        public Spec withMaxTurnDeg(double degrees) {
            return new Spec(intakeReachIn, chassisClearanceIn, startOffsetIn, pressOffsetIn, ballInboard, degrees, cyclesPerPollen, maxCycles);
        }

        public Spec withCycles(int perPollen, int max) {
            return new Spec(intakeReachIn, chassisClearanceIn, startOffsetIn, pressOffsetIn, ballInboard, maxTurnDeg, perPollen, max);
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Spec)) {
                return false;
            }
            Spec s = (Spec) o;
            return intakeReachIn == s.intakeReachIn && chassisClearanceIn == s.chassisClearanceIn && startOffsetIn == s.startOffsetIn
                    && pressOffsetIn == s.pressOffsetIn && ballInboard == s.ballInboard && maxTurnDeg == s.maxTurnDeg
                    && cyclesPerPollen == s.cyclesPerPollen && maxCycles == s.maxCycles;
        }

        @Override
        public int hashCode() {
            return java.util.Arrays.hashCode(new double[] {intakeReachIn, chassisClearanceIn, startOffsetIn, pressOffsetIn,
                    ballInboard ? 1.0 : 0.0, maxTurnDeg, cyclesPerPollen, maxCycles});
        }
    }

    /**
     * How many push-and-release cycles a pick-up of {@code pollen} POLLEN makes: {@code Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN} for each, but at
     * least one and never more than {@code Auto.FLOWER_WIGGLE_MAX_CYCLES}.
     */
    public static int cyclesFor(Spec spec, int pollen) {
        return Math.min(spec.maxCycles, Math.max(1, pollen * spec.cyclesPerPollen));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // the answer
    // ---------------------------------------------------------------------------------------------------------------

    /**
     * The wiggle of one wheel, in the wall frame (see the class comment): where the robot stands, how far it turns and whether the wheel reaches.
     * The robot starts the swing at {@code (standoffIn, lateralIn)} facing the wall (heading 180 degrees) and turns by {@code turnRad}.
     */
    public static final class Solution {
        /** The real wheel (LEFT or RIGHT) this solution is for. */
        public final IntakeWheel wheel;
        /** The numbers it was derived from. */
        public final Spec spec;
        /** How far the pose point stands from the wall, in inches: the standoff, swing included. */
        public final double standoffIn;
        /** How far along the wall the pose point is from the ball's middle line, in inches (sign: toward the wall frame's +y). */
        public final double lateralIn;
        /**
         * How far the robot turns in the push, in radians, positive = counter-clockwise seen from above. It is the turn that carries the wheel TOWARD
         * the ball (clockwise for a left wheel with the ball inboard of it, counter-clockwise for a right wheel). When the wheel cannot reach, this is
         * the turn that brings it closest, within the biggest turn allowed.
         */
        public final double turnRad;
        /** True if the wheel gets within the press offset of the ball without breaking a rule: the pick-up works with these numbers. */
        public final boolean reaches;
        /** The closest the middle of the wheel gets to the middle of the ball during the swing, in inches (the press offset when it reaches). */
        public final double closestIn;
        /**
         * How many more inches the flat wheels would have to stick out in front of the chassis for the pick-up to work (0 if it works already);
         * {@link Double#POSITIVE_INFINITY} if no wheel length helps (then the turn limit or the stop line is the problem).
         */
        public final double missingReachIn;
        /** The standoff a straight press-in would need (no swing): the chassis front on its clearance line, or the wheel on the stop line. */
        public final double noSwingStandoffIn;
        /** The distance from the pose point to the middle of the wheel, in inches: the lever arm of the turn. */
        public final double leverArmIn;

        Solution(IntakeWheel wheel, Spec spec, double standoffIn, double lateralIn, double turnRad, boolean reaches, double closestIn,
                 double missingReachIn, double noSwingStandoffIn, double leverArmIn) {
            this.wheel = wheel;
            this.spec = spec;
            this.standoffIn = standoffIn;
            this.lateralIn = lateralIn;
            this.turnRad = turnRad;
            this.reaches = reaches;
            this.closestIn = closestIn;
            this.missingReachIn = missingReachIn;
            this.noSwingStandoffIn = noSwingStandoffIn;
            this.leverArmIn = leverArmIn;
        }

        /** The turn of the push in degrees (positive = counter-clockwise). */
        public double turnDeg() {
            return Math.toDegrees(turnRad);
        }

        /** How much farther from the wall the swing makes the robot stand than a straight press would, in inches. */
        public double swingExtraStandoffIn() {
            return standoffIn - noSwingStandoffIn;
        }

        /** The robot pose where the swing starts (the approach ends), in the wall frame. */
        public Pose2 startPose() {
            return new Pose2(standoffIn, lateralIn, Math.PI);
        }

        /** The robot pose at the end of the push, in the wall frame: the start pose turned by {@link #turnRad}. */
        public Pose2 pushedPose() {
            return new Pose2(standoffIn, lateralIn, Math.PI + turnRad);
        }

        /**
         * In plain words, why the wheel does not reach, with the number the team needs; empty if it does. Used by the validator and the gallery.
         */
        public String shortfallText() {
            if (reaches) {
                return "";
            }
            String longer = Double.isInfinite(missingReachIn)
                    ? "no length of wheel would help: the turn limit (" + oneDecimal(spec.maxTurnDeg) + " degrees) or the stop line is in the way"
                    : "the flat wheels would have to stick out " + oneDecimal(spec.intakeReachIn + missingReachIn) + " in in front of the chassis, "
                    + oneDecimal(missingReachIn) + " in more than they do (Robot.INTAKE_REACH_IN)";
            return "with the chassis kept " + oneDecimal(spec.chassisClearanceIn) + " in off the plate through the whole " + oneDecimal(Math.abs(turnDeg()))
                    + " degree swing (it stands " + oneDecimal(standoffIn) + " in from the wall, " + oneDecimal(swingExtraStandoffIn())
                    + " in farther than a straight press) the middle of the " + wheel + " wheel gets no closer than " + oneDecimal(closestIn)
                    + " in to the middle of the POLLEN, but the press needs " + oneDecimal(spec.pressOffsetIn) + " in; " + longer;
        }
    }

    private static final class Key {
        final IntakeWheel wheel;
        final Spec spec;

        Key(IntakeWheel wheel, Spec spec) {
            this.wheel = wheel;
            this.spec = spec;
        }

        @Override
        public boolean equals(Object o) {
            return o instanceof Key && ((Key) o).wheel == wheel && ((Key) o).spec.equals(spec);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(wheel, spec);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // the wall frame
    // ---------------------------------------------------------------------------------------------------------------

    /** The obstacles around one FLOWER in the wall frame: x = distance from the wall, y = distance along it from the ball's middle line. */
    private static final class Frame {
        final Rect chassisKeepOut;
        final Rect plateKeepOut;
        final Rect stopZone;
        final Rect field;

        Frame(Spec spec) {
            double half = BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN / 2.0;
            Rect plate = new Rect(0.0, BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN, -half, half);
            chassisKeepOut = plate.inflate(spec.chassisClearanceIn);
            plateKeepOut = plate.inflate(BioBuzzConstants.Auto.FLOWER_KEEPOUT_MARGIN_IN);
            stopZone = new Rect(0.0, ballDepthIn(), -half, half);
            // only the wall matters here (the other walls are a long way off): a half-plane, written as a box that is very much bigger than the robot
            field = new Rect(0.0, BioBuzzConstants.Field.SIZE_IN, -BioBuzzConstants.Field.SIZE_IN, BioBuzzConstants.Field.SIZE_IN);
        }
    }

    /** How far from the wall the middle of the POLLEN stack is (and so the press stop line: the wheel never goes past it). */
    static double ballDepthIn() {
        return BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN;
    }

    /** Forward distance, from the pose point, of the middle of a flat wheel (inches). */
    static double wheelForwardIn() {
        return Footprint.frontExtentIn() - BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0;
    }

    /** The lever arm: how far the middle of a flat wheel is from the pose point, in inches. */
    public static double leverArmIn() {
        return Math.hypot(wheelForwardIn(), BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // solving
    // ---------------------------------------------------------------------------------------------------------------

    /**
     * Derives the wiggle of {@code wheel} (LEFT or RIGHT) for the numbers in {@code spec}. The answer is cached: the same wheel and numbers
     * always give the same {@link Solution}.
     */
    public static Solution solve(IntakeWheel wheel, Spec spec) {
        if (wheel == null || wheel.isAuto()) {
            throw new IllegalArgumentException("a wiggle is solved for LEFT or RIGHT, not " + wheel + "; WheelChoice picks one for AUTO");
        }
        Key key = new Key(wheel, spec);
        synchronized (CACHE) {
            Solution cached = CACHE.get(key);
            if (cached != null) {
                return cached;
            }
        }
        Solution fresh = derive(wheel, spec);
        synchronized (CACHE) {
            if (CACHE.size() >= CACHE_LIMIT) {
                CACHE.clear();
            }
            CACHE.put(key, fresh);
        }
        return fresh;
    }

    /** How far along the wall the pose point stands from the ball's middle line so that the wheel starts {@code startOffsetIn} to one side of the ball. */
    private static double lateralOfPose(IntakeWheel wheel, Spec spec) {
        // In the wall frame the robot's left is toward -y (facing the wall, 180 degrees). The wheel is {@code sign * OFFSET} to the robot's left of the pose point,
        // so it is that far toward -y; the ball is startOffset away from the wheel toward the center line (inboard) or away from it (outboard).
        double s = wheel.sign();
        double toBall = spec.ballInboard ? 1.0 : -1.0;
        // wheel y = -s * toBall * start (the ball is at y = 0, on the inboard side of the wheel); pose point y = wheel y + s * OFFSET
        return s * (BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN - toBall * spec.startOffsetIn);
    }

    private static Solution derive(IntakeWheel wheel, Spec spec) {
        double yPose = lateralOfPose(wheel, spec);
        Frame frame = new Frame(spec);
        double[] exact = findStandoff(frame, spec, wheel, yPose, true, true);
        double noSwing = noSwingStandoff(spec);
        double lever = leverArmIn();
        if (exact != null) {
            Turn t = turnFor(spec, wheel, exact[0], yPose);
            return new Solution(wheel, spec, exact[0], yPose, exact[1], true, t.closest, 0.0, noSwing, lever);
        }
        // The wheel cannot get to the ball. Still derive a pose (the legal one closest to the wall, turned as far toward the ball as the limit allows) so the
        // diagram and the timing have something to show, and work out how much longer the wheels would have to be.
        double[] closest = findStandoff(frame, spec, wheel, yPose, false, true);
        double depth = closest != null ? closest[0] : scanEnd(spec);
        Turn t = turnFor(spec, wheel, depth, yPose);
        return new Solution(wheel, spec, depth, yPose, t.phi, false, t.closest, missingReach(spec, wheel, yPose), noSwing, lever);
    }

    /** The standoff of a straight press with no swing: the chassis front on its clearance line, or the wheel's tip on the stop line, whichever is farther out. */
    static double noSwingStandoff(Spec spec) {
        double chassis = BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN + spec.chassisClearanceIn + Footprint.frontExtentIn() - spec.intakeReachIn;
        double wheelOnStopLine = ballDepthIn() + BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0 + wheelForwardIn();
        return Math.max(chassis, wheelOnStopLine);
    }

    /** The farthest standoff worth looking at: farther out than this the wheel's circle cannot reach the ball at all. */
    private static double scanEnd(Spec spec) {
        return ballDepthIn() + leverArmIn() + spec.pressOffsetIn + Math.abs(lateralOfPose(IntakeWheel.LEFT, spec)) + 1.0;
    }

    /**
     * The smallest standoff (distance of the pose point from the wall) at which the swing is legal, scanning outward from the wall: returns
     * {standoff, turn in radians}, or null if none is. With {@code requireReach} the wheel must also reach the ball (get within the press offset
     * within the biggest turn); without it only the rules are asked, with the turn that brings the wheel closest. With {@code refine} the edge is found
     * exactly by halving; without it the first standoff of the scan that works is returned (enough to know that one exists).
     */
    private static double[] findStandoff(Frame frame, Spec spec, IntakeWheel wheel, double yPose, boolean requireReach, boolean refine) {
        double from = Footprint.frontExtentIn();                  // the wheel tips on the wall: closer than this is inside the wall
        double to = scanEnd(spec);
        double previous = Double.NaN;
        for (double d = from; d <= to; d += SCAN_STEP_IN) {
            if (acceptable(frame, spec, wheel, d, yPose, requireReach)) {
                double bad = previous;
                double good = d;
                if (refine && !Double.isNaN(bad)) {
                    for (int i = 0; i < HALVINGS; i++) {
                        double mid = (bad + good) / 2.0;
                        if (acceptable(frame, spec, wheel, mid, yPose, requireReach)) {
                            good = mid;
                        } else {
                            bad = mid;
                        }
                    }
                }
                return new double[] {good, turnFor(spec, wheel, good, yPose).phi};
            }
            previous = d;
        }
        return null;
    }

    private static boolean acceptable(Frame frame, Spec spec, IntakeWheel wheel, double depth, double yPose, boolean requireReach) {
        Turn t = turnFor(spec, wheel, depth, yPose);
        if (requireReach && !t.reaches) {
            return false;
        }
        return swingIsLegal(frame, spec, wheel, depth, yPose, t.phi);
    }

    /**
     * Every rule of the swing: the chassis, the stop line, the other wheel and the wall, at every pose from the start to the pushed pose. A handful of
     * poses (the start, the middle, the end) are tried first, because most standoffs the search looks at break a rule there and that is a cheap way to say
     * so; only a standoff that passes them is swept exactly ({@link Sweep}), which is what makes the answer sure.
     */
    private static boolean swingIsLegal(Frame frame, Spec spec, IntakeWheel wheel, double depth, double yPose, double phi) {
        double m = DERIVE_MARGIN_IN;
        Sweep.Depth chassis = PressRules.chassisOffPlate(frame.chassisKeepOut, spec.intakeReachIn, m);
        Sweep.Depth stop = PressRules.wheelShortOfStopLine(wheel, frame.stopZone, m);
        Sweep.Depth other = PressRules.wheelOffPlate(wheel.other(), frame.plateKeepOut, m);
        Sweep.Depth inside = PressRules.insideField(frame.field, 0.0, m);
        for (int i = 0; i <= QUICK_POSES; i++) {
            Pose2 p = new Pose2(depth, yPose, Math.PI + phi * i / QUICK_POSES);
            if (chassis.at(p) > 0.0 || stop.at(p) > 0.0 || other.at(p) > 0.0 || inside.at(p) > 0.0) {
                return false;
            }
        }
        Pose2 start = new Pose2(depth, yPose, Math.PI);
        Pose2 pushed = new Pose2(depth, yPose, Math.PI + phi);
        DrivePath swing = DrivePath.straight(start, pushed);
        return !Sweep.run(swing, chassis).broken() && !Sweep.run(swing, stop).broken() && !Sweep.run(swing, other).broken()
                && !Sweep.run(swing, inside).broken();
    }

    /** How many more inches of wheel reach would make the pick-up work: found by halving, because more reach never makes it harder. */
    private static double missingReach(Spec spec, IntakeWheel wheel, double yPose) {
        double cap = Footprint.frontExtentIn();                  // the chassis front could not be farther back than the pose point
        if (spec.intakeReachIn >= cap || !works(spec.withIntakeReach(cap), wheel, yPose)) {
            return Double.POSITIVE_INFINITY;
        }
        double bad = spec.intakeReachIn;
        double good = cap;
        for (int i = 0; i < REACH_HALVINGS; i++) {
            double mid = (bad + good) / 2.0;
            if (works(spec.withIntakeReach(mid), wheel, yPose)) {
                good = mid;
            } else {
                bad = mid;
            }
        }
        return good - spec.intakeReachIn;
    }

    private static boolean works(Spec spec, IntakeWheel wheel, double yPose) {
        return findStandoff(new Frame(spec), spec, wheel, yPose, true, false) != null;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // the turn
    // ---------------------------------------------------------------------------------------------------------------

    /** The push for one standoff: how far to turn, whether that reaches the ball and how close the wheel gets. */
    static final class Turn {
        /** Signed turn in radians (positive = counter-clockwise). */
        final double phi;
        final boolean reaches;
        final double closest;

        Turn(double phi, boolean reaches, double closest) {
            this.phi = phi;
            this.reaches = reaches;
            this.closest = closest;
        }
    }

    /**
     * The turn of the push for a robot whose pose point is {@code depth} from the wall and {@code yPose} along it, facing the wall.
     *
     * <p>The middle of the wheel is {@code L} (the lever arm) from the pose point, so it moves on a circle of radius L when the robot turns in place. The
     * ball's middle is {@code dv} from the pose point. The distance between the wheel and the ball after turning by {@code phi} is, by the law of cosines,
     * {@code sqrt(L^2 + dv^2 - 2 L dv cos(phi - base))}, where {@code base} is the turn at which the wheel points straight at the ball (the closest it can ever be, |dv - L|).
     * Setting that equal to the press offset and solving for phi gives the first turn, in the direction that carries the wheel toward the ball, at which
     * the two circles meet: {@code phi = base - acos(kappa)} with {@code kappa = (L^2 + dv^2 - press^2) / (2 L dv)}. If kappa is above 1 the circles never meet.
     */
    static Turn turnFor(Spec spec, IntakeWheel wheel, double depth, double yPose) {
        double a = wheelForwardIn();
        double b = BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN;
        double lever = Math.hypot(a, b);
        double psi = Math.atan2(wheel.sign() * b, a);                 // direction of the wheel as seen from the pose point, in the robot's frame
        double vx = ballDepthIn() - depth;
        double vy = -yPose;
        double dv = Math.hypot(vx, vy);
        double press = spec.pressOffsetIn;
        // the turn at which the wheel points straight at the ball (heading = direction of the ball minus psi, and facing the wall is 180 degrees)
        double base = Angles.normalize(Math.atan2(vy, vx) - psi - Math.PI);
        double dir = Math.sin(base) >= 0.0 ? 1.0 : -1.0;               // the way to turn so that the wheel gets closer to the ball
        double beta = Angles.normalize(dir * base);                    // how far along that way the closest approach lies (0 to PI)
        double maxTurn = Math.toRadians(spec.maxTurnDeg);
        double startDistance = Math.sqrt(Math.max(0.0, lever * lever + dv * dv - 2.0 * lever * dv * Math.cos(base)));
        if (startDistance <= press) {
            return new Turn(0.0, true, startDistance);                 // the wheel is already within the press offset: no turn needed
        }
        double kappa = (lever * lever + dv * dv - press * press) / (2.0 * lever * dv);
        if (kappa <= 1.0) {
            double first = beta - Math.acos(kappa);
            if (first > 0.0 && first <= maxTurn) {
                return new Turn(dir * first, true, press);
            }
        }
        double u = Math.max(0.0, Math.min(maxTurn, beta));             // the best the turn limit allows
        double distance = Math.sqrt(Math.max(0.0, lever * lever + dv * dv - 2.0 * lever * dv * Math.cos(u - beta)));
        return new Turn(dir * u, false, distance);
    }

    /** The distance between the middle of the wheel and the middle of the ball for a pose of the robot in the wall frame, in inches. */
    static double wheelToBallIn(Pose2 wallPose, IntakeWheel wheel) {
        return Footprint.wheelCenter(wallPose, wheel).distanceTo(new Point2(ballDepthIn(), 0.0));
    }

    private static String oneDecimal(double v) {
        return String.format(Locale.ROOT, "%.1f", v);
    }
}
