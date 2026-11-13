package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;

/**
 * One immutable step of an autonomous route. A route is written ONCE, in the RED frame; the BLUE
 * version is the half turn of every pose and point ({@link Alliance#fromRed}), done by
 * {@link AutoRoute#forAlliance}.
 *
 * <p>Pure Java (no FTC or Pedro imports) so a PC can validate and draw routes. Use
 * {@link AutoRoute#builder(String)} to create steps; the constructors are public only so tests can
 * build odd steps directly.
 */
public abstract class Step {
    private final StepKind kind;
    private final String note;

    Step(StepKind kind, String note) {
        this.kind = kind;
        this.note = note == null ? "" : note;
    }

    public final StepKind kind() {
        return kind;
    }

    /** Free text for people (why this step exists, what the old code did). Never empty-null: "" if none. */
    public final String note() {
        return note;
    }

    /** The same step with a different note. */
    public abstract Step withNote(String note);

    /** The half-turn image of this step about the field center (red <-> blue). */
    abstract Step halfTurn(double fieldSize);

    /** A short one-line description for lists and tooltips. */
    public abstract String describe();

    static String fmt(double v) {
        return String.format(Locale.ROOT, "%.1f", v);
    }

    static String fmt(Pose2 p) {
        return String.format(Locale.ROOT, "(%.1f, %.1f, %.0f deg)", p.x, p.y, p.headingDegrees());
    }

    private static Pose2 rotate(Pose2 p, double size) {
        return Alliance.BLUE.fromRed(p, size);
    }

    private static Point2 rotate(Point2 p, double size) {
        return p.fromRed(Alliance.BLUE, size);
    }

    // ---------------------------------------------------------------------------------------------

    /** Where the robot is placed before the match. Rule G304 decides which placements are legal. */
    public static final class Start extends Step {
        private final Pose2 pose;

        public Start(Pose2 pose, String note) {
            super(StepKind.START, note);
            if (pose == null) {
                throw new IllegalArgumentException("start pose must not be null");
            }
            this.pose = pose;
        }

        public Pose2 pose() {
            return pose;
        }

        @Override
        public Step withNote(String note) {
            return new Start(pose, note);
        }

        @Override
        Step halfTurn(double fieldSize) {
            return new Start(rotate(pose, fieldSize), note());
        }

        @Override
        public String describe() {
            return "START at " + fmt(pose);
        }
    }

    // ---------------------------------------------------------------------------------------------

    /**
     * Follow a path from wherever the previous step ended to {@link #target()}. With no via points the
     * path is a straight line (Pedro {@code line}); each via point is a control point of a Bezier curve
     * (Pedro {@code curve}), so the robot is pulled toward it but does NOT pass through it. End a DRIVE
     * at a point to force the robot through it.
     */
    public static final class Drive extends Step {
        private final List<Point2> via;
        private final Pose2 target;
        private final HeadingMode mode;
        private final Point2 facePoint;
        private final boolean faceWithBack;
        private final double speedFraction;
        private final String label;
        private final boolean wallContact;
        private final boolean allowUnderHive;

        public Drive(List<Point2> via, Pose2 target, HeadingMode mode, Point2 facePoint, boolean faceWithBack,
                     double speedFraction, String label, boolean wallContact, String note) {
            this(via, target, mode, facePoint, faceWithBack, speedFraction, label, wallContact, false, note);
        }

        /**
         * @param allowUnderHive true if this leg may drive UNDER the HIVE, along the lane between its two foot bars; the validator then checks
         *                       it against the exact solid (foot bars and leg stubs, with {@code Auto.HIVE_LANE_MARGIN_IN} on both sides)
         *                       instead of the whole footprint box. See {@link #allowsUnderHive()}.
         */
        public Drive(List<Point2> via, Pose2 target, HeadingMode mode, Point2 facePoint, boolean faceWithBack,
                     double speedFraction, String label, boolean wallContact, boolean allowUnderHive, String note) {
            super(StepKind.DRIVE, note);
            if (target == null || mode == null) {
                throw new IllegalArgumentException("a DRIVE needs a target pose and a heading mode");
            }
            if (mode == HeadingMode.FACE_POINT && facePoint == null) {
                throw new IllegalArgumentException("FACE_POINT needs the point to face");
            }
            if (!(speedFraction > 0.0 && speedFraction <= 1.0)) {
                throw new IllegalArgumentException("speed fraction must be in (0, 1], got " + speedFraction);
            }
            this.via = Collections.unmodifiableList(new ArrayList<Point2>(via));
            this.target = target;
            this.mode = mode;
            this.facePoint = facePoint;
            this.faceWithBack = faceWithBack;
            this.speedFraction = speedFraction;
            this.label = label == null ? "" : label;
            this.wallContact = wallContact;
            this.allowUnderHive = allowUnderHive;
        }

        /** Bezier control points between the previous end pose and the target (empty = straight line). */
        public List<Point2> via() {
            return via;
        }

        /**
         * The target pose. Its x and y are where the leg ends. Its heading is only used by LINEAR and
         * CONSTANT; for the tangent and face-point modes the heading follows from the geometry (the value
         * given here, copied from the old code, is ignored just as Pedro ignores it).
         */
        public Pose2 target() {
            return target;
        }

        public HeadingMode mode() {
            return mode;
        }

        /** The point to keep facing, for FACE_POINT only (else null). */
        public Point2 facePoint() {
            return facePoint;
        }

        /** FACE_POINT only: true when the robot's BACK (shooter) keeps pointing at the point. */
        public boolean faceWithBack() {
            return faceWithBack;
        }

        /** Fraction (0, 1] of the cruise speed to drive at. */
        public double speedFraction() {
            return speedFraction;
        }

        /** Optional name of the leg ("" if none), e.g. the old code's "path1". */
        public String label() {
            return label;
        }

        /** True if the leg touches a field wall on purpose. A note only: the wall rule is the same for every leg. */
        public boolean isWallContact() {
            return wallContact;
        }

        /**
         * True if this leg may drive UNDER the HIVE (default false). A robot 12 to 13 in tall fits under the cells (the lowest is 30.65 in up)
         * and can drive north-south along the open lane between the two foot bars, about 20 in shorter than going around the frame. The
         * default keeps every route OUT of the whole HIVE footprint (G409 / G417: never be under a HIVE that is tipping). A leg with this
         * flag is checked against the exact solid instead: the foot bars and the leg stubs, with {@code Auto.HIVE_LANE_MARGIN_IN} to spare on
         * both sides, and the validator WARNs if the HIVE could still tip while the robot is under it (before the camera has confirmed it at
         * rest, see {@code TipRules}). The flag changes only the HIVE check of this one leg: walls, FLOWERS and the center line still apply.
         */
        public boolean allowsUnderHive() {
            return allowUnderHive;
        }

        public Drive withVia(List<Point2> newVia) {
            return new Drive(newVia, target, mode, facePoint, faceWithBack, speedFraction, label, wallContact, allowUnderHive, note());
        }

        public Drive withSpeed(double fraction) {
            return new Drive(via, target, mode, facePoint, faceWithBack, fraction, label, wallContact, allowUnderHive, note());
        }

        public Drive withLabel(String newLabel) {
            return new Drive(via, target, mode, facePoint, faceWithBack, speedFraction, newLabel, wallContact, allowUnderHive, note());
        }

        public Drive withWallContact(boolean contact) {
            return new Drive(via, target, mode, facePoint, faceWithBack, speedFraction, label, contact, allowUnderHive, note());
        }

        public Drive withAllowUnderHive(boolean allow) {
            return new Drive(via, target, mode, facePoint, faceWithBack, speedFraction, label, wallContact, allow, note());
        }

        @Override
        public Step withNote(String note) {
            return new Drive(via, target, mode, facePoint, faceWithBack, speedFraction, label, wallContact, allowUnderHive, note);
        }

        @Override
        Step halfTurn(double fieldSize) {
            List<Point2> v = new ArrayList<Point2>();
            for (Point2 p : via) {
                v.add(rotate(p, fieldSize));
            }
            return new Drive(v, rotate(target, fieldSize), mode, facePoint == null ? null : rotate(facePoint, fieldSize),
                    faceWithBack, speedFraction, label, wallContact, allowUnderHive, note());
        }

        @Override
        public String describe() {
            String name = label.isEmpty() ? "" : " " + label;
            String curve = via.isEmpty() ? "" : " curve(" + via.size() + ")";
            return "DRIVE" + name + " " + mode.name().toLowerCase(Locale.ROOT).replace('_', ' ') + curve
                    + " to " + fmt(target.x) + ", " + fmt(target.y) + (allowUnderHive ? " (may go under the HIVE)" : "");
        }
    }

    // ---------------------------------------------------------------------------------------------

    /**
     * Take POLLEN from a source. The robot first goes to {@link #robotPose()}, then takes the POLLEN there.
     *
     * <p>For a FLOWER the step is a PRESS-IN (see {@link FlowerPress}). The two flat intake wheels are 12.2 in apart, too
     * far to pinch one 2.8 in POLLEN, so ONE of them presses while the robot stands to one side of the FLOWER's center line:
     * {@link #wheel()} names it (LEFT or RIGHT, the robot's own left and right). {@link #robotPose()} is the LINE-UP pose,
     * the place the straight final approach starts from: facing the wall squarely with that wheel on the FLOWER's center
     * line. From there the robot drives straight in until the wheel overlaps the bottom POLLEN, dwells with the intake on,
     * and backs {@link #retreatLengthIn()} inches straight out; the step ends at that retreat pose. The press and retreat
     * poses are derived from the FLOWER, so route authors never write them. A FLOWER INTAKE without a wheel is refused when
     * the step is made.
     */
    public static final class Intake extends Step {
        private final IntakeSource source;
        private final Point2 target;
        private final Pose2 robotPose;
        private final double dwellSec;
        private final int expectedCount;
        private final double retreatLengthIn;
        private final IntakeWheel wheel;

        /** An INTAKE from a GARDEN or the floor. (A FLOWER needs to name its wheel: see the constructor with an IntakeWheel.) */
        public Intake(IntakeSource source, Point2 target, Pose2 robotPose, double dwellSec, int expectedCount,
                      String note) {
            this(source, target, robotPose, dwellSec, expectedCount, BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN, null, note);
        }

        /**
         * @param retreatLengthIn FLOWER only: how far the robot backs straight out after the dwell, in inches (the default
         *                        is {@code Auto.FLOWER_RETREAT_LENGTH_IN}; a shorter retreat is a WARN)
         * @param wheel           FLOWER only (required there, ignored elsewhere): which flat wheel presses the POLLEN
         */
        public Intake(IntakeSource source, Point2 target, Pose2 robotPose, double dwellSec, int expectedCount,
                      double retreatLengthIn, IntakeWheel wheel, String note) {
            super(StepKind.INTAKE, note);
            if (source == null || target == null || robotPose == null) {
                throw new IllegalArgumentException("an INTAKE needs a source, a target point and a robot pose");
            }
            if (!(dwellSec >= 0.0)) {
                throw new IllegalArgumentException("dwell seconds must be >= 0, got " + dwellSec);
            }
            if (expectedCount < 1) {
                throw new IllegalArgumentException("expected POLLEN count must be >= 1, got " + expectedCount);
            }
            if (!(retreatLengthIn >= 0.0)) {
                throw new IllegalArgumentException("retreat length must be >= 0, got " + retreatLengthIn);
            }
            if (source == IntakeSource.FLOWER && wheel == null) {
                throw new IllegalArgumentException("a FLOWER INTAKE must say which flat intake wheel presses the POLLEN"
                        + " (IntakeWheel.LEFT or RIGHT): the two wheels are too far apart to straddle it");
            }
            this.source = source;
            this.target = target;
            this.robotPose = robotPose;
            this.dwellSec = dwellSec;
            this.expectedCount = expectedCount;
            this.retreatLengthIn = retreatLengthIn;
            this.wheel = source == IntakeSource.FLOWER ? wheel : null;
        }

        public IntakeSource source() {
            return source;
        }

        /** The point being taken from: a POLLEN, the FLOWER center, or the middle of the GARDEN POLLEN line. */
        public Point2 target() {
            return target;
        }

        /** The robot pose to take it from (for a FLOWER: the line-up pose where the straight final approach starts). */
        public Pose2 robotPose() {
            return robotPose;
        }

        /** Seconds spent at the pose with the intake running (for a FLOWER: pressed in). */
        public double dwellSec() {
            return dwellSec;
        }

        /** How many POLLEN this step is expected to add to what the robot holds. */
        public int expectedCount() {
            return expectedCount;
        }

        /** FLOWER only: how far the robot backs straight out after the dwell, in inches. */
        public double retreatLengthIn() {
            return retreatLengthIn;
        }

        /**
         * FLOWER only: which flat intake wheel presses the POLLEN (the robot's own LEFT or RIGHT; it is the same for the
         * red and the blue alliance). Null for a GARDEN or floor INTAKE.
         */
        public IntakeWheel wheel() {
            return wheel;
        }

        /** The same step with another retreat length. */
        public Intake withRetreatLengthIn(double inches) {
            return new Intake(source, target, robotPose, dwellSec, expectedCount, inches, wheel, note());
        }

        @Override
        public Step withNote(String note) {
            return new Intake(source, target, robotPose, dwellSec, expectedCount, retreatLengthIn, wheel, note);
        }

        @Override
        Step halfTurn(double fieldSize) {
            // a half turn of the field turns the whole robot with it, so its left wheel stays its left wheel
            return new Intake(source, rotate(target, fieldSize), rotate(robotPose, fieldSize), dwellSec,
                    expectedCount, retreatLengthIn, wheel, note());
        }

        @Override
        public String describe() {
            return "INTAKE " + source + " +" + expectedCount + " from " + fmt(robotPose)
                    + (wheel == null ? "" : " with the " + wheel + " wheel");
        }
    }

    // ---------------------------------------------------------------------------------------------

    /**
     * Launch POLLEN at a cell of the own HIVE. The shooting pose is either explicit or null
     * ("AUTO"): then the runtime scoring-pose solver picks it, and the validator and diagrams use the
     * nominal standoff pose computed from the constants.
     */
    public static final class Shoot extends Step {
        private final ShootTarget target;
        private final Pose2 pose;
        private final int count;
        private final double fixedSeconds;

        /**
         * @param pose         the shooting pose, or null to let the solver choose (AUTO)
         * @param fixedSeconds if not NaN, how long the whole step takes (overrides the timing model)
         */
        public Shoot(ShootTarget target, Pose2 pose, int count, double fixedSeconds, String note) {
            super(StepKind.SHOOT, note);
            if (target == null) {
                throw new IllegalArgumentException("a SHOOT needs a target");
            }
            if (count < 1) {
                throw new IllegalArgumentException("a SHOOT must fire at least 1 POLLEN, got " + count);
            }
            if (!Double.isNaN(fixedSeconds) && !(fixedSeconds >= 0.0)) {
                throw new IllegalArgumentException("fixed seconds must be >= 0 or NaN, got " + fixedSeconds);
            }
            this.target = target;
            this.pose = pose;
            this.count = count;
            this.fixedSeconds = fixedSeconds;
        }

        public ShootTarget target() {
            return target;
        }

        /** The explicit shooting pose, or null when the solver chooses it at runtime. */
        public Pose2 poseOrNull() {
            return pose;
        }

        public boolean isAutoPose() {
            return pose == null;
        }

        /** Number of POLLEN fired. */
        public int count() {
            return count;
        }

        /** True if the step has a fixed duration that overrides the timing model. */
        public boolean hasFixedSeconds() {
            return !Double.isNaN(fixedSeconds);
        }

        /** The fixed duration in seconds, or NaN if the timing model decides. */
        public double fixedSeconds() {
            return fixedSeconds;
        }

        public Shoot withFixedSeconds(double seconds) {
            return new Shoot(target, pose, count, seconds, note());
        }

        @Override
        public Step withNote(String note) {
            return new Shoot(target, pose, count, fixedSeconds, note);
        }

        @Override
        Step halfTurn(double fieldSize) {
            // A fixed side flips (red audience cell <-> blue rear cell); see ShootTarget#halfTurn.
            return new Shoot(target.halfTurn(), pose == null ? null : rotate(pose, fieldSize), count, fixedSeconds,
                    note());
        }

        @Override
        public String describe() {
            return "SHOOT " + count + " at " + target + (pose == null ? " from the solver's pose" : " from " + fmt(pose));
        }
    }

    // ---------------------------------------------------------------------------------------------

    /** Do nothing for a while (for example to let the launcher finish). */
    public static final class Wait extends Step {
        private final double seconds;

        public Wait(double seconds, String note) {
            super(StepKind.WAIT, note);
            if (!(seconds >= 0.0)) {
                throw new IllegalArgumentException("wait seconds must be >= 0, got " + seconds);
            }
            this.seconds = seconds;
        }

        public double seconds() {
            return seconds;
        }

        @Override
        public Step withNote(String note) {
            return new Wait(seconds, note);
        }

        @Override
        Step halfTurn(double fieldSize) {
            return this;
        }

        @Override
        public String describe() {
            return "WAIT " + fmt(seconds) + " s";
        }
    }

    // ---------------------------------------------------------------------------------------------

    /**
     * Stand still at the current pose until the camera has seen the HIVE tip, and give up at {@link #giveUpSec()}. It is
     * for routes that assume the alliance partner shoots the first tip ({@link FirstTip#PARTNER}): the other cell only
     * comes up when the partner's volley has tipped the HIVE, and partner teams tip at different times, so the robot
     * does not guess. A time never makes it shoot (rule G417: only a cell that faces up may be launched into): it waits
     * until its camera sees the cell UP and steady, and only then goes on to the volley after this step.
     *
     * <p>{@code giveUpSec} is a DEADLINE on the route clock (seconds since the start of AUTO), not a length of time and
     * not a guess of when the partner tips. When the clock reaches it and the tip has not been seen, the robot skips
     * the volleys that need the tip and goes on with the safe steps (LEAVE, PARK). Choose it so that the steps after
     * the wait still fit before the AUTO limit: the analysis reports the latest such time
     * ({@link RouteAnalysis#latestGiveUpSec}) and the checker warns if the deadline is later.
     *
     * <p>The analysis (not the robot) must still guess when the tip happens, to estimate the time and the points: it
     * uses {@code Auto.PARTNER_FIRST_TIP_DONE_SEC} and reports how late a tip could be and still work.
     */
    public static final class WaitForTip extends Step {
        private final double giveUpSec;

        public WaitForTip(double giveUpSec, String note) {
            super(StepKind.WAIT_FOR_TIP, note);
            if (!(giveUpSec > 0.0)) {
                throw new IllegalArgumentException("the give-up time of a WAIT_FOR_TIP is a route-clock time and must be > 0 s, got "
                        + giveUpSec);
            }
            this.giveUpSec = giveUpSec;
        }

        /** The route-clock time in seconds at which the robot stops waiting if it has not seen the tip; then it skips the volleys that need it. */
        public double giveUpSec() {
            return giveUpSec;
        }

        @Override
        public Step withNote(String note) {
            return new WaitForTip(giveUpSec, note);
        }

        @Override
        Step halfTurn(double fieldSize) {
            return this;
        }

        @Override
        public String describe() {
            return "WAIT_FOR_TIP until the camera sees the HIVE tipped, giving up at " + fmt(giveUpSec) + " s on the route clock";
        }
    }

    // ---------------------------------------------------------------------------------------------

    /** Finish partly inside the own LOADING ZONE (AUTO PARK, 5 points). */
    public static final class Park extends Step {
        private final Pose2 pose;

        public Park(Pose2 pose, String note) {
            super(StepKind.PARK, note);
            if (pose == null) {
                throw new IllegalArgumentException("park pose must not be null");
            }
            this.pose = pose;
        }

        public Pose2 pose() {
            return pose;
        }

        @Override
        public Step withNote(String note) {
            return new Park(pose, note);
        }

        @Override
        Step halfTurn(double fieldSize) {
            return new Park(rotate(pose, fieldSize), note());
        }

        @Override
        public String describe() {
            return "PARK at " + fmt(pose);
        }
    }
}
