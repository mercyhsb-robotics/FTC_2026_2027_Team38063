package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;

/**
 * An autonomous route as DATA: an id, a title, and an ordered list of immutable {@link Step}s.
 *
 * <p>A route is authored once, in the RED frame ({@link #alliance()} is RED). {@link #forAlliance}
 * returns the BLUE version, which is the half turn of every pose and point about the field center
 * (NOT a mirror, see {@link Alliance}). Everything else (timing, points, validation, diagrams) is
 * derived from the steps by the other classes of this package.
 *
 * <p>Adding an auto is ONE data entry in {@link RouteCatalog}:
 * <pre>
 * AutoRoute.builder("red_b1").title("Red B1").startTile("B1")
 *     .start(35.0, 9.2, -90)
 *     .shootUp(4)
 *     .driveReverseTangent(35.8, 90, -90)
 *     .park(12, 90, 0)
 *     .build();
 * </pre>
 * Headings are given in DEGREES (counter-clockwise, 0 = +x) like Pedro's {@code PoseFactory.degrees()}.
 */
public final class AutoRoute {
    private final String id;
    private final String title;
    private final String description;
    private final String startTile;
    private final Alliance alliance;
    private final List<Step> steps;

    private AutoRoute(String id, String title, String description, String startTile, Alliance alliance,
                      List<Step> steps) {
        this.id = id;
        this.title = title;
        this.description = description;
        this.startTile = startTile;
        this.alliance = alliance;
        this.steps = Collections.unmodifiableList(new ArrayList<Step>(steps));
    }

    /** Stable machine name, e.g. "red_b1". It never changes once a route is in use. */
    public String id() {
        return id;
    }

    public String title() {
        return title;
    }

    public String description() {
        return description;
    }

    /** The start tile label, e.g. "B1" (column A-F, row 1-6, in the RED frame). */
    public String startTile() {
        return startTile;
    }

    /** The alliance whose frame the poses are in: RED for authored routes, BLUE for the derived ones. */
    public Alliance alliance() {
        return alliance;
    }

    public List<Step> steps() {
        return steps;
    }

    public Step step(int index) {
        return steps.get(index);
    }

    public int size() {
        return steps.size();
    }

    /** The pose the robot is placed at (the first step is always a START). */
    public Pose2 startPose() {
        return ((Step.Start) steps.get(0)).pose();
    }

    /**
     * This route in the given alliance's frame. For the same alliance it returns this route; for the
     * other one it returns a new route with every pose and point turned half a turn about (72, 72) and
     * every fixed HIVE side flipped. Applying it twice gives back the original.
     */
    public AutoRoute forAlliance(Alliance target) {
        if (target == alliance) {
            return this;
        }
        List<Step> turned = new ArrayList<Step>();
        for (Step s : steps) {
            turned.add(s.halfTurn(BioBuzzConstants.Field.SIZE_IN));
        }
        return new AutoRoute(id, title, description, startTile, target, turned);
    }

    // ---- derived values ------------------------------------------------------------------------

    /** Plays the route through (poses, times, POLLEN, HIVE state) in its own alliance's frame. */
    public RouteAnalysis analyze() {
        return RouteAnalysis.of(this);
    }

    /** Estimated duration in seconds, from the placeholder {@link TimingModel}. */
    public double estimatedSeconds() {
        return analyze().totalSeconds();
    }

    /**
     * AUTO points the route can claim if everything goes to plan: LEAVE 3 if it ends off every wall,
     * PARK 5 if it ends overlapping the own LOADING ZONE, 20 per HIVE TIP its shots reach.
     */
    public int claimablePoints() {
        return analyze().points();
    }

    /** POLLEN taken in minus POLLEN fired over the whole route. */
    public int pollenBalance() {
        return analyze().pollenBalance();
    }

    /** The robot pose after each step (the effective end heading of every DRIVE included). */
    public List<Pose2> endPoses() {
        List<Pose2> out = new ArrayList<Pose2>();
        for (RouteAnalysis.StepInfo info : analyze().steps()) {
            out.add(info.to);
        }
        return out;
    }

    @Override
    public String toString() {
        return "AutoRoute{" + id + ", " + alliance + ", " + steps.size() + " steps}";
    }

    public static Builder builder(String id) {
        return new Builder(id);
    }

    // ---------------------------------------------------------------------------------------------

    /**
     * Fluent builder. Step methods add a step; the modifier methods ({@link #via}, {@link #speed},
     * {@link #label}, {@link #wallContact}, {@link #launchSeconds}, {@link #note}) change the step
     * added last. {@link #build()} checks the structure (one START first, sane numbers); whether the
     * route is a good idea on the real field is the job of {@link RouteValidator}.
     */
    public static final class Builder {
        private final String id;
        private String title;
        private String description = "";
        private String startTile = "";
        private final List<Step> steps = new ArrayList<Step>();

        private Builder(String id) {
            if (id == null || id.trim().isEmpty()) {
                throw new IllegalArgumentException("a route needs an id");
            }
            this.id = id.trim();
            this.title = this.id;
        }

        public Builder title(String title) {
            this.title = title;
            return this;
        }

        public Builder description(String description) {
            this.description = description;
            return this;
        }

        public Builder startTile(String startTile) {
            this.startTile = startTile;
            return this;
        }

        // ---- START -----------------------------------------------------------------------------

        /** The start pose in the RED frame (inches, heading in degrees). Must be the first step. */
        public Builder start(double x, double y, double headingDeg) {
            steps.add(new Step.Start(Pose2.ofDegrees(x, y, headingDeg), ""));
            return this;
        }

        // ---- DRIVE -----------------------------------------------------------------------------

        /** Front of the robot along the travel direction (heading ignored). */
        public Builder driveTangent(double x, double y) {
            return driveTangent(x, y, 0.0);
        }

        /** As {@link #driveTangent(double, double)}, keeping the (ignored) heading the old code wrote. */
        public Builder driveTangent(double x, double y, double nominalHeadingDeg) {
            return addDrive(x, y, nominalHeadingDeg, HeadingMode.TANGENT, null, false);
        }

        /** Back (shooter) of the robot along the travel direction, i.e. driving backwards (heading ignored). */
        public Builder driveReverseTangent(double x, double y) {
            return driveReverseTangent(x, y, 0.0);
        }

        /** As {@link #driveReverseTangent(double, double)}, keeping the (ignored) heading the old code wrote. */
        public Builder driveReverseTangent(double x, double y, double nominalHeadingDeg) {
            return addDrive(x, y, nominalHeadingDeg, HeadingMode.REVERSE_TANGENT, null, false);
        }

        /** Turn evenly from the heading the leg starts with to the given heading. */
        public Builder driveLinear(double x, double y, double headingDeg) {
            return addDrive(x, y, headingDeg, HeadingMode.LINEAR, null, false);
        }

        /** Turn to the given heading at the start and hold it along the leg (strafing if needed). */
        public Builder driveConstant(double x, double y, double headingDeg) {
            return addDrive(x, y, headingDeg, HeadingMode.CONSTANT, null, false);
        }

        /** Keep the FRONT of the robot facing the point (px, py) along the whole leg. */
        public Builder driveFacing(double x, double y, double px, double py) {
            return addDrive(x, y, 0.0, HeadingMode.FACE_POINT, new Point2(px, py), false);
        }

        /** Keep the BACK (shooter) of the robot facing the point (px, py) along the whole leg. */
        public Builder driveBackFacing(double x, double y, double px, double py) {
            return addDrive(x, y, 0.0, HeadingMode.FACE_POINT, new Point2(px, py), true);
        }

        private Builder addDrive(double x, double y, double headingDeg, HeadingMode mode, Point2 face, boolean back) {
            steps.add(new Step.Drive(Collections.<Point2>emptyList(), Pose2.ofDegrees(x, y, headingDeg), mode, face,
                    back, 1.0, "", false, ""));
            return this;
        }

        /** Adds a Bezier control point to the DRIVE added last: the path bends toward (x, y) without passing through it. */
        public Builder via(double x, double y) {
            Step.Drive d = lastDrive("via");
            List<Point2> v = new ArrayList<Point2>(d.via());
            v.add(new Point2(x, y));
            return replaceLast(d.withVia(v));
        }

        /** Drives the DRIVE added last at this fraction (0, 1] of the cruise speed. */
        public Builder speed(double fraction) {
            return replaceLast(lastDrive("speed").withSpeed(fraction));
        }

        /** Names the DRIVE added last. */
        public Builder label(String label) {
            return replaceLast(lastDrive("label").withLabel(label));
        }

        /**
         * Marks the DRIVE added last as touching a wall on purpose (a note for people and the picture). It buys NO
         * extra room: touching a wall is legal, pushing into it is an ERROR for this leg like for any other.
         */
        public Builder wallContact() {
            return replaceLast(lastDrive("wallContact").withWallContact(true));
        }

        // ---- INTAKE ----------------------------------------------------------------------------

        public Builder intakeFloor(double targetX, double targetY, double robotX, double robotY, double robotHeadingDeg,
                                   int count) {
            return intake(IntakeSource.FLOOR, targetX, targetY, robotX, robotY, robotHeadingDeg,
                    BioBuzzConstants.Auto.INTAKE_DWELL_FLOOR_SEC, count);
        }

        public Builder intakeFlower(double targetX, double targetY, double robotX, double robotY, double robotHeadingDeg,
                                    int count) {
            return intake(IntakeSource.FLOWER, targetX, targetY, robotX, robotY, robotHeadingDeg,
                    BioBuzzConstants.Auto.INTAKE_DWELL_FLOWER_SEC, count);
        }

        public Builder intakeGarden(double targetX, double targetY, double robotX, double robotY, double robotHeadingDeg,
                                    int count) {
            return intake(IntakeSource.GARDEN, targetX, targetY, robotX, robotY, robotHeadingDeg,
                    BioBuzzConstants.Auto.INTAKE_DWELL_GARDEN_SEC, count);
        }

        /** General INTAKE: where from, what point, which robot pose, how long, how many POLLEN. */
        public Builder intake(IntakeSource source, double targetX, double targetY, double robotX, double robotY,
                              double robotHeadingDeg, double dwellSec, int count) {
            steps.add(new Step.Intake(source, new Point2(targetX, targetY),
                    Pose2.ofDegrees(robotX, robotY, robotHeadingDeg), dwellSec, count, ""));
            return this;
        }

        // ---- SHOOT -----------------------------------------------------------------------------

        /** Fire {@code count} POLLEN at the UP cell from the pose the solver picks at runtime (AUTO pose). */
        public Builder shootUp(int count) {
            steps.add(new Step.Shoot(ShootTarget.upCell(), null, count, Double.NaN, ""));
            return this;
        }

        /** Fire {@code count} POLLEN at the UP cell from an explicit pose. */
        public Builder shootUp(double x, double y, double headingDeg, int count) {
            steps.add(new Step.Shoot(ShootTarget.upCell(), Pose2.ofDegrees(x, y, headingDeg), count, Double.NaN, ""));
            return this;
        }

        /** Fire at a fixed side of the own HIVE (it must be the UP side at that moment, rule G417). */
        public Builder shootSide(Side side, int count) {
            steps.add(new Step.Shoot(ShootTarget.fixedSide(side), null, count, Double.NaN, ""));
            return this;
        }

        public Builder shootSide(Side side, double x, double y, double headingDeg, int count) {
            steps.add(new Step.Shoot(ShootTarget.fixedSide(side), Pose2.ofDegrees(x, y, headingDeg), count, Double.NaN,
                    ""));
            return this;
        }

        /** Makes the SHOOT added last take exactly this long, instead of the timing model's estimate. */
        public Builder launchSeconds(double seconds) {
            Step last = last("launchSeconds");
            if (!(last instanceof Step.Shoot)) {
                throw new IllegalStateException("launchSeconds() applies to a SHOOT, but the last step is " + last.kind());
            }
            return replaceLast(((Step.Shoot) last).withFixedSeconds(seconds));
        }

        // ---- WAIT and PARK ---------------------------------------------------------------------

        public Builder waitSeconds(double seconds) {
            steps.add(new Step.Wait(seconds, ""));
            return this;
        }

        /** Finish partly inside the own LOADING ZONE at this pose. */
        public Builder park(double x, double y, double headingDeg) {
            steps.add(new Step.Park(Pose2.ofDegrees(x, y, headingDeg), ""));
            return this;
        }

        // ---- any step --------------------------------------------------------------------------

        /** Sets the free-text note of the step added last. */
        public Builder note(String note) {
            return replaceLast(last("note").withNote(note));
        }

        // ---- build -----------------------------------------------------------------------------

        public AutoRoute build() {
            if (steps.isEmpty()) {
                throw new IllegalStateException("route " + id + " has no steps");
            }
            if (steps.get(0).kind() != StepKind.START) {
                throw new IllegalStateException("route " + id + " must begin with a START step");
            }
            for (int i = 1; i < steps.size(); i++) {
                if (steps.get(i).kind() == StepKind.START) {
                    throw new IllegalStateException("route " + id + " has a second START at step " + (i + 1));
                }
            }
            return new AutoRoute(id, title == null ? id : title, description == null ? "" : description,
                    startTile == null ? "" : startTile, Alliance.RED, steps);
        }

        private Step last(String what) {
            if (steps.isEmpty()) {
                throw new IllegalStateException(what + "() needs a step to apply to, but none was added yet");
            }
            return steps.get(steps.size() - 1);
        }

        private Step.Drive lastDrive(String what) {
            Step last = last(what);
            if (!(last instanceof Step.Drive)) {
                throw new IllegalStateException(what + "() applies to a DRIVE, but the last step is " + last.kind());
            }
            return (Step.Drive) last;
        }

        private Builder replaceLast(Step replacement) {
            steps.set(steps.size() - 1, replacement);
            return this;
        }
    }
}
