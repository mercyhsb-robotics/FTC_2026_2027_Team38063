package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
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
 *     .start(35.0, 9.77, -90)         // front edge ON the audience wall: the center of rotation is 9.77 in from it
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
    private final FirstTip firstTip;
    private final List<Step> steps;

    private AutoRoute(String id, String title, String description, String startTile, Alliance alliance,
                      FirstTip firstTip, List<Step> steps) {
        this.id = id;
        this.title = title;
        this.description = description;
        this.startTile = startTile;
        this.alliance = alliance;
        this.firstTip = firstTip;
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

    /**
     * Who the route assumes shoots the first tip of the HIVE: this robot ({@link FirstTip#SELF}, the default) or the
     * alliance partner ({@link FirstTip#PARTNER}).
     */
    public FirstTip firstTip() {
        return firstTip;
    }

    /**
     * Route time in seconds at which the partner's tip is assumed to be done
     * ({@code Auto.PARTNER_FIRST_TIP_DONE_SEC}) when the route assumes {@link FirstTip#PARTNER}; NaN for SELF.
     */
    public double partnerTipSec() {
        return firstTip == FirstTip.PARTNER ? BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC : Double.NaN;
    }

    /** One line for people: "we shoot the first tip" or "partner tips first, assumed at 12.0 s". */
    public String firstTipSummary() {
        return firstTip.summary(partnerTipSec());
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
     * The same route started from another pose (in this route's own alliance frame): every step after the START stays exactly as it is, and so do the
     * id, the title, the notes and the start tile's name (a label of where the route was written to start, not a computed value). It is how the search
     * for a better place to set the robot up, and the test "what if the robot is placed an inch off?", ask the validator about the same route from a
     * different start.
     */
    public AutoRoute withStartPose(Pose2 start) {
        List<Step> moved = new ArrayList<Step>(steps);
        moved.set(0, new Step.Start(start, steps.get(0).note()));
        return new AutoRoute(id, title, description, startTile, alliance, firstTip, moved);
    }

    /**
     * This route in the given alliance's frame. For the same alliance it returns this route; for the
     * other one it returns a new route with every pose and point turned half a turn about the field center (Field.CENTER_IN, CENTER_IN) and
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
        return new AutoRoute(id, title, description, startTile, target, firstTip, turned);
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
     * AUTO points OUR robot can claim if everything goes to plan: LEAVE 3 if it ends off every wall,
     * PARK 5 if it ends overlapping the own LOADING ZONE, 20 per HIVE TIP its own volleys cause. A tip that the
     * alliance partner is assumed to shoot ({@link FirstTip#PARTNER}) is not counted here, see
     * {@link RouteAnalysis#partnerTipPoints()}.
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
     * {@link #label}, {@link #wallContact}, {@link #allowUnderHive}, {@link #launchSeconds}, {@link #note}) change the step
     * added last. {@link #build()} checks the structure (one START first, sane numbers); whether the
     * route is a good idea on the real field is the job of {@link RouteValidator}.
     */
    public static final class Builder {
        private final String id;
        private String title;
        private String description = "";
        private String startTile = "";
        private FirstTip firstTip = FirstTip.SELF;
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

        /**
         * Says who shoots the first tip of the HIVE. The default is {@link FirstTip#SELF}: this robot's pre-loads tip
         * it. See {@link #partnerTipsFirst()} for the other family.
         */
        public Builder firstTip(FirstTip firstTip) {
            if (firstTip == null) {
                throw new IllegalArgumentException("firstTip must not be null");
            }
            this.firstTip = firstTip;
            return this;
        }

        /**
         * This route assumes the alliance PARTNER shoots the first tip: the HIVE tips by itself at
         * {@code Auto.PARTNER_FIRST_TIP_DONE_SEC} and the OTHER cell comes up then. Our robot goes to that side
         * and must not launch into it before (use {@link #waitForTip}). The drive team must agree the time with
         * the partner team before the match.
         */
        public Builder partnerTipsFirst() {
            return firstTip(FirstTip.PARTNER);
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

        /**
         * Lets the DRIVE added last go UNDER the HIVE, along the open lane between its two foot bars (a 12 to 13 in tall robot fits under the
         * cells; the lane saves about 20 in against going around the frame). Without this flag every leg must stay out of the whole HIVE
         * footprint (G409 / G417: never be under a HIVE that is tipping). With it the leg is checked against the exact solid, with
         * {@code Auto.HIVE_LANE_MARGIN_IN} on both sides, and the validator WARNs if the HIVE could still tip while the robot is under it.
         * Put a {@link #waitForTip} (or a shot you know has not tipped it) before it so the camera has seen the HIVE at rest.
         */
        public Builder allowUnderHive() {
            return replaceLast(lastDrive("allowUnderHive").withAllowUnderHive(true));
        }

        // ---- INTAKE ----------------------------------------------------------------------------

        public Builder intakeFloor(double targetX, double targetY, double robotX, double robotY, double robotHeadingDeg,
                                   int count) {
            return intake(IntakeSource.FLOOR, targetX, targetY, robotX, robotY, robotHeadingDeg,
                    BioBuzzConstants.Auto.INTAKE_DWELL_FLOOR_SEC, count);
        }

        /**
         * Empty a FLOWER by WIGGLING a flat intake wheel into its bottom POLLEN (see {@link FlowerPress} and {@link FlowerWiggle}). Give the FLOWER's center
         * and how many POLLEN you want; nothing else. The program works out the rest: where the robot stands (off center, beside the ball), how far it
         * turns toward the ball, the cycles, and which wheel does the work (the cheaper one for the legs before and after, see {@link WheelChoice}). It is
         * the same as {@code intakeFlower(targetX, targetY, IntakeWheel.AUTO, count)}. The robot goes to the line-up pose by an implicit straight move
         * that is checked like any other, so put a DRIVE before this step if the straight line would hit something. Use {@link #retreat} to change how
         * far the robot backs out afterwards.
         */
        public Builder intakeFlower(double targetX, double targetY, int count) {
            return intakeFlower(targetX, targetY, IntakeWheel.AUTO, count);
        }

        /** As {@link #intakeFlower(double, double, int)}, forcing the wheel: {@link IntakeWheel#LEFT} or {@link IntakeWheel#RIGHT} (the robot's own), or AUTO to let the program choose. */
        public Builder intakeFlower(double targetX, double targetY, IntakeWheel wheel, int count) {
            steps.add(Step.Intake.flower(new Point2(targetX, targetY), count, BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN, wheel, ""));
            return this;
        }

        /** FLOWER INTAKE added last: how far the robot backs straight out after the last cycle of the wiggle, in inches. */
        public Builder retreat(double inches) {
            Step last = last("retreat");
            if (!(last instanceof Step.Intake)) {
                throw new IllegalStateException("retreat() applies to an INTAKE, but the last step is " + last.kind());
            }
            return replaceLast(((Step.Intake) last).withRetreatLengthIn(inches));
        }

        public Builder intakeGarden(double targetX, double targetY, double robotX, double robotY, double robotHeadingDeg,
                                    int count) {
            return intake(IntakeSource.GARDEN, targetX, targetY, robotX, robotY, robotHeadingDeg,
                    BioBuzzConstants.Auto.INTAKE_DWELL_GARDEN_SEC, count);
        }

        /**
         * General INTAKE: where from, what point, which robot pose, how long, how many POLLEN. For a FLOWER the robot pose and the dwell are IGNORED (the
         * pose is derived and the time comes from the wiggle): it is the same as {@code intakeFlower(targetX, targetY, count)}.
         */
        public Builder intake(IntakeSource source, double targetX, double targetY, double robotX, double robotY,
                              double robotHeadingDeg, double dwellSec, int count) {
            if (source == IntakeSource.FLOWER) {
                return intakeFlower(targetX, targetY, count);
            }
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

        /**
         * Stand still until the camera has seen the HIVE tip, and give up at {@code giveUpSec} on the route clock (seconds
         * since the start of AUTO: {@code waitForTip(22.0)} means "stop waiting at 22 s"). Put it before the first volley at
         * the cell that comes up with the partner's tip, in a route that assumes {@link #partnerTipsFirst()}: that cell only
         * comes up after the tip, launching into a cell that faces down is a G417 violation, and a SHOOT at that cell
         * without a WAIT_FOR_TIP before it is an ERROR in the checker, whatever the clock says (partner teams tip at
         * different times, so the robot shoots after it SEES the tip, never at a time). If the deadline passes first, the
         * robot skips the volleys that need the tip and goes on with LEAVE and PARK. Pick the latest time that still lets
         * the rest of the route finish: the checker tells you ({@link RouteAnalysis#latestGiveUpSec}).
         */
        public Builder waitForTip(double giveUpSec) {
            steps.add(new Step.WaitForTip(giveUpSec, ""));
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
                    startTile == null ? "" : startTile, Alliance.RED, firstTip, steps);
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
