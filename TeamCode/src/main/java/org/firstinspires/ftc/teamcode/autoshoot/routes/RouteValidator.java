package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.HiveSolid;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotShape;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;

/**
 * Checks a route against the real field geometry and the AUTO rules of the BIOBUZZ manual (TU09),
 * and reports every finding as an {@link Issue}. It answers "can this route work, and does it break a
 * rule?" before a robot ever drives it. It never throws for a bad route; it reports.
 *
 * <p>The checks (rule numbers are the manual's):
 * <ol>
 *   <li><b>START, G304.</b> The whole robot footprint (the perimeter rectangle, NOT centered on the pose point: see {@code RobotShape}, at the
 *       start heading) is inside the field and inside the own half, touches a perimeter wall (within
 *       {@code START_WALL_CONTACT_TOLERANCE_IN}), and is not in the own LOADING ZONE (G304.E), not touching
 *       a FLOWER (G304.D) and not attached to the HIVE frame (G304.B). A start that stands clear of a FLOWER plate but over
 *       the end of its wall bracket (it is 0.36 in longer than the plate, at a height the robot reaches) is a WARN.</li>
 *   <li><b>Every leg, 9.2/G412, G417/G409, G415.</b> Every DRIVE leg and every unplanned move to an INTAKE,
 *       SHOOT or PARK pose is swept by {@link Sweep}: sampled every {@code PATH_SAMPLE_STEP_IN} (and every
 *       {@code TURN_SAMPLE_STEP_DEG} while turning) and proven not to miss anything between two samples. The footprint
 *       (a rectangle: the corners count) must stay inside the field (it may touch a wall but never overlap it by more
 *       than {@code FIELD_BOUNDS_TOLERANCE_IN}, on every step), out of the HIVE frame grown by
 *       {@code HIVE_KEEPOUT_MARGIN_IN}, and out of every FLOWER plate (grown by {@code FLOWER_KEEPOUT_MARGIN_IN}) with
 *       ONE exception: the one flat wheel that presses into the FLOWER an INTAKE step takes POLLEN from may enter that
 *       plate, see the next item. A DRIVE with {@code allowUnderHive} is the other exception: it is checked against the
 *       exact HIVE solid (the foot bars and leg stubs, {@link HiveSolid}) with {@code HIVE_LANE_MARGIN_IN} on both sides
 *       instead of the whole frame, so it may drive along the lane between the feet; {@link TipRules} WARNs when the HIVE
 *       could still tip while the robot is there.</li>
 *   <li><b>FLOWER wiggle, 9.7/G415 ({@link FlowerPress}, {@link FlowerWiggle}).</b> A FLOWER INTAKE is a straight approach to the wall
 *       with one flat wheel beside the ball, a turn on the spot toward the ball and back (once per POLLEN) and a straight retreat. The poses
 *       are derived, so the approach is square to the wall and the retreat is straight by construction; what is checked is every pose of every
 *       leg, the TURN included, swept with the real footprint: the CHASSIS (the footprint without the front
 *       {@code Robot.INTAKE_REACH_IN}, where only the two flat wheels stick out) stays {@code FLOWER_CHASSIS_CLEARANCE_IN}
 *       off the plate at every pose of the swing, the pressing wheel's disc may be over the plate but never nearer the wall than the POLLEN's center line,
 *       the other wheel stays out of it, and nothing leaves the field. The wheel must be able to REACH the ball once the chassis has been stood far
 *       enough out to clear the plate through the swing (ERROR with the missing inches of wheel reach); with the wheel left to AUTO the choice
 *       ({@link WheelChoice}) must find a legal wheel, else it is an ERROR; a retreat shorter than {@code FLOWER_RETREAT_LENGTH_IN} is a WARN.</li>
 *   <li><b>AUTO sides, G402.</b> The robot center stays on the own side of the center line at all times.</li>
 *   <li><b>POLLEN, G407 and 10.3.1.</b> The robot starts with 4 pre-loads; an INTAKE adds, a SHOOT removes;
 *       more than 4 held, or firing more than is held, is an ERROR. Every FLOWER and GARDEN holds 4; taking
 *       from one that is empty is a WARN; taking from the other alliance's GARDEN or a FLOWER on the other
 *       half is an ERROR (G402).</li>
 *   <li><b>Time, G401.</b> The estimated total must fit in 30 s minus {@code TIME_SAFETY_MARGIN_SEC}: an ERROR
 *       beyond 30 s, a WARN inside the margin.</li>
 *   <li><b>End pose, 10.5.4/G402.</b> The robot ends outside the HIVE frame footprint (ERROR) and wholly on
 *       the own side (WARN if the footprint reaches over the center line). A robot that ends in the LOADING ZONE touching a wall
 *       (or within {@code LEAVE_WALL_MARGIN_IN} of it) keeps its PARK but loses its LEAVE (FIRST's Q&amp;A QA21: any touch of
 *       any perimeter wall at the end of AUTO, the one inside the zone too): WARN.</li>
 *   <li><b>SHOOT pose, G417.</b> An explicit shooting pose is a WARN unless the launch point is on the open
 *       side of the target cell, {@code SHOOT_STANDOFF_MIN_IN}..{@code SHOOT_STANDOFF_MAX_IN} from its aim
 *       point, with the back within {@code SHOOT_AIM_TOLERANCE_DEG} of it. A fixed-side SHOOT at a cell that is
 *       DOWN at that moment is a WARN (only an upward-facing CELL may be launched into). Which cell is UP is
 *       simulated: it starts per Fig 10-2 and flips after every TIP.</li>
 *   <li><b>PARK, 10.5.4.</b> A PARK step must end overlapping the own LOADING ZONE (ERROR), and overlapping
 *       it by less than {@code PARK_MIN_OVERLAP_IN} is a WARN.</li>
 *   <li><b>Unplanned moves.</b> An explicit INTAKE/SHOOT/PARK pose farther than {@code IMPLICIT_MOVE_WARN_IN}
 *       from where the robot is gets a WARN: add a DRIVE so the path is planned and checked.</li>
 *   <li><b>HIVE tips, G417 and G409.</b> A SHOOT must aim at a cell that faces UP when the POLLEN leave (a WARN for
 *       a cell that is DOWN; in a route that assumes the partner shoots the first tip, a SHOOT at the cell that comes
 *       up with the partner's tip is an ERROR unless a WAIT_FOR_TIP before it lasts until that tip is confirmed (the tip plus
 *       the settle time the camera needs), because the robot shoots only after its camera has SEEN the cell UP), a timed
 *       volley that goes on firing after the tip is a WARN, a tip with no POLLEN to spare is a WARN (QA46: plan one POLLEN more
 *       than FIRST's calibration needs), and for {@code TIP_SPILL_CLEAR_SEC} after every tip the robot stays out of the spill
 *       zone of the cell that tipped. A partner route with little slack (the latest partner tip that still works is close to
 *       the assumed one) gets a WARN. These live in {@link TipRules}.</li>
 * </ol>
 */
public final class RouteValidator {
    private static final double EPS = 1e-9;

    private final FieldModel model;
    private final TimingModel timing;

    public RouteValidator() {
        this(new FieldModel(), new TimingModel());
    }

    public RouteValidator(FieldModel model, TimingModel timing) {
        this.model = model;
        this.timing = timing;
    }

    /**
     * As {@link #RouteValidator(FieldModel, TimingModel)} for a robot whose flat intake wheels stick out
     * {@code intakeReachIn} inches in front of the chassis instead of {@code Robot.INTAKE_REACH_IN}. Used to ask "what if
     * the wheels are shorter (or longer) than hoped?" and by the tests. The wiggle of every FLOWER pick-up is derived for that reach
     * ({@link FlowerWiggle}), so the analysis, the wheel choice and the findings all see the same robot.
     */
    public RouteValidator(FieldModel model, TimingModel timing, double intakeReachIn) {
        this(model, timing.withFlowerSpec(timing.flowerSpec().withIntakeReach(intakeReachIn)));
    }

    /** Validates a route in the frame of its own alliance. */
    public List<Issue> validate(AutoRoute route) {
        RouteAnalysis a = RouteAnalysis.of(route, model, timing);
        List<Issue> out = new ArrayList<Issue>();
        checkStart(a, out);
        for (RouteAnalysis.StepInfo info : a.steps()) {
            checkMotion(a, info, out);
            checkPollen(a, info, out);
            checkShoot(a, info, out);
            checkPark(a, info, out);
            checkUnplannedMove(info, out);
        }
        TipRules.check(a, out);
        checkTime(a, out);
        checkEnd(a, out);
        Collections.sort(out, new Comparator<Issue>() {
            @Override
            public int compare(Issue x, Issue y) {
                int ix = x.stepIndex < 0 ? Integer.MAX_VALUE : x.stepIndex;
                int iy = y.stepIndex < 0 ? Integer.MAX_VALUE : y.stepIndex;
                if (ix != iy) {
                    return ix < iy ? -1 : 1;
                }
                return x.severity.compareTo(y.severity);
            }
        });
        return Collections.unmodifiableList(out);
    }

    /** Convenience: validate with the default field and timing model. */
    public static List<Issue> check(AutoRoute route) {
        return new RouteValidator().validate(route);
    }

    /** True if no ERROR is among the issues. */
    public static boolean isRunnable(List<Issue> issues) {
        for (Issue i : issues) {
            if (i.isError()) {
                return false;
            }
        }
        return true;
    }

    // ---- 1. START (G304) ---------------------------------------------------------------------------

    private void checkStart(RouteAnalysis a, List<Issue> out) {
        AllianceField f = a.field();
        Pose2 start = a.route().startPose();
        Footprint fp = Footprint.of(start);
        double tol = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
        String at = "start footprint at " + pose(start);

        double outside = fp.overshootOutside(f.fieldRect());
        if (outside > tol) {
            out.add(error(0, Rule.START_G304, at + " sticks out of the field by " + in(outside) + " (G304.A)"
                    + wallHint(start, f.fieldRect())));
        }
        double past = 0.0;
        for (int i = 0; i < 4; i++) {
            past = Math.max(past, f.pastCenterLine(fp.corner(i).x));
        }
        if (past > tol) {
            out.add(error(0, Rule.START_G304, at + " reaches " + in(past)
                    + " into the opponent half; the robot must start fully inside its own columns (G304.A)"));
        }
        Rect b = fp.bounds();
        double size = f.fieldRect().maxX;
        double gap = Math.min(Math.min(b.minX, b.minY), Math.min(size - b.maxX, size - b.maxY));
        if (gap > BioBuzzConstants.Auto.START_WALL_CONTACT_TOLERANCE_IN) {
            out.add(error(0, Rule.START_G304, at + " is " + in(gap) + " away from the nearest wall; it must touch a"
                    + " perimeter wall (G304.C, tolerance " + in(BioBuzzConstants.Auto.START_WALL_CONTACT_TOLERANCE_IN) + ")"));
        }
        if (fp.overlaps(f.ownLoadingZone())) {
            out.add(error(0, Rule.START_G304, at + " overlaps the own LOADING ZONE " + f.ownLoadingZone()
                    + "; no robot may start there (G304.E)"));
        }
        for (int i = 0; i < f.flowerPlates().size(); i++) {
            if (fp.overlaps(f.flowerPlates().get(i))) {
                out.add(error(0, Rule.START_G304, at + " touches FLOWER " + (i + 1)
                        + "; a start may not contact a FLOWER (G304.D)"));
            } else if (BioBuzzConstants.Robot.HEIGHT_IN > BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN
                    && fp.overlaps(f.flowerBracket(i))) {
                // The bracket is longer than the plate, so a robot can stand clear of the plate and still be over the bracket.
                out.add(warn(0, Rule.START_G304, at + " is over the wall bracket of FLOWER " + (i + 1) + " (it sticks out "
                        + in(BioBuzzConstants.Field.FLOWER_BRACKET_PROTRUSION_IN) + " from the wall, " + in(BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN)
                        + " along it, at " + in(BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN) + " to " + in(BioBuzzConstants.Field.FLOWER_BRACKET_TOP_Z_IN)
                        + " above the floor): any part of the robot taller than " + in(BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN)
                        + " there would be touching the FLOWER (G304.D). Slide the start along the wall"));
            }
        }
        if (fp.overlaps(f.hiveFrame())) {
            out.add(error(0, Rule.START_G304, at + " overlaps the HIVE frame " + f.hiveFrame()
                    + "; a start may not be attached to a FIELD element (G304.B)"));
        }
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

    // ---- 2 and 3. every leg: field, HIVE, FLOWERS, sides -----------------------------------------------

    private void checkMotion(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        FlowerPress press = info.press;
        DrivePath leg = info.path != null ? info.path : info.approach;
        if (leg != null) {
            // a DRIVE, or the implicit move to a pose (for a FLOWER: to its line-up pose): the WHOLE robot is checked
            checkLeg(a, info, leg, describe(info), null, out);
        }
        if (press != null) {
            // the pick-up itself: here, and only here, the plate of this one FLOWER is judged by the pick-up rules (the chassis keeps its clearance, only the
            // pressing wheel may be over the plate) instead of "stay out"; the SWING is a leg like the others, swept at every pose of the turn
            String pickup = describeStep(info);
            checkLeg(a, info, press.approach, pickup + ", the straight approach to the FLOWER", press, out);
            checkLeg(a, info, press.swing, pickup + ", the wiggle (the turn on the spot toward the POLLEN)", press, out);
            checkLeg(a, info, press.retreatPath, pickup + ", the retreat from the FLOWER", press, out);
            checkPress(a, info, press, out);
        }
    }

    /**
     * Sweeps every rule over one leg ({@link LegSweeps}) and reports what it finds. With {@code press == null} the whole footprint must stay out of every
     * FLOWER plate. With a press, the plate of THAT FLOWER is treated by the pick-up rules instead: the chassis may not come within
     * {@code FLOWER_CHASSIS_CLEARANCE_IN} of it at ANY pose of the leg, the swing included (I2), the pressing wheel disc may be over it but never nearer the
     * wall than the POLLEN's center line (I3) and the other wheel disc may not enter it at all. The walls, the HIVE keep-out, the other plates and the
     * center line apply to the whole robot as always.
     */
    private void checkLeg(RouteAnalysis a, RouteAnalysis.StepInfo info, DrivePath leg, String what, FlowerPress press,
                          List<Issue> out) {
        final AllianceField f = a.field();
        // Only the planned path of a DRIVE that says so may use the lane under the HIVE; the implicit moves to a pose may not.
        final boolean underHive = info.step instanceof Step.Drive && ((Step.Drive) info.step).allowsUnderHive() && leg == info.path;
        final HiveSolid solid = f.model().hiveSolid();
        final double laneMargin = BioBuzzConstants.Auto.HIVE_LANE_MARGIN_IN;
        final double fieldTol = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
        LegSweeps sweeps = LegSweeps.run(f, leg, press, underHive);
        Sweep.Result bounds = sweeps.bounds;
        Sweep.Result hive = sweeps.hive;
        Sweep.Result side = sweeps.side;
        Sweep.Result[] flowers = sweeps.flowers;
        Sweep.Result chassis = sweeps.chassis;
        Sweep.Result pressing = sweeps.pressing;
        Sweep.Result otherWheel = sweeps.otherWheel;

        if (bounds.broken()) {
            out.add(error(info.index, Rule.FIELD_BOUNDS, what + ": the robot footprint leaves the field by up to "
                    + in(bounds.worst + fieldTol + EPS) + " (first at " + pose(bounds.firstPose) + "); a robot cannot drive or turn through"
                    + " a wall"));
        }
        if (hive.broken() && underHive) {
            out.add(error(info.index, Rule.HIVE_KEEPOUT, what + ": the robot footprint comes within " + in(laneMargin) + " of the HIVE's foot bars or"
                    + " leg stubs (it may drive under the HIVE, along the lane between the feet, but it needs " + in(laneMargin)
                    + " to spare on both sides) from " + in(hive.firstArc) + " to " + in(hive.lastArc) + " along the move, first at "
                    + pose(hive.firstPose) + "; the lane is " + in(solid.narrowestLane()[1] - solid.narrowestLane()[0]) + " wide where the leg stubs"
                    + " narrow it, x " + in(solid.narrowestLane()[0]) + " to " + in(solid.narrowestLane()[1])));
        } else if (hive.broken()) {
            out.add(error(info.index, Rule.HIVE_KEEPOUT, what + ": the robot footprint enters the HIVE frame keep-out ("
                    + "frame + " + in(BioBuzzConstants.Auto.HIVE_KEEPOUT_MARGIN_IN) + ") from "
                    + in(hive.firstArc) + " to " + in(hive.lastArc) + " along the move, first at " + pose(hive.firstPose)
                    + "; a robot is kept out of the whole HIVE unless the DRIVE says allowUnderHive"));
        }
        for (int i = 0; i < flowers.length; i++) {
            if (flowers[i].broken()) {
                out.add(error(info.index, Rule.FLOWER_KEEPOUT, what + ": the robot footprint runs into the plate of FLOWER "
                        + (i + 1) + " (first at " + pose(flowers[i].firstPose) + "); only the wheels of an INTAKE step that"
                        + " takes POLLEN from this FLOWER may be over its plate, and only during its wiggle"));
            }
        }
        if (chassis.broken()) {
            double reach = press.spec.intakeReachIn;
            out.add(error(info.index, Rule.FLOWER_KEEPOUT, what + ": the CHASSIS (the robot body, without the " + in(reach)
                    + " the flat wheels stick out in front) comes within " + in(press.spec.chassisClearanceIn)
                    + " of the plate of FLOWER " + (press.flower + 1) + " by up to " + in(chassis.worst + EPS) + " (first at "
                    + pose(chassis.firstPose) + "); only a wheel may be over the plate, and the turn of the wiggle swings the chassis corner toward it"));
        }
        if (pressing.broken()) {
            out.add(error(info.index, Rule.FLOWER_PRESS, what + ": the " + press.wheel + " wheel goes " + in(pressing.worst + EPS)
                    + " past the press stop line of FLOWER " + (press.flower + 1) + " (" + in(f.flowerPressStopLineIn())
                    + " from the wall: the POLLEN's center line), first at " + pose(pressing.firstPose)
                    + "; the wheel must stay beside and in front of the ball, never deeper than its middle"));
        }
        if (otherWheel.broken()) {
            out.add(error(info.index, Rule.FLOWER_KEEPOUT, what + ": the " + press.wheel.other() + " wheel (the one that does NOT"
                    + " press) enters the plate of FLOWER " + (press.flower + 1) + " (first at " + pose(otherWheel.firstPose)
                    + "); only the ONE pressing wheel may be over it"));
        }
        if (side.broken()) {
            out.add(error(info.index, Rule.SIDE_G402, what + ": the robot center crosses the center line into the opponent"
                    + " half by up to " + in(side.worst + EPS) + " (first at " + pose(side.firstPose) + "); in AUTO a "
                    + a.route().alliance() + " robot must stay on its own side"));
        }
    }

    /**
     * What a FLOWER pick-up needs besides the geometry of its legs: that the wheel can actually REACH the ball (ERROR, with the missing inches of wheel
     * reach), that the wheel was chosen (ERROR if AUTO finds neither wheel legal) and a retreat that is long enough (WARN).
     */
    private void checkPress(RouteAnalysis a, RouteAnalysis.StepInfo info, FlowerPress press, List<Issue> out) {
        String what = describeStep(info) + " at FLOWER " + (press.flower + 1);
        WheelChoice.Choice choice = info.choice;
        if (choice != null && choice.requested.isAuto() && !choice.anyLegal()) {
            out.add(error(info.index, Rule.FLOWER_PRESS, what + ": " + choice.reason()));
        } else if (!press.reaches()) {
            out.add(error(info.index, Rule.FLOWER_PRESS, what + ": the " + press.wheel + " wheel cannot reach the POLLEN: " + press.solution.shortfallText()));
        }
        double minRetreat = BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN;
        if (press.retreatLengthIn < minRetreat - EPS) {
            out.add(warn(info.index, Rule.FLOWER_PRESS, what + ": the retreat is only " + in(press.retreatLengthIn)
                    + "; back out straight for at least " + in(minRetreat) + " so the wheels are clear of the POLLEN and the plate"));
        }
    }

    // ---- 4. POLLEN (G407, 10.3.1, G402) ----------------------------------------------------------------

    private void checkPollen(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        if (info.step instanceof Step.Intake) {
            Step.Intake intake = (Step.Intake) info.step;
            if (info.pollenAfter > BioBuzzConstants.Auto.POLLEN_CAPACITY) {
                out.add(error(info.index, Rule.CAPACITY_G407, describe(info) + ": the robot would hold " + info.pollenAfter
                        + " POLLEN (" + info.pollenBefore + " + " + intake.expectedCount() + "); at most "
                        + BioBuzzConstants.Auto.POLLEN_CAPACITY + " may be controlled at once"));
            }
            if (intake.source() == IntakeSource.FLOOR) {
                boolean shotBefore = false;
                for (int i = 0; i < info.index; i++) {
                    shotBefore |= a.step(i).step instanceof Step.Shoot;
                }
                if (!shotBefore) {
                    out.add(warn(info.index, Rule.SOURCE, describe(info) + ": no POLLEN lies on the floor at match start"
                            + " (all 40 are in FLOWERS, GARDENS or robots); a floor intake only finds POLLEN that earlier"
                            + " shots missed or a HIVE tip spilled"));
                }
            } else if (info.sourceName == null) {
                out.add(warn(info.index, Rule.SOURCE, describe(info) + ": the target point " + intake.target() + " is not within "
                        + in(BioBuzzConstants.Auto.INTAKE_TARGET_MATCH_RADIUS_IN) + " of any "
                        + (intake.source() == IntakeSource.FLOWER ? "FLOWER" : "GARDEN") + "; there is nothing to take"));
            }
            if (info.sourceOffLimits) {
                out.add(error(info.index, Rule.SOURCE, describe(info) + ": takes POLLEN from " + info.sourceName
                        + ", which lies on the opposing side; controlling elements there in AUTO is a MAJOR FOUL (G402)"));
            } else if (info.sourceName != null && info.sourceRemainingBefore >= 0
                    && info.sourceRemainingBefore < intake.expectedCount()) {
                out.add(warn(info.index, Rule.SOURCE, describe(info) + ": expects +" + intake.expectedCount() + " from "
                        + info.sourceName + " but only " + info.sourceRemainingBefore + " POLLEN are left there"));
            }
        } else if (info.step instanceof Step.Shoot) {
            Step.Shoot sh = (Step.Shoot) info.step;
            if (info.pollenBefore <= 0) {
                out.add(error(info.index, Rule.CAPACITY_G407, describe(info) + ": fires " + sh.count()
                        + " POLLEN but the robot holds none"));
            } else if (sh.count() > info.pollenBefore) {
                out.add(error(info.index, Rule.CAPACITY_G407, describe(info) + ": fires " + sh.count()
                        + " POLLEN but the robot holds only " + info.pollenBefore));
            }
        }
    }

    // ---- 7. SHOOT pose and target ------------------------------------------------------------------------

    private void checkShoot(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Shoot)) {
            return;
        }
        Step.Shoot sh = (Step.Shoot) info.step;
        // (Whether the target cell faces UP when the POLLEN leave is checked with the other tip rules, in TipRules.)
        if (sh.isAutoPose()) {
            return;     // the solver picks a pose on the standoff band by construction
        }
        CellSpec cell = a.field().cell(info.targetSide);
        Pose2 p = info.shootPose;
        String what = describe(info) + " from " + pose(p);
        if (!ShotGeometry.onOpeningSide(p, cell)) {
            out.add(warn(info.index, Rule.SHOOT_POSE, what + ": the launch point is on the wrong side of the HIVE; the "
                    + cell + " opens toward " + (cell.side == Side.AUDIENCE ? "the audience wall" : "the rear wall")));
        }
        double d = ShotGeometry.standoff(p, cell);
        if (d < BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN || d > BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN) {
            out.add(warn(info.index, Rule.SHOOT_POSE, what + ": the launch point is " + in(d) + " from the aim point of the "
                    + cell + "; the standoff band is " + in(BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN) + " to "
                    + in(BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN) + " for the fixed-angle shooter"));
        }
        double aim = Math.toDegrees(ShotGeometry.aimError(p, cell));
        if (aim > BioBuzzConstants.Auto.SHOOT_AIM_TOLERANCE_DEG) {
            out.add(warn(info.index, Rule.SHOOT_POSE, what + ": the back of the robot is " + String.format(Locale.ROOT, "%.0f", aim)
                    + " deg away from the aim point of the " + cell + " (tolerance "
                    + String.format(Locale.ROOT, "%.0f", BioBuzzConstants.Auto.SHOOT_AIM_TOLERANCE_DEG) + " deg)"));
        }
    }

    // ---- 8. PARK -------------------------------------------------------------------------------------------

    private void checkPark(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Park)) {
            return;
        }
        Pose2 p = ((Step.Park) info.step).pose();
        Rect zone = a.field().ownLoadingZone();
        Rect clipped = Footprint.of(p).clippedBounds(zone);
        if (clipped == null) {
            out.add(error(info.index, Rule.PARK_ZONE, describe(info) + ": the robot at " + pose(p)
                    + " is not even partly inside the own LOADING ZONE " + zone + "; that earns no PARK (10.5.4)"));
            return;
        }
        double depth = Math.min(clipped.maxX - clipped.minX, clipped.maxY - clipped.minY);
        if (depth < BioBuzzConstants.Auto.PARK_MIN_OVERLAP_IN) {
            out.add(warn(info.index, Rule.PARK_ZONE, describe(info) + ": the robot at " + pose(p) + " overlaps the LOADING ZONE"
                    + " by only " + in(depth) + "; with the +/-1 in field tolerance a PARK needs at least "
                    + in(BioBuzzConstants.Auto.PARK_MIN_OVERLAP_IN) + " to be safe (Fig 10-7)"));
        }
    }

    // ---- 9. unplanned moves --------------------------------------------------------------------------------

    private void checkUnplannedMove(RouteAnalysis.StepInfo info, List<Issue> out) {
        if (info.approach == null) {
            return;
        }
        // A FLOWER pick-up has no authored pose: its line-up pose is derived, and the move to it is a planned, swept leg like any other.
        boolean explicit = (info.step instanceof Step.Intake && info.press == null) || info.step instanceof Step.Park
                || (info.step instanceof Step.Shoot && !info.shootPoseIsNominal);
        Pose2 target = info.approach.end();
        double d = info.from.distanceTo(target);
        if (explicit && d > BioBuzzConstants.Auto.IMPLICIT_MOVE_WARN_IN) {
            out.add(warn(info.index, Rule.IMPLICIT_MOVE, describe(info) + ": the pose " + pose(target) + " is " + in(d)
                    + " from where the previous step leaves the robot " + pose(info.from)
                    + "; add a DRIVE so that the path is planned and checked"));
        }
    }

    // ---- 5. time (G401) ------------------------------------------------------------------------------------

    private void checkTime(RouteAnalysis a, List<Issue> out) {
        double total = a.totalSeconds();
        double limit = BioBuzzConstants.Auto.AUTO_DURATION_SEC;
        double margin = BioBuzzConstants.Auto.TIME_SAFETY_MARGIN_SEC;
        if (total > limit) {
            out.add(error(-1, Rule.TIME_G401, "the estimated time " + sec(total) + " is longer than the " + sec(limit)
                    + " AUTO period (G401); the robot would be stopped mid-route"));
        } else if (total > limit - margin) {
            out.add(warn(-1, Rule.TIME_G401, "the estimated time " + sec(total) + " is inside the " + sec(margin)
                    + " safety margin before the " + sec(limit) + " limit; the timing numbers are placeholders"));
        }
    }

    // ---- 6. end pose -----------------------------------------------------------------------------------------

    private void checkEnd(RouteAnalysis a, List<Issue> out) {
        AllianceField f = a.field();
        int last = a.steps().size() - 1;
        Pose2 end = a.endPose();
        Footprint fp = Footprint.of(end);
        if (fp.overlaps(f.hiveFrame())) {
            out.add(error(last, Rule.END_POSE, "the route ends at " + pose(end) + " inside the HIVE frame footprint "
                    + f.hiveFrame() + "; TELEOP would start with the robot caught under the HIVE (G409/G417)"));
        }
        if (a.park() && !a.leave()) {
            // QA21: LEAVE is judged at the end of AUTO and any touch of the perimeter wall loses it, also the wall in the LOADING ZONE
            double margin = BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN;
            out.add(warn(last, Rule.LEAVE_QA21, "the robot ends at " + pose(end) + " in the LOADING ZONE but only " + in(Math.max(0.0, a.endWallGapIn()))
                    + " from the " + a.endNearestWall() + ": PARK (" + BioBuzzConstants.Auto.PARK_POINTS + ") still counts, but a robot that touches ANY wall at the end of AUTO,"
                    + " the one inside the LOADING ZONE too, loses its LEAVE (" + BioBuzzConstants.Auto.LEAVE_POINTS + ", FIRST's Q&A QA21). To claim both, end"
                    + " " + in(margin) + " or more off every wall: the zone is " + in(BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN)
                    + " deep, so a robot standing " + in(margin) + " off its wall still reaches " + in(BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - margin)
                    + " into it"));
        }
        double reach = 0.0;
        for (int i = 0; i < 4; i++) {
            reach = Math.max(reach, f.pastCenterLine(fp.corner(i).x));
        }
        if (reach > EPS && f.pastCenterLine(end.x) <= EPS) {
            out.add(warn(last, Rule.END_POSE, "the route ends at " + pose(end) + " with the robot reaching " + in(reach)
                    + " over the center line into the opponent half"));
        }
    }

    // ---- helpers ---------------------------------------------------------------------------------------------

    /** "step 3 (INTAKE)": the step by itself, whatever leg of it is being talked about. */
    static String describeStep(RouteAnalysis.StepInfo info) {
        return "step " + (info.index + 1) + " (" + info.step.kind() + ")";
    }

    static String describe(RouteAnalysis.StepInfo info) {
        String label = info.step instanceof Step.Drive && !((Step.Drive) info.step).label().isEmpty()
                ? " '" + ((Step.Drive) info.step).label() + "'" : "";
        String suffix = info.path == null && info.approach != null ? ", the move to its pose" : "";
        return "step " + (info.index + 1) + " (" + info.step.kind() + label + suffix + ")";
    }

    static String pose(Pose2 p) {
        return String.format(Locale.ROOT, "(%.1f, %.1f, %.0f deg)", p.x, p.y, p.headingDegrees());
    }

    static String in(double v) {
        return String.format(Locale.ROOT, "%.1f in", v);
    }

    static String sec(double v) {
        return String.format(Locale.ROOT, "%.1f s", v);
    }

    private static Issue error(int step, Rule rule, String message) {
        return new Issue(Severity.ERROR, step, rule, message);
    }

    private static Issue warn(int step, Rule rule, String message) {
        return new Issue(Severity.WARN, step, rule, message);
    }
}
