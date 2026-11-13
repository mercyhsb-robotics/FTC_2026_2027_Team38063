package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
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
 *   <li><b>START, G304.</b> The whole robot footprint (a rotated {@code LENGTH x WIDTH} rectangle at the
 *       start heading) is inside the field and inside the own half, touches a perimeter wall (within
 *       {@code START_WALL_CONTACT_TOLERANCE_IN}), and is not in the own LOADING ZONE (G304.E), not touching
 *       a FLOWER (G304.D) and not attached to the HIVE frame (G304.B).</li>
 *   <li><b>Every leg, 9.2/G412, G417/G409, G415.</b> Every DRIVE leg and every unplanned move to an INTAKE,
 *       SHOOT or PARK pose is sampled every {@code PATH_SAMPLE_STEP_IN} (and every
 *       {@code TURN_SAMPLE_STEP_DEG} while turning in place). The footprint must stay inside the field (a
 *       small allowance only on steps marked wall-contact), out of the HIVE frame grown by
 *       {@code HIVE_KEEPOUT_MARGIN_IN}, and out of the FLOWER plates except the FLOWER an INTAKE step
 *       targets.</li>
 *   <li><b>AUTO sides, G402.</b> The robot center stays on the own side of the center line at all times.</li>
 *   <li><b>POLLEN, G407 and 10.3.1.</b> The robot starts with 4 pre-loads; an INTAKE adds, a SHOOT removes;
 *       more than 4 held, or firing more than is held, is an ERROR. Every FLOWER and GARDEN holds 4; taking
 *       from one that is empty is a WARN; taking from the other alliance's GARDEN or a FLOWER on the other
 *       half is an ERROR (G402).</li>
 *   <li><b>Time, G401.</b> The estimated total must fit in 30 s minus {@code TIME_SAFETY_MARGIN_SEC}: an ERROR
 *       beyond 30 s, a WARN inside the margin.</li>
 *   <li><b>End pose, 10.5.4/G402.</b> The robot ends outside the HIVE frame footprint (ERROR) and wholly on
 *       the own side (WARN if the footprint reaches over the center line).</li>
 *   <li><b>SHOOT pose, G417.</b> An explicit shooting pose is a WARN unless the launch point is on the open
 *       side of the target cell, {@code SHOOT_STANDOFF_MIN_IN}..{@code SHOOT_STANDOFF_MAX_IN} from its aim
 *       point, with the back within {@code SHOOT_AIM_TOLERANCE_DEG} of it. A fixed-side SHOOT at a cell that is
 *       DOWN at that moment is a WARN (only an upward-facing CELL may be launched into). Which cell is UP is
 *       simulated: it starts per Fig 10-2 and flips after every TIP.</li>
 *   <li><b>PARK, 10.5.4.</b> A PARK step must end overlapping the own LOADING ZONE (ERROR), and overlapping
 *       it by less than {@code PARK_MIN_OVERLAP_IN} is a WARN.</li>
 *   <li><b>Unplanned moves.</b> An explicit INTAKE/SHOOT/PARK pose farther than {@code IMPLICIT_MOVE_WARN_IN}
 *       from where the robot is gets a WARN: add a DRIVE so the path is planned and checked.</li>
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
            out.add(error(0, Rule.START_G304, at + " sticks out of the field by " + in(outside) + " (G304.A)"));
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
            }
        }
        if (fp.overlaps(f.hiveFrame())) {
            out.add(error(0, Rule.START_G304, at + " overlaps the HIVE frame " + f.hiveFrame()
                    + "; a start may not be attached to a FIELD element (G304.B)"));
        }
    }

    // ---- 2 and 3. every leg: field, HIVE, FLOWERS, sides -----------------------------------------------

    /** Collects the samples of one leg that break one rule, so the report shows one line per leg and rule. */
    private static final class Run {
        int count;
        double firstArc;
        double lastArc;
        Pose2 firstPose;
        double worst;

        void add(double arc, Pose2 pose, double amount) {
            if (count == 0) {
                firstArc = arc;
                firstPose = pose;
            }
            count++;
            lastArc = arc;
            worst = Math.max(worst, amount);
        }
    }

    private void checkMotion(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        DrivePath leg = info.path != null ? info.path : info.approach;
        if (leg == null) {
            return;
        }
        AllianceField f = a.field();
        Step step = info.step;
        boolean wallContact = (step instanceof Step.Drive && ((Step.Drive) step).isWallContact())
                || (step instanceof Step.Intake && ((Step.Intake) step).source().isWallContact());
        double fieldTol = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN
                + (wallContact ? BioBuzzConstants.Auto.WALL_CONTACT_ALLOWANCE_IN : 0.0);
        boolean[] exempt = exemptFlowers(a, info);

        List<Pose2> poses = new ArrayList<Pose2>(
                leg.turnInPlaceSamples(Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG)));
        List<Double> arcs = new ArrayList<Double>();
        for (int i = 0; i < poses.size(); i++) {
            arcs.add(0.0);
        }
        poses.addAll(leg.sample(BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN));
        arcs.addAll(leg.sampleArcLengths(BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN));

        Run bounds = new Run();
        Run hive = new Run();
        Run side = new Run();
        Run[] flowers = new Run[f.flowerPlates().size()];
        for (int i = 0; i < flowers.length; i++) {
            flowers[i] = new Run();
        }
        Rect hiveKeepOut = f.hiveKeepOut();
        for (int k = 0; k < poses.size(); k++) {
            Pose2 p = poses.get(k);
            double arc = arcs.get(k);
            Footprint fp = Footprint.of(p);
            double over = fp.overshootOutside(f.fieldRect());
            if (over > fieldTol + EPS) {
                bounds.add(arc, p, over);
            }
            if (fp.overlaps(hiveKeepOut)) {
                hive.add(arc, p, 0);
            }
            for (int i = 0; i < flowers.length; i++) {
                if (!exempt[i] && fp.overlaps(f.flowerKeepOut(i))) {
                    flowers[i].add(arc, p, 0);
                }
            }
            double past = f.pastCenterLine(p.x);
            if (past > EPS) {
                side.add(arc, p, past);
            }
        }

        String what = describe(info);
        if (bounds.count > 0) {
            out.add(error(info.index, Rule.FIELD_BOUNDS, what + ": the robot footprint leaves the field by up to "
                    + in(bounds.worst) + " (first at " + pose(bounds.firstPose) + "); a robot cannot drive or turn through"
                    + " a wall"));
        }
        if (hive.count > 0) {
            out.add(error(info.index, Rule.HIVE_KEEPOUT, what + ": the robot footprint enters the HIVE frame keep-out ("
                    + "frame + " + in(BioBuzzConstants.Auto.HIVE_KEEPOUT_MARGIN_IN) + ") from "
                    + in(hive.firstArc) + " to " + in(hive.lastArc) + " along the move, first at " + pose(hive.firstPose)));
        }
        for (int i = 0; i < flowers.length; i++) {
            if (flowers[i].count > 0) {
                out.add(error(info.index, Rule.FLOWER_KEEPOUT, what + ": the robot footprint runs into the plate of FLOWER "
                        + (i + 1) + " (first at " + pose(flowers[i].firstPose) + "); only an INTAKE step may touch the"
                        + " FLOWER it takes POLLEN from"));
            }
        }
        if (side.count > 0) {
            out.add(error(info.index, Rule.SIDE_G402, what + ": the robot center crosses the center line into the opponent"
                    + " half by up to " + in(side.worst) + " (first at " + pose(side.firstPose) + "); in AUTO a "
                    + a.route().alliance() + " robot must stay on its own side"));
        }
    }

    /** FLOWERS the leg may touch: the one an INTAKE step right before or after (or the step itself) takes POLLEN from. */
    private boolean[] exemptFlowers(RouteAnalysis a, RouteAnalysis.StepInfo info) {
        boolean[] exempt = new boolean[a.field().flowerPlates().size()];
        for (int idx = info.index - 1; idx <= info.index + 1; idx++) {
            if (idx < 0 || idx >= a.steps().size()) {
                continue;
            }
            Step s = a.step(idx).step;
            boolean relevant = idx == info.index || (info.step instanceof Step.Drive);
            if (relevant && s instanceof Step.Intake && ((Step.Intake) s).source() == IntakeSource.FLOWER) {
                int f = a.field().nearestFlower(((Step.Intake) s).target());
                if (f >= 0) {
                    exempt[f] = true;
                }
            }
        }
        return exempt;
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
        if (!info.targetWasUp) {
            Side upNow = info.upSideBefore;
            out.add(warn(info.index, Rule.SHOOT_TARGET, describe(info) + ": fires at the " + info.targetSide
                    + " cell, but at this point the " + upNow + " cell is the UP one; launching into a cell that faces"
                    + " down is a G417 violation"));
        }
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
        boolean explicit = info.step instanceof Step.Intake || info.step instanceof Step.Park
                || (info.step instanceof Step.Shoot && !info.shootPoseIsNominal);
        double d = info.from.distanceTo(info.to);
        if (explicit && d > BioBuzzConstants.Auto.IMPLICIT_MOVE_WARN_IN) {
            out.add(warn(info.index, Rule.IMPLICIT_MOVE, describe(info) + ": the pose " + pose(info.to) + " is " + in(d)
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

    private static String describe(RouteAnalysis.StepInfo info) {
        String label = info.step instanceof Step.Drive && !((Step.Drive) info.step).label().isEmpty()
                ? " '" + ((Step.Drive) info.step).label() + "'" : "";
        String suffix = info.path == null && info.approach != null ? ", the move to its pose" : "";
        return "step " + (info.index + 1) + " (" + info.step.kind() + label + suffix + ")";
    }

    private static String pose(Pose2 p) {
        return String.format(Locale.ROOT, "(%.1f, %.1f, %.0f deg)", p.x, p.y, p.headingDegrees());
    }

    private static String in(double v) {
        return String.format(Locale.ROOT, "%.1f in", v);
    }

    private static String sec(double v) {
        return String.format(Locale.ROOT, "%.1f s", v);
    }

    private static Issue error(int step, Rule rule, String message) {
        return new Issue(Severity.ERROR, step, rule, message);
    }

    private static Issue warn(int step, Rule rule, String message) {
        return new Issue(Severity.WARN, step, rule, message);
    }
}
