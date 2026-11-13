package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Finds the closest feasible place to shoot into a given UP cell from the robot's current pose.
 *
 * <p>A candidate shooting pose has its ball exit on the side the opening faces, inside a cone around
 * the opening's centre line, at a range inside the measured shot-table band, with the shooter (the
 * robot's back) pointed straight at the aim point. A candidate is feasible if the robot's footprint at
 * that pose is inside the field and clear of the HIVE frame and the FLOWERS, and a route to it exists that the
 * whole robot can drive without touching a wall or an obstacle. Among the feasible candidates the cheapest (travel +
 * turning + small preferences) wins.
 *
 * <p>Where it sits in the data flow: when the driver holds the shoot button, the PLAN state of
 * {@link AutoShootController} calls {@link #solve} with the robot's pose (from odometry) and the cell the
 * {@link HiveTracker} says is UP. The answer is a {@link Solution}: the pose to drive to and the route. The
 * controller later also calls {@link #standingPoseFree} to check small camera nudges stay legal. The planner is
 * built once at INIT from the {@link FieldModel} and a {@link Config}.
 *
 * <p>Vocabulary used below:
 * <ul>
 *   <li><b>Aim point</b>: the center of the cell's opening ({@link CellSpec#originX}, {@link CellSpec#originY}).</li>
 *   <li><b>Ball exit</b>: where the POLLEN leaves the shooter, a fixed spot behind the robot's center.</li>
 *   <li><b>Range</b>: horizontal distance from the ball exit to the aim point, in inches. The shot table only has
 *       flywheel speeds for a band of ranges, so only those are searched.</li>
 *   <li><b>Cone</b> and <b>azimuth</b>: the opening is a 20 in wide slot, so it looks narrower the farther you
 *       stand to the side. The robot may stand anywhere inside a fan (cone) around the line that runs straight out
 *       from the opening. Azimuth is the angle around the aim point from that line, in degrees, counter-clockwise
 *       positive; 0 is dead in front of the opening.</li>
 *   <li><b>Footprint</b>: the robot's rectangle on the floor. <b>Keep-out</b>: a box the robot must not touch.
 *       <b>Margin</b>: extra empty space kept around a keep-out, beyond the robot's own size.</li>
 *   <li><b>Cost</b>: one number that lets the search compare candidates. It is measured in "inches of driving":
 *       real driving distance plus small penalties for turning, for a range away from the preferred one, and for
 *       standing off the center line. The lowest cost wins.</li>
 * </ul>
 *
 * <p>How it searches: a plain brute-force grid. For every range from the lower to the upper end of the band (in
 * steps) and every azimuth in the cone (in steps) it builds one candidate pose, throws it away if the robot would
 * not fit or cannot get there, and keeps the cheapest survivor. That is only a few hundred candidates, which is
 * fast enough to do when the button is pressed.
 *
 * <p><b>Why the route is checked as a rectangle, not a point.</b> The road to a candidate is first found for a single
 * dot, the robot's center ({@link PathPlanner}); a dot has no corners, so that road keeps the dot only half a robot
 * width from a wall. But the robot is a 17.8 in square and it TURNS while it drives (see {@link HeadingProfile}), and a
 * square that is turned needs more room: turned 25 degrees it needs 11.8 in from its center to the wall, not 8.9 in,
 * and turned 45 degrees it needs 12.6 in. A robot that starts against a wall facing the wrong way and turns on its way
 * to the shooting spot can therefore push a corner through the wall, although the dot never goes near it and the
 * standing pose at the end is fine. So every candidate's road is walked with the real rotated rectangle
 * ({@link RouteSweep}) against the four walls, the HIVE frame and the FLOWER plates. A road that fails is repaired,
 * cheapest fix first:
 * <ol>
 *   <li>turn only where there is room: keep the starting heading while the robot is still close to the wall, do the
 *       whole turn on the stretch of the road where the robot could spin on the spot, and be finished before the road
 *       gets close to a wall again;</li>
 *   <li>a detour: drive straight out to open floor, turn on the way across, and come back in (a bend is added to the
 *       trip if the turn needs a longer road to stay gentle enough for the robot to follow);</li>
 *   <li>if neither is safe the candidate is thrown away exactly like a pose that is blocked. If every candidate is
 *       thrown away the answer is {@link Status#NO_FEASIBLE_POSE} and the controller stops instead of driving.</li>
 * </ol>
 * The repaired turn window travels in the {@link Solution} so the drive follows exactly what was checked.
 *
 * <p>Frame: Pedro field frame, inches and radians (0 = +x, counter-clockwise positive).
 */
public final class ScoringPlanner {

    /**
     * Everything the planner needs that is not field geometry. The fields are public so unit tests can change
     * them; the robot uses {@link #fromConstants}. Lengths are inches, angles degrees unless the name says otherwise.
     */
    public static final class Config {
        /** Ball exit position relative to the robot's rotation center: forward (negative = toward the back). */
        public double launchForwardIn;
        /** Ball exit position relative to the rotation center: sideways, positive = robot's left. */
        public double launchLeftIn;
        /** Robot size front to back. */
        public double robotLengthIn;
        /** Robot size side to side. */
        public double robotWidthIn;
        /** Extra clearance kept around the HIVE frame and FLOWERS beyond the robot's own size. */
        public double keepOutMarginIn;
        /** How close the robot's edge may come to a perimeter wall (0 = touching). */
        public double wallClearanceIn;
        /** Shortest range the search tries (the shot table's shortest row plus a safety margin). */
        public double rangeMinIn;
        /** Longest range the search tries (the shot table's longest row minus a safety margin). */
        public double rangeMaxIn;
        /** The range preferred when several poses are about equally good. */
        public double preferredRangeIn;
        /** Half-angle of the shooting cone around the opening's center line, in degrees. */
        public double coneHalfAngleDeg;
        /** Search step across the cone, in degrees. */
        public double coneStepDeg;
        /** Search step across the range band, in inches. */
        public double rangeStepIn;
        /** Cost of turning the robot, in inches of driving per degree turned. */
        public double rotationCostPerDeg;
        /** Cost of a range away from {@link #preferredRangeIn}, in inches of driving per inch of difference. */
        public double rangeCostPerIn;
        /** Cost of standing off the opening's center line, in inches of driving per degree of azimuth. */
        public double azimuthCostPerDeg;
        /** The route check looks at the robot at least every this many inches of driving (see {@link RouteSweep}). */
        public double sweepStepIn;
        /** ... and at least every this many degrees of turning. */
        public double sweepStepDeg;
        /** The route check forgives this many inches of squeeze, for rounding. */
        public double sweepToleranceIn;
        /** The route check never looks finer than this many inches. */
        public double sweepMinStepIn;
        /** The most degrees per inch of driving a squeezed-in turn may ask of the robot. */
        public double maxTurnDegPerIn;
        /** A squeezed-in turn is finished this many inches before the stretch with room to turn ends. */
        public double turnSettleIn;
        /** The drive drops route points closer together than this many inches (see {@link HeadingProfile}). */
        public double minLegIn;
        /** A detour looks for its bend at points around a ring, one every this many degrees. */
        public double detourAngleStepDeg;

        /**
         * The configuration the robot runs with, from {@link BioBuzzConstants} and the shot table. The usable range
         * band is the table's range shrunk by {@code RANGE_MARGIN_IN} at each end, so the planner never picks a
         * pose right at the edge of what has been measured.
         */
        public static Config fromConstants(ShotTable table) {
            Config c = new Config();
            c.launchForwardIn = BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN;
            c.launchLeftIn = BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN;
            c.robotLengthIn = BioBuzzConstants.Robot.LENGTH_IN;
            c.robotWidthIn = BioBuzzConstants.Robot.WIDTH_IN;
            c.keepOutMarginIn = BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN;
            c.wallClearanceIn = BioBuzzConstants.AutoShoot.WALL_CLEARANCE_IN;
            c.rangeMinIn = table.minRange() + BioBuzzConstants.Shooter.RANGE_MARGIN_IN;
            c.rangeMaxIn = table.maxRange() - BioBuzzConstants.Shooter.RANGE_MARGIN_IN;
            c.preferredRangeIn = BioBuzzConstants.AutoShoot.PREFERRED_RANGE_IN;
            c.coneHalfAngleDeg = BioBuzzConstants.AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG;
            c.coneStepDeg = BioBuzzConstants.AutoShoot.SHOOT_CONE_STEP_DEG;
            c.rangeStepIn = BioBuzzConstants.AutoShoot.SHOOT_RANGE_STEP_IN;
            c.rotationCostPerDeg = BioBuzzConstants.AutoShoot.PLAN_ROTATION_COST_IN_PER_DEG;
            c.rangeCostPerIn = BioBuzzConstants.AutoShoot.PLAN_RANGE_COST_IN_PER_IN;
            c.azimuthCostPerDeg = BioBuzzConstants.AutoShoot.PLAN_AZIMUTH_COST_IN_PER_DEG;
            c.sweepStepIn = BioBuzzConstants.AutoShoot.PATH_SWEEP_STEP_IN;
            c.sweepStepDeg = BioBuzzConstants.AutoShoot.PATH_SWEEP_STEP_DEG;
            c.sweepToleranceIn = BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN;
            c.sweepMinStepIn = BioBuzzConstants.AutoShoot.PATH_SWEEP_MIN_STEP_IN;
            c.maxTurnDegPerIn = BioBuzzConstants.AutoShoot.PATH_MAX_TURN_DEG_PER_IN;
            c.turnSettleIn = BioBuzzConstants.AutoShoot.PATH_TURN_SETTLE_IN;
            c.minLegIn = BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN;
            c.detourAngleStepDeg = BioBuzzConstants.AutoShoot.PATH_DETOUR_ANGLE_STEP_DEG;
            return c;
        }
    }

    /** Whether the search found a usable pose. */
    public enum Status {
        /** A feasible pose was found. */
        OK,
        /** Every candidate was blocked (wall, HIVE frame, FLOWER) or unreachable. The controller faults. */
        NO_FEASIBLE_POSE
    }

    /** The planner's answer. {@code pose}, {@code waypoints} etc. are only meaningful when status is OK. */
    public static final class Solution {
        public final Status status;
        /** The cell this answer is for. */
        public final CellSpec cell;
        /**
         * Where to put the robot's rotation center and which way to face (field inches / radians). The ball exit
         * is then at the chosen range and azimuth and the shooter points at the aim point. Null if not OK.
         */
        public final Pose2 pose;
        /** Distance from the ball exit to the aim point at that pose, in inches. */
        public final double rangeIn;
        /** Angle around the aim point from the opening's center line, in degrees (counter-clockwise positive). */
        public final double azimuthDeg;
        /** Length of the driving route from the robot's pose when asked, in inches. */
        public final double travelIn;
        /** The score the search minimized (lower is better), in "inches of driving". */
        public final double cost;
        /** Points to drive through, ending at pose.x, pose.y (the start is not included). Each is {x, y} in inches. */
        public final List<double[]> waypoints;
        /**
         * How the robot must turn along the route (see {@link HeadingProfile}): keep the heading it has now for this
         * many inches of the route before turning. 0 means "start turning at once", which is the ordinary rule. The
         * route was checked against the walls with exactly this turn, so the drive has to use it.
         */
        public final double turnHoldIn;
        /** Be finished turning to the final heading this many inches before the route ends (0 = turn right up to the end). */
        public final double turnSettleIn;

        Solution(Status status, CellSpec cell, Pose2 pose, double rangeIn, double azimuthDeg,
                 double travelIn, double cost, List<double[]> waypoints, double turnHoldIn, double turnSettleIn) {
            this.status = status;
            this.cell = cell;
            this.pose = pose;
            this.rangeIn = rangeIn;
            this.azimuthDeg = azimuthDeg;
            this.travelIn = travelIn;
            this.cost = cost;
            this.waypoints = waypoints;
            this.turnHoldIn = turnHoldIn;
            this.turnSettleIn = turnSettleIn;
        }

        /** True if a feasible pose was found (status OK). */
        public boolean ok() {
            return status == Status.OK;
        }

        @Override
        public String toString() {
            return ok() ? String.format("%s -> %s range %.1f in, azimuth %.1f deg, travel %.1f in",
                    cell, pose, rangeIn, azimuthDeg, travelIn) : status + " for " + cell;
        }
    }

    private final FieldModel field;
    private final Config cfg;
    // Route finder for DRIVING (robot treated as a point, so its keep-outs are grown by the robot's size).
    private final PathPlanner travel;
    // Keep-outs for STANDING (the real footprint box is tested, so they are only grown by the margin).
    private final List<Rect> standingObstacles = new ArrayList<Rect>();
    // Where the robot's footprint may be when standing: the field shrunk by the wall clearance only.
    private final Rect standingBounds;
    // The wall-safety check of a whole route: the real rotated footprint against the walls, HIVE frame and FLOWERS.
    private final RouteSweep sweep;
    // Route finder used when a route has to detour: the robot's center may only be where it can turn freely (the
    // field shrunk by its full reach, not just half its width).
    private final PathPlanner safeTravel;
    private final Rect turnSafeBounds;

    /** Builds the planner once; the two "inflate" blocks below prepare the driving and standing keep-outs. */
    public ScoringPlanner(FieldModel field, Config cfg) {
        this.field = field;
        this.cfg = cfg;

        // The route check comes first because it knows the robot's REACH: the distance from the rotation center to the
        // farthest corner (the "circumscribed radius", the smallest circle that holds the whole robot whatever way it
        // faces). Taking it from the same Footprint the check uses means the planner and the check cannot disagree about
        // how big the robot is. halfMin = half of the shorter side.
        RouteSweep.Settings settings = new RouteSweep.Settings();
        settings.stepIn = cfg.sweepStepIn;
        settings.stepDeg = cfg.sweepStepDeg;
        settings.toleranceIn = cfg.sweepToleranceIn;
        settings.minStepIn = cfg.sweepMinStepIn;
        sweep = new RouteSweep(field.fieldRect(), cfg.wallClearanceIn, field.hardObstacles(), cfg.keepOutMarginIn,
                cfg.robotLengthIn, cfg.robotWidthIn, settings);
        double halfDiag = sweep.reachIn();
        double halfMin = 0.5 * Math.min(cfg.robotLengthIn, cfg.robotWidthIn);

        // Driving: the robot is a point; obstacles grow by its circumscribed radius (it may rotate while
        // it drives) and the field shrinks by its smallest half-extent. (Using the smaller half-size for the walls
        // lets poses that sit flush against a wall, such as the audience-wall shot, stay reachable. The cost is that a
        // route planned this way can squeeze a TURNING robot against a wall; that is what the route check and the
        // repairs below are for.)
        List<Rect> inflated = new ArrayList<Rect>();
        for (Rect o : field.hardObstacles()) {
            inflated.add(o.inflate(halfDiag + cfg.keepOutMarginIn));
        }
        Rect driveBounds = field.fieldRect().inflate(-(halfMin + cfg.wallClearanceIn));
        travel = new PathPlanner(driveBounds, inflated);

        // Standing: the exact footprint box is tested, so obstacles only grow by the margin.
        for (Rect o : field.hardObstacles()) {
            standingObstacles.add(o.inflate(cfg.keepOutMarginIn));
        }
        standingBounds = field.fieldRect().inflate(-cfg.wallClearanceIn);

        // The detour planner only uses places where the robot may face any way.
        turnSafeBounds = field.fieldRect().inflate(-(sweep.reachIn() + cfg.wallClearanceIn));
        List<Rect> turnSafeObstacles = new ArrayList<Rect>();
        for (Rect o : field.hardObstacles()) {
            turnSafeObstacles.add(o.inflate(sweep.reachIn() + cfg.keepOutMarginIn));
        }
        safeTravel = new PathPlanner(turnSafeBounds, turnSafeObstacles);
    }

    /**
     * True if the robot, standing at this pose, is inside the field and clear of the fixed obstacles. Used to
     * test one candidate pose, and by the controller before it accepts a camera nudge of the hold pose.
     */
    public boolean standingPoseFree(Pose2 pose) {
        Rect box = ShotGeometry.footprint(pose, cfg.robotLengthIn, cfg.robotWidthIn);
        // Inside the field (shrunk by the wall clearance)?
        if (box.minX < standingBounds.minX || box.maxX > standingBounds.maxX
                || box.minY < standingBounds.minY || box.maxY > standingBounds.maxY) {
            return false;
        }
        // Not on top of the HIVE frame or a FLOWER (each grown by the margin)?
        for (Rect o : standingObstacles) {
            if (box.overlaps(o)) {
                return false;
            }
        }
        return true;
    }

    /**
     * The cheapest feasible shooting pose for the cell, or a Solution with status NO_FEASIBLE_POSE.
     *
     * @param robot where the robot is now (field frame); the route and the cost start from here
     * @param cell  the UP cell to shoot into
     */
    public Solution solve(Pose2 robot, CellSpec cell) {
        Solution best = null;
        // How many cone steps fit on each side of the center line (the 1e-9 forgives floating-point rounding).
        int coneSteps = (int) Math.floor(cfg.coneHalfAngleDeg / cfg.coneStepDeg + 1e-9);
        for (double r = cfg.rangeMinIn; r <= cfg.rangeMaxIn + 1e-9; r += cfg.rangeStepIn) {
            // Azimuth order 0, -step, +step, -2 step, +2 step ...: a strict "cheaper" test then keeps the
            // earliest candidate (smallest range, then most central) on ties, which makes the result deterministic.
            best = better(best, evaluate(robot, cell, r, 0.0, best));
            for (int k = 1; k <= coneSteps; k++) {
                double az = Math.toRadians(cfg.coneStepDeg * k);
                best = better(best, evaluate(robot, cell, r, -az, best));
                best = better(best, evaluate(robot, cell, r, az, best));
            }
        }
        if (best == null) {
            return new Solution(Status.NO_FEASIBLE_POSE, cell, null, 0, 0, 0, Double.POSITIVE_INFINITY,
                    new ArrayList<double[]>(), 0.0, 0.0);
        }
        return best;
    }

    /** Keeps whichever of the two is cheaper; a candidate must win by more than a hair (1e-9) to replace the best. */
    private static Solution better(Solution best, Solution candidate) {
        if (candidate != null && (best == null || candidate.cost < best.cost - 1e-9)) {
            return candidate;
        }
        return best;
    }

    /**
     * Builds and scores one candidate; null if it is not feasible.
     *
     * @param rangeIn    horizontal distance from the ball exit to the aim point, in inches
     * @param azimuthRad angle around the aim point from the opening's center line, radians, counter-clockwise positive
     * @param best       the cheapest feasible answer found so far (or null); a candidate that cannot beat it is not
     *                   worth the (slower) wall-safety check and is dropped at once
     */
    private Solution evaluate(Pose2 robot, CellSpec cell, double rangeIn, double azimuthRad, Solution best) {
        // 1. Where would the ball exit be? Start at the aim point and step "range" out along the direction
        //    the opening faces, swung sideways by the azimuth.
        double dir = cell.facingHeading + azimuthRad;
        double lx = cell.originX + rangeIn * Math.cos(dir);
        double ly = cell.originY + rangeIn * Math.sin(dir);
        // 2. Which pose of the robot's center and heading puts the exit there, with the back pointing at the aim point?
        Pose2 pose = ShotGeometry.poseForLaunchPoint(lx, ly, cell.originX, cell.originY,
                cfg.launchForwardIn, cfg.launchLeftIn);
        // 3. Reject poses where the robot would not fit (walls, HIVE frame, FLOWERS) ...
        if (!standingPoseFree(pose)) {
            return null;
        }
        // ... or that it cannot drive to.
        List<double[]> waypoints = travel.plan(robot.x, robot.y, pose.x, pose.y);
        if (waypoints == null) {
            return null;
        }
        // 4. Score it: driving distance, plus small penalties (see the Config cost fields) that break near-ties.
        double travelIn = PathPlanner.length(robot.x, robot.y, waypoints);
        double turnDeg = Math.toDegrees(Angles.distance(robot.heading, pose.heading));
        double azDeg = Math.toDegrees(Math.abs(azimuthRad));
        double extraCost = cfg.rotationCostPerDeg * turnDeg
                + cfg.rangeCostPerIn * Math.abs(rangeIn - cfg.preferredRangeIn)
                + cfg.azimuthCostPerDeg * azDeg;
        double cost = travelIn + extraCost;
        // Cannot beat the answer we already have (by more than the tie hair that "better" uses): no need to check it.
        if (best != null && cost >= best.cost - 1e-9) {
            return null;
        }
        // 5. The route so far treats the robot as a dot. Check the real turning rectangle along the whole route, and
        //    repair the route if it fails; if no repair is safe the candidate is dropped like any blocked pose.
        SafeRoute safe = makeSafe(robot, pose, waypoints);
        if (safe == null) {
            return null;
        }
        if (safe.waypoints != waypoints) {
            // A detour is longer: score the route that will really be driven.
            travelIn = PathPlanner.length(robot.x, robot.y, safe.waypoints);
            cost = travelIn + extraCost;
        }
        return new Solution(Status.OK, cell, pose, rangeIn, Math.toDegrees(azimuthRad), travelIn, cost,
                safe.waypoints, safe.holdIn, safe.settleIn);
    }

    /** A route that passed the wall-safety check, with the turn window it was checked with. */
    private static final class SafeRoute {
        final List<double[]> waypoints;
        final double holdIn;
        final double settleIn;

        SafeRoute(List<double[]> waypoints, double holdIn, double settleIn) {
            this.waypoints = waypoints;
            this.holdIn = holdIn;
            this.settleIn = settleIn;
        }
    }

    /**
     * Turns the dot route into a route the real robot can drive without touching a wall or an obstacle, or returns null
     * if none is found. It tries, in order, the cheapest fix that works:
     * <ol>
     *   <li>The route as it is, with the ordinary rule (turn evenly over the whole way). Most routes pass.</li>
     *   <li>The same route, but turning only where there is room: keep the starting heading while the robot is still
     *       close to a wall, turn where the robot is far enough from everything that it could spin on the spot, and
     *       arrive already facing the final way.</li>
     *   <li>A detour: first move to the nearest spot with room to turn, turn on the way across, and move in to the
     *       pose at the end.</li>
     * </ol>
     */
    private SafeRoute makeSafe(Pose2 robot, Pose2 pose, List<double[]> route) {
        if (sweep.isSafe(HeadingProfile.build(robot, route, pose.heading))) {
            return new SafeRoute(route, 0.0, 0.0);
        }
        SafeRoute turnedWhereThereIsRoom = turnWhereThereIsRoom(robot, pose, route);
        if (turnedWhereThereIsRoom != null) {
            return turnedWhereThereIsRoom;
        }
        return detourToWhereTheRobotCanTurn(robot, pose);
    }

    /**
     * Repair 2: keep the route, but give the turn a window. The route is walked in {@code sweepStepIn} steps and each
     * step marked "free" if the robot could face any way there (see {@link RouteSweep#anyHeadingIsSafeAt}). The turn is
     * then done inside a run of free steps, longest run first, and checked; the first run that works wins. A run that
     * is too short for the turn (more than {@code maxTurnDegPerIn} degrees per inch) is skipped.
     */
    private SafeRoute turnWhereThereIsRoom(Pose2 robot, Pose2 pose, List<double[]> route) {
        double turnDeg = Math.toDegrees(Angles.distance(robot.heading, pose.heading));
        HeadingProfile plain = HeadingProfile.build(robot, route, pose.heading);
        // No turn to hide (or no route): choosing when to turn cannot help.
        if (turnDeg <= 1e-9 || plain.isEmpty()) {
            return null;
        }
        double length = plain.length();
        // Walk the route and collect the runs of free steps as {from, to} distances.
        List<double[]> runs = new ArrayList<double[]>();
        double runFrom = Double.NaN;
        double lastFree = Double.NaN;
        int steps = (int) Math.ceil(length / cfg.sweepStepIn);
        for (int k = 0; k <= steps; k++) {
            double s = Math.min(length, k * cfg.sweepStepIn);
            Pose2 at = plain.poseAt(s);
            if (sweep.anyHeadingIsSafeAt(at.x, at.y)) {
                if (Double.isNaN(runFrom)) {
                    runFrom = s;
                }
                lastFree = s;
            } else if (!Double.isNaN(runFrom)) {
                runs.add(new double[]{runFrom, lastFree});
                runFrom = Double.NaN;
            }
        }
        if (!Double.isNaN(runFrom)) {
            runs.add(new double[]{runFrom, lastFree});
        }
        // Try the longest run first (the longest run turns the robot most gently).
        while (!runs.isEmpty()) {
            int longest = 0;
            for (int i = 1; i < runs.size(); i++) {
                if (runs.get(i)[1] - runs.get(i)[0] > runs.get(longest)[1] - runs.get(longest)[0]) {
                    longest = i;
                }
            }
            double[] run = runs.remove(longest);
            SafeRoute tried = tryWindow(robot, pose, route, run[0], run[1], turnDeg);
            if (tried != null) {
                return tried;
            }
        }
        return null;
    }

    /**
     * Checks one turn window on a route. The robot keeps its heading for {@code holdIn} inches, then turns, and the turn
     * must be finished {@code turnSettleIn} inches before {@code freeEndIn}, the distance at which the robot's free
     * stretch ends (the extra inches let the real robot's heading catch up with the planned one while it is still far
     * from the wall). Returns null if there is no room for the turn, if the turn would be faster than
     * {@code maxTurnDegPerIn}, or if the route then fails the wall check.
     */
    private SafeRoute tryWindow(Pose2 robot, Pose2 pose, List<double[]> route, double holdIn, double freeEndIn,
                                double turnDeg) {
        HeadingProfile plain = HeadingProfile.build(robot, route, pose.heading);
        if (turnDeg <= 1e-9) {
            // Nothing to turn: the heading is the same everywhere, so only the route itself needs checking.
            return sweep.isSafe(plain) ? new SafeRoute(route, 0.0, 0.0) : null;
        }
        double turnEndIn = freeEndIn - cfg.turnSettleIn;
        if (turnEndIn <= holdIn) {
            return null;
        }
        double settleIn = plain.length() - turnEndIn;
        HeadingProfile windowed = HeadingProfile.build(robot, route, pose.heading, holdIn, settleIn);
        // The robot cannot turn faster than it can spin, so a window that squeezes the turn too hard is no use.
        if (windowed.maxTurnRateDegPerIn() > cfg.maxTurnDegPerIn) {
            return null;
        }
        return sweep.isSafe(windowed) ? new SafeRoute(route, holdIn, settleIn) : null;
    }

    /**
     * Repair 3: a detour. Walk straight to the nearest spot where the robot may face any way (the field shrunk by the
     * robot's full reach), find a route from there to the matching spot near the pose that avoids the keep-outs
     * grown by the same reach, turn on the way, and finish with a straight move in to the pose. The turn window is the
     * middle stretch, where there is room by construction. If that stretch is too short for a turn that is gentle
     * enough, a bend is added to make it longer (see {@link #lengthened}).
     */
    private SafeRoute detourToWhereTheRobotCanTurn(Pose2 robot, Pose2 pose) {
        double[] leaveAt = nearestTurnSafePoint(robot.x, robot.y);
        double[] arriveAt = nearestTurnSafePoint(pose.x, pose.y);
        List<double[]> middle = safeTravel.plan(leaveAt[0], leaveAt[1], arriveAt[0], arriveAt[1]);
        if (middle == null) {
            return null;
        }
        double turnDeg = Math.toDegrees(Angles.distance(robot.heading, pose.heading));
        // Room the turn needs: the turn at the fastest allowed rate, plus the settle distance after it, plus one
        // short-leg limit at each end of the turn (HeadingProfile moves a turn edge that falls that close to a corner of
        // the route onto the corner, which can shorten the turn by that much).
        double needIn = turnDeg / cfg.maxTurnDegPerIn + cfg.turnSettleIn + 2.0 * cfg.minLegIn;
        double haveIn = PathPlanner.length(leaveAt[0], leaveAt[1], middle);
        if (haveIn < needIn) {
            middle = lengthened(leaveAt, middle, needIn - haveIn);
            if (middle == null) {
                return null;
            }
        }
        List<double[]> route = new ArrayList<double[]>();
        route.add(leaveAt);
        route.addAll(middle);
        double[] last = route.get(route.size() - 1);
        if (Math.hypot(pose.x - last[0], pose.y - last[1]) > 1e-9) {
            // (Skipped when the pose is itself a place with room to turn: the route already ends on it.)
            route.add(new double[]{pose.x, pose.y});
        }
        double holdIn = Math.hypot(leaveAt[0] - robot.x, leaveAt[1] - robot.y);
        double freeEndIn = HeadingProfile.build(robot, route, pose.heading).length()
                - Math.hypot(pose.x - arriveAt[0], pose.y - arriveAt[1]);
        return tryWindow(robot, pose, route, holdIn, freeEndIn, turnDeg);
    }

    /**
     * Makes the route {@code from -> middle} about {@code extraIn} inches longer by putting one bend into its longest
     * straight leg (two sides of a triangle are longer than the third). It is used when a robot pressed against a wall
     * has to turn a long way but the two places with room to turn are close together, so a gentle turn needs a longer
     * trip.
     *
     * <p>Which bend: the leg from A to B becomes A to P to B. The points P for which the trip has exactly the length
     * wanted (|AP| + |PB| = |AB| + extra) lie on an ellipse around A and B (an "ellipse" is a squashed circle; when A
     * and B are the same point it is a plain circle). The planner tries points around it, one every
     * {@code detourAngleStepDeg} degrees, throws away those that are outside the area where the robot may turn freely or
     * whose two new legs hit a keep-out, and keeps the one closest to the middle of the field, which is away from the
     * wall the robot is leaning on.
     *
     * @param from    where the route starts (a place with room to turn)
     * @param middle  the rest of the route after {@code from}
     * @return the new rest of the route (it has one more point), or null if no bend fits
     */
    private List<double[]> lengthened(double[] from, List<double[]> middle, double extraIn) {
        List<double[]> pts = new ArrayList<double[]>();
        pts.add(from);
        pts.addAll(middle);
        // The longest leg is the one most likely to have room for a bend.
        int leg = 0;
        double longest = -1.0;
        for (int i = 0; i + 1 < pts.size(); i++) {
            double len = Math.hypot(pts.get(i + 1)[0] - pts.get(i)[0], pts.get(i + 1)[1] - pts.get(i)[1]);
            if (len > longest) {
                longest = len;
                leg = i;
            }
        }
        double[] a = pts.get(leg);
        double[] b = pts.get(leg + 1);
        double chord = Math.hypot(b[0] - a[0], b[1] - a[1]);
        // The ellipse: centered on the middle of the leg, long axis along the leg (any direction when the leg has no
        // length), half-length halfSide along it and "minor" across it.
        double ux = chord > 1e-9 ? (b[0] - a[0]) / chord : 1.0;
        double uy = chord > 1e-9 ? (b[1] - a[1]) / chord : 0.0;
        double midX = 0.5 * (a[0] + b[0]);
        double midY = 0.5 * (a[1] + b[1]);
        double halfSide = 0.5 * (chord + extraIn);
        double halfChord = 0.5 * chord;
        double minor = Math.sqrt(Math.max(0.0, halfSide * halfSide - halfChord * halfChord));
        double[] apex = null;
        double apexFromCenter = Double.POSITIVE_INFINITY;
        double step = Math.toRadians(cfg.detourAngleStepDeg);
        for (double phi = 0.0; phi < 2.0 * Math.PI - 1e-9; phi += step) {
            double along = halfSide * Math.cos(phi);
            double across = minor * Math.sin(phi);
            double[] p = {midX + ux * along - uy * across, midY + uy * along + ux * across};
            double fromCenter = Math.hypot(p[0] - field.center(), p[1] - field.center());
            if (fromCenter < apexFromCenter && turnSafeBounds.contains(p[0], p[1])
                    && safeTravel.segmentClear(a[0], a[1], p[0], p[1])
                    && safeTravel.segmentClear(p[0], p[1], b[0], b[1])) {
                apex = p;
                apexFromCenter = fromCenter;
            }
        }
        if (apex == null) {
            return null;
        }
        List<double[]> out = new ArrayList<double[]>();
        for (int i = 1; i <= leg; i++) {
            out.add(pts.get(i));
        }
        out.add(apex);
        for (int i = leg + 1; i < pts.size(); i++) {
            out.add(pts.get(i));
        }
        return out;
    }

    /** The closest point to (x, y) inside the area where the robot's center leaves room to turn (see turnSafeBounds). */
    private double[] nearestTurnSafePoint(double x, double y) {
        return new double[]{Math.max(turnSafeBounds.minX, Math.min(turnSafeBounds.maxX, x)),
                Math.max(turnSafeBounds.minY, Math.min(turnSafeBounds.maxY, y))};
    }

    /** The field model this planner was built with. */
    public FieldModel field() {
        return field;
    }

    /** The configuration this planner was built with. */
    public Config config() {
        return cfg;
    }
}
