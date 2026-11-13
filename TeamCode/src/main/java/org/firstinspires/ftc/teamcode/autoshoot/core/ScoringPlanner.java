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
 * that pose is inside the field and clear of the HIVE frame and the FLOWERS, and a collision-free route
 * to it exists. Among the feasible candidates the cheapest (travel + turning + small preferences) wins.
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

        // Half the diagonal = distance from the robot's center to a corner (its "circumscribed radius", the
        // smallest circle that holds the whole robot). halfMin = half of the shorter side.
        double halfDiag = 0.5 * Math.hypot(cfg.robotLengthIn, cfg.robotWidthIn);
        double halfMin = 0.5 * Math.min(cfg.robotLengthIn, cfg.robotWidthIn);

        // Driving: the robot is a point; obstacles grow by its circumscribed radius (it may rotate while
        // it drives) and the field shrinks by its smallest half-extent. (Using the smaller half-size for the walls
        // lets poses that sit flush against a wall, such as the audience-wall shot, stay reachable.)
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

        // The route check, and the detour planner that only uses places where the robot may face any way.
        RouteSweep.Settings settings = new RouteSweep.Settings();
        settings.stepIn = cfg.sweepStepIn;
        settings.stepDeg = cfg.sweepStepDeg;
        settings.toleranceIn = cfg.sweepToleranceIn;
        settings.minStepIn = cfg.sweepMinStepIn;
        sweep = new RouteSweep(field.fieldRect(), cfg.wallClearanceIn, field.hardObstacles(), cfg.keepOutMarginIn,
                cfg.robotLengthIn, cfg.robotWidthIn, settings);
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
            SafeRoute tried = tryWindow(robot, pose, route, run[0], length - run[1], turnDeg, run[1] - run[0]);
            if (tried != null) {
                return tried;
            }
        }
        return null;
    }

    /**
     * Checks one turn window: the route with the start heading held for {@code holdIn} and the turn finished
     * {@code settleIn} before the end. {@code windowIn} is the length left for turning. Null if the turn would be
     * faster than allowed or the check fails.
     */
    private SafeRoute tryWindow(Pose2 robot, Pose2 pose, List<double[]> route, double holdIn, double settleIn,
                                double turnDeg, double windowIn) {
        if (turnDeg > cfg.maxTurnDegPerIn * windowIn) {
            return null;
        }
        HeadingProfile windowed = HeadingProfile.build(robot, route, pose.heading, holdIn, settleIn);
        return sweep.isSafe(windowed) ? new SafeRoute(route, holdIn, settleIn) : null;
    }

    /**
     * Repair 3: a detour. Walk straight to the nearest spot where the robot may face any way (the field shrunk by the
     * robot's full reach), find a route from there to the matching spot near the pose that avoids the keep-outs
     * grown by the same reach, turn on the way, and finish with a straight move in to the pose. The turn window is the
     * middle stretch, where there is room by construction.
     */
    private SafeRoute detourToWhereTheRobotCanTurn(Pose2 robot, Pose2 pose) {
        double[] leaveAt = nearestTurnSafePoint(robot.x, robot.y);
        double[] arriveAt = nearestTurnSafePoint(pose.x, pose.y);
        List<double[]> middle = safeTravel.plan(leaveAt[0], leaveAt[1], arriveAt[0], arriveAt[1]);
        if (middle == null) {
            return null;
        }
        List<double[]> route = new ArrayList<double[]>();
        route.add(leaveAt);
        route.addAll(middle);
        route.add(new double[]{pose.x, pose.y});
        double holdIn = Math.hypot(leaveAt[0] - robot.x, leaveAt[1] - robot.y);
        double settleIn = Math.hypot(pose.x - arriveAt[0], pose.y - arriveAt[1]);
        double length = HeadingProfile.build(robot, route, pose.heading).length();
        double turnDeg = Math.toDegrees(Angles.distance(robot.heading, pose.heading));
        return tryWindow(robot, pose, route, holdIn, settleIn, turnDeg, length - holdIn - settleIn);
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
