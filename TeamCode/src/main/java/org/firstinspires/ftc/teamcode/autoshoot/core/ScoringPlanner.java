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

        Solution(Status status, CellSpec cell, Pose2 pose, double rangeIn, double azimuthDeg,
                 double travelIn, double cost, List<double[]> waypoints) {
            this.status = status;
            this.cell = cell;
            this.pose = pose;
            this.rangeIn = rangeIn;
            this.azimuthDeg = azimuthDeg;
            this.travelIn = travelIn;
            this.cost = cost;
            this.waypoints = waypoints;
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
            best = better(best, evaluate(robot, cell, r, 0.0));
            for (int k = 1; k <= coneSteps; k++) {
                double az = Math.toRadians(cfg.coneStepDeg * k);
                best = better(best, evaluate(robot, cell, r, -az));
                best = better(best, evaluate(robot, cell, r, az));
            }
        }
        if (best == null) {
            return new Solution(Status.NO_FEASIBLE_POSE, cell, null, 0, 0, 0, Double.POSITIVE_INFINITY,
                    new ArrayList<double[]>());
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
     */
    private Solution evaluate(Pose2 robot, CellSpec cell, double rangeIn, double azimuthRad) {
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
        double cost = travelIn
                + cfg.rotationCostPerDeg * turnDeg
                + cfg.rangeCostPerIn * Math.abs(rangeIn - cfg.preferredRangeIn)
                + cfg.azimuthCostPerDeg * azDeg;
        return new Solution(Status.OK, cell, pose, rangeIn, Math.toDegrees(azimuthRad), travelIn, cost, waypoints);
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
