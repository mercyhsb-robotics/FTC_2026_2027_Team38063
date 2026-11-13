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
 */
public final class ScoringPlanner {

    /** Everything the planner needs that is not field geometry. */
    public static final class Config {
        public double launchForwardIn;
        public double launchLeftIn;
        public double robotLengthIn;
        public double robotWidthIn;
        public double keepOutMarginIn;
        public double wallClearanceIn;
        public double rangeMinIn;
        public double rangeMaxIn;
        public double preferredRangeIn;
        public double coneHalfAngleDeg;
        public double coneStepDeg;
        public double rangeStepIn;
        public double rotationCostPerDeg;
        public double rangeCostPerIn;
        public double azimuthCostPerDeg;

        /** The configuration the robot runs with, from {@link BioBuzzConstants} and the shot table. */
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

    public enum Status {
        OK,
        NO_FEASIBLE_POSE
    }

    /** The planner's answer. {@code pose}, {@code waypoints} etc. are only meaningful when status is OK. */
    public static final class Solution {
        public final Status status;
        public final CellSpec cell;
        public final Pose2 pose;
        public final double rangeIn;
        public final double azimuthDeg;
        public final double travelIn;
        public final double cost;
        /** Points to drive through, ending at pose.x, pose.y (the start is not included). */
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
    private final PathPlanner travel;
    private final List<Rect> standingObstacles = new ArrayList<Rect>();
    private final Rect standingBounds;

    public ScoringPlanner(FieldModel field, Config cfg) {
        this.field = field;
        this.cfg = cfg;

        double halfDiag = 0.5 * Math.hypot(cfg.robotLengthIn, cfg.robotWidthIn);
        double halfMin = 0.5 * Math.min(cfg.robotLengthIn, cfg.robotWidthIn);

        // Driving: the robot is a point; obstacles grow by its circumscribed radius (it may rotate while
        // it drives) and the field shrinks by its smallest half-extent.
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

    /** True if the robot, standing at this pose, is inside the field and clear of the fixed obstacles. */
    public boolean standingPoseFree(Pose2 pose) {
        Rect box = ShotGeometry.footprint(pose, cfg.robotLengthIn, cfg.robotWidthIn);
        if (box.minX < standingBounds.minX || box.maxX > standingBounds.maxX
                || box.minY < standingBounds.minY || box.maxY > standingBounds.maxY) {
            return false;
        }
        for (Rect o : standingObstacles) {
            if (box.overlaps(o)) {
                return false;
            }
        }
        return true;
    }

    /** The cheapest feasible shooting pose for the cell, or a Solution with status NO_FEASIBLE_POSE. */
    public Solution solve(Pose2 robot, CellSpec cell) {
        Solution best = null;
        int coneSteps = (int) Math.floor(cfg.coneHalfAngleDeg / cfg.coneStepDeg + 1e-9);
        for (double r = cfg.rangeMinIn; r <= cfg.rangeMaxIn + 1e-9; r += cfg.rangeStepIn) {
            // Azimuth order 0, -step, +step, -2 step, +2 step ...: a strict "cheaper" test then keeps the
            // earliest (most central) candidate on ties, which makes the result deterministic.
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

    private static Solution better(Solution best, Solution candidate) {
        if (candidate != null && (best == null || candidate.cost < best.cost - 1e-9)) {
            return candidate;
        }
        return best;
    }

    /** Builds and scores one candidate; null if it is not feasible. */
    private Solution evaluate(Pose2 robot, CellSpec cell, double rangeIn, double azimuthRad) {
        double dir = cell.facingHeading + azimuthRad;
        double lx = cell.originX + rangeIn * Math.cos(dir);
        double ly = cell.originY + rangeIn * Math.sin(dir);
        Pose2 pose = ShotGeometry.poseForLaunchPoint(lx, ly, cell.originX, cell.originY,
                cfg.launchForwardIn, cfg.launchLeftIn);
        if (!standingPoseFree(pose)) {
            return null;
        }
        List<double[]> waypoints = travel.plan(robot.x, robot.y, pose.x, pose.y);
        if (waypoints == null) {
            return null;
        }
        double travelIn = PathPlanner.length(robot.x, robot.y, waypoints);
        double turnDeg = Math.toDegrees(Angles.distance(robot.heading, pose.heading));
        double azDeg = Math.toDegrees(Math.abs(azimuthRad));
        double cost = travelIn
                + cfg.rotationCostPerDeg * turnDeg
                + cfg.rangeCostPerIn * Math.abs(rangeIn - cfg.preferredRangeIn)
                + cfg.azimuthCostPerDeg * azDeg;
        return new Solution(Status.OK, cell, pose, rangeIn, Math.toDegrees(azimuthRad), travelIn, cost, waypoints);
    }

    public FieldModel field() {
        return field;
    }

    public Config config() {
        return cfg;
    }
}
