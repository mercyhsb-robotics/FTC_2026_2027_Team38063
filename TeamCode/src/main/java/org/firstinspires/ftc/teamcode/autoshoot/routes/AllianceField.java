package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;

/**
 * The field as one alliance sees it: its own half, its own LOADING ZONE and GARDEN, the keep-outs,
 * and the zones of the other alliance. All the numbers are written for RED in
 * {@link BioBuzzConstants} and turned half a turn for BLUE, so a route that was derived for BLUE is
 * checked against exactly the geometry a blue robot meets.
 */
public final class AllianceField {
    private final FieldModel model;
    private final Alliance alliance;
    private final double size;
    private final double center;
    private final List<Rect> plates;

    public AllianceField(FieldModel model, Alliance alliance) {
        this.model = model;
        this.alliance = alliance;
        this.size = model.size();
        this.center = model.center();
        List<Rect> built = new ArrayList<Rect>();
        for (int i = 0; i < BioBuzzConstants.Field.FLOWER_CENTERS_IN.length; i++) {
            built.add(alongWallBox(flowerWallPoint(i), flowerInward(i), BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN / 2.0,
                    BioBuzzConstants.Field.FLOWER_PLATE_DEPTH_IN));
        }
        this.plates = Collections.unmodifiableList(built);
    }

    public Alliance alliance() {
        return alliance;
    }

    public FieldModel model() {
        return model;
    }

    public Rect fieldRect() {
        return model.fieldRect();
    }

    public Rect hiveFrame() {
        return model.hiveFrame();
    }

    /**
     * The HIVE frame footprint grown by {@link BioBuzzConstants.Auto#HIVE_KEEPOUT_MARGIN_IN} on every side: where no
     * part of the robot may be. It is a solid box: the gaps under and inside the frame count as solid too (see the
     * constant for what the 12.13 in tall robot can really hit and why).
     */
    public Rect hiveKeepOut() {
        return model.hiveFrame().inflate(BioBuzzConstants.Auto.HIVE_KEEPOUT_MARGIN_IN);
    }

    /**
     * The four FLOWER plates (no margin) at their real size, {@code Field.FLOWER_PLATE_DEPTH_IN x
     * FLOWER_PLATE_ALONG_WALL_IN} (4.75 x 5.87 in), standing on their wall and centered on the FLOWER, in the order of
     * {@code Field.FLOWER_CENTERS_IN}. (The path planner of the TeleOp uses the slightly larger boxes of
     * {@link FieldModel#flowerKeepOuts()} instead.)
     */
    public List<Rect> flowerPlates() {
        return plates;
    }

    /** One FLOWER plate. */
    public Rect flowerPlate(int index) {
        return plates.get(index);
    }

    /** A FLOWER plate grown by {@link BioBuzzConstants.Auto#FLOWER_KEEPOUT_MARGIN_IN}: where the robot may not be. */
    public Rect flowerKeepOut(int index) {
        return plates.get(index).inflate(BioBuzzConstants.Auto.FLOWER_KEEPOUT_MARGIN_IN);
    }

    /**
     * The plate grown by {@link BioBuzzConstants.Auto#FLOWER_CHASSIS_CLEARANCE_IN}: where the CHASSIS (the robot without
     * its intake strip) may not be while its wheels are pressed into this FLOWER.
     */
    public Rect flowerChassisKeepOut(int index) {
        return plates.get(index).inflate(BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN);
    }

    /** The center of FLOWER {@code index}. */
    public Point2 flowerCenter(int index) {
        double[] c = BioBuzzConstants.Field.FLOWER_CENTERS_IN[index];
        return new Point2(c[0], c[1]);
    }

    // ---- FLOWER geometry: which wall, how far out, how far along ----------------------------------------------

    /**
     * The unit vector that points from the wall FLOWER {@code index} stands on INTO the field: (1, 0) for the left wall,
     * (-1, 0) for the right one, (0, 1) for the audience wall and (0, -1) for the rear wall.
     */
    public Point2 flowerInward(int index) {
        Point2 c = flowerCenter(index);
        double toLeft = c.x;
        double toRight = size - c.x;
        double toAudience = c.y;
        double toRear = size - c.y;
        double nearest = Math.min(Math.min(toLeft, toRight), Math.min(toAudience, toRear));
        if (nearest == toLeft) {
            return new Point2(1.0, 0.0);
        }
        if (nearest == toRight) {
            return new Point2(-1.0, 0.0);
        }
        if (nearest == toAudience) {
            return new Point2(0.0, 1.0);
        }
        return new Point2(0.0, -1.0);
    }

    /** The point of the wall that is straight behind the FLOWER's center line (where its POLLEN stack stands). */
    public Point2 flowerWallPoint(int index) {
        Point2 c = flowerCenter(index);
        Point2 n = flowerInward(index);
        double out = BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN;
        return new Point2(c.x - n.x * out, c.y - n.y * out);
    }

    /** How far a point is out from the FLOWER's wall, in inches (positive = on the field side of the wall). */
    public double flowerWallDistance(int index, Point2 p) {
        Point2 w = flowerWallPoint(index);
        Point2 n = flowerInward(index);
        return (p.x - w.x) * n.x + (p.y - w.y) * n.y;
    }

    /**
     * How far a point is along the wall from the FLOWER's center line, in inches (the sign only says which way along
     * the wall). A robot lined up on the FLOWER has a lateral offset near 0.
     */
    public double flowerLateral(int index, Point2 p) {
        Point2 c = flowerCenter(index);
        Point2 n = flowerInward(index);
        return (p.x - c.x) * -n.y + (p.y - c.y) * n.x;
    }

    /** The heading (radians) of a robot that faces FLOWER {@code index}'s wall squarely, front toward the wall. */
    public double flowerFacingHeading(int index) {
        Point2 n = flowerInward(index);
        return Math.atan2(-n.y, -n.x);
    }

    /**
     * The press stop line: how far from the wall (inches) the FRONT EDGE of the robot stops when the wheels are pressed
     * into the POLLEN. It is the POLLEN's field-side edge (the center line 2.4 in plus the 1.4 in radius = 3.8 in) minus
     * {@code Auto.FLOWER_PRESS_DEPTH_IN}, and never closer to the wall than the POLLEN's center line.
     */
    public double flowerPressStopLineIn() {
        double centerLine = BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN;
        double edge = centerLine + BioBuzzConstants.Auto.POLLEN_DIAMETER_IN / 2.0;
        return Math.max(centerLine, edge - BioBuzzConstants.Auto.FLOWER_PRESS_DEPTH_IN);
    }

    /**
     * The part of FLOWER {@code index}'s plate that is nearer the wall than the press stop line: no part of the robot
     * may enter it, not even the wheels.
     */
    public Rect flowerPressZone(int index) {
        Point2 w = flowerWallPoint(index);
        Point2 n = flowerInward(index);
        double half = BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN / 2.0;
        double stop = flowerPressStopLineIn();
        return alongWallBox(w, n, half, stop);
    }

    /**
     * How far the wheels must reach in front of the chassis (inches) to press the POLLEN with the chassis kept
     * {@code FLOWER_CHASSIS_CLEARANCE_IN} off the plate: plate depth + clearance - press stop line.
     */
    public double flowerRequiredReachIn() {
        return BioBuzzConstants.Field.FLOWER_PLATE_DEPTH_IN + BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN
                - flowerPressStopLineIn();
    }

    /** The box from the wall out to {@code depth} inches, {@code half} inches either side of the wall point. */
    private static Rect alongWallBox(Point2 wall, Point2 n, double half, double depth) {
        // the tangent along the wall is (-n.y, n.x)
        double ax = wall.x + -n.y * half;
        double ay = wall.y + n.x * half;
        double bx = wall.x + n.x * depth - -n.y * half;
        double by = wall.y + n.y * depth - n.x * half;
        return new Rect(ax, bx, ay, by);
    }

    /** True if the FLOWER is on this alliance's half (the other half is off limits in AUTO, rule G402). */
    public boolean isOwnFlower(int index) {
        return isOwnSide(flowerCenter(index).x);
    }

    // ---- sides ---------------------------------------------------------------------------------------

    /** This alliance's half of the field. */
    public Rect ownHalf() {
        return alliance == Alliance.RED ? new Rect(0, center, 0, size) : new Rect(center, size, 0, size);
    }

    /** True if x lies on this alliance's side of the center line (the center line itself counts as own). */
    public boolean isOwnSide(double x) {
        return alliance == Alliance.RED ? x <= center + 1e-9 : x >= center - 1e-9;
    }

    /** Signed distance a point is past the center line into the opponent half (<= 0 on the own side). */
    public double pastCenterLine(double x) {
        return alliance == Alliance.RED ? x - center : center - x;
    }

    // ---- zones of any alliance -----------------------------------------------------------------------

    /** The LOADING ZONE of the given alliance. */
    public Rect loadingZone(Alliance a) {
        Rect red = new Rect(BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MAX_IN);
        return a == Alliance.RED ? red : red.rotated180(center, center);
    }

    /** The GARDEN zone of the given alliance. */
    public Rect garden(Alliance a) {
        Rect red = new Rect(BioBuzzConstants.Auto.RED_GARDEN_X_MIN_IN, BioBuzzConstants.Auto.RED_GARDEN_X_MAX_IN,
                BioBuzzConstants.Auto.RED_GARDEN_Y_MIN_IN, BioBuzzConstants.Auto.RED_GARDEN_Y_MAX_IN);
        return a == Alliance.RED ? red : red.rotated180(center, center);
    }

    /** Centers of the 4 POLLEN lying in the GARDEN of the given alliance at match start. */
    public List<Point2> gardenPollen(Alliance a) {
        List<Point2> out = new ArrayList<Point2>();
        for (double x : BioBuzzConstants.Auto.RED_GARDEN_POLLEN_X_IN) {
            Point2 red = new Point2(x, BioBuzzConstants.Auto.RED_GARDEN_POLLEN_Y_IN);
            out.add(a == Alliance.RED ? red : new Point2(size - red.x, size - red.y));
        }
        return Collections.unmodifiableList(out);
    }

    public Rect ownLoadingZone() {
        return loadingZone(alliance);
    }

    public Rect ownGarden() {
        return garden(alliance);
    }

    /** The middle of the own GARDEN POLLEN line: the natural target point of a GARDEN intake. */
    public Point2 ownGardenPollenCenter() {
        List<Point2> p = gardenPollen(alliance);
        double sx = 0;
        double sy = 0;
        for (Point2 q : p) {
            sx += q.x;
            sy += q.y;
        }
        return new Point2(sx / p.size(), sy / p.size());
    }

    // ---- HIVE cells ----------------------------------------------------------------------------------

    public CellSpec cell(Side side) {
        return model.cell(alliance, side);
    }

    /** The cell of this alliance's HIVE that is UP when the match starts. */
    public Side initialUpSide() {
        return model.initialUpSide(alliance);
    }

    /**
     * The plan view footprint of a cell of the given alliance: the 20 in wide cell on its HIVE axis,
     * reaching out from the pivot line on its side. An UP cell is the long box that points out
     * of the frame; a DOWN cell overhangs the frame the other way.
     */
    public Rect cellPlan(Alliance a, Side side, boolean up) {
        if (a == Alliance.BLUE) {
            // the blue cell on side S is the half-turn image of the red cell on the opposite side
            return cellPlan(Alliance.RED, side.opposite(), up).rotated180(center, center);
        }
        double near = up ? BioBuzzConstants.Auto.UP_CELL_PLAN_OUT_NEAR_IN : BioBuzzConstants.Auto.DOWN_CELL_PLAN_OUT_NEAR_IN;
        double far = up ? BioBuzzConstants.Auto.UP_CELL_PLAN_OUT_FAR_IN : BioBuzzConstants.Auto.DOWN_CELL_PLAN_OUT_FAR_IN;
        double axis = BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN;
        double half = BioBuzzConstants.Field.CELL_OPENING_WIDTH_IN / 2.0;
        double y0 = center + side.ySign() * near;
        double y1 = center + side.ySign() * far;
        return new Rect(axis - half, axis + half, y0, y1);
    }

    // ---- intake sources ------------------------------------------------------------------------------

    /** The nearest FLOWER within the matching radius of the point, or -1. */
    public int nearestFlower(Point2 p) {
        int best = -1;
        double bestDist = BioBuzzConstants.Auto.INTAKE_TARGET_MATCH_RADIUS_IN;
        for (int i = 0; i < BioBuzzConstants.Field.FLOWER_CENTERS_IN.length; i++) {
            double d = flowerCenter(i).distanceTo(p);
            if (d <= bestDist) {
                bestDist = d;
                best = i;
            }
        }
        return best;
    }

    /** The alliance whose GARDEN zone is within the matching radius of the point, or null if neither is. */
    public Alliance nearestGarden(Point2 p) {
        Alliance best = null;
        double bestDist = BioBuzzConstants.Auto.INTAKE_TARGET_MATCH_RADIUS_IN;
        for (Alliance a : Alliance.values()) {
            Rect g = garden(a);
            double dx = Math.max(Math.max(g.minX - p.x, 0.0), p.x - g.maxX);
            double dy = Math.max(Math.max(g.minY - p.y, 0.0), p.y - g.maxY);
            double d = Math.hypot(dx, dy);
            if (d <= bestDist) {
                bestDist = d;
                best = a;
            }
        }
        return best;
    }

    /** The nominal shooting pose in front of the given cell of this alliance's HIVE. */
    public Pose2 nominalShootPose(Side side) {
        return ShotGeometry.nominalPose(cell(side));
    }
}
