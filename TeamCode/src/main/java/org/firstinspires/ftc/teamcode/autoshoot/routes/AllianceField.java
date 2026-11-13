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

    public AllianceField(FieldModel model, Alliance alliance) {
        this.model = model;
        this.alliance = alliance;
        this.size = model.size();
        this.center = model.center();
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

    /** The HIVE frame grown by {@link BioBuzzConstants.Auto#HIVE_KEEPOUT_MARGIN_IN}: where no leg may enter. */
    public Rect hiveKeepOut() {
        return model.hiveFrame().inflate(BioBuzzConstants.Auto.HIVE_KEEPOUT_MARGIN_IN);
    }

    /** The four FLOWER plates (no margin), in the order of {@code Field.FLOWER_CENTERS_IN}. */
    public List<Rect> flowerPlates() {
        return model.flowerKeepOuts();
    }

    /** A FLOWER plate grown by {@link BioBuzzConstants.Auto#FLOWER_KEEPOUT_MARGIN_IN}. */
    public Rect flowerKeepOut(int index) {
        return model.flowerKeepOuts().get(index).inflate(BioBuzzConstants.Auto.FLOWER_KEEPOUT_MARGIN_IN);
    }

    /** The center of FLOWER {@code index}. */
    public Point2 flowerCenter(int index) {
        double[] c = BioBuzzConstants.Field.FLOWER_CENTERS_IN[index];
        return new Point2(c[0], c[1]);
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
