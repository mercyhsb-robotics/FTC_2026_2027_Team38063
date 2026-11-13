package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The exact solid of the HIVE below a given height, seen from above: the parts of the HIVE a robot of that height can touch. The whole
 * HIVE footprint ({@link FieldModel#hiveFrame()}) is a plain box, which is right for "never go near it" but throws away the one thing
 * that makes the HIVE special for a low robot: between its two feet the floor is OPEN, a lane a 12 to 13 in tall robot can drive along
 * under the cells (the lowest point of any cell is 30.65 in up).
 *
 * <p>What is solid below the robot's height (all numbers from the official field CAD, in {@code BioBuzzConstants.Field}):
 * <ul>
 *   <li><b>Two foot bars</b>, 1.98 in wide, 38.94 in long and 2.15 in tall, one at each side of the frame. They hold the HIVE to the floor.
 *       A robot cannot drive over one (a 2.15 in step) so it can only go BETWEEN them, north-south.</li>
 *   <li><b>Four leg stubs.</b> A leg is a tube that starts on the end of a foot bar and leans inward as it climbs, 0.28 in toward the center
 *       along x and 0.45 in along y for every inch of height. At 12 to 13 in it has moved about 3.2 in in along x and 5.2 in along y, so
 *       what is left of it below the robot is a short slanted stub on each end of each bar. They narrow the lane at the two ends of the
 *       frame (x 50.61 to 90.73 in the wall-corner frame); between the stubs the lane is as wide as the gap between the bars (x 47.92 to
 *       93.42).</li>
 * </ul>
 * Every piece is a {@link ConvexPolygon}. Together they are SMALLER than the footprint box: the box also contains the lane.
 *
 * <p>Who uses it: {@link FieldModel#hiveSolid()} builds it for the field's height limit ({@code Auto.HIVE_SOLID_HEIGHT_IN}). The route
 * validator checks a step that says {@code allowUnderHive} against these pieces (with {@code Auto.HIVE_LANE_MARGIN_IN} on both sides)
 * instead of the box, and the TeleOp planner does the same when {@code AutoShoot.ALLOW_UNDER_HIVE} is on.
 *
 * <p>Words used: <b>plan</b> is the view from above; <b>stub</b> is the part of a leg that is lower than the robot (the rest of the leg
 * passes over it); <b>lane</b> is the open floor between the left pieces and the right pieces. The HIVE is the same for both alliances (one
 * frame carries both HIVEs), so the solid is its own half-turn image.
 */
public final class HiveSolid {
    private final double heightIn;
    private final List<ConvexPolygon> pieces;
    private final Rect envelope;
    private final double center;

    private HiveSolid(double center, double heightIn, List<ConvexPolygon> pieces, Rect envelope) {
        this.center = center;
        this.heightIn = heightIn;
        this.pieces = Collections.unmodifiableList(pieces);
        this.envelope = envelope;
    }

    /**
     * The solid below {@code heightIn} for a HIVE frame centered on the field center, from the CAD numbers in {@code BioBuzzConstants.Field}.
     * Below the foot of a leg only the bars exist; a taller robot gets longer leg stubs.
     *
     * @param fieldCenter the field center on both axes, in inches (the HIVE frame is centered on it)
     * @param heightIn    the robot's height: parts of the HIVE higher than this do not count
     */
    public static HiveSolid of(double fieldCenter, double heightIn) {
        List<ConvexPolygon> pieces = new ArrayList<ConvexPolygon>();
        double halfWidth = BioBuzzConstants.Field.HIVE_FRAME_WIDTH_X_IN / 2.0;
        double halfDepth = BioBuzzConstants.Field.HIVE_FRAME_DEPTH_Y_IN / 2.0;
        double innerHalfGap = BioBuzzConstants.Field.HIVE_FOOT_BAR_INNER_HALF_GAP_IN;
        // the two foot bars: left (x below the center) and right
        for (int sideX = -1; sideX <= 1; sideX += 2) {
            double inner = fieldCenter + sideX * innerHalfGap;
            double outer = fieldCenter + sideX * halfWidth;
            pieces.add(ConvexPolygon.of(new Rect(inner, outer, fieldCenter - halfDepth, fieldCenter + halfDepth)));
        }
        // the four leg stubs: the leg's middle line starts on the bar and leans toward the center
        double climb = heightIn - BioBuzzConstants.Field.HIVE_LEG_FOOT_Z_IN;
        if (climb > 0.0) {
            double r = BioBuzzConstants.Field.HIVE_LEG_HALF_WIDTH_IN;
            for (int sideX = -1; sideX <= 1; sideX += 2) {
                for (int sideY = -1; sideY <= 1; sideY += 2) {
                    double footX = fieldCenter + sideX * BioBuzzConstants.Field.HIVE_LEG_FOOT_HALF_SPAN_X_IN;
                    double footY = fieldCenter + sideY * BioBuzzConstants.Field.HIVE_LEG_FOOT_HALF_SPAN_Y_IN;
                    double topX = footX - sideX * BioBuzzConstants.Field.HIVE_LEG_LEAN_X_PER_Z * climb;
                    double topY = footY - sideY * BioBuzzConstants.Field.HIVE_LEG_LEAN_Y_PER_Z * climb;
                    List<Point2> corners = new ArrayList<Point2>();
                    for (int cx = -1; cx <= 1; cx += 2) {
                        for (int cy = -1; cy <= 1; cy += 2) {
                            corners.add(new Point2(footX + cx * r, footY + cy * r));
                            corners.add(new Point2(topX + cx * r, topY + cy * r));
                        }
                    }
                    pieces.add(ConvexPolygon.hull(corners));
                }
            }
        }
        Rect envelope = pieces.get(0).bounds();
        double minX = envelope.minX;
        double maxX = envelope.maxX;
        double minY = envelope.minY;
        double maxY = envelope.maxY;
        for (ConvexPolygon p : pieces) {
            Rect b = p.bounds();
            minX = Math.min(minX, b.minX);
            maxX = Math.max(maxX, b.maxX);
            minY = Math.min(minY, b.minY);
            maxY = Math.max(maxY, b.maxY);
        }
        return new HiveSolid(fieldCenter, heightIn, pieces, new Rect(minX, maxX, minY, maxY));
    }

    /** The robot height this solid is for, in inches. */
    public double heightIn() {
        return heightIn;
    }

    /** Every solid piece, seen from above: the two foot bars first, then the four leg stubs (none if the robot is lower than a leg's foot). */
    public List<ConvexPolygon> pieces() {
        return pieces;
    }

    /** The box around all pieces. For the real HIVE this is the footprint of the two foot bars (the stubs stay inside it). */
    public Rect envelope() {
        return envelope;
    }

    /**
     * The signed gap between a robot footprint and the nearest piece: POSITIVE = that many inches of clear floor, 0 = touching a foot bar or
     * a stub, NEGATIVE = overlapping one (how deep). A robot in the lane has a positive gap equal to its distance to the nearer side.
     */
    public double signedGap(Footprint footprint) {
        ConvexPolygon robot = footprint.asPolygon();
        double best = Double.POSITIVE_INFINITY;
        for (ConvexPolygon p : pieces) {
            best = Math.min(best, robot.signedGap(p));
        }
        return best;
    }

    /**
     * The open floor in a row {@code y} of the frame, as {min x, max x}: from the nearest solid on the left (the red side, below the center in x)
     * to the nearest on the right. A robot that must pass this row fits if it is narrower than the interval (plus its margins). Returns null
     * if the row is outside the frame.
     */
    public double[] laneAtRow(double y) {
        double left = Double.NEGATIVE_INFINITY;
        double right = Double.POSITIVE_INFINITY;
        boolean inside = false;
        for (ConvexPolygon p : pieces) {
            double[] range = p.xRangeAt(y);
            if (range == null) {
                continue;
            }
            inside = true;
            if (p.bounds().maxX <= center) {
                left = Math.max(left, range[1]);
            } else {
                right = Math.min(right, range[0]);
            }
        }
        return inside ? new double[] {left, right} : null;
    }

    /**
     * The narrowest the lane gets anywhere between the two ends of the frame, as {min x, max x}: the tips of the leg stubs, which are the
     * pieces that reach farthest toward the center. A robot (with its margins) that fits here fits the whole lane.
     */
    public double[] narrowestLane() {
        double left = Double.NEGATIVE_INFINITY;
        double right = Double.POSITIVE_INFINITY;
        for (ConvexPolygon p : pieces) {
            Rect b = p.bounds();
            if (b.maxX <= center) {
                left = Math.max(left, b.maxX);
            } else {
                right = Math.min(right, b.minX);
            }
        }
        return new double[] {left, right};
    }

    /** The solid turned half a turn about the field center: the same solid (the frame is symmetric), kept so a test can prove it. */
    public List<ConvexPolygon> halfTurnedPieces() {
        List<ConvexPolygon> out = new ArrayList<ConvexPolygon>();
        for (ConvexPolygon p : pieces) {
            out.add(p.rotated180(center, center));
        }
        return out;
    }
}
