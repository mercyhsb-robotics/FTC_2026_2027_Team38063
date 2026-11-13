package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Turns one frame's AprilTags into what the shooter needs to know about each HIVE cell of one alliance:
 * is the cell UP or DOWN, how far away is its aim point, which way must the robot turn to point the
 * shooter at it, and where does that put the robot on the field.
 *
 * <p><b>UP or DOWN.</b> The stickers of both cells of a HIVE lie in one plane. Read upright from the
 * UP cell's side, the tag IDs run left to right in increasing order; the other cell's sticker, seen
 * across the gap, is turned half a turn, so its IDs run right to left. That left-to-right order is
 * checked with {@code tx} only, which does not depend on any 3D axis convention. When 3D positions
 * are available the tag HEIGHT (UP row about 50 in, DOWN row about 36 in) is a second, independent
 * vote; if the two votes disagree the cluster is reported UNKNOWN.
 *
 * <p><b>Geometry.</b> The tags of a cluster lie along the field x axis at known offsets from the
 * cluster centre, so with the robot's heading (from odometry, which is reliable) every tag position
 * gives an estimate of the tag-row centre; the estimates are averaged, so seeing only some of the four
 * tags works. The aim point is a fixed offset from the tag row, and the implied robot position is the
 * known tag-row position minus the measured one.
 */
public final class ClusterAnalyzer {

    /** Numbers the analyzer needs from outside the field model. */
    public static final class Config {
        public double launchForwardIn;
        public double launchLeftIn;
        public double tagHeightToleranceIn;
        public double orderMinTxSpanDeg;

        public static Config fromConstants() {
            Config c = new Config();
            c.launchForwardIn = BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN;
            c.launchLeftIn = BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN;
            c.tagHeightToleranceIn = BioBuzzConstants.Vision.TAG_HEIGHT_TOLERANCE_IN;
            c.orderMinTxSpanDeg = BioBuzzConstants.Vision.ORDER_MIN_TX_SPAN_DEG;
            return c;
        }
    }

    private final FieldModel field;
    private final Config cfg;

    public ClusterAnalyzer(FieldModel field, Config cfg) {
        this.field = field;
        this.cfg = cfg;
    }

    /**
     * @param own            the alliance whose HIVE we shoot into; tags of the other alliance are ignored
     * @param robotHeadingRad the robot's heading from odometry
     * @param tags           every tag in the frame
     * @return one view per cell of {@code own} that has at least one tag in the frame
     */
    public List<ClusterView> analyze(Alliance own, double robotHeadingRad, List<TagObservation> tags) {
        List<List<TagObservation>> bySide = new ArrayList<List<TagObservation>>();
        for (int i = 0; i < Side.values().length; i++) {
            bySide.add(new ArrayList<TagObservation>());
        }
        for (TagObservation t : tags) {
            CellSpec cell = field.cellForTag(t.id);
            if (cell != null && cell.alliance == own) {
                bySide.get(cell.side.ordinal()).add(t);
            }
        }
        List<ClusterView> views = new ArrayList<ClusterView>();
        for (Side side : Side.values()) {
            List<TagObservation> group = bySide.get(side.ordinal());
            if (!group.isEmpty()) {
                views.add(analyzeCell(field.cell(own, side), robotHeadingRad, group));
            }
        }
        return views;
    }

    private ClusterView analyzeCell(CellSpec cell, double heading, List<TagObservation> group) {
        ClusterView.Orientation orientation = orientationOf(cell, group);

        double meanAzimuthSin = 0.0;
        double meanAzimuthCos = 0.0;
        for (TagObservation t : group) {
            meanAzimuthSin += Math.sin(t.azimuthRad);
            meanAzimuthCos += Math.cos(t.azimuthRad);
        }
        double meanAzimuth = Math.atan2(meanAzimuthSin, meanAzimuthCos);

        boolean allPositioned = true;
        for (TagObservation t : group) {
            allPositioned &= t.hasPosition;
        }

        if (orientation == ClusterView.Orientation.UP && allPositioned) {
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            // The sticker runs along field x; that direction in the robot frame:
            double exF = cosH;
            double exL = -sinH;
            double cf = 0.0;
            double cl = 0.0;
            for (TagObservation t : group) {
                double s = cell.tagXSign * cell.tagOffsetAlongSticker(cell.tagIndex(t.id));
                cf += t.forwardIn - s * exF;
                cl += t.leftIn - s * exL;
            }
            cf /= group.size();
            cl /= group.size();

            // From the tag-row centre to the aim point, in the field and then in the robot frame.
            double dOx = cell.originX - cell.axisX;
            double dOy = cell.originY - cell.tagRowY;
            double aimF = cf + (dOx * cosH + dOy * sinH);
            double aimL = cl + (-dOx * sinH + dOy * cosH);

            double vf = aimF - cfg.launchForwardIn;
            double vl = aimL - cfg.launchLeftIn;
            double range = Math.hypot(vf, vl);
            double aimError = Angles.error(Math.PI, Math.atan2(vl, vf));

            // Robot position implied by the known tag-row centre and the measured one.
            double impliedX = cell.axisX - (cf * cosH - cl * sinH);
            double impliedY = cell.tagRowY - (cf * sinH + cl * cosH);
            return new ClusterView(cell, group.size(), orientation, true, cf, cl, range,
                    aimError, true, impliedX, impliedY);
        }

        double aimError = Angles.error(Math.PI, meanAzimuth);
        boolean bearingUsable = orientation == ClusterView.Orientation.UP;
        return new ClusterView(cell, group.size(), orientation, false, Double.NaN, Double.NaN, Double.NaN,
                bearingUsable ? aimError : Double.NaN, bearingUsable, Double.NaN, Double.NaN);
    }

    private ClusterView.Orientation orientationOf(CellSpec cell, List<TagObservation> group) {
        ClusterView.Orientation byOrder = ClusterView.Orientation.UNKNOWN;
        if (group.size() >= 2) {
            double meanIdx = 0.0;
            double meanTx = 0.0;
            double minTx = Double.POSITIVE_INFINITY;
            double maxTx = Double.NEGATIVE_INFINITY;
            for (TagObservation t : group) {
                meanIdx += cell.tagIndex(t.id);
                meanTx += t.txDeg;
                minTx = Math.min(minTx, t.txDeg);
                maxTx = Math.max(maxTx, t.txDeg);
            }
            meanIdx /= group.size();
            meanTx /= group.size();
            if (maxTx - minTx >= cfg.orderMinTxSpanDeg) {
                double slope = 0.0;
                for (TagObservation t : group) {
                    slope += (cell.tagIndex(t.id) - meanIdx) * (t.txDeg - meanTx);
                }
                if (slope > 0.0) {
                    byOrder = ClusterView.Orientation.UP;
                } else if (slope < 0.0) {
                    byOrder = ClusterView.Orientation.DOWN;
                }
            }
        }

        ClusterView.Orientation byHeight = ClusterView.Orientation.UNKNOWN;
        boolean heights = true;
        double meanZ = 0.0;
        for (TagObservation t : group) {
            heights &= t.hasPosition;
            meanZ += t.heightIn;
        }
        boolean heightImplausible = false;
        if (heights) {
            meanZ /= group.size();
            double upErr = Math.abs(meanZ - cell.tagRowZ);
            double downErr = Math.abs(meanZ - cell.downTagRowZ);
            if (Math.min(upErr, downErr) <= cfg.tagHeightToleranceIn) {
                byHeight = upErr < downErr ? ClusterView.Orientation.UP : ClusterView.Orientation.DOWN;
            } else {
                heightImplausible = true;
            }
        }

        if (heightImplausible) {
            return ClusterView.Orientation.UNKNOWN;
        }
        if (byOrder == ClusterView.Orientation.UNKNOWN) {
            return byHeight;
        }
        if (byHeight == ClusterView.Orientation.UNKNOWN || byHeight == byOrder) {
            return byOrder;
        }
        return ClusterView.Orientation.UNKNOWN;
    }
}
