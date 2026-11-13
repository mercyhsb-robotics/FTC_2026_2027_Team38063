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
 *
 * <p><b>Where it sits in the data flow.</b> {@code LimelightVision} makes {@link TagObservation}s, which travel
 * in a {@link VisionFrame}; this class turns them into {@link ClusterView}s; those go to {@link HiveTracker}
 * (which cell is up?), {@link PoseCorrector} (is odometry drifting?) and {@code AutoShootController} (how far
 * and which way to turn). The TeleOp and the Limelight Diagnostic OpMode call {@link #analyze} once per loop.
 * It is plain Java with no hardware, so the unit tests feed it tags from a simulated camera.
 *
 * <p>Words used here: a "cluster" is the (up to) four tags on one cell; the "tag row" is the horizontal line
 * those tags sit on; "tx" is how many degrees a tag is to the right (+) or left (-) of the image centre; the
 * "implied position" is "where the robot must be standing for the tags to look the way they do".
 */
public final class ClusterAnalyzer {

    /** Numbers the analyzer needs from outside the field model. */
    public static final class Config {
        /** Ball exit position: inches forward of the robot's rotation center (negative: shooter fires out the back). */
        public double launchForwardIn;
        /** Ball exit position: inches left of the robot's center line (negative = right). */
        public double launchLeftIn;
        /** A cluster whose tag height is farther than this from BOTH the UP and DOWN rows is not trusted (inches). */
        public double tagHeightToleranceIn;
        /** The left-to-right order of tag IDs is used only if the tags are at least this far apart (degrees of tx). */
        public double orderMinTxSpanDeg;

        /** The values in {@code BioBuzzConstants} (launch point, tag height tolerance, order span). */
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
     * Groups the frame's tags by cell and analyzes each group. Call once per loop.
     *
     * @param own            the alliance whose HIVE we shoot into; tags of the other alliance are ignored
     * @param robotHeadingRad the robot's heading from odometry (field frame, radians, counter-clockwise from +x)
     * @param tags           every tag in the frame
     * @return one view per cell of {@code own} that has at least one tag in the frame
     */
    public List<ClusterView> analyze(Alliance own, double robotHeadingRad, List<TagObservation> tags) {
        // One bucket of tags per side (AUDIENCE, REAR) of our HIVE. Tags that belong to the other alliance's
        // HIVE, or that are not HIVE tags at all (cellForTag gives null), are skipped.
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

    /** Analyzes the tags of ONE cell: UP/DOWN first, then (only for an UP cell) range, aim and implied position. */
    private ClusterView analyzeCell(CellSpec cell, double heading, List<TagObservation> group) {
        ClusterView.Orientation orientation = orientationOf(cell, group);

        // The average direction to the cluster. Directions are averaged as (sin, cos) pairs rather than as plain
        // numbers because our camera looks out the back, so tag directions sit near +/-PI where the angle wraps
        // around: plain-averaging +179 and -179 degrees would give 0 (straight ahead) instead of 180 (behind).
        double meanAzimuthSin = 0.0;
        double meanAzimuthCos = 0.0;
        for (TagObservation t : group) {
            meanAzimuthSin += Math.sin(t.azimuthRad);
            meanAzimuthCos += Math.cos(t.azimuthRad);
        }
        double meanAzimuth = Math.atan2(meanAzimuthSin, meanAzimuthCos);

        // Range and position need a measured distance for every tag (the "Full 3D" pipeline setting).
        boolean allPositioned = true;
        for (TagObservation t : group) {
            allPositioned &= t.hasPosition;
        }

        // Only an UP cluster gets geometry: the field model knows where an UP cell's tags and aim point are.
        if (orientation == ClusterView.Orientation.UP && allPositioned) {
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            // The sticker runs along field x; that direction in the robot frame:
            double exF = cosH;
            double exL = -sinH;
            // Find the middle of the tag row (cf = forward, cl = left, robot frame). Each tag sits at a known
            // offset along the sticker from that middle, so (measured tag position) - (its offset) is one guess for
            // the middle. Average the guesses: any subset of the four tags gives an answer.
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

            // Range and bearing are measured from the ball exit, not the robot center, because that is where
            // the ball starts. The shooter fires out the back, which is "PI" in the robot frame, so the aim
            // error is (bearing to the aim point) - PI: positive means turn counter-clockwise.
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

        // No 3D position (or not UP): the best we can offer is a bearing toward the cluster's tags, and only
        // if the cell reads UP, because nobody should aim at a DOWN or unknown cell.
        double aimError = Angles.error(Math.PI, meanAzimuth);
        boolean bearingUsable = orientation == ClusterView.Orientation.UP;
        return new ClusterView(cell, group.size(), orientation, false, Double.NaN, Double.NaN, Double.NaN,
                bearingUsable ? aimError : Double.NaN, bearingUsable, Double.NaN, Double.NaN);
    }

    /** Decides UP / DOWN / UNKNOWN for one cell from two independent clues (ID order, and tag height). */
    private ClusterView.Orientation orientationOf(CellSpec cell, List<TagObservation> group) {
        // Clue 1: the left-to-right ORDER of the tag IDs in the image. Needs at least two tags that are far
        // enough apart in tx to tell which is left of which.
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
                // "slope" says whether tx rises with the tag index: positive = IDs run left to right = UP,
                // negative = IDs run right to left = DOWN. (It is a covariance, so it works for any 2 to 4 tags.)
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

        // Clue 2: the HEIGHT of the tags above the floor (only if every tag has a 3D position). The UP row is
        // about 50 in high and the DOWN row about 36 in, so the average height says which one we are looking at.
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
                heightImplausible = true;     // matches neither row: a bad reading, or not really our tags
            }
        }

        // Combine the clues. A height that fits neither row spoils everything; with only one clue we use it;
        // two clues must agree, and if they contradict each other we say UNKNOWN rather than guess.
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
