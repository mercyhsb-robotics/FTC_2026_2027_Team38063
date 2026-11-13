package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Keeps the robot's belief about which cell of its alliance's HIVE is UP.
 *
 * <p>The belief starts from the known match-start state (or the state carried over from autonomous)
 * and is changed only by evidence, with hysteresis so one bad frame cannot send the robot to the
 * other side of the field:
 * <ul>
 *   <li>A cluster that reads UP or DOWN says which cell is up. A different answer must repeat for
 *       several NEW camera frames over a minimum time before the belief changes.</li>
 *   <li>When a HIVE tips, the sticker plane faces away from the side the robot was shooting from, so
 *       all of that HIVE's tags vanish. Tags seen steadily and then lost (with a healthy camera link
 *       and the robot on that side) are treated as a suspected tip: the belief flips to the other
 *       side, marked unconfirmed until the other side's cluster is actually seen.</li>
 *   <li>The driver can force the belief with a button; that always wins.</li>
 * </ul>
 */
public final class HiveTracker {

    public static final class Config {
        public int confirmFrames;
        public double confirmSec;
        public double stableSeenSec;
        public double lostSec;

        public static Config fromConstants() {
            Config c = new Config();
            c.confirmFrames = BioBuzzConstants.Vision.HIVE_CONFIRM_FRAMES;
            c.confirmSec = BioBuzzConstants.Vision.HIVE_CONFIRM_SEC;
            c.stableSeenSec = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
            c.lostSec = BioBuzzConstants.Vision.HIVE_LOST_SEC;
            return c;
        }
    }

    /** What changed during one update. */
    public enum Event {
        NONE,
        CONFIRMED,
        CHANGED_BY_VISION,
        SUSPECTED_TIP,
        CHANGED_BY_DRIVER
    }

    private final FieldModel field;
    private final Config cfg;

    private Side upSide;
    private boolean confirmed;
    private int changes;

    private Side candidate;
    private int candidateFrames;
    private double candidateSince;

    private boolean seeing;
    private double seenSince;
    private double lastSeenSec;
    private double noTagsSince = Double.NaN;

    public HiveTracker(FieldModel field, Alliance alliance, Config cfg) {
        this.field = field;
        this.cfg = cfg;
        reset(field.initialUpSide(alliance), false);
    }

    /** Sets the belief (match start, carried over from autonomous, or after a pose reset). */
    public void reset(Side up, boolean isConfirmed) {
        this.upSide = up;
        this.confirmed = isConfirmed;
        this.candidate = null;
        this.candidateFrames = 0;
        this.seeing = false;
        this.noTagsSince = Double.NaN;
    }

    public Side upSide() {
        return upSide;
    }

    /** True if the camera has actually seen the UP cell's cluster since the belief last changed. */
    public boolean confirmed() {
        return confirmed;
    }

    /** How many times the belief has changed since construction (any cause). */
    public int changeCount() {
        return changes;
    }

    /** Which side of the HIVE a robot at this pose is on. */
    public static Side sideOf(Pose2 robot, FieldModel field) {
        return robot.y < field.center() ? Side.AUDIENCE : Side.REAR;
    }

    /** The driver says which side is up; this wins over everything. */
    public Event forceUp(Side side) {
        if (side != upSide) {
            changes++;
        }
        reset(side, false);
        return Event.CHANGED_BY_DRIVER;
    }

    public Event toggle() {
        return forceUp(upSide.opposite());
    }

    /**
     * Call once per loop.
     *
     * @param nowSec    the logic clock
     * @param frame     this loop's camera snapshot
     * @param views     {@link ClusterAnalyzer} output for this frame (ignored unless the frame is usable)
     * @param robotSide the side of the HIVE the robot is standing on
     */
    public Event update(double nowSec, VisionFrame frame, List<ClusterView> views, Side robotSide) {
        if (!frame.usable() || !frame.newFrame) {
            return Event.NONE;      // a dead, stale or repeated frame teaches us nothing
        }

        if (views.isEmpty()) {
            if (Double.isNaN(noTagsSince)) {
                noTagsSince = nowSec;
            }
            return checkLost(nowSec, robotSide);
        }

        if (!seeing) {
            seeing = true;
            seenSince = nowSec;
        }
        lastSeenSec = nowSec;
        noTagsSince = Double.NaN;

        Side evidence = evidenceOf(views);
        if (evidence == null) {
            return Event.NONE;
        }
        if (evidence == upSide) {
            candidate = null;
            candidateFrames = 0;
            if (!confirmed) {
                confirmed = true;
                return Event.CONFIRMED;
            }
            return Event.NONE;
        }
        if (candidate != evidence) {
            candidate = evidence;
            candidateFrames = 0;
            candidateSince = nowSec;
        }
        candidateFrames++;
        if (candidateFrames >= cfg.confirmFrames && nowSec - candidateSince >= cfg.confirmSec) {
            upSide = evidence;
            confirmed = true;
            candidate = null;
            candidateFrames = 0;
            seeing = true;
            seenSince = nowSec;
            changes++;
            return Event.CHANGED_BY_VISION;
        }
        return Event.NONE;
    }

    private Event checkLost(double nowSec, Side robotSide) {
        if (!seeing || robotSide != upSide) {
            return Event.NONE;
        }
        boolean steady = lastSeenSec - seenSince >= cfg.stableSeenSec;
        boolean gone = nowSec - noTagsSince >= cfg.lostSec;
        if (steady && gone) {
            upSide = upSide.opposite();
            confirmed = false;
            seeing = false;
            candidate = null;
            candidateFrames = 0;
            changes++;
            return Event.SUSPECTED_TIP;
        }
        return Event.NONE;
    }

    /** The side the clusters in this frame agree is UP, or null if they say nothing or disagree. */
    private Side evidenceOf(List<ClusterView> views) {
        Side result = null;
        for (ClusterView v : views) {
            Side says;
            if (v.orientation == ClusterView.Orientation.UP) {
                says = v.cell.side;
            } else if (v.orientation == ClusterView.Orientation.DOWN) {
                says = v.cell.side.opposite();
            } else {
                continue;
            }
            if (result != null && result != says) {
                return null;
            }
            result = says;
        }
        return result;
    }

    public FieldModel field() {
        return field;
    }
}
