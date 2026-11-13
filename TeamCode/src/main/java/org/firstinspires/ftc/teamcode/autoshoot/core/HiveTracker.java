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
 *
 * <p><b>Why a "belief" at all?</b> Only one cell of the HIVE is tipped UP and scorable, and it flips every time
 * enough POLLEN is launched into it. The robot cannot be sure which one is up at any instant (the camera may see
 * nothing for a moment), so it keeps a best guess, the {@link #upSide() belief}, and a flag saying whether the
 * camera or the driver has actually checked it ({@link #confirmed() confirmed}). While a camera is working, the
 * shot gate refuses to feed until the belief is confirmed, because shooting at the down cell is a foul (rule G417).
 * "Hysteresis" means the belief is hard to change: it takes several agreeing frames, so it does not flip back
 * and forth on a single noisy reading (like a thermostat that waits for a real temperature change).
 *
 * <p>Data flow: {@link ClusterAnalyzer} turns each camera frame into {@link ClusterView}s; the TeleOp passes them
 * to {@link #update} every loop and calls {@link #toggle} when the driver presses the "other cell is up" button.
 * {@code AutoShootController} reads {@link #upSide()} to choose which cell to drive to and calls
 * {@link #flipBecauseUnseen()}. The TeleOp also saves and restores the belief through {@link PoseStore}.
 * Time comes in as a parameter (never a clock), so tests can replay any sequence of frames.
 */
public final class HiveTracker {

    /** The tuning numbers of the tracker; the defaults come from {@code BioBuzzConstants.Vision.HIVE_*}. */
    public static final class Config {
        /** How many NEW camera frames in a row must give the same, different answer before the belief changes. */
        public int confirmFrames;
        /** ... and those frames must span at least this many seconds. */
        public double confirmSec;
        /** The up cell's tags must have been seen this long (seconds) for losing them to count as a tip. */
        public double stableSeenSec;
        /** Tags must then be missing for at least this long (seconds, with the camera healthy) to be called a tip. */
        public double lostSec;

        /** The values in {@code BioBuzzConstants.Vision}. */
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
        /** Nothing worth reporting happened. */
        NONE,
        /** The belief did not change, but the camera has now seen the up cell, so it counts as confirmed. */
        CONFIRMED,
        /** Several frames in a row convinced the tracker that the OTHER cell is up, so the belief flipped. */
        CHANGED_BY_VISION,
        /** The belief flipped without positive proof (the up cell's tags vanished, or were never seen): unconfirmed. */
        SUSPECTED_TIP,
        /** The driver set the belief with the button. */
        CHANGED_BY_DRIVER
    }

    private final FieldModel field;
    private final Config cfg;

    // The belief: which side's cell is UP, whether it has been confirmed, and how many times it has changed.
    private Side upSide;
    private boolean confirmed;
    private int changes;

    // A rival answer from the camera that has not yet been repeated often enough to be believed:
    // which side it says is up, how many NEW frames in a row said so, and when the first of them arrived.
    private Side candidate;
    private int candidateFrames;
    private double candidateSince;

    // Tip detection. "seeing" = we have been seeing our alliance's tags (since seenSince); lastSeenSec is the
    // most recent time we saw any; noTagsSince is when the current run of tag-less frames began (NaN = none).
    private boolean seeing;
    private double seenSince;
    private double lastSeenSec;
    private double noTagsSince = Double.NaN;

    /** Starts with the belief the match starts in (see {@link FieldModel#initialUpSide}), not yet confirmed. */
    public HiveTracker(FieldModel field, Alliance alliance, Config cfg) {
        this.field = field;
        this.cfg = cfg;
        reset(field.initialUpSide(alliance), false);
    }

    /** Sets the belief (match start, carried over from autonomous, or after a pose reset). */
    public void reset(Side up, boolean isConfirmed) {
        this.upSide = up;
        this.confirmed = isConfirmed;
        // Throw away any half-built evidence: it was about the old belief.
        this.candidate = null;
        this.candidateFrames = 0;
        this.seeing = false;
        this.noTagsSince = Double.NaN;
    }

    /** Which cell the robot currently believes is UP. */
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
        // The HIVE sits on the field's center line (y = Field.CENTER_IN, 70.674 in); smaller y is the audience end, larger y the rear end.
        return robot.y < field.center() ? Side.AUDIENCE : Side.REAR;
    }

    /**
     * The driver says which side is up; this wins over everything and counts as confirmed (the
     * driver can see the HIVE, and it is the way to proceed when the camera is not working).
     */
    public Event forceUp(Side side) {
        if (side != upSide) {
            changes++;
        }
        reset(side, true);
        return Event.CHANGED_BY_DRIVER;
    }

    /**
     * The controller stood at the scoring pose and never saw this side's tags, so the other cell is
     * probably the UP one. Flips the belief, unconfirmed.
     */
    public Event flipBecauseUnseen() {
        changes++;
        reset(upSide.opposite(), false);
        return Event.SUSPECTED_TIP;
    }

    /** The driver's "the other cell is up" button: forces the belief to the opposite side (confirmed). */
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

        // A good new frame with none of our alliance's tags: possibly a tip. Start (or continue) the clock on
        // how long they have been missing and let checkLost decide.
        if (views.isEmpty()) {
            if (Double.isNaN(noTagsSince)) {
                noTagsSince = nowSec;
            }
            return checkLost(nowSec, robotSide);
        }

        // We can see tags: remember since when, and cancel any "missing" timer.
        if (!seeing) {
            seeing = true;
            seenSince = nowSec;
        }
        lastSeenSec = nowSec;
        noTagsSince = Double.NaN;

        // What do this frame's clusters say about which side is up? (null = nothing clear, or they disagree.)
        Side evidence = evidenceOf(views);
        if (evidence == null) {
            return Event.NONE;
        }
        // The camera agrees with the belief: drop any rival answer and mark the belief confirmed.
        if (evidence == upSide) {
            candidate = null;
            candidateFrames = 0;
            if (!confirmed) {
                confirmed = true;
                return Event.CONFIRMED;
            }
            return Event.NONE;
        }
        // The camera disagrees. Count how many NEW frames in a row say the same rival answer; a different
        // answer restarts the count. Only after enough frames spread over enough time do we change our mind.
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

    /** Called on each new, healthy frame that shows none of our alliance's tags: is this what a HIVE tip looks like? */
    private Event checkLost(double nowSec, Side robotSide) {
        // Only counts if we had been seeing the tags, and we are on the side of the cell we believe is up
        // (a tip hides the tags from the side we were shooting from).
        if (!seeing || robotSide != upSide) {
            return Event.NONE;
        }
        // "steady": the tags were seen for a good stretch first, so this is a real loss, not a lucky one-off sighting.
        // "gone": and they have now been missing for long enough that a blink of the camera does not explain it.
        boolean steady = lastSeenSec - seenSince >= cfg.stableSeenSec;
        boolean gone = nowSec - noTagsSince >= cfg.lostSec;
        if (steady && gone) {
            // Suspected tip: believe the other cell is up now, but do not call it confirmed until we see its tags.
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
            // A cluster that reads UP means its own cell is up; one that reads DOWN means the OTHER cell is.
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

    /** The field model this tracker was built with. */
    public FieldModel field() {
        return field;
    }
}
