package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Remembers the robot's pose, alliance and HIVE belief between OpModes (autonomous to TeleOp) in plain
 * static fields, which live as long as the robot controller app does.
 *
 * <p>The pose is only handed back if it is fresh, so a pose left over from an earlier practice run is
 * never mistaken for where the robot is now. Time is a parameter so tests do not need a clock.
 */
public final class PoseStore {

    /** A saved snapshot. */
    public static final class Snapshot {
        public final Pose2 pose;
        public final Alliance alliance;
        public final Side hiveUpSide;
        public final boolean hiveConfirmed;

        Snapshot(Pose2 pose, Alliance alliance, Side hiveUpSide, boolean hiveConfirmed) {
            this.pose = pose;
            this.alliance = alliance;
            this.hiveUpSide = hiveUpSide;
            this.hiveConfirmed = hiveConfirmed;
        }
    }

    private static Snapshot saved;
    private static long savedAtNanos;

    private PoseStore() {
    }

    public static synchronized void save(Pose2 pose, Alliance alliance, Side hiveUpSide, boolean hiveConfirmed,
                                         long nowNanos) {
        saved = new Snapshot(pose, alliance, hiveUpSide, hiveConfirmed);
        savedAtNanos = nowNanos;
    }

    /** @return the saved snapshot if it is no older than {@code maxAgeSec}, otherwise null */
    public static synchronized Snapshot load(long nowNanos, double maxAgeSec) {
        if (saved == null) {
            return null;
        }
        double ageSec = (nowNanos - savedAtNanos) / 1.0e9;
        return ageSec >= 0.0 && ageSec <= maxAgeSec ? saved : null;
    }

    public static synchronized void clear() {
        saved = null;
    }
}
