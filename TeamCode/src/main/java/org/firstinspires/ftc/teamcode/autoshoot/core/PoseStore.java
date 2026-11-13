package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Remembers the robot's pose, alliance and HIVE belief between OpModes (autonomous to TeleOp) in plain
 * static fields, which live as long as the robot controller app does.
 *
 * <p>The pose is only handed back if it is fresh, so a pose left over from an earlier practice run is
 * never mistaken for where the robot is now. Time is a parameter so tests do not need a clock.
 *
 * <p>Why it exists: every OpMode builds its own objects (drive, tracker, controller) and tells the odometry
 * where the robot starts, so without help it would have to assume the default start pose. Something has to
 * remember where the robot ended and what it believed about the HIVE, and hand that to the next OpMode. A static
 * field is the simple way to pass data from one OpMode to the next while the app stays open. (If the Robot
 * Controller app is restarted, the store is empty again.)
 *
 * <p>Who uses it today: {@code BioBuzzRobot} (TeleOp) saves in its {@code stop()} and loads in
 * {@code init()}, {@code init_loop()} and {@code start()}. The autonomous OpModes do not save yet.
 * Callers pass {@code System.nanoTime()} as {@code nowNanos}.
 */
public final class PoseStore {

    /** A saved snapshot. */
    public static final class Snapshot {
        /** Where the robot was when it was saved (field frame: inches, radians). */
        public final Pose2 pose;
        /** Which alliance the driver had chosen. */
        public final Alliance alliance;
        /** Which cell of this alliance's HIVE the robot believed was UP. */
        public final Side hiveUpSide;
        /** True if that belief had been confirmed by the camera or the driver (see {@code HiveTracker}). */
        public final boolean hiveConfirmed;

        Snapshot(Pose2 pose, Alliance alliance, Side hiveUpSide, boolean hiveConfirmed) {
            this.pose = pose;
            this.alliance = alliance;
            this.hiveUpSide = hiveUpSide;
            this.hiveConfirmed = hiveConfirmed;
        }
    }

    // The one remembered snapshot (null = nothing saved) and the System.nanoTime() reading when it was saved.
    private static Snapshot saved;
    private static long savedAtNanos;

    private PoseStore() {
    }

    /**
     * Remembers a snapshot, replacing any earlier one. The methods are {@code synchronized} so two threads can
     * never see a half-written snapshot.
     *
     * @param nowNanos the time of the save, from {@code System.nanoTime()} (nanoseconds)
     */
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
        // Too old is rejected (see the class comment). A negative age ("saved in the future") is not trusted either.
        return ageSec >= 0.0 && ageSec <= maxAgeSec ? saved : null;
    }

    /** Forgets the saved snapshot (used by tests, so one test cannot leak into the next). */
    public static synchronized void clear() {
        saved = null;
    }
}
