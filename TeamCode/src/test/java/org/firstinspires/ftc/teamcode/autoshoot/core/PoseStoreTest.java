package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

import org.junit.After;
import org.junit.Test;

/**
 * Checks {@link PoseStore}, the small memory that carries the robot's pose, alliance and HIVE belief from one OpMode to
 * the next. Thanks to it, an OpMode can start where the previous one stopped instead of assuming the default start
 * pose. (PoseStore's own docs name autonomous to TeleOp as the main use. Today the TeleOp saves when it stops and loads
 * when it starts.)
 *
 * <p>How it works: PoseStore keeps its data in plain {@code static} fields, which live until the robot controller app
 * closes. It also takes the time as a parameter instead of reading a clock, so the tests pass in made-up times (in
 * nanoseconds) and get the same answer every run. No robot is needed.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*PoseStoreTest"} to run only this
 * class).
 *
 * <p>A 'fixture' is the setup and cleanup scaffolding around a test. Here it is just the {@link #clean} method, which
 * empties the store after every test.
 */
public class PoseStoreTest {
    /**
     * Nanoseconds in one second. The TeleOp feeds PoseStore {@code System.nanoTime()}, which counts in nanoseconds, so
     * the tests do too.
     */
    private static final long SEC = 1_000_000_000L;

    /**
     * Runs after every test and empties the store. The store is static, so without this a pose saved by one test would
     * leak into the next and make the results depend on the order the tests run in.
     */
    @After
    public void clean() {
        PoseStore.clear();
    }

    /**
     * Situation: a snapshot is saved at t = 100 s, then loaded 30 s later, 400 s later, and at t = 50 s (before it was
     *     saved), always with a 300 s age limit.
     * Expected: the 30 s load returns the very same pose with the saved alliance, up side and confirmed flag; the other
     *     two loads return null.
     * Why: a pose left over from an earlier practice run must never be mistaken for where the robot is now, and a clock
     *     that jumps backwards is not trusted. (The TeleOp passes POSE_CARRYOVER_MAX_AGE_SEC, 300 s.)
     */
    @Test
    public void aFreshSnapshotComesBackAndAnOldOneDoesNot() {
        Pose2 p = Pose2.ofDegrees(60, 20, -90);
        PoseStore.save(p, Alliance.RED, Side.REAR, true, 100 * SEC);
        PoseStore.Snapshot s = PoseStore.load(130 * SEC, 300.0);
        assertNotNull(s);
        // assertSame checks it is the very same Pose2 object that was saved, not a look-alike copy.
        assertSame(p, s.pose);
        assertEquals(Alliance.RED, s.alliance);
        assertEquals(Side.REAR, s.hiveUpSide);
        assertEquals(true, s.hiveConfirmed);
        assertNull("older than the limit", PoseStore.load(500 * SEC, 300.0));
        assertNull("a clock that went backwards is never trusted", PoseStore.load(50 * SEC, 300.0));
    }

    /**
     * Situation: nothing has ever been saved; then a snapshot is saved and clear() is called.
     * Expected: load gives null both times.
     * Why: with no saved pose the TeleOp must fall back to its default start pose, not crash or use stale data.
     */
    @Test
    public void nothingSavedAndClearBothGiveNull() {
        assertNull(PoseStore.load(1, 300.0));
        PoseStore.save(Pose2.ofDegrees(1, 2, 3), Alliance.BLUE, Side.AUDIENCE, false, 0);
        PoseStore.clear();
        assertNull(PoseStore.load(1, 300.0));
    }
}
