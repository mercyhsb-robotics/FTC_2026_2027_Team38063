package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

import org.junit.After;
import org.junit.Test;

public class PoseStoreTest {
    private static final long SEC = 1_000_000_000L;

    @After
    public void clean() {
        PoseStore.clear();
    }

    @Test
    public void aFreshSnapshotComesBackAndAnOldOneDoesNot() {
        Pose2 p = Pose2.ofDegrees(60, 20, -90);
        PoseStore.save(p, Alliance.RED, Side.REAR, true, 100 * SEC);
        PoseStore.Snapshot s = PoseStore.load(130 * SEC, 300.0);
        assertNotNull(s);
        assertSame(p, s.pose);
        assertEquals(Alliance.RED, s.alliance);
        assertEquals(Side.REAR, s.hiveUpSide);
        assertEquals(true, s.hiveConfirmed);
        assertNull("older than the limit", PoseStore.load(500 * SEC, 300.0));
        assertNull("a clock that went backwards is never trusted", PoseStore.load(50 * SEC, 300.0));
    }

    @Test
    public void nothingSavedAndClearBothGiveNull() {
        assertNull(PoseStore.load(1, 300.0));
        PoseStore.save(Pose2.ofDegrees(1, 2, 3), Alliance.BLUE, Side.AUDIENCE, false, 0);
        PoseStore.clear();
        assertNull(PoseStore.load(1, 300.0));
    }
}
