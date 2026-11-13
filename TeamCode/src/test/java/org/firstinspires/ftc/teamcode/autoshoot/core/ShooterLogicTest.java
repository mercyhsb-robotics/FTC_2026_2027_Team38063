package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

/**
 * Checks the shooter's decision logic, without a real flywheel: {@link ShotTable} (which flywheel speed to use for a
 * given distance), {@link Debounce} and {@link FlywheelReady} (is the flywheel 'at speed' yet?), and
 * {@link ShotCounter} (did a ball just leave?).
 *
 * <p>How it works: plain unit tests with no robot. The first groups call one small class with hand-made numbers. The
 * ShotCounter group uses a tiny simulated flywheel, {@code playVolley} below, that spins up, then gets knocked down by
 * each ball, stepped 10 ms at a time. The real FlywheelReady and ShotCounter classes watch that fake speed exactly as
 * they would watch the real encoder.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*ShooterLogicTest"} to run only
 * this class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>tps (ticks per second):</b> the flywheel's speed unit. The motor encoder counts 'ticks' as it turns, so
 *       ticks per second is its speed. The robot's manual launch speed is about 1200 tps.</li>
 *   <li><b>Range:</b> the horizontal distance in inches from the ball exit to the HIVE cell's aim point.</li>
 *   <li><b>Window:</b> the allowed band of speeds around the target in which the flywheel counts as 'at
 *       speed'.</li>
 *   <li><b>Debounce:</b> ignoring flicker. The output only changes after the input has stayed changed for a set
 *       time, so one noisy reading cannot flip it. Here switching on is slow and switching off is fast (a simple kind
 *       of hysteresis: the on and off rules differ on purpose).</li>
 *   <li><b>Dip:</b> the sudden drop in flywheel speed when a ball is squeezed through it. Counting dips is how the
 *       robot counts shots.</li>
 *   <li><b>Refractory time:</b> a short quiet period after a counted shot, during which no new shot is
 *       counted.</li>
 *   <li><b>Tolerance / EPS (epsilon):</b> how far two decimal numbers may differ and still count as equal, to allow
 *       for rounding. Here one billionth.</li>
 * </ul>
 */
public class ShooterLogicTest {
    /** Allowed rounding error when comparing two decimal numbers: one billionth. */
    private static final double EPS = 1e-9;

    // ---------------------------------------------------------------- ShotTable

    /**
     * Situation: a table with rows 30 in -> 1000 tps, 40 in -> 1200 tps and 50 in -> 1250 tps, asked for ranges below,
     *     on, between and above its rows.
     * Expected: exact speeds on the rows, a straight-line blend between them (35 in -> 1100, 45 in -> 1225), and the
     *     nearest row's speed outside the table (10 in -> 1000, 500 in -> 1250).
     * Why: the shooter looks its flywheel speed up in a measured table. Outside what was measured we must not
     *     extrapolate to a made-up speed.
     */
    @Test
    public void tableInterpolatesLinearlyAndClampsOutsideItsRows() {
        ShotTable t = new ShotTable(new double[][]{{30, 1000}, {40, 1200}, {50, 1250}});
        assertEquals(1000, t.velocityAt(10), EPS);
        assertEquals(1000, t.velocityAt(30), EPS);
        assertEquals(1100, t.velocityAt(35), EPS);
        assertEquals(1200, t.velocityAt(40), EPS);
        assertEquals(1225, t.velocityAt(45), EPS);
        assertEquals(1250, t.velocityAt(500), EPS);
    }

    /**
     * Situation: a table that covers 30 to 50 in.
     * Expected: inRange is true from 30 up to and including 50. With a 1.5 in margin the usable band shrinks to
     *     31.5-48.5: 32 is in, 30.5 and 49 are out. minRange and maxRange report 30 and 50.
     * Why: the planner keeps a margin so it never aims at the very edge of what was measured, and the controller
     *     refuses to fire when the range is outside the table.
     */
    @Test
    public void tableRangeChecksUseTheMarginAtBothEnds() {
        ShotTable t = new ShotTable(new double[][]{{30, 1000}, {50, 1250}});
        assertTrue(t.inRange(30));
        assertTrue(t.inRange(50));
        assertFalse(t.inRange(29.9));
        assertFalse(t.inRange(50.1));
        assertTrue(t.inRange(32, 1.5));
        assertFalse(t.inRange(30.5, 1.5));
        assertFalse(t.inRange(49, 1.5));
        assertEquals(30, t.minRange(), EPS);
        assertEquals(50, t.maxRange(), EPS);
    }

    /**
     * Situation: tables that make no sense: empty, a repeated range, ranges that go down, a speed of 0, a negative
     *     range, and a row with three numbers.
     * Expected: each one is rejected with an IllegalArgumentException.
     * Why: a typo in BioBuzzConstants.Shooter.SHOT_TABLE should fail loudly at INIT instead of sending the flywheel to
     *     a nonsense speed.
     */
    @Test
    public void tableRejectsBadRows() {
        expectBad(new double[][]{});
        expectBad(new double[][]{{30, 1000}, {30, 1100}});
        expectBad(new double[][]{{40, 1000}, {30, 1100}});
        expectBad(new double[][]{{30, 0}});
        expectBad(new double[][]{{-1, 100}});
        expectBad(new double[][]{{30, 1000, 5}});
    }

    /** Fails the test unless building a ShotTable from these rows throws IllegalArgumentException. */
    private static void expectBad(double[][] rows) {
        try {
            new ShotTable(rows);
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // fine
        }
    }

    /**
     * Situation: the real SHOT_TABLE from BioBuzzConstants.
     * Expected: it builds, has at least 2 rows, and its smallest range is below its largest.
     * Why: interpolation needs two points. This is a quick guard against someone leaving a one-row table when retuning.
     */
    @Test
    public void theShippedConstantsFormAValidTable() {
        ShotTable t = ShotTable.fromConstants();
        assertTrue(t.rows() >= 2);
        assertTrue(t.minRange() < t.maxRange());
    }

    // ---------------------------------------------------------------- Debounce / FlywheelReady

    /**
     * Situation: a Debounce that needs 0.10 s of 'true' to switch on and 0.03 s of 'false' to switch off, fed a noisy
     *     on/off signal at the times shown.
     * Expected: a glitch restarts the 'on' timer; it switches on only after an unbroken 0.10 s (exactly 0.10 s counts);
     *     once on, a single 'false' sample does not switch it off, but 0.03 s of 'false' in a row does.
     * Why: the shooter's 'at speed' flag must not flicker on one noisy speed reading, yet must drop quickly when a ball
     *     slows the flywheel.
     */
    @Test
    public void debounceNeedsAContinuousRunInEitherDirection() {
        Debounce d = new Debounce(0.10, 0.03);
        assertFalse(d.update(true, 0.00));
        assertFalse(d.update(true, 0.05));
        assertFalse(d.update(false, 0.06));   // glitch resets the rising timer
        assertFalse(d.update(true, 0.07));
        assertFalse(d.update(true, 0.16));    // only 0.09 s since 0.07
        assertTrue(d.update(true, 0.17));     // 0.10 s since 0.07: exactly enough
        assertTrue(d.update(false, 0.20));
        assertTrue(d.update(true, 0.21));     // a one-sample drop does not release it
        assertTrue(d.update(false, 0.30));
        assertFalse(d.update(false, 0.33));   // 0.03 s of false in a row: it releases
    }

    /**
     * Situation: a FlywheelReady with a window of 5% below and 5% above the target plus 20 tps of slack, 0.10 s to rise
     *     and 0.02 s to fall, a target of 1000 tps, updated every 0.02 s.
     * Expected: the window is 930 to 1070 tps; speeds that only pass through the window during spin-up are not 'ready';
     *     staying inside for 0.10 s is; a drop to 800 tps un-readies it after about 0.02 s.
     * Why: the feeder may only run while the flywheel is truly at speed, and must stop quickly after a ball slows it.
     *     The 20 tps slack covers encoder rounding noise.
     */
    @Test
    public void flywheelReadyUsesAWindowAndRisesSlowlyButFallsFast() {
        FlywheelReady r = new FlywheelReady(0.05, 0.05, 20, 0.10, 0.02);
        // Window: 1000 x (1 - 0.05) - 20 = 930 tps at the bottom and 1000 x (1 + 0.05) + 20 = 1070 tps at the top.
        double target = 1000;
        assertEquals(930, r.lowerBound(target), EPS);
        assertEquals(1070, r.upperBound(target), EPS);
        double t = 0;
        // Passing through the window during spin-up is not ready.
        assertFalse(r.update(t += 0.02, true, target, 900));
        assertFalse(r.update(t += 0.02, true, target, 940));
        assertFalse(r.update(t += 0.02, true, target, 990));
        // Staying in the window long enough is.
        boolean ready = false;
        for (int i = 0; i < 10; i++) {
            ready = r.update(t += 0.02, true, target, 1000);
        }
        assertTrue(ready);
        // A dip below the window drops it quickly.
        r.update(t += 0.02, true, target, 800);
        assertFalse(r.update(t += 0.02, true, target, 800));
        // Not running means never ready.
        assertFalse(r.update(t += 0.02, false, target, 1000));
    }

    // ---------------------------------------------------------------- ShotCounter

    /**
     * Plays a four-ball volley through the real ready/counter pair and returns what the counter saw.
     *
     * <p>A tiny scripted simulation of a flywheel: each 10 ms step the fake flywheel speed {@code v} moves 8% of the
     * way to the 1200 tps target (spin-up). The first ball arrives at 1.5 s and the others every 0.45 s; a ball knocks
     * the speed down at once to {@code target x (1 - dip depth)}. Then the real FlywheelReady and ShotCounter are
     * updated with that speed, as the robot's loop does, for 600 steps (6 s). It is not 'closed-loop': the balls arrive
     * on schedule whatever the logic decides (the full closed-loop simulation is in AutoShootSequenceTest).
     *
     * <p>It simplifies: the dip is instant and its recovery is the same smooth curve as the spin-up, and there is no
     * motor or encoder noise unless a test asks for a small dip to stand in for it.
     *
     * @param dipDepthFrac one entry per ball: how deep its dip is as a fraction of the target (0.12 = 12% drop)
     * @param feeding whether the feeder is running; the counter ignores dips while it is off
     * @return the number of shots the counter counted
     */
    private int playVolley(double[] dipDepthFrac, boolean feeding) {
        double target = 1200;
        FlywheelReady ready = new FlywheelReady(0.06, 0.06, 20, 0.10, 0.03);
        ShotCounter counter = new ShotCounter(0.05, 20, 0.15);
        double dt = 0.01;
        double t = 0;
        double v = 0;
        // The counter starts disarmed; it arms only after the flywheel has been at speed (so spin-up is never counted).
        counter.start(t);
        int ball = 0;
        double dipUntil = -1;
        double nextBallAt = 1.5;
        for (int step = 0; step < 600; step++) {
            t += dt;
            // First-order spin-up toward the target.
            v += (target - v) * 0.08;
            if (ball < dipDepthFrac.length && t >= nextBallAt) {
                v = target * (1.0 - dipDepthFrac[ball]);
                // (dipUntil is set but never read: a leftover. The dip simply recovers through the spin-up line above.)
                dipUntil = t + 0.05;
                ball++;
                nextBallAt = t + 0.45;
            }
            boolean atSpeed = ready.update(t, true, target, v);
            counter.update(t, feeding, atSpeed, target, v);
        }
        return counter.count();
    }

    /**
     * Situation: four balls fed 0.45 s apart, each knocking the flywheel down by 12% (a realistic dip), feeder on.
     * Expected: the counter reports exactly 4 shots.
     * Why: the shot count ends the volley (rule G407: at most 4 POLLEN). Counting a ball twice, or missing one, would
     *     end it too early or too late.
     */
    @Test
    public void fourRealisticBallDipsAreCountedExactlyOnce() {
        assertEquals(4, playVolley(new double[]{0.12, 0.12, 0.12, 0.12}, true));
    }

    /**
     * Situation: (a) no balls at all, just the flywheel spinning up with the feeder on; (b) two tiny wiggles of 2% and
     *     3%.
     * Expected: 0 shots both times.
     * Why: spin-up and encoder noise change the speed but are not balls. A counted dip must be deeper than the dip
     *     threshold, 1200 x 0.05 + 20 = 80 tps below the target, so 24 and 36 tps wiggles do not count.
     */
    @Test
    public void spinUpAndNoiseAreNeverCounted() {
        assertEquals(0, playVolley(new double[]{}, true));
        assertEquals(0, playVolley(new double[]{0.02, 0.03}, true));       // inside the noise band
    }

    /**
     * Situation: two big 15% dips while the feeder is NOT running.
     * Expected: 0 shots.
     * Why: only a feeder pushing a ball through can launch one, so any other speed change is not a shot.
     */
    @Test
    public void dipsWhileTheFeederIsOffAreIgnored() {
        assertEquals(0, playVolley(new double[]{0.15, 0.15}, false));
    }

    /**
     * Situation: a counter with a 0.50 s refractory time, a target of 1000 tps (dip threshold 1000 x 0.95 - 20 =
     *     930 tps) and 'at speed' always true.
     * Expected: 900 tps at 0.2 s counts as shot 1; a dip to 850 tps at 0.30 s, inside the quiet period, is ignored; the
     *     counter re-arms once 0.50 s have passed (0.80 s) and a dip at 0.90 s is shot 2. The total is 2 and
     *     secondsSinceLastShot is 0 right after a shot.
     * Why: while the flywheel recovers its speed can wobble through the threshold more than once for the same ball. It
     *     must still count once.
     */
    @Test
    public void aSecondDipInsideTheRefractoryWindowIsNotCounted() {
        double target = 1000;
        ShotCounter c = new ShotCounter(0.05, 20, 0.50);
        c.start(0);
        assertFalse(c.update(0.1, true, true, target, 1000));     // arms
        assertTrue(c.update(0.2, true, true, target, 900));       // shot 1
        assertFalse(c.update(0.25, true, true, target, 1000));    // still refractory, not re-armed
        assertFalse(c.update(0.30, true, true, target, 850));     // dip while disarmed
        assertFalse(c.update(0.80, true, true, target, 1000));    // re-arms now (refractory over)
        assertTrue(c.update(0.90, true, true, target, 900));      // shot 2
        assertEquals(2, c.count());
        assertEquals(0.0, c.secondsSinceLastShot(0.90), EPS);
    }
}
