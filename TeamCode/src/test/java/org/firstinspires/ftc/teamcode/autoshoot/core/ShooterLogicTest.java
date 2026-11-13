package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

public class ShooterLogicTest {
    private static final double EPS = 1e-9;

    // ---------------------------------------------------------------- ShotTable

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

    @Test
    public void tableRejectsBadRows() {
        expectBad(new double[][]{});
        expectBad(new double[][]{{30, 1000}, {30, 1100}});
        expectBad(new double[][]{{40, 1000}, {30, 1100}});
        expectBad(new double[][]{{30, 0}});
        expectBad(new double[][]{{-1, 100}});
        expectBad(new double[][]{{30, 1000, 5}});
    }

    private static void expectBad(double[][] rows) {
        try {
            new ShotTable(rows);
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // fine
        }
    }

    @Test
    public void theShippedConstantsFormAValidTable() {
        ShotTable t = ShotTable.fromConstants();
        assertTrue(t.rows() >= 2);
        assertTrue(t.minRange() < t.maxRange());
    }

    // ---------------------------------------------------------------- Debounce / FlywheelReady

    @Test
    public void debounceNeedsAContinuousRunInEitherDirection() {
        Debounce d = new Debounce(0.10, 0.03);
        assertFalse(d.update(true, 0.00));
        assertFalse(d.update(true, 0.05));
        assertFalse(d.update(false, 0.06));   // glitch resets the rising timer
        assertFalse(d.update(true, 0.07));
        assertFalse(d.update(true, 0.16));    // only 0.09 s since 0.07
        assertTrue(d.update(true, 0.17));
        assertTrue(d.update(false, 0.20));
        assertTrue(d.update(true, 0.21));     // a one-sample drop does not release it
        assertTrue(d.update(false, 0.30));
        assertFalse(d.update(false, 0.33));
    }

    @Test
    public void flywheelReadyUsesAWindowAndRisesSlowlyButFallsFast() {
        FlywheelReady r = new FlywheelReady(0.05, 0.05, 20, 0.10, 0.02);
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

    /** Plays a four-ball volley through the real ready/counter pair and returns what the counter saw. */
    private int playVolley(double[] dipDepthFrac, boolean feeding) {
        double target = 1200;
        FlywheelReady ready = new FlywheelReady(0.06, 0.06, 20, 0.10, 0.03);
        ShotCounter counter = new ShotCounter(0.05, 20, 0.15);
        double dt = 0.01;
        double t = 0;
        double v = 0;
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
                dipUntil = t + 0.05;
                ball++;
                nextBallAt = t + 0.45;
            }
            boolean atSpeed = ready.update(t, true, target, v);
            counter.update(t, feeding, atSpeed, target, v);
        }
        return counter.count();
    }

    @Test
    public void fourRealisticBallDipsAreCountedExactlyOnce() {
        assertEquals(4, playVolley(new double[]{0.12, 0.12, 0.12, 0.12}, true));
    }

    @Test
    public void spinUpAndNoiseAreNeverCounted() {
        assertEquals(0, playVolley(new double[]{}, true));
        assertEquals(0, playVolley(new double[]{0.02, 0.03}, true));       // inside the noise band
    }

    @Test
    public void dipsWhileTheFeederIsOffAreIgnored() {
        assertEquals(0, playVolley(new double[]{0.15, 0.15}, false));
    }

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
