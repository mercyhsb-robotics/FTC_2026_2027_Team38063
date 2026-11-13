package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.junit.Test;

/**
 * The starting values of the route timing model (from the benchmark research) and the time per shot that is WORKED OUT from the feeder power.
 *
 * <p>How it works: no robot; the constants and the model are read directly. The first test pins the starting values on purpose: when the
 * robot has been measured and a constant changes, change its line here too (and its marker and PLACEHOLDERS entry in BioBuzzConstants).
 */
public class TimingStartingValuesTest {
    private static final double EPS = 1e-9;

    /** Situation: the values of BioBuzzConstants.Auto after the research. Expected: the researched starting values, all still unmeasured (ADD) placeholders. */
    @Test
    public void theStartingValuesAreTheResearchedOnes() {
        assertEquals(48.0, BioBuzzConstants.Auto.CRUISE_SPEED_IN_PER_SEC, 0.0);
        assertEquals(110.0, BioBuzzConstants.Auto.ACCEL_IN_PER_SEC2, 0.0);
        assertEquals(150.0, BioBuzzConstants.Auto.TURN_RATE_DEG_PER_SEC, 0.0);
        assertEquals(0.4, BioBuzzConstants.Auto.SETTLE_SEC, 0.0);
        assertEquals(3.0, BioBuzzConstants.Auto.INTAKE_DWELL_GARDEN_SEC, 0.0);
        assertEquals("the floor dwell does not change", 2.0, BioBuzzConstants.Auto.INTAKE_DWELL_FLOOR_SEC, 0.0);
        assertEquals(1.5, BioBuzzConstants.Auto.AIM_SPINUP_SEC, 0.0);
        assertEquals(0.6, BioBuzzConstants.Auto.PLAN_SHOT_SUCCESS, 0.0);
        assertEquals(5.5, BioBuzzConstants.Auto.TIP_SETTLE_PLAN_SEC, 0.0);
        for (String name : new String[] {"Auto.CRUISE_SPEED_IN_PER_SEC", "Auto.ACCEL_IN_PER_SEC2", "Auto.TURN_RATE_DEG_PER_SEC", "Auto.SETTLE_SEC",
                "Auto.INTAKE_DWELL_FLOOR_SEC", "Auto.INTAKE_DWELL_GARDEN_SEC", "Auto.AIM_SPINUP_SEC", "Auto.TIME_PER_SHOT_SEC"}) {
            assertTrue(name + " is an unmeasured placeholder", Arrays.asList(BioBuzzConstants.PLACEHOLDERS).contains(name));
        }
        for (String name : new String[] {"Auto.PLAN_SHOT_SUCCESS", "Auto.TIP_SETTLE_PLAN_SEC"}) {
            assertTrue(name + " is a TUNE value, not a placeholder", !Arrays.asList(BioBuzzConstants.PLACEHOLDERS).contains(name));
        }
    }

    /**
     * Situation: a 100 in leg and a 90 degree turn at the starting values.
     * Expected: 100 / 48 + 48 / 110 = 2.52 s of driving (the research quotes about 2.8 s with the old 0.3 s settle: 2.52 + 0.3), and 0.6 s for the turn.
     */
    @Test
    public void aHundredInchLegAndANinetyDegreeTurnCostWhatTheResearchSaid() {
        TimingModel m = new TimingModel();
        assertEquals(100.0 / 48.0 + 48.0 / 110.0, m.translationSeconds(100.0, 1.0), EPS);
        assertEquals(2.52, m.translationSeconds(100.0, 1.0), 0.005);
        assertEquals(90.0 / 150.0, m.turnSeconds(Math.toRadians(90.0)), EPS);
    }

    /** Situation: the shipped feeder power of 0.3. Expected: 0.30 / 0.3 = 1.0 s between two POLLEN, the number the model and the shipped constant use. */
    @Test
    public void theTimePerShotAtTheShippedFeederPowerIsOneSecond() {
        assertEquals(0.3, BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER, 0.0);
        assertEquals(1.0, BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, EPS);
        assertEquals(BioBuzzConstants.Auto.timePerShotAt(BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER), BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, 0.0);
        assertEquals(BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, new TimingModel().timePerShotSec(), 0.0);
    }

    /** Situation: other feeder powers. Expected: 0.5 s at 0.6, 0.375 s at 0.8, and never under the 0.35 s floor, which is what full power gives (0.30 would be too fast). */
    @Test
    public void aFasterFeederShortensItDownToTheFloor() {
        assertEquals(0.5, BioBuzzConstants.Auto.timePerShotAt(0.6), EPS);
        assertEquals(0.375, BioBuzzConstants.Auto.timePerShotAt(0.8), EPS);
        assertEquals(0.35, BioBuzzConstants.Auto.timePerShotAt(1.0), EPS);
        assertEquals(0.35, BioBuzzConstants.Auto.TIME_PER_SHOT_MIN_SEC, 0.0);
        assertEquals(0.30, BioBuzzConstants.Auto.FEED_SEC_PER_POLLEN_AT_FULL_POWER, 0.0);
        double last = Double.POSITIVE_INFINITY;
        for (double power = 0.05; power <= 1.0001; power += 0.05) {
            double t = BioBuzzConstants.Auto.timePerShotAt(power);
            assertTrue("never under the floor at power " + power, t >= BioBuzzConstants.Auto.TIME_PER_SHOT_MIN_SEC);
            assertTrue("a stronger feeder is never slower at power " + power, t <= last);
            last = t;
        }
    }

    /** Situation: a feeder that is stopped or run backward. Expected: no POLLEN ever leaves, so the time is infinite instead of a division by zero. */
    @Test
    public void aFeederThatDoesNotRunForwardNeverFeeds() {
        assertEquals(Double.POSITIVE_INFINITY, BioBuzzConstants.Auto.timePerShotAt(0.0), 0.0);
        assertEquals(Double.POSITIVE_INFINITY, BioBuzzConstants.Auto.timePerShotAt(-0.3), 0.0);
    }

    /** Situation: a volley of 4 with the model. Expected: the aim and spin-up plus four feeds of the derived time: 1.5 + 4 x 1.0 = 5.5 s. */
    @Test
    public void aVolleyOfFourCostsTheAimPlusFourDerivedFeeds() {
        Step.Shoot four = (Step.Shoot) AutoRoute.builder("a").start(0, 0, 0).shootUp(4).build().step(1);
        assertEquals(BioBuzzConstants.Auto.AIM_SPINUP_SEC + 4.0 * (BioBuzzConstants.Auto.FEED_SEC_PER_POLLEN_AT_FULL_POWER
                / BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER), new TimingModel().shootSeconds(four), EPS);
        assertEquals(5.5, new TimingModel().shootSeconds(four), EPS);
    }
}
