package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkTipStop}: the numbers that decide whether the spare POLLEN of a volley is kept or spent (the ball flight, the feeder's stop
 * latency and the time between POLLEN) must make sense. The shipped ones pass; a flight or a time between POLLEN that is not positive, or a negative
 * latency, is an error (a latency of 0 is allowed: a feeder that stops at once).
 *
 * <p>How it works: no robot. The check takes the numbers as parameters.
 */
public class ConstantsCheckTipStopTest {

    private static ConstantsCheck.Report check(double flight, double latency, double perShot) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkTipStop(r, flight, latency, perShot);
        return r;
    }

    /** Situation: the shipped numbers, and a latency of exactly 0. Expected: no error, in the single check and in the whole INIT check. */
    @Test
    public void theShippedNumbersAndAFeederThatStopsAtOnceAreAccepted() {
        assertTrue(check(BioBuzzConstants.Auto.BALL_FLIGHT_SEC, BioBuzzConstants.Auto.TIP_STOP_LATENCY_SEC, BioBuzzConstants.Auto.TIME_PER_SHOT_SEC)
                .errors.isEmpty());
        assertTrue(check(BioBuzzConstants.Auto.BALL_FLIGHT_SEC, 0.0, BioBuzzConstants.Auto.TIME_PER_SHOT_SEC).errors.isEmpty());
        for (String e : ConstantsCheck.run().errors) {
            assertFalse(e, e.contains("TIP_STOP_LATENCY_SEC") || e.contains("BALL_FLIGHT_SEC"));
        }
    }

    /** Situation: a flight of 0, a time between POLLEN of 0 or a negative latency. Expected: an ERROR that names the constants. */
    @Test
    public void numbersThatCannotBeRightAreErrors() {
        ConstantsCheck.Report noFlight = check(0.0, 0.85, 0.9);
        assertEquals(noFlight.errors.toString(), 1, noFlight.errors.size());
        assertTrue(noFlight.errors.get(0), noFlight.errors.get(0).contains("BALL_FLIGHT_SEC"));
        ConstantsCheck.Report noPace = check(0.5, 0.85, 0.0);
        assertEquals(noPace.errors.toString(), 1, noPace.errors.size());
        assertTrue(noPace.errors.get(0), noPace.errors.get(0).contains("TIME_PER_SHOT_SEC"));
        ConstantsCheck.Report negative = check(0.5, -0.1, 0.9);
        assertEquals(negative.errors.toString(), 1, negative.errors.size());
        assertTrue(negative.errors.get(0), negative.errors.get(0).contains("TIP_STOP_LATENCY_SEC"));
    }
}
