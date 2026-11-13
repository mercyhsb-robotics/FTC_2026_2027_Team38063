package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkPlanningNumbers}: the numbers the route estimate is worked out from must make sense. The feeder power is divided into the time
 * between two POLLEN (so it must be above 0), the chance that a POLLEN lands is a probability (more than 0, at most 1) and a tip cannot settle in negative time.
 *
 * <p>How it works: no robot. The check takes the numbers as parameters.
 */
public class ConstantsCheckPlanningTest {

    private static ConstantsCheck.Report check(double feederPower, double shotSuccess, double tipSettle) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkPlanningNumbers(r, feederPower, shotSuccess, tipSettle);
        return r;
    }

    /** Situation: the shipped numbers, and the edges that are allowed (a shot chance of exactly 1, a settle of 0). Expected: no error. */
    @Test
    public void theShippedPlanningNumbersPassTheInitCheck() {
        assertTrue(check(BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER, BioBuzzConstants.Auto.PLAN_SHOT_SUCCESS,
                BioBuzzConstants.Auto.TIP_SETTLE_PLAN_SEC).errors.isEmpty());
        assertTrue(check(1.0, 1.0, 0.0).errors.isEmpty());
        for (String e : ConstantsCheck.run().errors) {
            assertTrue(e, !e.contains("PLAN_SHOT_SUCCESS") && !e.contains("TIP_SETTLE_PLAN_SEC") && !e.contains("FEEDER_FORWARD_POWER ("));
        }
    }

    /** Situation: a feeder power of 0, a shot chance of 0, 1.2 or 0 again, and a negative settle. Expected: one ERROR each, naming the constant. */
    @Test
    public void numbersThatCannotBeRightAreErrors() {
        ConstantsCheck.Report noFeeder = check(0.0, 0.6, 5.5);
        assertEquals(noFeeder.errors.toString(), 1, noFeeder.errors.size());
        assertTrue(noFeeder.errors.get(0), noFeeder.errors.get(0).contains("FEEDER_FORWARD_POWER"));
        for (double chance : new double[] {0.0, 1.2, -0.1, Double.NaN}) {
            ConstantsCheck.Report bad = check(0.3, chance, 5.5);
            assertEquals(chance + " " + bad.errors, 1, bad.errors.size());
            assertTrue(bad.errors.get(0), bad.errors.get(0).contains("PLAN_SHOT_SUCCESS"));
        }
        ConstantsCheck.Report negative = check(0.3, 0.6, -1.0);
        assertEquals(negative.errors.toString(), 1, negative.errors.size());
        assertTrue(negative.errors.get(0), negative.errors.get(0).contains("TIP_SETTLE_PLAN_SEC"));
    }
}
