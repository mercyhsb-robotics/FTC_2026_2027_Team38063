package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkReapproachHeading}: the heading error at which a turned robot is sent back to PLAN
 * ({@code AutoShoot.REAPPROACH_HEADING_DEG}) must be larger than the heading error APPROACH hands over with
 * ({@code APPROACH_HANDOFF_DEG}). Otherwise the controller loops between PLAN and ALIGN.
 *
 * <p>How it works: no robot. The check takes the two numbers as parameters, so one test passes the shipped ones and the others pass
 * numbers that break the rule.
 */
public class ConstantsCheckReapproachTest {

    private static ConstantsCheck.Report check(double reapproachDeg, double handoffDeg) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkReapproachHeading(r, reapproachDeg, handoffDeg);
        return r;
    }

    /** Situation: the shipped numbers (45 and 25 degrees). Expected: no error, in the single check and in the whole INIT check. */
    @Test
    public void theShippedNumbersPass() {
        assertTrue(check(BioBuzzConstants.AutoShoot.REAPPROACH_HEADING_DEG, BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_DEG).errors.isEmpty());
        for (String e : ConstantsCheck.run().errors) {
            assertFalse(e, e.contains("REAPPROACH_HEADING_DEG"));
        }
    }

    /**
     * Situation: the re-plan heading is equal to, or below, the hand-over heading.
     * Expected: an ERROR that names both constants and the loop. This is the case that would fail without the check: a robot handed over
     * at 25 degrees would be re-planned at once.
     */
    @Test
    public void aReplanHeadingThatIsNotLargerThanTheHandoverHeadingIsAnError() {
        for (double[] bad : new double[][] {{25.0, 25.0}, {20.0, 25.0}, {0.0, 25.0}}) {
            ConstantsCheck.Report r = check(bad[0], bad[1]);
            assertEquals(r.errors.toString(), 1, r.errors.size());
            assertTrue(r.errors.get(0), r.errors.get(0).contains("REAPPROACH_HEADING_DEG"));
            assertTrue(r.errors.get(0), r.errors.get(0).contains("APPROACH_HANDOFF_DEG"));
            assertTrue(r.errors.get(0), r.errors.get(0).contains("loop"));
        }
    }
}
