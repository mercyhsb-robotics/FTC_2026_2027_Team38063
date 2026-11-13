package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkSpillZone}: the G409 spill zone of a tipped HIVE cell must reach farther out than the cell itself hangs, because what
 * falls out lands beyond the cell's own mouth. The first version of the zone stopped at the cell (plus a few inches) and a robot standing where the
 * balls land was reported clean; this check refuses a reach that would do that again.
 *
 * <p>How it works: no robot. The check takes the numbers as parameters, so one test passes the shipped ones and the others pass numbers that break the
 * rule.
 */
public class ConstantsCheckSpillZoneTest {

    private static ConstantsCheck.Report check(double reach, double margin, double clear) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkSpillZone(r, reach, BioBuzzConstants.Auto.DOWN_CELL_PLAN_OUT_FAR_IN, margin, clear);
        return r;
    }

    /** Situation: the shipped numbers (40 in, 6 in, 3 s). Expected: no error, in the single check and in the whole INIT check. */
    @Test
    public void theShippedNumbersPass() {
        assertTrue(check(BioBuzzConstants.Auto.TIP_SPILL_REACH_IN, BioBuzzConstants.Auto.TIP_SPILL_MARGIN_IN,
                BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC).errors.isEmpty());
        for (String e : ConstantsCheck.run().errors) {
            assertFalse(e, e.contains("TIP_SPILL"));
        }
    }

    /**
     * Situation: a reach that stops at, or short of, the far end of the tipped cell (25.13 in out).
     * Expected: an ERROR that names TIP_SPILL_REACH_IN. This is the case that fails without the check: it is the first version's zone, which ended 26 in
     * out and guarded nothing past the cell.
     */
    @Test
    public void aReachThatStopsAtTheCellItselfIsAnError() {
        for (double reach : new double[] {BioBuzzConstants.Auto.DOWN_CELL_PLAN_OUT_FAR_IN, 20.0, 0.0}) {
            ConstantsCheck.Report r = check(reach, BioBuzzConstants.Auto.TIP_SPILL_MARGIN_IN, BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC);
            assertEquals(r.errors.toString(), 1, r.errors.size());
            assertTrue(r.errors.get(0), r.errors.get(0).contains("TIP_SPILL_REACH_IN"));
            assertTrue(r.errors.get(0), r.errors.get(0).contains("lands beyond"));
        }
    }

    /** Situation: a negative margin or a negative clear time. Expected: an ERROR that names both constants. */
    @Test
    public void aNegativeMarginOrTimeIsAnError() {
        for (double[] bad : new double[][] {{-1.0, 3.0}, {6.0, -0.1}}) {
            ConstantsCheck.Report r = check(BioBuzzConstants.Auto.TIP_SPILL_REACH_IN, bad[0], bad[1]);
            assertEquals(r.errors.toString(), 1, r.errors.size());
            assertTrue(r.errors.get(0), r.errors.get(0).contains("TIP_SPILL_MARGIN_IN"));
            assertTrue(r.errors.get(0), r.errors.get(0).contains("TIP_SPILL_CLEAR_SEC"));
        }
    }
}
