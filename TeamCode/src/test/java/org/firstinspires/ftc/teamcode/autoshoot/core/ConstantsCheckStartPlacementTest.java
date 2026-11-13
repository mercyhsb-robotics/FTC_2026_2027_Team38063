package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkStartPlacementMargin}: how far off a hand-placed start may be (inches along the wall, degrees of heading). The shipped numbers
 * pass; a negative margin is an error; a margin so big that nothing a person could place would pass is a warning; a margin of exactly 0 is allowed.
 */
public class ConstantsCheckStartPlacementTest {

    private static ConstantsCheck.Report check(double marginIn, double marginDeg) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPlacementMargin(r, marginIn, marginDeg);
        return r;
    }

    /** Situation: the shipped margin (1 in, 3 degrees) and a margin of 0. Expected: nothing to say. */
    @Test
    public void theShippedMarginAndNoMarginAreAccepted() {
        ConstantsCheck.Report shipped = check(BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN, BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_DEG);
        assertTrue(shipped.errors.toString() + shipped.warnings, shipped.errors.isEmpty() && shipped.warnings.isEmpty());
        ConstantsCheck.Report none = check(0.0, 0.0);
        assertTrue(none.errors.isEmpty() && none.warnings.isEmpty());
    }

    /** Situation: a negative margin in either number, or not a number. Expected: an ERROR that names the constants. */
    @Test
    public void aNegativeMarginIsAnError() {
        for (double[] m : new double[][] {{-1.0, 3.0}, {1.0, -3.0}, {Double.NaN, 3.0}}) {
            ConstantsCheck.Report r = check(m[0], m[1]);
            assertEquals(r.errors.toString(), 1, r.errors.size());
            assertTrue(r.errors.get(0), r.errors.get(0).contains("START_PLACEMENT_MARGIN_IN") && r.errors.get(0).contains("START_PLACEMENT_MARGIN_DEG"));
        }
    }

    /** Situation: a margin of 5 in, and one of 20 degrees. Expected: a warning (not an error): almost nothing would pass it. */
    @Test
    public void aHugeMarginIsAWarning() {
        for (double[] m : new double[][] {{5.0, 3.0}, {1.0, 20.0}}) {
            ConstantsCheck.Report r = check(m[0], m[1]);
            assertTrue(r.errors.toString(), r.errors.isEmpty());
            assertEquals(r.warnings.toString(), 1, r.warnings.size());
        }
    }

    private static ConstantsCheck.Report words(double toleranceDeg) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkPlacementWords(r, toleranceDeg);
        return r;
    }

    /** Situation: the shipped tolerance of the placement words (1 degree), none (0) and 10 degrees. Expected: accepted. */
    @Test
    public void theShippedToleranceOfThePlacementWordsIsAccepted() {
        assertTrue(words(BioBuzzConstants.Auto.PLACEMENT_ALIGNED_TOLERANCE_DEG).errors.isEmpty());
        assertTrue(words(0.0).errors.isEmpty());
        assertTrue(words(10.0).errors.isEmpty());
    }

    /** Situation: a negative tolerance, 45 degrees, and more. Expected: an ERROR that names the constant (a robot turned 40 degrees would be called straight). */
    @Test
    public void aToleranceThatCallsEveryHeadingStraightIsAnError() {
        for (double bad : new double[] {-1.0, 45.0, 60.0, Double.NaN}) {
            ConstantsCheck.Report r = words(bad);
            assertEquals(bad + ": " + r.errors, 1, r.errors.size());
            assertTrue(r.errors.get(0), r.errors.get(0).contains("PLACEMENT_ALIGNED_TOLERANCE_DEG"));
        }
    }
}
