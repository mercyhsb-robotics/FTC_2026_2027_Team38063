package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkStartSearch}: the grid of the search for a legal start (a place every so many inches along a wall, a heading every so
 * many degrees). The shipped numbers pass; a step of 0 or less is an error (the search would never move on), and a heading step that does not divide 90
 * evenly is a warning (the search would skip the headings that point the robot straight at a wall).
 */
public class ConstantsCheckStartSearchTest {

    private static ConstantsCheck.Report check(double stepIn, double headingStepDeg) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkStartSearch(r, stepIn, headingStepDeg);
        return r;
    }

    /** Situation: the shipped grid, and a few other sensible ones (every degree, every 15 degrees, a coarse 1 in). Expected: no error and no warning. */
    @Test
    public void theShippedGridAndOtherSensibleOnesAreAccepted() {
        ConstantsCheck.Report shipped = check(BioBuzzConstants.Auto.START_SEARCH_STEP_IN, BioBuzzConstants.Auto.START_SEARCH_HEADING_STEP_DEG);
        assertTrue(shipped.errors.toString(), shipped.errors.isEmpty());
        assertTrue(shipped.warnings.toString(), shipped.warnings.isEmpty());
        for (double deg : new double[] {1.0, 2.0, 3.0, 5.0, 6.0, 9.0, 10.0, 15.0, 90.0}) {
            ConstantsCheck.Report r = check(1.0, deg);
            assertTrue(deg + ": " + r.errors + r.warnings, r.errors.isEmpty() && r.warnings.isEmpty());
        }
    }

    /** Situation: a step of 0 inches, a negative one, a heading step of 0 and one above 90 degrees. Expected: an ERROR that names the constant. */
    @Test
    public void aStepThatCannotMoveTheSearchOnIsAnError() {
        ConstantsCheck.Report noStep = check(0.0, 5.0);
        assertEquals(noStep.errors.toString(), 1, noStep.errors.size());
        assertTrue(noStep.errors.get(0), noStep.errors.get(0).contains("START_SEARCH_STEP_IN"));
        assertEquals(1, check(-0.25, 5.0).errors.size());
        ConstantsCheck.Report noTurn = check(0.25, 0.0);
        assertEquals(noTurn.errors.toString(), 1, noTurn.errors.size());
        assertTrue(noTurn.errors.get(0), noTurn.errors.get(0).contains("START_SEARCH_HEADING_STEP_DEG"));
        assertEquals(1, check(0.25, 120.0).errors.size());
    }

    /** Situation: a heading step of 7 degrees, which skips 0, 90, 180 and -90. Expected: a warning (not an error) that names the constant. */
    @Test
    public void aHeadingStepThatSkipsTheStraightHeadingsIsAWarning() {
        ConstantsCheck.Report r = check(0.25, 7.0);
        assertTrue(r.errors.toString(), r.errors.isEmpty());
        assertEquals(r.warnings.toString(), 1, r.warnings.size());
        assertTrue(r.warnings.get(0), r.warnings.get(0).contains("START_SEARCH_HEADING_STEP_DEG"));
        assertFalse("the whole INIT check of the shipped constants stays clean", ConstantsCheck.run().warnings.toString().contains("START_SEARCH"));
    }
}
