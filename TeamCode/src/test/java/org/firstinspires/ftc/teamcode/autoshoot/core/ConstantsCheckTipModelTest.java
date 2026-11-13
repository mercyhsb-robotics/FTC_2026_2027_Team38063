package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.routes.TipModel;

/**
 * {@link ConstantsCheck#checkTipModel} refuses tip constants that disagree with FIRST's own calibration of every HIVE cell: an empty cell
 * tips with the 8th POLLEN (not with 7) and a cell holding the 3 staged NECTAR tips with the 3rd POLLEN (not with 2). The shipped numbers
 * must pass it, and the earlier weighted rule of the team (a NECTAR is 1/5 of a tip, a POLLEN 1/8) must fail it, because that rule said
 * that 3 NECTAR + 3 POLLEN do not tip.
 *
 * <p>How it works: no robot. The check takes the numbers as parameters, so each test starts from the shipped numbers and changes one of
 * them; the last test shows that the check agrees with the route planner's own {@link TipModel} for a whole grid of other numbers.
 */
public class ConstantsCheckTipModelTest {
    private static final int STAGED = BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL;
    private static final int NUM = BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR;
    private static final int DEN = BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR;
    private static final int THRESHOLD = BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD;
    private static final int MARGIN = BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN;
    private static final int PRELOADS = BioBuzzConstants.Auto.PRELOAD_POLLEN;

    private static ConstantsCheck.Report check(int staged, int numerator, int denominator, int threshold, int margin) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkTipModel(r, staged, numerator, denominator, threshold, margin, PRELOADS);
        return r;
    }

    /** Situation: the shipped numbers (NECTAR = 5/3 POLLEN, a cell tips at 8, a spare of 1). Expected: no error and no warning. */
    @Test
    public void theShippedTipConstantsReproduceFirstsTable() {
        ConstantsCheck.Report r = check(STAGED, NUM, DEN, THRESHOLD, MARGIN);
        assertTrue(r.errors.toString(), r.errors.isEmpty());
        assertTrue(r.warnings.toString(), r.warnings.isEmpty());
        // and the full check that the robot runs at INIT has no finding about them either
        for (String e : ConstantsCheck.run().errors) {
            assertFalse(e, e.contains("tip constants"));
        }
    }

    /**
     * Situation: the old weighted rule, 1/5 of a tip per NECTAR and 1/8 per POLLEN, written as a weight of 8/5 POLLEN per NECTAR.
     * Expected: an ERROR that says 3 NECTAR + 3 POLLEN do not tip, against FIRST's table. This is the case that fails without the change.
     */
    @Test
    public void theOldWeightedRuleFailsBecauseItSaidThreeNectarPlusThreePollenDoNotTip() {
        ConstantsCheck.Report r = check(STAGED, 8, 5, THRESHOLD, MARGIN);
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("3 NECTAR + 3 POLLEN do not tip"));
        assertTrue(r.errors.get(0), r.errors.get(0).contains("the first tip needs only 3 POLLEN"));
    }

    /** Situation: NECTAR counted as 2 POLLEN (2/1), too heavy. Expected: 3 NECTAR + 2 POLLEN would tip, which FIRST says they do not. */
    @Test
    public void aNectarWorthTwoPollenIsTooHeavy() {
        ConstantsCheck.Report r = check(STAGED, 2, 1, THRESHOLD, MARGIN);
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("3 NECTAR + 2 POLLEN tip a HIVE cell"));
    }

    /** Situation: a cell that tips at 7 POLLEN-equivalents. Expected: 7 POLLEN would tip an empty cell, which FIRST says they do not. */
    @Test
    public void aThresholdOfSevenSaysSevenPollenTipAnEmptyCell() {
        ConstantsCheck.Report r = check(STAGED, NUM, DEN, 7, MARGIN);
        assertTrue(r.errors.toString(), r.errors.size() >= 1);
        boolean emptyCell = false;
        for (String e : r.errors) {
            emptyCell |= e.contains("7 POLLEN tip an empty HIVE cell");
        }
        assertTrue(r.errors.toString(), emptyCell);
    }

    /** Situation: a cell that needs 9. Expected: 8 POLLEN would not tip an empty cell, which FIRST says they do. */
    @Test
    public void aThresholdOfNineSaysEightPollenDoNotTipAnEmptyCell() {
        ConstantsCheck.Report r = check(STAGED, NUM, DEN, 9, MARGIN);
        boolean emptyCell = false;
        for (String e : r.errors) {
            emptyCell |= e.contains("8 POLLEN do not tip an empty HIVE cell");
        }
        assertTrue(r.errors.toString(), emptyCell);
    }

    /** Situation: a weight of zero or a zero denominator. Expected: an ERROR and no division by zero or crash. */
    @Test
    public void nonsenseNumbersAreAnErrorNotACrash() {
        assertEquals(1, check(STAGED, 0, DEN, THRESHOLD, MARGIN).errors.size());
        assertEquals(1, check(STAGED, NUM, 0, THRESHOLD, MARGIN).errors.size());
        assertEquals(1, check(STAGED, NUM, DEN, 0, MARGIN).errors.size());
        assertEquals(1, check(STAGED, -5, DEN, THRESHOLD, MARGIN).errors.size());
    }

    /** Situation: a different number of staged NECTAR. Expected: an ERROR, because FIRST's table is written for the 3 of Fig 10-2. */
    @Test
    public void theStagedNectarMustBeThree() {
        ConstantsCheck.Report r = check(4, NUM, DEN, THRESHOLD, MARGIN);
        boolean named = false;
        for (String e : r.errors) {
            named |= e.contains("Auto.NECTAR_STAGED_IN_UP_CELL");
        }
        assertTrue(r.errors.toString(), named);
    }

    /** Situation: a negative planning margin is an ERROR; one that the 4 pre-loads cannot give for the first tip is a WARNING. */
    @Test
    public void theMarginMustBeSomethingARouteCanReach() {
        assertEquals(1, check(STAGED, NUM, DEN, THRESHOLD, -1).errors.size());
        ConstantsCheck.Report zero = check(STAGED, NUM, DEN, THRESHOLD, 0);
        assertTrue(zero.errors.isEmpty() && zero.warnings.isEmpty());
        ConstantsCheck.Report one = check(STAGED, NUM, DEN, THRESHOLD, 1);
        assertTrue(one.errors.isEmpty() && one.warnings.isEmpty());
        ConstantsCheck.Report two = check(STAGED, NUM, DEN, THRESHOLD, 2);
        assertTrue(two.errors.isEmpty());
        assertEquals("3 + 2 = 5 POLLEN into the first cell is more than the 4 pre-loads", 1, two.warnings.size());
        assertTrue(two.warnings.get(0), two.warnings.get(0).contains("Auto.TIP_PLANNING_MARGIN_POLLEN"));
    }

    /**
     * Situation: every weight num/den (1/1 to 9/5) with every threshold from 5 to 10. Expected: the check reports "no error" exactly when the
     * route planner's {@link TipModel#tips(int, int, int, int, int)} reproduces FIRST's four official rows with the same numbers.
     * Why: the check repeats the planner's one-line test (it may not call it); this keeps the two copies from drifting apart.
     */
    @Test
    public void theCheckAgreesWithTheRoutePlannersOwnArithmeticForEveryWeightAndThreshold() {
        int agreeing = 0;
        for (int numerator = 1; numerator <= 9; numerator++) {
            for (int denominator = 1; denominator <= 5; denominator++) {
                for (int threshold = 5; threshold <= 10; threshold++) {
                    boolean planner = !TipModel.tips(0, 7, numerator, denominator, threshold) && TipModel.tips(0, 8, numerator, denominator, threshold)
                            && !TipModel.tips(3, 2, numerator, denominator, threshold) && TipModel.tips(3, 3, numerator, denominator, threshold);
                    ConstantsCheck.Report r = check(STAGED, numerator, denominator, threshold, MARGIN);
                    assertEquals(numerator + "/" + denominator + " tips at " + threshold + ": " + r.errors, planner, r.errors.isEmpty());
                    agreeing += planner ? 1 : 0;
                }
            }
        }
        assertTrue("some numbers do pass (5/3 at 8 is one of them)", agreeing > 0);
        assertTrue(agreeing < 9 * 5 * 6);
    }
}
