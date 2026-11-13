package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.junit.Test;

/**
 * The tip rule of FIRST's own calibration: a HIVE cell tips when its POLLEN plus 5/3 POLLEN for every NECTAR add up to 8
 * POLLEN-equivalents, which in whole numbers is {@code nectar x 5 + pollen x 3 >= 8 x 3}. FIRST's field documents (the Event
 * Field Setup Guide section 12.3, the Field Acceptance Checklist 2.3 to 2.6) fix two facts that every real field must show:
 * an empty cell tips with the 8th POLLEN and not with 7, and a cell holding the 3 staged NECTAR tips with the 3rd POLLEN and
 * not with 2. The tests spell the numbers out (3, 5, 8, 24) on purpose: they are the official figures and the whole point of
 * the rule, and {@link #theFiguresAreFirstsCalibration} fails if somebody changes them without rethinking the routes.
 */
public class TipModelTest {

    @Test
    public void theFiguresAreFirstsCalibration() {
        assertEquals(3, BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL);
        assertEquals("NECTAR = 5/3 POLLEN", 5, BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR);
        assertEquals(3, BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR);
        assertEquals(5.0 / 3.0, BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT, 1e-12);
        assertEquals("a cell tips at 8 POLLEN-equivalents", 8, BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD);
        assertEquals(1, BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN);
    }

    // ---- the official table (Setup Guide 12.3, Checklist 2.3 to 2.6) ---------------------------------------

    @Test
    public void threeNectarPlusThreePollenTipButThreeNectarPlusTwoPollenDoNot() {
        assertFalse("3 NECTAR + 2 POLLEN: 15 + 6 = 21 < 24, FIRST: no tip", TipModel.tips(3, 2));
        assertTrue("3 NECTAR + 3 POLLEN: 15 + 9 = 24 >= 24, FIRST: tips", TipModel.tips(3, 3));
        assertFalse("the 3 staged NECTAR alone do not tip the HIVE", TipModel.tips(3, 0));
    }

    @Test
    public void anEmptyCellTipsWithTheEighthPollenButNotWithTheSeventh() {
        assertFalse("0 + 7 x 3 = 21 < 24, FIRST: no tip", TipModel.tips(0, 7));
        assertTrue("0 + 8 x 3 = 24 >= 24, FIRST: tips", TipModel.tips(0, 8));
    }

    @Test
    public void fiveNectarTipAndFourDoNot() {
        assertTrue("5 x 5 = 25 >= 24: the team's rule of thumb", TipModel.tips(5, 0));
        assertFalse("4 x 5 = 20 < 24", TipModel.tips(4, 0));
    }

    @Test
    public void theOldWeightedRuleWasWrongAboutTheStagedCell() {
        // The earlier rule was nectar/5 + pollen/8 >= 1: 3/5 + 3/8 = 0.975 said "3 NECTAR + 3 POLLEN do not tip" and
        // "3 NECTAR + 4 POLLEN do". FIRST says the 3rd POLLEN tips. This is the case that fails without the change.
        assertTrue(3.0 / 5.0 + 3.0 / 8.0 < 1.0);
        assertTrue("the official answer is a tip", TipModel.tips(3, 3));
        // and the first tip needs 3 POLLEN, not 4
        assertEquals(3, TipModel.firstTipPollen());
    }

    @Test
    public void aMixOfBothAddsUpInPollenEquivalents() {
        // 1 NECTAR = 5/3 POLLEN: 1 NECTAR + 6 POLLEN = 5 + 18 = 23 < 24 does not tip, 1 NECTAR + 7 POLLEN = 26 does
        assertFalse(TipModel.tips(1, 6));
        assertTrue(TipModel.tips(1, 7));
        // 2 NECTAR + 5 POLLEN = 10 + 15 = 25 tips, 2 NECTAR + 4 POLLEN = 22 does not
        assertTrue(TipModel.tips(2, 5));
        assertFalse(TipModel.tips(2, 4));
        // 4 NECTAR + 1 POLLEN = 20 + 3 = 23 does not, + 2 POLLEN = 26 does
        assertFalse(TipModel.tips(4, 1));
        assertTrue(TipModel.tips(4, 2));
    }

    // ---- exact arithmetic ---------------------------------------------------------------------------------

    @Test
    public void theRuleUsesWholeNumbersSoNoRoundingCanFlipTheAnswer() {
        // 3 NECTAR + 3 POLLEN are exactly 8 POLLEN-equivalents. With a rounded decimal weight the sum is just below 8 and a
        // floating point test says "no tip", the opposite of FIRST's table; with whole numbers it is 24 >= 24.
        assertTrue("3 x 1.66 + 3 = 7.98 would say no tip", 3 * 1.66 + 3 < 8.0);
        assertTrue("3 x 5 + 3 x 3 = 24 = 8 x 3", TipModel.tips(3, 3));
        assertEquals(24, 3 * BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR + 3 * BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR);
        assertEquals(8 * BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR,
                BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD * BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR);
        // the exactness holds for every small cell, not only the official ones: compare with exact fractions
        for (int nectar = 0; nectar <= 8; nectar++) {
            for (int pollen = 0; pollen <= 12; pollen++) {
                boolean exact = nectar * 5 + pollen * 3 >= 24;
                assertEquals(nectar + " NECTAR + " + pollen + " POLLEN", exact, TipModel.tips(nectar, pollen));
            }
        }
    }

    @Test
    public void onlyAWeightFromFiveThirdsUpToTwoAgreesWithTheOfficialTable() {
        // Any NECTAR weight w with 5/3 <= w < 2 reproduces the four official rows; a lighter or a heavier one does not. This is
        // why the constant can be VERIFY with a range: 5/3 is the lightest weight that still tips 3 NECTAR + 3 POLLEN.
        int[][] weights = {{5, 3}, {7, 4}, {9, 5}, {11, 6}, {19, 10}};          // 1.667, 1.75, 1.8, 1.833, 1.9
        for (int[] w : weights) {
            assertTrue(w[0] + "/" + w[1], officialTable(w[0], w[1]));
        }
        assertFalse("8/5 = 1.6: 3 NECTAR + 3 POLLEN would not tip", officialTable(8, 5));
        assertFalse("13/8 = 1.625", officialTable(13, 8));
        assertFalse("2 = 2/1: 3 NECTAR + 2 POLLEN would tip", officialTable(2, 1));
        assertFalse("21/10 = 2.1", officialTable(21, 10));
    }

    private static boolean officialTable(int numerator, int denominator) {
        int threshold = BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD;
        return !TipModel.tips(3, 2, numerator, denominator, threshold) && TipModel.tips(3, 3, numerator, denominator, threshold)
                && !TipModel.tips(0, 7, numerator, denominator, threshold) && TipModel.tips(0, 8, numerator, denominator, threshold);
    }

    // ---- how many POLLEN, how many spare -------------------------------------------------------------------

    @Test
    public void pollenStillNeededCountsWhatIsMissing() {
        assertEquals("the first tip: 3 POLLEN, one of the 4 pre-loads is spare", 3, TipModel.pollenStillNeeded(3, 0));
        assertEquals(1, TipModel.pollenStillNeeded(3, 2));
        assertEquals("already enough", 0, TipModel.pollenStillNeeded(3, 3));
        assertEquals("a cell with no NECTAR: 8 POLLEN, two volleys", 8, TipModel.pollenStillNeeded(0, 0));
        assertEquals(3, TipModel.pollenStillNeeded(0, 5));
        assertEquals("4 NECTAR are 20 of the 24", 2, TipModel.pollenStillNeeded(4, 0));
        assertEquals(0, TipModel.pollenStillNeeded(5, 0));
        assertEquals(3, TipModel.firstTipPollen());
        assertEquals(8, TipModel.laterTipPollen());
        assertEquals(3, TipModel.minimumPollen(3));
        assertEquals(8, TipModel.minimumPollen(0));
    }

    @Test
    public void aRoutePlansOnePollenMoreThanTheMinimum() {
        assertEquals("4 for the first tip (the 4 pre-loads)", 4, TipModel.planningPollen(3));
        assertEquals("9 for a later tip (FIRST's Q&A QA46: a field needed 9 instead of 8)", 9, TipModel.planningPollen(0));
    }

    @Test
    public void theSpareIsWhatIsPlannedBeyondTheMinimum() {
        assertEquals("4 into the staged cell: 1 spare", 1, TipModel.spare(3, 4));
        assertEquals("exactly 3: no spare", 0, TipModel.spare(3, 3));
        assertEquals("only 2: short of the minimum", -1, TipModel.spare(3, 2));
        assertEquals("8 into an empty cell: no spare", 0, TipModel.spare(0, 8));
        assertEquals(1, TipModel.spare(0, 9));
    }

    // ---- the words ----------------------------------------------------------------------------------------

    @Test
    public void aVolleyIsWorkedOutInWordsWithTheRealSums() {
        // 4 pre-loads at the staged cell: it tips on the 3rd, the 4th stays in the robot as the spare
        assertEquals("the UP cell holds 3 NECTAR and 0 POLLEN of ours; this volley fires up to 4: 3 NECTAR x 5/3 + 3 POLLEN = 8 POLLEN-equivalents, "
                        + "the 8 that tip a cell, so the HIVE tips on POLLEN 3 of the 4 and the volley stops there; the last 1 stays in the robot as the spare",
                TipModel.explainVolley(3, 0, 4));
        // exactly the minimum: the words say there is no spare
        String exact = TipModel.explainVolley(3, 0, 3);
        assertTrue(exact, exact.contains("so the HIVE tips on POLLEN 3 of the 3: NO spare, a field that needs one more POLLEN would not tip"));
        // one short: the sum is 7 and the words say how many more POLLEN it would take
        String short2 = TipModel.explainVolley(3, 0, 2);
        assertTrue(short2, short2.contains("3 NECTAR x 5/3 + 2 POLLEN = 7 POLLEN-equivalents, short of the 8 that tip a cell, so no tip yet (1 more POLLEN into this cell would tip it)"));
        // the POLLEN that are in the cell already count: 4 + 4 into a cell with no NECTAR is exactly the 8 that tips it
        String second = TipModel.explainVolley(0, 4, 4);
        assertTrue(second, second.contains("holds 0 NECTAR and 4 POLLEN of ours; this volley fires up to 4: 0 NECTAR x 5/3 + 8 POLLEN = 8 POLLEN-equivalents, the 8 that tip a cell, so the HIVE tips on POLLEN 4 of the 4: NO spare"));
        // and the first half volley into the empty cell is only half a tip
        String half = TipModel.explainVolley(0, 0, 4);
        assertTrue(half, half.contains("= 4 POLLEN-equivalents, short of the 8 that tip a cell, so no tip yet (4 more POLLEN"));
    }

    @Test
    public void aTimedVolleyIsWorkedOutAsOneThatCannotStop() {
        // a volley that feeds for a fixed time fires all 4 POLLEN: the 4th goes into a cell that is already tipping down
        String timed = TipModel.explainVolley(3, 0, 4, false);
        assertTrue(timed, timed.contains("so the HIVE tips on POLLEN 3 of the 4; this volley is TIMED and cannot stop, so the last 1 goes into a cell that is already tipping down"));
        assertFalse(timed, timed.contains("stays in the robot"));
        // two POLLEN too many: "go", not "goes"
        assertTrue(TipModel.explainVolley(0, 6, 4, false).contains("the last 2 go into a cell that is already tipping down"));
        // with no spare there is nothing to say about the end of the volley, stopping or not
        assertEquals(TipModel.explainVolley(3, 0, 3, true), TipModel.explainVolley(3, 0, 3, false));
        // the three-argument form is the camera-gated volley
        assertEquals(TipModel.explainVolley(3, 0, 4, true), TipModel.explainVolley(3, 0, 4));
    }

    @Test
    public void thePollenEquivalentsAreWrittenExactly() {
        assertEquals("24 thirds", "8", TipModel.equivalentsText(3, 3));
        assertEquals("23 thirds", "7 2/3", TipModel.equivalentsText(1, 6));
        assertEquals("20 thirds", "6 2/3", TipModel.equivalentsText(4, 0));
        assertEquals("5 thirds", "1 2/3", TipModel.equivalentsText(1, 0));
        assertEquals("an empty cell", "0", TipModel.equivalentsText(0, 0));
        assertEquals("5/3", TipModel.weightText());
    }

    @Test
    public void theRuleInWordsQuotesTheRealNumbers() {
        String words = TipModel.explainRule();
        assertTrue(words, words.contains("POLLEN plus 5/3 POLLEN for every NECTAR add up to 8 POLLEN-equivalents"));
        assertTrue(words, words.contains("an empty cell tips with the 8th POLLEN and 7 do not"));
        assertTrue(words, words.contains("a cell with the 3 staged NECTAR tips with the 3rd POLLEN and 2 do not"));
        assertTrue(words, words.contains("the FIRST tip needs only 3 POLLEN (the robot starts with 4, so 1 is spare)"));
        assertTrue(words, words.contains("each LATER tip needs 8 POLLEN"));
        assertTrue(words, words.contains("plans 1 POLLEN more than the minimum, 4 and 9"));
        assertFalse("the old team figure is gone", words.contains("5 NECTAR, or 8 POLLEN, or a mix"));
        assertEquals("a cell tips at 8 POLLEN-equivalents (each POLLEN counts 1, each NECTAR 5/3)", TipModel.ruleShort());
    }

    @Test
    public void ordinalsAreWrittenTheEnglishWay() {
        assertEquals("1st", TipModel.ordinal(1));
        assertEquals("2nd", TipModel.ordinal(2));
        assertEquals("3rd", TipModel.ordinal(3));
        assertEquals("4th", TipModel.ordinal(4));
        assertEquals("8th", TipModel.ordinal(8));
        assertEquals("11th", TipModel.ordinal(11));
        assertEquals("12th", TipModel.ordinal(12));
        assertEquals("13th", TipModel.ordinal(13));
        assertEquals("21st", TipModel.ordinal(21));
        assertEquals("22nd", TipModel.ordinal(22));
    }
}
