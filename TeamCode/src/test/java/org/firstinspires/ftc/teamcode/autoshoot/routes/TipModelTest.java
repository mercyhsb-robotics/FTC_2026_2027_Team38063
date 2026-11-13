package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.junit.Test;

/**
 * The weighted tip rule: a HIVE cell tips when {@code nectar x 8 + pollen x 5 >= 40}, that is 5 NECTAR, or 8 POLLEN,
 * or a mix. The tests spell the numbers out (3, 4, 5, 8, 40) on purpose: they are the team's figures and the whole
 * point of the rule, and {@link #theThresholdsAreTheTeamsFigures} fails if somebody changes them without
 * rethinking the routes.
 */
public class TipModelTest {

    @Test
    public void theThresholdsAreTheTeamsFigures() {
        assertEquals(3, BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL);
        assertEquals(5, BioBuzzConstants.Auto.NECTAR_TIP_THRESHOLD);
        assertEquals(8, BioBuzzConstants.Auto.POLLEN_TIP_THRESHOLD);
    }

    @Test
    public void threeNectarPlusFourPollenTipButThreePollenDoNot() {
        assertTrue("3 x 8 + 4 x 5 = 44 >= 40", TipModel.tips(3, 4));
        assertFalse("3 x 8 + 3 x 5 = 39 < 40: one POLLEN short", TipModel.tips(3, 3));
        assertFalse("the 3 staged NECTAR alone do not tip the HIVE", TipModel.tips(3, 0));
    }

    @Test
    public void fiveNectarOrEightPollenAloneTipAndOneLessDoesNot() {
        assertTrue(TipModel.tips(5, 0));
        assertFalse(TipModel.tips(4, 0));
        assertTrue(TipModel.tips(0, 8));
        assertFalse(TipModel.tips(0, 7));
    }

    @Test
    public void aMixOfBothTipsWhenTheSharesAddUpToOne() {
        // 2 NECTAR = 2/5 and 5 POLLEN = 5/8: 16 + 25 = 41 >= 40 tips; 2 NECTAR and 4 POLLEN is 36: it does not
        assertTrue(TipModel.tips(2, 5));
        assertFalse(TipModel.tips(2, 4));
        assertTrue(TipModel.tips(4, 2));
        assertFalse(TipModel.tips(4, 1));
    }

    @Test
    public void theRuleUsesWholeNumbersSoNoRoundingCanFlipTheAnswer() {
        // 3/5 + 3/8 = 0.975 is below 1 and 3/5 + 4/8 = 1.1 is above it; in whole numbers 39 < 40 <= 44
        assertEquals(39, 3 * BioBuzzConstants.Auto.POLLEN_TIP_THRESHOLD + 3 * BioBuzzConstants.Auto.NECTAR_TIP_THRESHOLD);
        assertEquals(44, 3 * BioBuzzConstants.Auto.POLLEN_TIP_THRESHOLD + 4 * BioBuzzConstants.Auto.NECTAR_TIP_THRESHOLD);
        // other thresholds give other answers: with 2 and 2 one NECTAR plus one POLLEN is a whole tip
        assertTrue(TipModel.tips(1, 1, 2, 2));
        assertFalse(TipModel.tips(1, 0, 2, 2));
    }

    @Test
    public void pollenStillNeededCountsWhatIsMissing() {
        assertEquals("the first tip: one volley", 4, TipModel.pollenStillNeeded(3, 0));
        assertEquals(1, TipModel.pollenStillNeeded(3, 3));
        assertEquals("already enough", 0, TipModel.pollenStillNeeded(3, 4));
        assertEquals("a cell with no NECTAR: two volleys", 8, TipModel.pollenStillNeeded(0, 0));
        assertEquals(3, TipModel.pollenStillNeeded(0, 5));
        assertEquals("4 NECTAR are 32 of the 40", 2, TipModel.pollenStillNeeded(4, 0));
        assertEquals(0, TipModel.pollenStillNeeded(5, 0));
        assertEquals(4, TipModel.firstTipPollen());
        assertEquals(8, TipModel.laterTipPollen());
    }

    @Test
    public void aVolleyIsWorkedOutInWordsWithTheRealSums() {
        assertEquals("the UP cell holds 3 NECTAR and 0 POLLEN of ours; this volley adds 4: 3 x 8 + 4 x 5 = 44, at least 40, so the HIVE tips",
                TipModel.explainVolley(3, 0, 4));
        // one POLLEN short: the sum is 39 and the words say how many more POLLEN it would take
        String short3 = TipModel.explainVolley(3, 0, 3);
        assertTrue(short3, short3.contains("3 x 8 + 3 x 5 = 39, short of 40, so no tip yet (1 more POLLEN into this cell would tip it)"));
        // the POLLEN that are in the cell already count: 4 + 4 into a cell with no NECTAR is exactly the 40 that tips
        String second = TipModel.explainVolley(0, 4, 4);
        assertTrue(second, second.contains("holds 0 NECTAR and 4 POLLEN of ours; this volley adds 4: 0 x 8 + 8 x 5 = 40, at least 40, so the HIVE tips"));
        // and the first half volley into the empty cell is only half a tip
        String half = TipModel.explainVolley(0, 0, 4);
        assertTrue(half, half.contains("= 20, short of 40, so no tip yet (4 more POLLEN"));
    }

    @Test
    public void theRuleInWordsQuotesTheRealNumbers() {
        String words = TipModel.explainRule();
        assertTrue(words, words.contains("NECTAR x 8 + POLLEN x 5 reaches 40"));
        assertTrue(words, words.contains("3 NECTAR, so 4 POLLEN tip it (3 x 8 + 4 x 5 = 44)"));
        assertTrue(words, words.contains("but 3 do not (3 x 8 + 3 x 5 = 39)"));
        assertTrue(words, words.contains("each later tip needs 8 POLLEN"));
    }
}
