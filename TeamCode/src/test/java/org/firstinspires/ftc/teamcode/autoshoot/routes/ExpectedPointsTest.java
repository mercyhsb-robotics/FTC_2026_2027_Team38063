package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.render.RouteGallery;
import org.junit.Test;

import java.util.Collections;

/**
 * The "expected points" estimate: the points a route can claim ({@link RouteAnalysis#points()}) assume that every POLLEN lands, but real shooters land only
 * about 6 of 10 ({@code Auto.PLAN_SHOT_SUCCESS}). The estimate weighs each HIVE TIP by the exact binomial chance that enough POLLEN land and keeps LEAVE and PARK
 * in full.
 *
 * <p>How it works: no robot and no field, only the route analysis and arithmetic done by hand in the comments.
 */
public class ExpectedPointsTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double P = BioBuzzConstants.Auto.PLAN_SHOT_SUCCESS;
    private static final double EPS = 1e-9;

    // ---- the binomial arithmetic ----------------------------------------------------------------------------

    /**
     * Situation: at least 3 of 4 POLLEN must land, each with the chance 0.6.
     * Expected: exactly 0.4752 = 4 x 0.6^3 x 0.4 + 0.6^4 = 0.3456 + 0.1296 (the first tip, about 47.5 percent).
     */
    @Test
    public void threeOfFourAtSixTenthsIsFortySevenPointFivePercent() {
        assertEquals(0.4752, TipModel.chanceAtLeast(4, 3, 0.6), EPS);
    }

    /** Situation: all 8 of 8 must land at 0.6. Expected: 0.6^8 = 0.01679616, about 1.7 percent: a second tip from one volley is not a plan. */
    @Test
    public void eightOfEightAtSixTenthsIsAboutTwoPercent() {
        assertEquals(0.01679616, TipModel.chanceAtLeast(8, 8, 0.6), EPS);
        assertEquals(0.0168, TipModel.chanceAtLeast(8, 8, 0.6), 5e-5);
    }

    /** Situation: the figures of the benchmark research (3 of 4 at 0.56, 0.68, 0.80; 3 of 8 at 0.6; 8 of 9 at 0.68). Expected: the same percentages. */
    @Test
    public void theResearchedFiguresComeOutAsPublished() {
        assertEquals(0.407, TipModel.chanceAtLeast(4, 3, 0.56), 5e-4);
        assertEquals(0.616, TipModel.chanceAtLeast(4, 3, 0.68), 5e-4);
        assertEquals(0.819, TipModel.chanceAtLeast(4, 3, 0.80), 5e-4);
        assertEquals(0.950, TipModel.chanceAtLeast(8, 3, 0.6), 5e-4);
        assertEquals(0.163, TipModel.chanceAtLeast(9, 8, 0.68), 5e-4);
    }

    /** Situation: the corners of the formula. Expected: needing nothing is certain, needing more than are fired is impossible, and 0 and 1 are certain either way. */
    @Test
    public void theCornersOfTheFormulaAreRight() {
        assertEquals(1.0, TipModel.chanceAtLeast(4, 0, 0.6), 0.0);
        assertEquals(1.0, TipModel.chanceAtLeast(0, 0, 0.6), 0.0);
        assertEquals(1.0, TipModel.chanceAtLeast(4, -2, 0.6), 0.0);
        assertEquals(0.0, TipModel.chanceAtLeast(3, 4, 0.99), 0.0);
        assertEquals(1.0, TipModel.chanceAtLeast(4, 4, 1.0), 0.0);
        assertEquals(0.0, TipModel.chanceAtLeast(4, 1, 0.0), 0.0);
        // at least one of n: the opposite of "none land"
        assertEquals(1.0 - Math.pow(0.4, 5), TipModel.chanceAtLeast(5, 1, 0.6), EPS);
    }

    /** Situation: every need from 0 to 6 of 6 shots at an unusual chance, compared with listing all 64 outcomes (which POLLEN land and which miss). Expected: the same. */
    @Test
    public void itAgreesWithListingEveryOutcome() {
        double p = 0.37;
        int shots = 6;
        for (int needed = 0; needed <= shots + 1; needed++) {
            double byListing = 0.0;
            for (int outcome = 0; outcome < (1 << shots); outcome++) {
                int landed = Integer.bitCount(outcome);
                double chance = 1.0;
                for (int shot = 0; shot < shots; shot++) {
                    chance *= (outcome >> shot & 1) == 1 ? p : 1.0 - p;
                }
                if (landed >= needed) {
                    byListing += chance;
                }
            }
            assertEquals("need " + needed, byListing, TipModel.chanceAtLeast(shots, needed, p), EPS);
        }
    }

    // ---- the expected points of a route ---------------------------------------------------------------------------

    /** Situation: the standard route: shoot the 4 pre-loads (the first tip needs 3), slide off the wall, leave through the left corridor and park. */
    private static RouteAnalysis standard() {
        return RouteAnalysisTest.corridorPark().build().analyze();
    }

    /**
     * Situation: LEAVE 3 + PARK 5 and one tip that needs 3 of the 4 pre-loads.
     * Expected: 8 + 20 x 0.4752 = 17.504 expected points, below the 28 the route can claim if every POLLEN lands.
     */
    @Test
    public void theStandardRouteExpectsLeaveAndParkInFullAndTheTipAtItsChance() {
        RouteAnalysis a = standard();
        assertEquals(28, a.points());
        assertEquals(0.4752, a.tipChance(a.step(1)), EPS);
        assertEquals(0.4752, a.expectedTips(P), EPS);
        assertEquals(20 * 0.4752, a.expectedTipPoints(), EPS);
        assertEquals(8 + 20 * 0.4752, a.expectedPoints(), EPS);
        assertTrue(a.expectedPoints() < a.points());
    }

    /** Situation: a route that shoots nothing. Expected: no tip is counted and the expectation is exactly its LEAVE and PARK. */
    @Test
    public void aRouteWithoutATipExpectsExactlyItsLeaveAndPark() {
        RouteAnalysis a = AutoRoute.builder("no_tip").start(58, FRONT, -90).driveConstant(30, 14.8, -90).driveReverseTangent(30, 100)
                .driveLinear(12, 108, 0).park(12, 108, 0).build().analyze();
        assertEquals(0, a.tips());
        assertEquals(8, a.points());
        assertEquals(8.0, a.expectedPoints(), 0.0);
        assertEquals(0.0, a.expectedTips(P), 0.0);
        assertTrue(a.expectedPointsText(), a.expectedPointsText().contains("no tip is counted"));
    }

    /**
     * Situation: two tips: 4 pre-loads at the UP cell (3 of 4), then 4 + 4 into the empty cell that comes up (8 of 8, because the cell holds no NECTAR).
     * Expected: the second tip cannot happen without the first (the robot skips a volley at a cell that never came up), so its chance is 0.4752 x 0.6^8,
     *     and the expected tips are 0.4752 + 0.4752 x 0.01679616.
     */
    @Test
    public void aSecondTipCountsOnlyWhenTheFirstOneHappened() {
        AutoRoute r = AutoRoute.builder("second").start(35, FRONT, -90)
                .shootUp(4).intakeFlower(2.4, 48, 3).shootUp(4).intakeFlower(2.4, 48, 4).shootUp(4).build();
        RouteAnalysis a = r.analyze();
        assertEquals(2, a.tips());
        double first = 0.4752;
        double second = Math.pow(0.6, 8);
        assertEquals(first, a.tipChance(a.step(1)), EPS);
        assertEquals("8 of the 8 POLLEN fired at that cell since it came up", second, a.tipChance(a.step(5)), EPS);
        assertEquals("the volley that does not tip has no chance of its own", 0.0, a.tipChance(a.step(3)), 0.0);
        assertEquals(first * second, a.tipChanceWithEarlierTips(a.step(5), P), EPS);
        assertEquals(first + first * second, a.expectedTips(P), EPS);
        assertEquals(20 * (first + first * second), a.expectedTipPoints(), EPS);
        assertTrue(a.expectedPointsText(), a.expectedPointsText().contains("tip 2 needs 8 of the 8"));
    }

    /** Situation: 4 + 3 + 2 POLLEN into the empty cell. Expected: the 9th POLLEN is the spare, so the tip needs 8 of the 9 that were fired at that cell. */
    @Test
    public void aSpareMakesTheSecondTipLikelierThanExactlyEight() {
        RouteAnalysis a = AutoRoute.builder("nine").start(35, FRONT, -90)
                .shootUp(4).shootSide(Side.REAR, 4).shootSide(Side.REAR, 3).shootSide(Side.REAR, 2).build().analyze();
        double nineTries = TipModel.chanceAtLeast(9, 8, P);
        assertEquals(nineTries, a.tipChance(a.step(4)), EPS);
        assertTrue("9 tries beat exactly 8", nineTries > Math.pow(P, 8));
    }

    /** Situation: the chance of each POLLEN is a parameter. Expected: at 1 every POLLEN lands and the estimate is the ceiling; at 0 only LEAVE and PARK are left. */
    @Test
    public void theEstimateRunsFromTheCeilingDownToLeaveAndPark() {
        RouteAnalysis a = standard();
        assertEquals(a.points(), a.expectedPoints(1.0), EPS);
        assertEquals(a.leavePoints() + a.parkPoints(), a.expectedPoints(0.0), EPS);
        assertTrue("a better shooter expects more", a.expectedPoints(0.8) > a.expectedPoints(0.6));
    }

    /** Situation: a route that assumes the partner shoots the first tip and tips nothing itself. Expected: the partner's 20 points are not in the expectation. */
    @Test
    public void thePartnersTipIsNeverPartOfTheExpectation() {
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            RouteAnalysis a = r.analyze();
            assertEquals(r.id(), 1, a.partnerTips());
            assertEquals(r.id(), 0.0, a.expectedTipPoints(), 0.0);
            assertEquals(r.id(), a.leavePoints() + a.parkPoints(), a.expectedPoints(), 0.0);
        }
    }

    // ---- the words ---------------------------------------------------------------------------------------------------

    /** Situation: the standard route. Expected: the sentence says it is an estimate, names the three of four and the 47.5 and 60 percent, and adds up. */
    @Test
    public void theTextIsPlainWordsAndLabelledAsAnEstimate() {
        String t = standard().expectedPointsText();
        assertTrue(t, t.startsWith("about 17.5 (an estimate, not a promise)"));
        assertTrue(t, t.contains("LEAVE 3 + PARK 5 are certain if the route gets there"));
        assertTrue(t, t.contains("plus 9.5 for the HIVE TIP"));
        assertTrue(t, t.contains("the tip needs 3 of the 4 POLLEN fired at the UP cell to land, which is 47.5 percent"));
        assertTrue(t, t.contains("Auto.PLAN_SHOT_SUCCESS"));
        assertTrue(t, t.contains("60.0 percent"));
        for (int i = 0; i < t.length(); i++) {
            assertTrue("plain ASCII", t.charAt(i) < 128);
        }
    }

    /** Situation: the gallery page of the whole catalog. Expected: the summary table has the column and every route's section has the labelled line. */
    @Test
    public void theGalleryShowsTheEstimateNextToTheCeilingForEveryRoute() {
        String html = RouteGallery.indexHtml(RouteCatalog.all());
        assertTrue(html.contains("<th>Expected points (estimate)</th>"));
        assertTrue(html.contains("<th>Our AUTO points</th>"));
        int lines = 0;
        for (int at = html.indexOf("Expected AUTO points (estimate)"); at >= 0; at = html.indexOf("Expected AUTO points (estimate)", at + 1)) {
            lines++;
        }
        assertEquals("one line in every route's table", RouteCatalog.all().size(), lines);
        assertTrue(html.contains("an estimate, not a promise"));
        assertFalse("the ceiling line keeps its old text", html.contains("the most this route can score"));
    }

    // ---- when the tip is complete ---------------------------------------------------------------------------------

    /**
     * Situation: the standard route. Expected: the tip is planned complete {@code Auto.TIP_SETTLE_PLAN_SEC} after the tipping POLLEN LEFT the shooter, which is
     *     one ball flight before it landed, and the text says whether that is before the buzzer or inside the 8 s transition (where it still counts as AUTO).
     */
    @Test
    public void aTipIsPlannedCompleteFiveAndAHalfSecondsAfterItsPollenLeaves() {
        RouteAnalysis a = standard();
        double lands = a.step(1).tipSec;
        double left = lands - BioBuzzConstants.Auto.BALL_FLIGHT_SEC;
        assertEquals(left + BioBuzzConstants.Auto.TIP_SETTLE_PLAN_SEC, a.lastTipCompleteSec(), EPS);
        assertEquals(left + BioBuzzConstants.Auto.TIP_SETTLE_PLAN_SEC, new TimingModel().tipCompleteAtSec(lands), EPS);
        assertTrue(a.lastTipCompleteText(), a.lastTipCompleteText().contains("before the 30.0 s buzzer"));

        // a slower settle moves it past the buzzer but not past TELEOP at 38 s: it still counts as AUTO
        TimingModel.Params slow = new TimingModel.Params();
        slow.tipCompletePlanSec = BioBuzzConstants.Auto.AUTO_DURATION_SEC;
        RouteAnalysis late = RouteAnalysis.of(RouteAnalysisTest.corridorPark().build(), new org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel(),
                new TimingModel(slow));
        assertTrue(late.lastTipCompleteText(), late.lastTipCompleteText().contains("still counts as AUTO"));
        slow.tipCompletePlanSec = BioBuzzConstants.Auto.AUTO_DURATION_SEC + BioBuzzConstants.Auto.TRANSITION_SEC;
        RouteAnalysis tooLate = RouteAnalysis.of(RouteAnalysisTest.corridorPark().build(), new org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel(),
                new TimingModel(slow));
        assertTrue(tooLate.lastTipCompleteText(), tooLate.lastTipCompleteText().contains("would count as TELEOP"));
    }

    /** Situation: a route with no volley of its own. Expected: no completion time (NaN) and no text. */
    @Test
    public void aRouteThatTipsNothingHasNoTipCompletionTime() {
        RouteAnalysis a = AutoRoute.builder("none").start(58, FRONT, -90).waitSeconds(1.0).build().analyze();
        assertTrue(Double.isNaN(a.lastTipCompleteSec()));
        assertEquals("", a.lastTipCompleteText());
        assertEquals(Collections.emptyList(), a.tipEvents());
    }
}
