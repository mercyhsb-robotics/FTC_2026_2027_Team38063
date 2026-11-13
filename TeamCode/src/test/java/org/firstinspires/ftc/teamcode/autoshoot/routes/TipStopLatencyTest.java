package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * The spare POLLEN of a volley: is it kept, or is it launched into the HIVE cell while that cell swings down?
 *
 * <p>The route model used to say a camera-gated volley "stops exactly at the tip", so the 4th pre-load of the standard first volley (the 3rd POLLEN
 * tips the cell, the 4th is the spare) stayed in the robot. Nothing in the project supports zero delay: the tipping POLLEN is still in the air for
 * about 0.5 s ({@code Auto.BALL_FLIGHT_SEC}), the camera sees the tip only when the cell has swung far enough for its tags to look different, and the
 * feeder pauses only after they have been gone a moment ({@code Auto.TIP_STOP_LATENCY_SEC}, about 0.85 s in all, to be measured), while the
 * next POLLEN is fed 0.9 s after the last ({@code Auto.TIME_PER_SHOT_SEC}). So the spare is probably launched into a cell that is already swinging.
 * The plan counts it as fired ({@code StepInfo.firedAfterTip}) and the validator warns.
 *
 * <p>How it works: no robot. {@link TipModel#launchedBeforeTheFeederStops} is the arithmetic and is tested on its own (with its boundaries), then the
 * route analysis and the validator are asked about the standard volley with the shipped numbers and with a feeder that stops in time
 * ({@link FastFeederRobot}). The closed-loop version with the real {@code AutoShootController} is in {@code AutoShootSequenceTest}.
 */
public class TipStopLatencyTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double SHOT = BioBuzzConstants.Auto.TIME_PER_SHOT_SEC;
    private static final double FLIGHT = BioBuzzConstants.Auto.BALL_FLIGHT_SEC;
    private static final double LATENCY = BioBuzzConstants.Auto.TIP_STOP_LATENCY_SEC;

    private static AutoRoute standardVolley() {
        return AutoRoute.builder("standard").start(35, FRONT, -90).shootUp(4).build();
    }

    // ---- the arithmetic ------------------------------------------------------------------------------------

    /**
     * Situation: POLLEN 0.9 s apart, a flight of 0.5 s and several stop latencies, with spares of 0 to 3.
     * Expected: the j-th POLLEN after the tipping one (it leaves j x 0.9 s later) is launched if j x 0.9 < 0.5 + latency: latency 0.85 launches
     *     one (0.9 < 1.35, 1.8 is not), 0.3 launches none, 1.5 launches two (0.9 and 1.8 < 2.0); never more than the spare, none for no spare.
     * Boundary: a latency of exactly 0.4 s (= 0.9 - 0.5) launches none, and 0.41 s launches one.
     */
    @Test
    public void thePollenLaunchedBeforeTheFeederStopsFollowTheTimes() {
        assertEquals(1, TipModel.launchedBeforeTheFeederStops(1, SHOT, FLIGHT, 0.85));
        assertEquals(1, TipModel.launchedBeforeTheFeederStops(3, SHOT, FLIGHT, 0.85));
        assertEquals(0, TipModel.launchedBeforeTheFeederStops(3, SHOT, FLIGHT, 0.3));
        assertEquals(0, TipModel.launchedBeforeTheFeederStops(3, SHOT, FLIGHT, 0.0));
        assertEquals(2, TipModel.launchedBeforeTheFeederStops(3, SHOT, FLIGHT, 1.5));
        assertEquals("never more than the spare", 1, TipModel.launchedBeforeTheFeederStops(1, SHOT, FLIGHT, 1.5));
        assertEquals("no spare, nothing to launch", 0, TipModel.launchedBeforeTheFeederStops(0, SHOT, FLIGHT, 5.0));
        assertEquals("exactly the time between POLLEN minus the flight: the feeder is just in time", 0,
                TipModel.launchedBeforeTheFeederStops(1, SHOT, FLIGHT, SHOT - FLIGHT));
        assertEquals("a hair more is too late", 1, TipModel.launchedBeforeTheFeederStops(1, SHOT, FLIGHT, SHOT - FLIGHT + 0.01));
    }

    // ---- the route plan --------------------------------------------------------------------------------------

    /**
     * Situation: the standard first volley (4 pre-loads at the UP cell) with the shipped numbers, and with a feeder that stops in time.
     * Expected: with the shipped numbers the 4th POLLEN is launched after the tip (firedAfterTip 1), all 4 leave the robot and the hopper is empty; the
     *     volley lasts 4 feeds. With a feeder that stops in time only 3 leave, the 4th stays, the volley lasts 3 feeds. The planned spare is 1 in both.
     * Why: the spare is what covers a field that needs one more POLLEN, not a POLLEN to carry into TELEOP.
     */
    @Test
    public void theSpareIsSpentWithTheShippedNumbersAndKeptByAFeederThatStopsInTime() {
        RouteAnalysis.StepInfo real = RouteAnalysis.of(standardVolley()).step(1);
        assertEquals(1, real.tipSpare);
        assertEquals(1, real.firedAfterTip);
        assertEquals(4, real.pollenBefore - real.pollenAfter);
        assertEquals(0, real.pollenAfter);
        assertEquals(real.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC + 4 * SHOT, real.endSec(), 1e-9);
        RouteAnalysis.StepInfo fast = FastFeederRobot.analyze(standardVolley()).step(1);
        assertEquals(1, fast.tipSpare);
        assertEquals(0, fast.firedAfterTip);
        assertEquals(3, fast.pollenBefore - fast.pollenAfter);
        assertEquals(1, fast.pollenAfter);
        assertEquals(fast.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC + 3 * SHOT, fast.endSec(), 1e-9);
    }

    /**
     * Situation: the plan for a feeder whose stop latency is varied around the break-even (the time between POLLEN minus the flight, 0.4 s).
     * Expected: at or below it the spare is kept, above it the spare is spent. The tip itself always lands a flight time after the tipping POLLEN left.
     * Why: this is the number the team measures on day one; the plan must flip exactly there.
     */
    @Test
    public void theSpareFlipsExactlyAtTheTimeBetweenPollenMinusTheFlight() {
        double breakEven = SHOT - FLIGHT;
        for (double latency : new double[] {0.0, 0.2, breakEven}) {
            TimingModel timing = new TimingModel().withTipStopLatencySec(latency);
            RouteAnalysis.StepInfo s = RouteAnalysis.of(standardVolley(), new FieldModel(), timing).step(1);
            assertEquals("latency " + latency, 0, s.firedAfterTip);
            assertEquals("latency " + latency, s.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC + 3 * SHOT + FLIGHT, s.tipSec, 1e-9);
        }
        for (double latency : new double[] {breakEven + 0.01, 0.85, 1.2}) {
            TimingModel timing = new TimingModel().withTipStopLatencySec(latency);
            RouteAnalysis.StepInfo s = RouteAnalysis.of(standardVolley(), new FieldModel(), timing).step(1);
            assertEquals("latency " + latency, 1, s.firedAfterTip);
            assertEquals("latency " + latency, s.launchSec + BioBuzzConstants.Auto.AIM_SPINUP_SEC + 3 * SHOT + FLIGHT, s.tipSec, 1e-9);
        }
    }

    /**
     * Situation: a volley of exactly the 3 POLLEN that tip the cell (no spare), and a volley with a spare, both at the shipped numbers.
     * Expected: the exact volley has nothing to spend, so no WARN of this kind (its own finding is the missing spare, the planning-margin rule).
     */
    @Test
    public void aVolleyWithNoSpareHasNothingToSpend() {
        RouteAnalysis.StepInfo exact = RouteAnalysis.of(AutoRoute.builder("exact").start(35, FRONT, -90).shootUp(3).build()).step(1);
        assertEquals(0, exact.tipSpare);
        assertEquals(0, exact.firedAfterTip);
        assertEquals(3, exact.pollenBefore - exact.pollenAfter);
        List<Issue> l = RouteValidator.check(AutoRoute.builder("exact").start(35, FRONT, -90).shootUp(3).build());
        for (Issue i : l) {
            assertFalse(i.toString(), i.message.contains("the camera cannot stop the feeder at once"));
        }
    }

    // ---- the validator ---------------------------------------------------------------------------------------

    /**
     * Situation: the standard first volley, validated with the shipped numbers and with a feeder that stops in time.
     * Expected: one WARN of the G417 rule on the volley. It says how long the tipping POLLEN is in the air, how long the feeder needs to stop, when the
     *     next POLLEN leaves, how many POLLEN are spent and how many of the spare are kept, and the two ways out (fire only what tips; or measure the
     *     real stop time, with the latency under which the spare stays). The same route for a feeder that stops in time has no finding.
     */
    @Test
    public void theValidatorWarnsThatTheSpareIsProbablySpentAndSaysHowToKeepIt() {
        List<Issue> l = RouteValidator.check(standardVolley());
        assertEquals(l.toString(), 1, l.size());
        Issue w = l.get(0);
        assertEquals(Severity.WARN, w.severity);
        assertEquals(Rule.SHOOT_TARGET, w.rule);
        assertEquals("the volley", 1, w.stepIndex);
        assertTrue(w.message, w.message.contains("the camera cannot stop the feeder at once"));
        assertTrue(w.message, w.message.contains("in the air for 0.50 s"));
        assertTrue(w.message, w.message.contains("stops about 0.85 s after it lands (Auto.TIP_STOP_LATENCY_SEC)"));
        assertTrue(w.message, w.message.contains("the next POLLEN leaves 0.90 s after the last"));
        assertTrue(w.message, w.message.contains("the next 1 POLLEN will probably be launched into the cell while it swings down"));
        assertTrue(w.message, w.message.contains("the plan counts 4 fired and 0 of the spare kept"));
        assertTrue(w.message, w.message.contains("Fire only 3 to keep it for sure"));
        assertTrue(w.message, w.message.contains("with a latency under 0.40 s the spare stays in the robot"));
        assertTrue(w.message, w.message.contains("G417"));
        assertTrue(w.message, w.message.contains("QA12"));
        assertTrue(FastFeederRobot.check(standardVolley()).isEmpty());
    }

    /**
     * Situation: a TIMED volley (an old OpMode that feeds for a fixed time) is unchanged: it fires every POLLEN whatever happens, and its WARN is still the
     * "TIMED" one, not the new one.
     */
    @Test
    public void aTimedVolleyKeepsItsOwnWarning() {
        AutoRoute timed = AutoRoute.builder("timed").start(35, FRONT, -90).shootUp(35, FRONT, -90, 4).launchSeconds(6.0).build();
        RouteAnalysis.StepInfo s = RouteAnalysis.of(timed).step(1);
        assertEquals(1, s.firedAfterTip);
        assertEquals(4, s.pollenBefore - s.pollenAfter);
        List<Issue> l = RouteValidator.check(timed);
        boolean timedWarn = false;
        for (Issue i : l) {
            assertFalse(i.toString(), i.message.contains("the camera cannot stop the feeder at once"));
            timedWarn |= i.message.contains("TIMED");
        }
        assertTrue(l.toString(), timedWarn);
    }

    /**
     * Situation: the constants themselves.
     * Expected: the shipped stop latency is the placeholder it is documented as (it is on the PLACEHOLDERS list), longer than the break-even of the
     *     shipped numbers (so the shipped plan spends the spare, as the documentation says), and the flight is positive.
     */
    @Test
    public void theShippedNumbersAreTheDocumentedOnes() {
        assertTrue(java.util.Arrays.asList(BioBuzzConstants.PLACEHOLDERS).contains("Auto.TIP_STOP_LATENCY_SEC"));
        assertTrue(LATENCY > SHOT - FLIGHT);
        assertTrue(FLIGHT > 0.0);
    }
}
