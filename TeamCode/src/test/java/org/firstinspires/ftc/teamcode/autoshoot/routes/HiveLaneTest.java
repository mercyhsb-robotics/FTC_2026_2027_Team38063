package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.HiveSolid;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * The lane under the HIVE: a robot 12 to 13 in tall fits between the two foot bars of the HIVE frame (the cells are 30.65 in up), which saves
 * about 20 in on the way from the audience side to the rear side. By default a route must stay out of the whole HIVE footprint (G409 / G417: never
 * be under a HIVE that is tipping). A DRIVE that says {@code allowUnderHive()} is checked against the exact solid instead (foot bars and leg stubs,
 * {@link HiveSolid}) with {@code Auto.HIVE_LANE_MARGIN_IN} on both sides, and the validator WARNs while the HIVE could still tip.
 *
 * <p>How it works: no robot. Routes that drive up the lane (the robot facing the audience wall, driving backwards with the shooter leading, at
 * x = 64 in, 6.7 in left of the field center) are validated for RED and for BLUE (the half turn), which must agree. Each rule has a negative twin: a leg
 * that is not allowed, a leg too close to a foot bar, a leg across a bar, and a wait that does not confirm the HIVE.
 *
 * <p>Words used: <b>lane</b> is the open floor between the foot bars; <b>stub</b> is the part of a leg lower than the robot; <b>margin</b> is the
 * space the robot must keep to spare on each side; <b>at rest</b> means the camera has confirmed a tip fully and nothing has been launched into the
 * HIVE since.
 */
public class HiveLaneTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double SIDE = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
    private static final double MARGIN = BioBuzzConstants.Auto.HIVE_LANE_MARGIN_IN;
    /** The left tip of the leg stubs for the robot height limit: the lane is open to the right of this. */
    private static final double STUB_TIP = HiveSolid.of(BioBuzzConstants.Field.CENTER_IN, BioBuzzConstants.Auto.HIVE_SOLID_HEIGHT_IN).narrowestLane()[0];
    /** The x of a robot facing -90 (side extents 8.9 in) whose red-side edge is exactly the lane margin from the stub tips. */
    private static final double LIMIT_X = STUB_TIP + MARGIN + SIDE;

    private static List<Issue> validate(AutoRoute.Builder b) {
        AutoRoute red = b.build();
        List<Issue> r = RouteValidator.check(red);
        List<Issue> blue = RouteValidator.check(red.forAlliance(Alliance.BLUE));
        assertEquals("red and blue findings differ:\n" + r + "\n" + blue, signature(r), signature(blue));
        return r;
    }

    private static List<String> signature(List<Issue> issues) {
        List<String> out = new ArrayList<String>();
        for (Issue i : issues) {
            out.add(i.severity + "/" + i.rule + "/" + i.stepIndex);
        }
        return out;
    }

    private static int count(List<Issue> issues, Rule rule, Severity severity) {
        int n = 0;
        for (Issue i : issues) {
            if (i.rule == rule && i.severity == severity) {
                n++;
            }
        }
        return n;
    }

    /** From the C1 start to the lane axis, then the lane leg (step 2), then out the other side. */
    private static AutoRoute.Builder laneRoute(double laneX, boolean flag) {
        AutoRoute.Builder b = AutoRoute.builder("lane").start(58.0, FRONT, -90)
                .driveConstant(laneX, 38.0, -90).label("to the lane")
                .driveConstant(laneX, 105.0, -90).label("the lane");
        if (flag) {
            b = b.allowUnderHive();
        }
        return b.driveConstant(58.0, 120.0, -90).label("out");
    }

    /**
     * Situation: the same leg up the middle of the lane (x = 64) with and without the flag.
     * Expected: without it the HIVE keep-out ERROR names step 3 and the whole-HIVE rule; with it there is no HIVE ERROR at all, and the one
     *     finding is the UNDER_HIVE WARN (the HIVE could still tip: nothing confirmed it at rest).
     */
    @Test
    public void theLaneIsAnErrorByDefaultAndOnlyAWarningWithTheFlag() {
        List<Issue> off = validate(laneRoute(64.0, false));
        assertEquals(off.toString(), 1, count(off, Rule.HIVE_KEEPOUT, Severity.ERROR));
        for (Issue i : off) {
            if (i.rule == Rule.HIVE_KEEPOUT) {
                assertEquals("the lane leg is step 3", 2, i.stepIndex);
                assertTrue(i.message, i.message.contains("allowUnderHive"));
            }
        }
        List<Issue> on = validate(laneRoute(64.0, true));
        assertEquals(on.toString(), 0, count(on, Rule.HIVE_KEEPOUT, Severity.ERROR));
        assertEquals(on.toString(), 1, count(on, Rule.UNDER_HIVE, Severity.WARN));
        assertEquals("nothing else is wrong with it", 1, on.size());
        Issue warn = on.get(0);
        assertEquals(2, warn.stepIndex);
        assertTrue(warn.message, warn.message.contains("could still tip") && warn.message.contains("WAIT_FOR_TIP"));
    }

    /**
     * Situation: the lane leg at the margin: the robot's red-side edge exactly 3.0 in from the leg stubs, then 0.1 in closer, then 0.1 in farther.
     * Expected: exactly at the margin is fine, 0.1 in closer is a HIVE ERROR that quotes the lane and its margin, 0.1 in farther is fine.
     * Why: the lane is checked against the exact stubs with the margin, not against the box.
     */
    @Test
    public void theLaneNeedsItsMarginOnTheLeftAndTheTouchingAndCloserIsCaught() {
        assertEquals(0, count(validate(laneRoute(LIMIT_X, true)), Rule.HIVE_KEEPOUT, Severity.ERROR));
        assertEquals(0, count(validate(laneRoute(LIMIT_X + 0.1, true)), Rule.HIVE_KEEPOUT, Severity.ERROR));
        List<Issue> close = validate(laneRoute(LIMIT_X - 0.1, true));
        assertEquals(close.toString(), 1, count(close, Rule.HIVE_KEEPOUT, Severity.ERROR));
        for (Issue i : close) {
            if (i.rule == Rule.HIVE_KEEPOUT) {
                assertTrue(i.message, i.message.contains("foot bars or leg stubs") && i.message.contains("the lane is"));
            }
        }
        // the approach to the lane axis must keep the margin too: ending the first leg 0.1 in closer is the first leg's error
        assertTrue(LIMIT_X > 60.0 && LIMIT_X < BioBuzzConstants.Field.CENTER_IN);
    }

    /**
     * Situation: a leg that crosses a foot bar (x = 45), and one that drives along the INSIDE of the bar at x = 53 (edge 44.1 in: across the bar).
     * Expected: both are HIVE ERRORs even with the flag: the foot bars are a 2.15 in step the robot cannot cross, and the legs are solid.
     */
    @Test
    public void theFlagNeverLetsARobotOverAFootBarOrIntoALeg() {
        for (double x : new double[] {45.0, 53.0}) {
            List<Issue> l = validate(laneRoute(x, true));
            assertTrue("x = " + x + ": " + l, count(l, Rule.HIVE_KEEPOUT, Severity.ERROR) >= 1);
        }
    }

    /**
     * Situation: the flag on the lane leg only; the two legs around it have none.
     * Expected: the legs before and after stay out of the box: a leg that ENTERS the footprint before the lane leg starts is an ERROR, and so is a
     *     leg after it that runs through the frame; the flag changes nothing but the leg it is on.
     */
    @Test
    public void theFlagIsOnlyForTheLegItIsOn() {
        // the approach leg runs from the start to (64, 60): it enters the footprint keep-out by itself
        AutoRoute.Builder b = AutoRoute.builder("x").start(58.0, FRONT, -90)
                .driveConstant(64.0, 60.0, -90).label("too far in")
                .driveConstant(64.0, 105.0, -90).label("the lane").allowUnderHive();
        List<Issue> l = validate(b);
        boolean firstLegFlagged = false;
        for (Issue i : l) {
            firstLegFlagged |= i.rule == Rule.HIVE_KEEPOUT && i.severity == Severity.ERROR && i.stepIndex == 1;
        }
        assertTrue("the leg without the flag keeps out of the footprint: " + l, firstLegFlagged);
        // the same lane leg followed by one that runs back through the frame
        AutoRoute.Builder c = laneRoute(64.0, true).driveConstant(58.0, 40.0, -90).label("back through");
        List<Issue> m = validate(c);
        boolean backFlagged = false;
        for (Issue i : m) {
            backFlagged |= i.rule == Rule.HIVE_KEEPOUT && i.severity == Severity.ERROR && i.stepIndex == 4;
        }
        assertTrue(m.toString(), backFlagged);
    }

    /**
     * Situation: the HIVE at rest or not: (a) a route with a SHOOT before the lane leg, (b) a partner route whose WAIT_FOR_TIP comes before it,
     *     (c) the same with a SHOOT between the wait and the lane, (d) no wait at all.
     * Expected: (a) and (d) WARN, (b) does not (the camera has confirmed the tip and nothing went in since), (c) WARNs again (a volley can tip it).
     * Why: the only moment the HIVE is known to be still is after the camera has seen it tip fully and before the next volley.
     */
    @Test
    public void onlyAConfirmedRestingHiveIsSafeToDriveUnder() {
        // (a) shoot at the cell first: the volley is what tips the HIVE
        List<Issue> a = validate(AutoRoute.builder("x").start(58.0, FRONT, -90).shootUp(58.0, FRONT, -90, 4)
                .driveConstant(64.0, 38.0, -90).driveConstant(64.0, 105.0, -90).allowUnderHive().driveConstant(58.0, 120.0, -90));
        assertEquals(a.toString(), 1, count(a, Rule.UNDER_HIVE, Severity.WARN));
        // (b) the partner tips it, the camera confirms it, then we drive under it
        AutoRoute.Builder partner = AutoRoute.builder("x").start(58.0, FRONT, -90).partnerTipsFirst().waitForTip(20.0)
                .waitSeconds(BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC)
                .driveConstant(64.0, 38.0, -90).driveConstant(64.0, 105.0, -90).allowUnderHive().driveConstant(58.0, 120.0, -90);
        List<Issue> b = validate(partner);
        assertEquals(b.toString(), 0, count(b, Rule.UNDER_HIVE, Severity.WARN));
        // (c) a volley between the wait and the lane makes the HIVE able to tip again
        AutoRoute.Builder volley = AutoRoute.builder("x").start(58.0, FRONT, -90).partnerTipsFirst().waitForTip(20.0)
                .shootUp(58.0, FRONT, -90, 4).waitSeconds(BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC)
                .driveConstant(64.0, 38.0, -90).driveConstant(64.0, 105.0, -90).allowUnderHive().driveConstant(58.0, 120.0, -90);
        List<Issue> c = validate(volley);
        assertEquals(c.toString(), 1, count(c, Rule.UNDER_HIVE, Severity.WARN));
        // (d) with no wait at all, and a lane leg that does not reach the frame, there is nothing to warn about
        List<Issue> d = validate(AutoRoute.builder("x").start(58.0, FRONT, -90).driveConstant(64.0, 38.0, -90).allowUnderHive());
        assertEquals(d.toString(), 0, count(d, Rule.UNDER_HIVE, Severity.WARN));
    }

    /**
     * Situation: a leg with the flag that is under the HIVE while a tip of the partner's is assumed (the partner is assumed to tip at 12 s).
     * Expected: the WARN says that the tip is assumed during the move.
     */
    @Test
    public void aTipDuringTheMoveIsNamedInTheWarning() {
        // the robot waits 6 s, then drives; the lane leg runs from about 7 to 13 s, which includes the assumed partner tip at 12 s
        AutoRoute.Builder b = AutoRoute.builder("x").start(58.0, FRONT, -90).partnerTipsFirst().waitSeconds(6.0)
                .driveConstant(64.0, 38.0, -90).driveConstant(64.0, 105.0, -90).allowUnderHive().speed(0.3);
        List<Issue> l = validate(b);
        boolean named = false;
        for (Issue i : l) {
            named |= i.rule == Rule.UNDER_HIVE && i.message.contains("tip is assumed at 12.0 s, during this move");
        }
        assertTrue(l.toString(), named);
    }

    /**
     * Situation: the flag through the builder and the half turn.
     * Expected: it is false by default, true after {@code allowUnderHive()}, survives the copies ({@code via}, {@code speed}, {@code label},
     *     {@code wallContact}, {@code note}) and the half turn for BLUE, shows in {@code describe()}, and cannot be put on a step that is not a DRIVE.
     */
    @Test
    public void theFlagIsPartOfTheStepAndSurvivesCopiesAndTheHalfTurn() {
        AutoRoute plain = AutoRoute.builder("x").start(58.0, FRONT, -90).driveConstant(64.0, 38.0, -90).build();
        assertFalse(((Step.Drive) plain.step(1)).allowsUnderHive());
        assertFalse(plain.step(1).describe(), plain.step(1).describe().contains("HIVE"));
        AutoRoute flagged = AutoRoute.builder("x").start(58.0, FRONT, -90).driveConstant(64.0, 38.0, -90).allowUnderHive().via(60.0, 20.0)
                .speed(0.5).label("lane").wallContact().note("because").build();
        Step.Drive drive = (Step.Drive) flagged.step(1);
        assertTrue(drive.allowsUnderHive());
        assertEquals("because", drive.note());
        assertTrue(drive.describe(), drive.describe().contains("may go under the HIVE"));
        assertTrue(((Step.Drive) flagged.forAlliance(Alliance.BLUE).step(1)).allowsUnderHive());
        assertTrue(((Step.Drive) flagged.forAlliance(Alliance.BLUE).forAlliance(Alliance.RED).step(1)).allowsUnderHive());
        assertTrue(((Step.Drive) drive.withNote("other")).allowsUnderHive());
        assertFalse(drive.withAllowUnderHive(false).allowsUnderHive());
        try {
            AutoRoute.builder("x").start(58.0, FRONT, -90).waitSeconds(1).allowUnderHive();
            fail("only a DRIVE can go under the HIVE");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("DRIVE"));
        }
    }

    /**
     * Situation: the way from the audience start to a pose beyond the HIVE, the lane against the left corridor around the frame.
     * Expected: the lane is more than 15 in (about 20 in) shorter and about half a second quicker at the cruise speed; both routes pass the
     *     HIVE check (the lane one only with the flag). The test prints the numbers quoted in docs/AUTOS.md.
     */
    @Test
    public void theLaneIsAboutTwentyInchesShorterThanGoingAround() {
        AutoRoute.Builder lane = AutoRoute.builder("lane").start(58.0, FRONT, -90)
                .driveConstant(64.0, 38.0, -90).driveConstant(64.0, 105.0, -90).allowUnderHive().driveConstant(58.0, 125.0, -90);
        AutoRoute.Builder around = AutoRoute.builder("around").start(58.0, FRONT, -90)
                .driveConstant(30.0, 45.0, -90).driveConstant(30.0, 105.0, -90).driveConstant(58.0, 125.0, -90);
        AutoRoute laneRoute = lane.build();
        AutoRoute aroundRoute = around.build();
        assertEquals(0, count(RouteValidator.check(laneRoute), Rule.HIVE_KEEPOUT, Severity.ERROR));
        assertEquals(0, count(RouteValidator.check(aroundRoute), Rule.HIVE_KEEPOUT, Severity.ERROR));
        double laneIn = length(laneRoute);
        double aroundIn = length(aroundRoute);
        double laneSec = laneRoute.estimatedSeconds();
        double aroundSec = aroundRoute.estimatedSeconds();
        System.out.println(String.format(Locale.ROOT, "LANE vs AROUND from the audience start to a pose beyond the HIVE: lane %.1f in (%.1f s), around %.1f in (%.1f s): "
                + "%.1f in and %.1f s saved", laneIn, laneSec, aroundIn, aroundSec, aroundIn - laneIn, aroundSec - laneSec));
        assertTrue("the lane saves " + (aroundIn - laneIn) + " in", aroundIn - laneIn > 15.0 && aroundIn - laneIn < 30.0);
        assertTrue("and time: " + (aroundSec - laneSec) + " s", aroundSec - laneSec > 0.3);
    }

    private static double length(AutoRoute route) {
        double sum = 0.0;
        for (RouteAnalysis.StepInfo info : route.analyze().steps()) {
            if (info.path != null) {
                sum += info.path.length();
            }
        }
        return sum;
    }

    /**
     * Situation: the default validator on a robot driving up the lane without the flag, for every x across the whole footprint.
     * Expected: always a HIVE ERROR: nothing but the flag lets a robot under the HIVE (the old, safe behaviour is the default).
     */
    @Test
    public void withoutTheFlagNoXAcrossTheFrameIsAllowed() {
        for (double x = 20.0; x <= 70.0; x += 2.5) {
            List<Issue> l = RouteValidator.check(AutoRoute.builder("x").start(x, FRONT, -90).driveConstant(x, 105.0, -90).build());
            boolean through = x > 34.0;       // a robot at x <= 34.04 passes the keep-out on the red-wall side (its edge is then 3.0 in clear)
            assertEquals("x = " + x + ": " + l, through, count(l, Rule.HIVE_KEEPOUT, Severity.ERROR) >= 1);
        }
    }
}
