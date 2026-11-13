package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.junit.Test;

/** Sanity of the timing estimate: physically plausible, monotone and driven by the constants. */
public class TimingModelTest {
    private static final double EPS = 1e-9;
    private final TimingModel model = new TimingModel();
    private final double v = BioBuzzConstants.Auto.CRUISE_SPEED_IN_PER_SEC;
    private final double a = BioBuzzConstants.Auto.ACCEL_IN_PER_SEC2;

    @Test
    public void aLongLegIsRampUpPlusCruisePlusRampDown() {
        double d = 100.0;
        assertEquals(d / v + v / a, model.translationSeconds(d, 1.0), EPS);
    }

    @Test
    public void aShortLegNeverReachesCruiseSpeed() {
        double d = 10.0;
        assertTrue("test premise: leg shorter than the ramp distance", d < v * v / a);
        assertEquals(2.0 * Math.sqrt(d / a), model.translationSeconds(d, 1.0), EPS);
    }

    @Test
    public void theTwoFormulasMeetAtTheRampDistance() {
        double d = v * v / a;
        double below = model.translationSeconds(d - 1e-6, 1.0);
        double above = model.translationSeconds(d + 1e-6, 1.0);
        assertEquals(below, above, 1e-4);
    }

    @Test
    public void longerIsSlowerAndSlowerSpeedFractionIsSlower() {
        double last = 0;
        for (double d = 1; d <= 200; d += 3) {
            double t = model.translationSeconds(d, 1.0);
            assertTrue("monotone in distance at " + d, t > last);
            last = t;
        }
        assertTrue(model.translationSeconds(80, 0.5) > model.translationSeconds(80, 1.0));
        assertEquals(0.0, model.translationSeconds(0.0, 1.0), EPS);
    }

    @Test
    public void turningUsesTheTurnRate() {
        double rate = BioBuzzConstants.Auto.TURN_RATE_DEG_PER_SEC;
        assertEquals(90.0 / rate, model.turnSeconds(Math.toRadians(90)), EPS);
        assertEquals(90.0 / rate, model.turnSeconds(Math.toRadians(-90)), EPS);
    }

    @Test
    public void aLegCostsTheLongerOfDrivingAndTurningPlusSettle() {
        double settle = BioBuzzConstants.Auto.SETTLE_SEC;
        // straight, no turn
        DrivePath straight = DrivePath.of(Pose2.ofDegrees(0, 0, 0),
                (Step.Drive) AutoRoute.builder("a").start(0, 0, 0).driveTangent(100, 0).build().step(1));
        assertEquals(model.translationSeconds(100, 1.0) + settle, model.legSeconds(straight, 1.0), EPS);

        // a 2 in leg with a 91 degree turn: the turn dominates
        DrivePath turning = DrivePath.of(Pose2.ofDegrees(62, 118.1, -91.1),
                (Step.Drive) AutoRoute.builder("a").start(0, 0, 0).driveReverseTangent(60, 118.1, -90).build().step(1));
        double turn = model.turnSeconds(turning.totalTurnRadians());
        assertTrue(turn > model.translationSeconds(2.0, 1.0));
        assertEquals(turn + settle, model.legSeconds(turning, 1.0), EPS);
    }

    @Test
    public void aLegThatDoesNotMoveCostsNothing() {
        DrivePath none = DrivePath.of(Pose2.ofDegrees(5, 5, 90),
                (Step.Drive) AutoRoute.builder("a").start(0, 0, 0).driveLinear(5, 5, 90).build().step(1));
        assertEquals(0.0, model.legSeconds(none, 1.0), EPS);
        assertEquals(0.0, model.moveSeconds(Pose2.ofDegrees(5, 5, 90), Pose2.ofDegrees(5, 5, 90)), EPS);
    }

    @Test
    public void anImplicitMoveIncludesTheTurnInPlace() {
        double settle = BioBuzzConstants.Auto.SETTLE_SEC;
        // B1 hold: same place, -90 -> -115 degrees = 25 degrees
        double t = model.moveSeconds(Pose2.ofDegrees(35, 9.2, -90), Pose2.ofDegrees(35, 9.2, -115));
        assertEquals(model.turnSeconds(Math.toRadians(25)) + settle, t, 1e-9);
    }

    @Test
    public void shootCostsAimPlusFeedPerPollenUnlessItHasAFixedDuration() {
        Step.Shoot four = (Step.Shoot) AutoRoute.builder("a").start(0, 0, 0).shootUp(4).build().step(1);
        Step.Shoot eight = (Step.Shoot) AutoRoute.builder("a").start(0, 0, 0).shootUp(8).build().step(1);
        assertEquals(BioBuzzConstants.Auto.AIM_SPINUP_SEC + 4 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC,
                model.shootSeconds(four), EPS);
        assertEquals(4 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC,
                model.shootSeconds(eight) - model.shootSeconds(four), EPS);
        Step.Shoot fixed = four.withFixedSeconds(5.0);
        assertEquals(5.0, model.shootSeconds(fixed), EPS);
    }

    @Test
    public void aVolleyThatStopsAtTheTipOnlyPaysForThePollenItFires() {
        // a camera-gated volley of 4 at the cell that holds the staged NECTAR tips on the 3rd POLLEN and stops there
        Step.Shoot four = (Step.Shoot) AutoRoute.builder("a").start(0, 0, 0).shootUp(4).build().step(1);
        assertEquals(BioBuzzConstants.Auto.AIM_SPINUP_SEC + 3 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC, model.shootSeconds(four, 3), EPS);
        assertEquals("all of them is the whole step", model.shootSeconds(four), model.shootSeconds(four, 4), EPS);
        // a timed volley is spread evenly over its POLLEN: the 3rd of 4 leaves three quarters of the way through
        Step.Shoot fixed = four.withFixedSeconds(8.0);
        assertEquals(6.0, model.shootSeconds(fixed, 3), EPS);
        assertEquals(8.0, model.shootSeconds(fixed, 4), EPS);
    }

    @Test
    public void intakeAndWaitCostTheirData() {
        Step.Intake in = (Step.Intake) AutoRoute.builder("a").start(0, 0, 0)
                .intake(IntakeSource.GARDEN, 5, 1.4, 9, 9.7, -90, 3.25, 4).build().step(1);
        Step.Wait w = (Step.Wait) AutoRoute.builder("a").start(0, 0, 0).waitSeconds(4.5).build().step(1);
        assertEquals(3.25, model.intakeSeconds(in), EPS);
        assertEquals(4.5, model.waitSeconds(w), EPS);
    }

    @Test
    public void distanceAfterFollowsTheSameSpeedProfileAsTheTravelTime() {
        for (double distance : new double[] {10.0, 100.0}) {      // 10 in is a triangle profile, 100 in has a cruise phase
            double total = model.translationSeconds(distance, 1.0);
            assertEquals("at rest at the start", 0.0, model.distanceAfter(distance, 1.0, 0.0), EPS);
            assertEquals("arrives exactly when the travel time is up", distance, model.distanceAfter(distance, 1.0, total), 1e-6);
            assertEquals("and stays there", distance, model.distanceAfter(distance, 1.0, total + 5.0), EPS);
            assertEquals("the profile is symmetric: half the distance at half the time", distance / 2.0,
                    model.distanceAfter(distance, 1.0, total / 2.0), 1e-6);
            double previous = -1.0;
            for (int i = 0; i <= 100; i++) {
                double d = model.distanceAfter(distance, 1.0, total * i / 100.0);
                assertTrue("never goes backwards: " + d + " after " + previous, d >= previous - 1e-9);
                previous = d;
            }
        }
        // the cruise phase moves at the cruise speed: 1 s in the middle of a long path is cruise-speed inches
        double mid = model.translationSeconds(200, 1.0) / 2.0;
        assertEquals(v, model.distanceAfter(200, 1.0, mid + 0.5) - model.distanceAfter(200, 1.0, mid - 0.5), 1e-6);
        // and speeding up at the acceleration: after 0.5 s from rest the robot has gone a/2 x 0.25 in
        assertEquals(0.5 * BioBuzzConstants.Auto.ACCEL_IN_PER_SEC2 * 0.25, model.distanceAfter(200, 1.0, 0.5), 1e-9);
        // a slower leg takes longer and is behind the faster one at the same moment
        assertTrue(model.distanceAfter(100, 0.5, 2.0) < model.distanceAfter(100, 1.0, 2.0));
    }

    @Test
    public void aWaitForTipLastsUntilTheTipIsConfirmedButNeverBeyondItsGiveUpTimeOnTheRouteClock() {
        double settle = model.tipSettleSec();
        assertEquals("the settle time is the one the camera tracker uses", BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC, settle, 0.0);
        assertTrue(settle > 0.0);
        // the tip is at 12 s: starting at 3 s the wait lasts until the camera has seen the cell steady for the settle time...
        assertEquals(9.0 + settle, model.waitForTipSeconds(3.0, 12.0, 30.0), EPS);
        assertEquals(12.0 + settle, model.confirmedAtSec(12.0), EPS);
        // ...a give-up time of 7 s (a point on the route clock, not a length) ends it after 4 s...
        assertEquals(4.0, model.waitForTipSeconds(3.0, 12.0, 7.0), EPS);
        // ...a give-up time that falls inside the settle time cuts the wait too: the tip is not confirmed yet, so the volley is skipped...
        assertEquals(9.0 + settle / 2.0, model.waitForTipSeconds(3.0, 12.0, 12.0 + settle / 2.0), EPS);
        // ...a give-up time that has passed already means no waiting at all, never a negative time...
        assertEquals(0.0, model.waitForTipSeconds(15.0, 20.0, 10.0), EPS);
        // ...a tip that has been confirmed already costs nothing...
        assertEquals(0.0, model.waitForTipSeconds(15.0, 12.0, 30.0), EPS);
        assertEquals(0.0, model.waitForTipSeconds(12.0 + settle, 12.0, 30.0), EPS);
        // ...but one that is over and not yet settled still costs the rest of the settle time
        assertEquals(settle / 2.0, model.waitForTipSeconds(12.0 + settle / 2.0, 12.0, 30.0), EPS);
        // ...and with no tip to wait for (NaN) the cell is UP already: no waiting
        assertEquals(0.0, model.waitForTipSeconds(3.0, Double.NaN, 30.0), EPS);
    }

    @Test
    public void aLongerSettleTimeMakesTheWaitLonger() {
        TimingModel.Params slow = new TimingModel.Params();
        slow.tipSettleSec = 2.0;
        assertEquals(11.0, new TimingModel(slow).waitForTipSeconds(3.0, 12.0, 30.0), EPS);
    }

    @Test
    public void theModelFollowsItsParameters() {
        TimingModel.Params fast = new TimingModel.Params();
        fast.cruiseInPerSec = v * 2;
        TimingModel faster = new TimingModel(fast);
        assertTrue(faster.translationSeconds(100, 1.0) < model.translationSeconds(100, 1.0));
        TimingModel.Params slowTurn = new TimingModel.Params();
        slowTurn.turnRateDegPerSec = 45;
        assertEquals(2.0, new TimingModel(slowTurn).turnSeconds(Math.toRadians(90)), EPS);
    }

    // ---- the FLOWER wiggle ----------------------------------------------------------------------------------------

    private static FlowerPress pickUp(int pollen) {
        return FlowerPress.plan(new AllianceField(new FieldModel(), Alliance.RED), WiggleTestRobot.spec(), 0,
                IntakeWheel.LEFT, IntakeWheel.LEFT, pollen,
                BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN);
    }

    @Test
    public void theWiggleCostsOneCycleOfTwoTurnsAndTwoHoldsForEveryPollen() {
        double halfTurn = Math.max(BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_SEC, model.turnSeconds(pickUp(1).turnRad()));
        double cycle = 2.0 * halfTurn + BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC + BioBuzzConstants.Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC;
        assertEquals("a half turn is the 0.35 s the constants say: the drive turns 17 degrees in much less", BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_SEC, halfTurn, EPS);
        for (int pollen = 1; pollen <= 4; pollen++) {
            assertEquals(pollen + " POLLEN", pollen * cycle, model.wiggleSeconds(pickUp(pollen)), EPS);
            assertEquals(cycle, model.wiggleCycleSeconds(pickUp(pollen)), EPS);
        }
        // never more cycles than the cap, however many POLLEN are asked for
        assertEquals(BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_CYCLES * cycle, model.wiggleSeconds(pickUp(40)), EPS);
    }

    @Test
    public void aSlowTurnRateLengthensTheHalfTurnBeyondTheConstant() {
        TimingModel.Params slow = new TimingModel.Params();
        slow.turnRateDegPerSec = 20.0;                         // 17 degrees take 0.84 s, more than the 0.35 s of the constant
        TimingModel slowModel = new TimingModel(slow);
        FlowerPress p = pickUp(1);
        double half = Math.abs(p.turnDeg()) / 20.0;
        assertTrue(half > BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_SEC);
        assertEquals(2.0 * half + BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC + BioBuzzConstants.Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC,
                slowModel.wiggleSeconds(p), EPS);
    }

    @Test
    public void theHoldsAndTheTurnTimeComeFromTheParameters() {
        TimingModel.Params longer = new TimingModel.Params();
        longer.wigglePushHoldSec = 1.0;
        longer.wiggleReleaseHoldSec = 2.0;
        longer.wiggleTurnSec = 0.5;
        assertEquals(2.0 * 0.5 + 1.0 + 2.0, new TimingModel(longer).wiggleSeconds(pickUp(1)), EPS);
    }

    @Test
    public void aFlowerPickUpIsTheSlowApproachTheWiggleAndTheSlowRetreat() {
        FlowerPress p = pickUp(3);
        double slow = BioBuzzConstants.Auto.FLOWER_APPROACH_SPEED_FRACTION;
        assertEquals(model.legSeconds(p.approach, slow) + model.wiggleSeconds(p) + model.legSeconds(p.retreatPath, slow), model.flowerSeconds(p), EPS);
        // more cycles per POLLEN make it longer, and only the wiggle part grows
        FlowerPress twice = FlowerPress.plan(new AllianceField(new FieldModel(), Alliance.RED), WiggleTestRobot.spec().withCycles(2, 6), 0,
                IntakeWheel.LEFT, IntakeWheel.LEFT, 3,
                BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN);
        assertEquals(6, twice.cycles);
        assertEquals(2.0 * model.wiggleSeconds(p), model.wiggleSeconds(twice), EPS);
        assertEquals(model.flowerSeconds(p) + model.wiggleSeconds(p), model.flowerSeconds(twice), EPS);
    }

    @Test
    public void aModelForOtherWheelsKeepsEverythingElseAndTheOriginalIsUnchanged() {
        TimingModel longer = model.withFlowerSpec(model.flowerSpec().withIntakeReach(5.0));
        assertEquals(5.0, longer.flowerSpec().intakeReachIn, EPS);
        assertEquals("the original is untouched", BioBuzzConstants.Robot.INTAKE_REACH_IN, model.flowerSpec().intakeReachIn, EPS);
        assertEquals(model.translationSeconds(100, 1.0), longer.translationSeconds(100, 1.0), EPS);
        assertEquals(model.tipSettleSec(), longer.tipSettleSec(), EPS);
    }
}
