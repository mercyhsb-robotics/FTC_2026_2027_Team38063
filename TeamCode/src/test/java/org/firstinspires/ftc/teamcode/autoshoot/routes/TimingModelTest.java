package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
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
    public void aWaitForTipLastsUntilTheTipButNeverBeyondItsTimeout() {
        // the tip is at 12 s: starting at 3 s the wait is 9 s...
        assertEquals(9.0, model.waitForTipSeconds(3.0, 12.0, 30.0), EPS);
        // ...a timeout shorter than that caps it...
        assertEquals(4.0, model.waitForTipSeconds(3.0, 12.0, 4.0), EPS);
        // ...a tip that is already over costs nothing...
        assertEquals(0.0, model.waitForTipSeconds(15.0, 12.0, 30.0), EPS);
        assertEquals(0.0, model.waitForTipSeconds(12.0, 12.0, 30.0), EPS);
        // ...and with no tip to wait for (NaN) the cell is UP already: no waiting
        assertEquals(0.0, model.waitForTipSeconds(3.0, Double.NaN, 30.0), EPS);
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
}
