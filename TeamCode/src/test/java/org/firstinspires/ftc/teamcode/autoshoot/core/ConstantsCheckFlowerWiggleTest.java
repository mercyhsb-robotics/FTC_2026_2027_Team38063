package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The checks of {@link ConstantsCheck} on the numbers of the FLOWER wiggle (the {@code Auto.FLOWER_*} constants): the shipped values are fine, and values that
 * cannot work together are reported at INIT, before the robot is switched on.
 *
 * <p>How it works: the shipped numbers go through the checker first (no complaint), then the same checker runs on numbers written in the test, each wrong in one way.
 * Every negative case would pass silently if the check it targets were removed. The values are plain parameters of the checker so a test can feed in bad ones.
 *
 * <p>Words used: the <b>start offset</b> is how far the middle of the working wheel stands from the middle of the POLLEN when the wiggle starts; the <b>press offset</b>
 * is how far it is at the end of the push; two things whose middles are the sum of their radii apart just touch.
 */
public class ConstantsCheckFlowerWiggleTest {
    /** The numbers the checker takes, starting from the shipped ones; a test changes the one it is about. */
    private static final class Values {
        double wheelDiameter = BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN;
        double pollenDiameter = BioBuzzConstants.Auto.POLLEN_DIAMETER_IN;
        double start = BioBuzzConstants.Auto.FLOWER_WHEEL_START_OFFSET_IN;
        double press = BioBuzzConstants.Auto.FLOWER_WHEEL_PRESS_OFFSET_IN;
        double maxTurnDeg = BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_TURN_DEG;
        double pushHold = BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC;
        double releaseHold = BioBuzzConstants.Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC;
        int cyclesPerPollen = BioBuzzConstants.Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN;
        int maxCycles = BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_CYCLES;
        double turnSec = BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_SEC;
        double timeout = BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC;
        double tolDeg = BioBuzzConstants.Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG;
        double approachLen = BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN;
        double approachSpeed = BioBuzzConstants.Auto.FLOWER_APPROACH_SPEED_FRACTION;
        double retreatLen = BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN;
        double clearance = BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN;

        ConstantsCheck.Report check() {
            ConstantsCheck.Report r = new ConstantsCheck.Report();
            ConstantsCheck.checkFlowerWiggle(r, wheelDiameter, pollenDiameter, start, press, maxTurnDeg, pushHold, releaseHold, cyclesPerPollen, maxCycles,
                    turnSec, timeout, tolDeg, approachLen, approachSpeed, retreatLen, clearance);
            return r;
        }
    }

    private static void assertOneError(String needle, ConstantsCheck.Report r) {
        assertEquals(r.errors.toString() + r.warnings, 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains(needle));
    }

    @Test
    public void theShippedWiggleNumbersAreAcceptedWithoutAComplaint() {
        ConstantsCheck.Report r = new Values().check();
        assertTrue(r.errors.toString(), r.errors.isEmpty());
        assertTrue(r.warnings.toString(), r.warnings.isEmpty());
        // and the whole shipped constants file, with them
        ConstantsCheck.Report all = ConstantsCheck.run();
        assertTrue(all.errors.toString(), all.ok());
        assertTrue(all.warnings.toString(), all.warnings.isEmpty());
    }

    @Test
    public void thePlaceholdersTheWiggleNeedsAreListedForTheTeamToMeasure() {
        java.util.List<String> listed = java.util.Arrays.asList(BioBuzzConstants.PLACEHOLDERS);
        for (String name : new String[] {"Auto.FLOWER_WHEEL_START_OFFSET_IN", "Auto.FLOWER_WHEEL_PRESS_OFFSET_IN", "Auto.FLOWER_WHEEL_BALL_INBOARD",
                "Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC", "Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC", "Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN",
                "Auto.FLOWER_WIGGLE_TURN_SEC", "Auto.FLOWER_CHASSIS_CLEARANCE_IN", "Robot.INTAKE_REACH_IN", "Robot.INTAKE_WHEEL_OFFSET_IN"}) {
            assertTrue(name + " is an estimate and must be on the list", listed.contains(name));
        }
        // the numbers of the old straight press-in are gone, so they cannot be listed or measured by mistake
        for (String gone : new String[] {"Auto.FLOWER_PRESS_DEPTH_IN", "Auto.INTAKE_DWELL_FLOWER_SEC", "Auto.FLOWER_HEADING_TOLERANCE_DEG", "Auto.FLOWER_ALIGN_TOLERANCE_IN"}) {
            assertTrue(gone + " is replaced by the wiggle", !listed.contains(gone));
        }
    }

    @Test
    public void aPressOffsetThatOnlyTouchesTheBallIsAnError() {
        Values v = new Values();
        v.press = v.wheelDiameter / 2.0 + v.pollenDiameter / 2.0;       // 2.8 in: the two just touch, nothing is pressed
        assertOneError("only touch the ball", v.check());
        v.press = -1.0;
        assertOneError("must be positive", v.check());
    }

    @Test
    public void aStartOffsetNoBiggerThanThePressOffsetIsAnErrorAndOneThatWouldTouchTheBallOnTheWayInIsAWarning() {
        Values v = new Values();
        v.start = v.press;
        assertOneError("must be more than FLOWER_WHEEL_PRESS_OFFSET_IN", v.check());
        v = new Values();
        v.start = 2.0;                                                    // between the press offset (1.8) and the two radii (2.8): it would drag the ball
        ConstantsCheck.Report r = v.check();
        assertTrue(r.errors.toString(), r.errors.isEmpty());
        assertEquals(r.warnings.toString(), 1, r.warnings.size());
        assertTrue(r.warnings.get(0), r.warnings.get(0).contains("already touch the ball on the way in"));
    }

    @Test
    public void theTurnLimitAndTheHeadingToleranceMustMakeSense() {
        Values v = new Values();
        v.maxTurnDeg = 0.0;
        assertOneError("MAX_TURN_DEG", v.check());
        v.maxTurnDeg = 60.0;
        assertOneError("at most 45 degrees", v.check());
        v = new Values();
        v.tolDeg = v.maxTurnDeg;                                          // the tolerance as big as the whole turn: any heading counts as arrived
        assertOneError("HEADING_TOLERANCE_DEG", v.check());
        v.tolDeg = 0.0;
        assertOneError("HEADING_TOLERANCE_DEG", v.check());
    }

    @Test
    public void theTimesAndTheCyclesMustMakeSense() {
        Values v = new Values();
        v.pushHold = -0.1;
        assertOneError("cannot be negative", v.check());
        v = new Values();
        v.turnSec = 0.0;
        assertOneError("TURN_SEC must be positive", v.check());
        v = new Values();
        v.timeout = v.turnSec;                                            // the timeout is not longer than a normal turn: every turn would be called stuck
        ConstantsCheck.Report r = v.check();
        assertTrue(r.errors.isEmpty());
        assertEquals(r.warnings.toString(), 1, r.warnings.size());
        assertTrue(r.warnings.get(0), r.warnings.get(0).contains("would be called stuck"));
        v = new Values();
        v.cyclesPerPollen = 0;
        assertOneError("CYCLES_PER_POLLEN", v.check());
        v = new Values();
        v.cyclesPerPollen = 4;
        v.maxCycles = 3;
        assertOneError("CYCLES_PER_POLLEN", v.check());
    }

    /**
     * Situation: 2 cycles per POLLEN (a plausible day-one measurement) with the shipped cap of 6 cycles; a whole FLOWER (4 POLLEN) needs 8.
     * Expected: an ERROR that says the cap is less than a whole FLOWER needs and what to raise it to; with a cap of 8 there is none.
     * Why: the cap used to be compared only with the cycles of ONE POLLEN, so 2 / 6 passed and a pick-up of 4 POLLEN silently delivered 3.
     */
    @Test
    public void theCycleLimitMustBeEnoughForAWholeFlower() {
        Values v = new Values();
        v.cyclesPerPollen = 2;
        v.maxCycles = 6;
        assertOneError("less than a whole FLOWER needs", v.check());
        assertTrue(v.check().errors.get(0), v.check().errors.get(0).contains("at least 8"));
        v.maxCycles = 8;
        assertTrue(v.check().errors.toString(), v.check().errors.isEmpty());
        // the shipped numbers (1 per POLLEN, cap 6) have room for the 4 POLLEN of a FLOWER
        assertTrue(new Values().check().errors.isEmpty());
    }

    @Test
    public void theApproachTheRetreatTheSpeedAndTheClearanceMustMakeSense() {
        Values v = new Values();
        v.approachLen = 0.0;
        assertOneError("must be positive", v.check());
        v = new Values();
        v.approachSpeed = 1.5;
        assertOneError("must be in (0, 1]", v.check());
        v = new Values();
        v.clearance = -0.1;
        assertOneError("cannot be negative", v.check());
    }
}
