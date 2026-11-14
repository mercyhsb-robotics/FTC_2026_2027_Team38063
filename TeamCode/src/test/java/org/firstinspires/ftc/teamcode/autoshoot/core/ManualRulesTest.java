package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Checks the manual controls' decisions (which of shoot, intake and clear-a-jam really happens when buttons are held
 * together) and their settings. Run with {@code ./gradlew :TeamCode:testDebugUnitTest}.
 *
 * <p>The motors themselves cannot be tested here (see {@code LauncherBrakeTest} for the flywheel stop); these tests
 * protect the rules: a jam button always works, shooting never runs together with intaking, and when nothing is held
 * nothing runs (the flywheel is told to stop, which brakes it).
 */
public class ManualRulesTest {

    /**
     * Situation: all 64 combinations of the six inputs (shoot trigger, intake trigger, indexer jam button, shooter jam button, roller
     * reverse button, feelers reverse button).
     * Expected: the answer never breaks the rules below, for every combination.
     * Why: it is the whole truth table, so no combination of held buttons can start two conflicting jobs or lose a reverse button.
     */
    @Test
    public void everyCombinationFollowsThePriorityRules() {
        for (int bits = 0; bits < 64; bits++) {
            boolean shootAsked = (bits & 1) != 0;
            boolean intakeAsked = (bits & 2) != 0;
            boolean indexerJam = (bits & 4) != 0;
            boolean shooterJam = (bits & 8) != 0;
            boolean rollerReverse = (bits & 16) != 0;
            boolean feelerReverse = (bits & 32) != 0;
            ManualRules.Decision d = ManualRules.decide(shootAsked, intakeAsked, indexerJam, shooterJam, rollerReverse, feelerReverse);
            String what = "shoot=" + shootAsked + " intake=" + intakeAsked + " indexerJam=" + indexerJam + " shooterJam=" + shooterJam
                    + " rollerReverse=" + rollerReverse + " feelerReverse=" + feelerReverse;
            boolean clearing = indexerJam || shooterJam;
            // Every reverse button always works, whatever else is held.
            assertEquals(what + ": the indexer reverse button always works", indexerJam, d.reverseIndexer);
            assertEquals(what + ": the shooter reverse button always works", shooterJam, d.reverseShooter);
            if (rollerReverse) {
                assertEquals(what + ": the roller reverse button always works", -1, d.rollerDirection);
            }
            if (feelerReverse) {
                assertEquals(what + ": the feelers reverse button always works", -1, d.feelerDirection);
            }
            // Shooting needs its trigger and no jam button.
            assertEquals(what, shootAsked && !clearing, d.shoot);
            // Never pull POLLEN in while shooting or clearing a jam.
            if (d.shoot || clearing) {
                assertTrue(what + ": nothing pulls in while shooting or clearing a jam", d.rollerDirection <= 0 && d.feelerDirection <= 0);
            }
            // Without the intake trigger and without a reverse button the intake is off.
            if (!intakeAsked && !rollerReverse) {
                assertEquals(what, 0, d.rollerDirection);
            }
            if (!intakeAsked && !feelerReverse) {
                assertEquals(what, 0, d.feelerDirection);
            }
        }
    }

    /**
     * Situation: the driver holds the intake trigger and also the roller reverse button.
     * Expected: the roller pushes out while the corner servos still pull in; with the feelers reverse as well, everything pushes out.
     * Why: each reverse beats pulling in only for its own mechanism, and holding both is how to push everything out.
     */
    @Test
    public void aReverseButtonBeatsPullingInForItsOwnMechanism() {
        ManualRules.Decision rollerOnly = ManualRules.decide(false, true, false, false, true, false);
        assertEquals(-1, rollerOnly.rollerDirection);
        assertEquals(1, rollerOnly.feelerDirection);
        ManualRules.Decision both = ManualRules.decide(false, true, false, false, true, true);
        assertEquals(-1, both.rollerDirection);
        assertEquals(-1, both.feelerDirection);
        assertTrue(both.pushingOut());
        assertFalse(both.pullingIn());
    }

    /**
     * Situation: only the intake trigger is held.
     * Expected: the roller and both corner servos pull POLLEN in (direction 1) and nothing else runs.
     * Why: it is the headline feature of the left trigger, and nothing else pins it.
     */
    @Test
    public void theIntakeTriggerAlonePullsInWithBothParts() {
        ManualRules.Decision d = ManualRules.decide(false, true, false, false, false, false);
        assertEquals(1, d.rollerDirection);
        assertEquals(1, d.feelerDirection);
        assertTrue(d.pullingIn());
        assertFalse(d.pushingOut());
        assertFalse(d.shoot);
        assertFalse(d.reverseIndexer);
        assertFalse(d.reverseShooter);
    }

    /**
     * Situation: the feeder gate for different shooting speeds.
     * Expected: at the normal speed the gate is the tested 950; it scales with the trimmed speed, so it is always below the shooting speed
     * (the flywheel can reach it) and about 79% of it.
     * Why: a fixed 950 would never let the feeder run at a trimmed speed of 950 or less, and would feed far too early at 1600.
     */
    @Test
    public void theFeederGateFollowsTheShootingSpeed() {
        double min = BioBuzzConstants.Shooter.MANUAL_MIN_TPS;
        double normal = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
        assertEquals("at the normal speed the gate is the tested one", min, ManualRules.feedThresholdTps(normal, min, normal), 1e-9);
        for (double speed = BioBuzzConstants.Manual.SPEED_MIN_TPS; speed <= BioBuzzConstants.Manual.SPEED_MAX_TPS; speed += 25.0) {
            double gate = ManualRules.feedThresholdTps(speed, min, normal);
            assertTrue("the gate (" + gate + ") must be below the shooting speed " + speed, gate < speed);
            assertEquals("the gate is the same share of the speed", min / normal, gate / speed, 1e-9);
        }
        assertTrue("a trimmed-up speed raises the gate", ManualRules.feedThresholdTps(1600.0, min, normal) > min);
        assertTrue("a trimmed-down speed lowers the gate", ManualRules.feedThresholdTps(800.0, min, normal) < min);
    }

    /**
     * Situation: the startup check for the Manual TeleOp only ({@code ConstantsCheck.runManual}) with the shipped constants.
     * Expected: no errors.
     * Why: the Manual TeleOp refuses to start on an error here, so the shipped settings must pass.
     */
    @Test
    public void theShippedManualSettingsPassTheManualOnlyCheck() {
        ConstantsCheck.Report report = ConstantsCheck.runManual();
        assertTrue(report.errors.toString(), report.ok());
    }

    /**
     * Situation: the trigger threshold and the jam powers are checked with good and bad values.
     * Expected: a threshold of 0, 1 or more, or below 0 is reported; the shipped one passes; a jam power of zero or above the limit is reported.
     * Why: a threshold of 1 or more disables the triggers; one of 0 would shoot from a resting trigger.
     */
    @Test
    public void theTriggerThresholdAndJamPowersAreChecked() {
        ConstantsCheck.Report good = new ConstantsCheck.Report();
        ConstantsCheck.checkTriggerThreshold(good, BioBuzzConstants.Controls.TRIGGER_THRESHOLD);
        ConstantsCheck.checkJamPowers(good, BioBuzzConstants.Shooter.FEEDER_REVERSE_POWER, BioBuzzConstants.Shooter.FLYWHEEL_UNJAM_POWER,
                BioBuzzConstants.Controls.UNJAM_MAX_POWER);
        assertTrue(good.errors.toString(), good.errors.isEmpty());
        for (double bad : new double[]{0.0, 1.0, 1.5, -0.2}) {
            ConstantsCheck.Report r = new ConstantsCheck.Report();
            ConstantsCheck.checkTriggerThreshold(r, bad);
            assertContains(r.errors.toString(), "TRIGGER_THRESHOLD");
        }
        ConstantsCheck.Report jam = new ConstantsCheck.Report();
        ConstantsCheck.checkJamPowers(jam, 0.0, 0.9, 0.5);
        assertContains(jam.errors.toString(), "FEEDER_REVERSE_POWER");
        assertContains(jam.errors.toString(), "FLYWHEEL_UNJAM_POWER");
    }

    /**
     * Situation: the driver holds the shoot trigger and the two intake reverse buttons.
     * Expected: it shoots and the intake pushes out.
     * Why: pushing a POLLEN out never conflicts with shooting.
     */
    @Test
    public void pushingOutWorksWhileShooting() {
        ManualRules.Decision d = ManualRules.decide(true, false, false, false, true, true);
        assertTrue(d.shoot);
        assertEquals(-1, d.rollerDirection);
        assertEquals(-1, d.feelerDirection);
    }

    /**
     * Situation: nothing is held.
     * Expected: nothing is asked for: no shooting, no intake, no jam clearing.
     * Why: in that case the OpMode tells the flywheel to stop, and the launcher brakes it (the driver letting go of the
     * shoot trigger is exactly this case).
     */
    @Test
    public void whenNothingIsHeldNothingRuns() {
        ManualRules.Decision d = ManualRules.decide(false, false, false, false, false, false);
        assertFalse(d.shoot);
        assertEquals(0, d.rollerDirection);
        assertEquals(0, d.feelerDirection);
        assertFalse(d.reverseIndexer);
        assertFalse(d.reverseShooter);
    }

    /**
     * Situation: the driver holds the shoot trigger, then lets go.
     * Expected: shoot is true while held and false the moment it is released.
     * Why: shooting is "hold", not a switch that stays on.
     */
    @Test
    public void shootingStopsTheMomentTheTriggerIsReleased() {
        assertTrue(ManualRules.decide(true, false, false, false, false, false).shoot);
        assertFalse(ManualRules.decide(false, false, false, false, false, false).shoot);
    }

    /**
     * Situation: the shooting speed is raised and lowered by the speed buttons.
     * Expected: each press moves it by the step, and it never leaves the allowed range.
     * Why: a driver who keeps pressing must not be able to set a speed the flywheel cannot reach or a stop-like one.
     */
    @Test
    public void theShootingSpeedStepsAndStaysInsideTheRange() {
        double step = BioBuzzConstants.Manual.SPEED_STEP_TPS;
        double min = BioBuzzConstants.Manual.SPEED_MIN_TPS;
        double max = BioBuzzConstants.Manual.SPEED_MAX_TPS;
        assertEquals(1225.0, ManualRules.trimShootSpeed(1200.0, 1, 25.0, 800.0, 1600.0), 1e-9);
        assertEquals(1175.0, ManualRules.trimShootSpeed(1200.0, -1, 25.0, 800.0, 1600.0), 1e-9);
        assertEquals(1600.0, ManualRules.trimShootSpeed(1590.0, 1, 25.0, 800.0, 1600.0), 1e-9);
        assertEquals(800.0, ManualRules.trimShootSpeed(810.0, -1, 25.0, 800.0, 1600.0), 1e-9);
        double speed = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
        for (int i = 0; i < 1000; i++) {
            speed = ManualRules.trimShootSpeed(speed, 1, step, min, max);
        }
        assertEquals("a thousand presses up stop at the maximum", max, speed, 1e-9);
        for (int i = 0; i < 1000; i++) {
            speed = ManualRules.trimShootSpeed(speed, -1, step, min, max);
        }
        assertEquals("a thousand presses down stop at the minimum", min, speed, 1e-9);
    }

    /**
     * Situation: the shipped settings of the Manual TeleOp and the jam buttons.
     * Expected: the startup check finds no error about them, and the eight buttons are all different.
     * Why: two jobs on one button would fire together (for example slow mode while clearing a jam).
     */
    @Test
    public void theShippedManualSettingsPassTheStartupCheck() {
        ConstantsCheck.Report report = ConstantsCheck.run();
        for (String error : report.errors) {
            assertFalse("unexpected manual-controls error: " + error, error.contains("Manual.") || error.contains("MANUAL_TARGET_TPS") || error.contains("REVERSE_POWER")
                    || error.contains("intake reverse button"));
        }
        Button[] all = {BioBuzzConstants.Manual.SLOW_MODE_BUTTON, BioBuzzConstants.Manual.SPEED_UP_BUTTON,
                BioBuzzConstants.Manual.SPEED_DOWN_BUTTON, BioBuzzConstants.Manual.SPEED_RESET_BUTTON,
                BioBuzzConstants.Controls.UNJAM_INDEXER_BUTTON, BioBuzzConstants.Controls.UNJAM_SHOOTER_BUTTON,
                BioBuzzConstants.Controls.REVERSE_INTAKE_BUTTON, BioBuzzConstants.Controls.REVERSE_FEELERS_BUTTON};
        for (int i = 0; i < all.length; i++) {
            for (int j = i + 1; j < all.length; j++) {
                assertNotEquals("two manual buttons are the same: " + all[i], all[i], all[j]);
            }
        }
    }

    /**
     * Situation: the startup check is given bad values (slow mode of zero, a speed range turned around, a maximum the motor cannot reach,
     * a normal speed outside the range, a step larger than the range, two buttons the same).
     * Expected: each mistake is reported by name.
     * Why: the check is what stops a typo in BioBuzzConstants from reaching the robot.
     */
    @Test
    public void badManualSettingsAreReported() {
        Button[] twoSame = {Button.LEFT_BUMPER, Button.LEFT_BUMPER};
        String[] names = {"Manual.SLOW_MODE_BUTTON", "Controls.UNJAM_INDEXER_BUTTON"};

        ConstantsCheck.Report good = new ConstantsCheck.Report();
        ConstantsCheck.checkManualControls(good, 0.35, 25.0, 800.0, 1600.0, 1200.0, new Button[]{Button.LEFT_BUMPER, Button.B}, names);
        assertTrue(good.errors.toString(), good.errors.isEmpty());

        assertContains(errorsFor(0.0, 25.0, 800.0, 1600.0, 1200.0), "SLOW_MODE_SCALE");
        assertContains(errorsFor(0.35, 25.0, 1600.0, 800.0, 1200.0), "SPEED_MIN_TPS");
        assertContains(errorsFor(0.35, 25.0, 800.0, 2900.0, 1200.0), "SPEED_MAX_TPS");
        assertContains(errorsFor(0.35, 25.0, 800.0, 1600.0, 1700.0), "MANUAL_TARGET_TPS");
        assertContains(errorsFor(0.35, 900.0, 800.0, 1600.0, 1200.0), "SPEED_STEP_TPS");
        ConstantsCheck.Report same = new ConstantsCheck.Report();
        ConstantsCheck.checkManualControls(same, 0.35, 25.0, 800.0, 1600.0, 1200.0, twoSame, names);
        assertContains(same.errors.toString(), "two jobs on one button");
    }

    /**
     * Situation: the reverse powers of the intake roller and the feelers are checked with good and bad values.
     * Expected: the shipped values and full power (1.0) pass; zero (does nothing), a negative number and anything above 1 are each reported by name.
     * Why: a power of zero would make the reverse button useless, and one above 1 is a typo.
     */
    @Test
    public void theReversePowersMustBeBetweenZeroAndOne() {
        ConstantsCheck.Report good = new ConstantsCheck.Report();
        ConstantsCheck.checkReversePowers(good, BioBuzzConstants.Controls.INTAKE_REVERSE_POWER, BioBuzzConstants.Controls.FEELER_REVERSE_POWER);
        ConstantsCheck.checkReversePowers(good, 1.0, 1.0);
        assertTrue(good.errors.toString(), good.errors.isEmpty());
        ConstantsCheck.Report bad = new ConstantsCheck.Report();
        ConstantsCheck.checkReversePowers(bad, 0.0, 1.5);
        assertContains(bad.errors.toString(), "INTAKE_REVERSE_POWER");
        assertContains(bad.errors.toString(), "FEELER_REVERSE_POWER");
        ConstantsCheck.Report negative = new ConstantsCheck.Report();
        ConstantsCheck.checkReversePowers(negative, -0.5, 1.0);
        assertContains(negative.errors.toString(), "INTAKE_REVERSE_POWER");
        assertEquals(1, negative.errors.size());
    }

    private static String errorsFor(double slow, double step, double min, double max, double normal) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkManualControls(r, slow, step, min, max, normal, new Button[]{Button.LEFT_BUMPER, Button.B},
                new String[]{"Manual.SLOW_MODE_BUTTON", "Controls.UNJAM_SHOOTER_BUTTON"});
        return r.errors.toString();
    }

    private static void assertContains(String text, String part) {
        assertTrue("expected '" + part + "' in: " + text, text.contains(part));
    }
}
