package org.firstinspires.ftc.teamcode.autoshoot.run;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Button;
import org.firstinspires.ftc.teamcode.autoshoot.core.ConstantsCheck;

/** The checks of the numbers in {@code BioBuzzConstants.Run}: the shipped ones are valid, and every kind of bad number is caught with a message that names it. */
public class RunSettingsCheckTest {

    @Test
    public void theShippedRunSettingsHaveNoErrorAndNoWarning() {
        ConstantsCheck.Report r = RunSettingsCheck.run();
        assertTrue("errors: " + r.errors, r.ok());
        assertTrue("warnings: " + r.warnings, r.warnings.isEmpty());
    }

    @Test
    public void thePickerButtonsAreTheFourDpadDirectionsAsTheStudentsAreTold() {
        assertEquals(Button.DPAD_UP, BioBuzzConstants.Run.MENU_PREVIOUS_ROUTE_BUTTON);
        assertEquals(Button.DPAD_DOWN, BioBuzzConstants.Run.MENU_NEXT_ROUTE_BUTTON);
        assertEquals(Button.DPAD_LEFT, BioBuzzConstants.Run.MENU_RED_BUTTON);
        assertEquals(Button.DPAD_RIGHT, BioBuzzConstants.Run.MENU_BLUE_BUTTON);
    }

    @Test
    public void twoMenuFunctionsOnOneButtonIsAnError() {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        RunSettingsCheck.checkMenuButtons(r, Button.DPAD_UP, Button.DPAD_UP, Button.DPAD_LEFT, Button.DPAD_RIGHT);
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("MENU_PREVIOUS_ROUTE_BUTTON") && r.errors.get(0).contains("MENU_NEXT_ROUTE_BUTTON"));
        ConstantsCheck.Report ok = new ConstantsCheck.Report();
        RunSettingsCheck.checkMenuButtons(ok, Button.DPAD_UP, Button.DPAD_DOWN, Button.DPAD_LEFT, Button.DPAD_RIGHT);
        assertTrue(ok.errors.isEmpty());
    }

    @Test
    public void aClockThatRunsPastTheAutoPeriodIsAnError() {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        RunSettingsCheck.checkClock(r, 31.0, 30.0);
        RunSettingsCheck.checkClock(r, 0.0, 30.0);
        assertEquals(r.errors.toString(), 2, r.errors.size());
        ConstantsCheck.Report ok = new ConstantsCheck.Report();
        RunSettingsCheck.checkClock(ok, 29.5, 30.0);
        RunSettingsCheck.checkClock(ok, 30.0, 30.0);
        assertTrue(ok.errors.isEmpty());
    }

    @Test
    public void badMoveToleranceAndTimeoutNumbersAreErrors() {
        // arguments: pos tol, loose in, speed, loose still s, turn done deg, pre-turn min deg, turn timeout s, timeout speed
        ConstantsCheck.Report ok = new ConstantsCheck.Report();
        RunSettingsCheck.checkMoves(ok, 1.5, 4.0, 6.0, 0.6, 4.0, 5.0, 2.5, 25.0);
        assertTrue(ok.errors.toString(), ok.errors.isEmpty());

        ConstantsCheck.Report zero = new ConstantsCheck.Report();
        RunSettingsCheck.checkMoves(zero, 0.0, 4.0, 6.0, 0.6, 4.0, 5.0, 2.5, 25.0);
        assertTrue(zero.errors.toString(), zero.errors.size() == 1 && zero.errors.get(0).contains("ARRIVE_POS_TOL_IN"));

        ConstantsCheck.Report loose = new ConstantsCheck.Report();
        RunSettingsCheck.checkMoves(loose, 3.0, 2.0, 6.0, 0.6, 4.0, 5.0, 2.5, 25.0);
        assertTrue(loose.errors.toString(), loose.errors.size() == 1 && loose.errors.get(0).contains("ARRIVE_LOOSE_IN"));

        ConstantsCheck.Report turn = new ConstantsCheck.Report();
        RunSettingsCheck.checkMoves(turn, 1.5, 4.0, 6.0, 0.6, 5.0, 5.0, 2.5, 25.0);
        assertTrue("a finished turn must not be big enough to start another: " + turn.errors, turn.errors.size() == 1 && turn.errors.get(0).contains("PRE_TURN_MIN_DEG"));

        ConstantsCheck.Report timeout = new ConstantsCheck.Report();
        RunSettingsCheck.checkMoves(timeout, 1.5, 4.0, 6.0, 0.6, 4.0, 5.0, 0.0, 25.0);
        assertTrue(timeout.errors.toString(), timeout.errors.size() == 1 && timeout.errors.get(0).contains("TURN_TIMEOUT_SEC"));
    }

    @Test
    public void aServoPowerAboveOneOrAtZeroAndANonPositiveFlowerToleranceAreErrors() {
        ConstantsCheck.Report ok = new ConstantsCheck.Report();
        RunSettingsCheck.checkIntake(ok, -1.0, 1.0, 1.0, 1.0);
        assertTrue(ok.errors.isEmpty());
        for (double[] bad : new double[][] {{-2.0, 1.0, 1.0, 1.0}, {-1.0, 0.0, 1.0, 1.0}, {-1.0, 1.0, 0.0, 1.0}, {-1.0, 1.0, 1.0, -1.0}}) {
            ConstantsCheck.Report r = new ConstantsCheck.Report();
            RunSettingsCheck.checkIntake(r, bad[0], bad[1], bad[2], bad[3]);
            assertEquals(r.errors.toString(), 1, r.errors.size());
        }
    }

    @Test
    public void aShootStepTimeoutShorterThanTheLongestVolleyIsAWarning() {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        RunSettingsCheck.checkShootStep(r, 6.0, 5.0, 3.5);
        assertTrue(r.errors.isEmpty());
        assertEquals(r.warnings.toString(), 1, r.warnings.size());
        ConstantsCheck.Report ok = new ConstantsCheck.Report();
        RunSettingsCheck.checkShootStep(ok, 12.0, 5.0, 3.5);
        assertTrue(ok.warnings.isEmpty() && ok.errors.isEmpty());
        ConstantsCheck.Report bad = new ConstantsCheck.Report();
        RunSettingsCheck.checkShootStep(bad, 0.0, 5.0, 3.5);
        assertEquals(1, bad.errors.size());
    }
}
