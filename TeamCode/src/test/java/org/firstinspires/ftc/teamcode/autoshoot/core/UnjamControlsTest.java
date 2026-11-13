package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Checks the settings of the jam-clearing buttons (hold a button to run the indexer or the shooter wheel slowly
 * BACKWARD so a stuck POLLEN comes free). Run with {@code ./gradlew :TeamCode:testDebugUnitTest}.
 *
 * <p>The motors themselves cannot be tested on a PC, so these tests protect the two things that can go wrong in
 * the settings: a reverse speed that is not slow (it could throw the ball or hurt the mechanism), and a button that
 * is already used for something else (pressing it would do two things at once).
 */
public class UnjamControlsTest {

    /**
     * Situation: the shipped constants.
     * Expected: both reverse powers are above zero (so they really run) and no more than the safety limit
     *     {@code Controls.UNJAM_MAX_POWER}.
     * Why: "slowly" is part of the requirement, and a typo such as 1.5 must never reach the robot.
     */
    @Test
    public void bothReversePowersAreSlowButRun() {
        double limit = BioBuzzConstants.Controls.UNJAM_MAX_POWER;
        double indexer = BioBuzzConstants.Shooter.FEEDER_REVERSE_POWER;
        double wheel = BioBuzzConstants.Shooter.FLYWHEEL_UNJAM_POWER;
        assertTrue("the indexer reverse power must be above 0: " + indexer, indexer > 0.0);
        assertTrue("the indexer reverse power must be slow (at most " + limit + "): " + indexer, indexer <= limit);
        assertTrue("the shooter reverse power must be above 0: " + wheel, wheel > 0.0);
        assertTrue("the shooter reverse power must be slow (at most " + limit + "): " + wheel, wheel <= limit);
    }

    /**
     * Situation: the shipped button choices.
     * Expected: the two jam-clearing buttons differ from each other and from the shoot, HIVE-toggle and pose-reset
     *     buttons.
     * Why: pressing a jam button while shooting must not also start the auto-shoot, flip the HIVE belief or reset the
     *     robot's position.
     */
    @Test
    public void theJamButtonsAreNotUsedForAnythingElse() {
        Button indexer = BioBuzzConstants.Controls.UNJAM_INDEXER_BUTTON;
        Button shooter = BioBuzzConstants.Controls.UNJAM_SHOOTER_BUTTON;
        assertNotEquals("the two jam buttons must differ", indexer, shooter);
        for (Button b : new Button[]{indexer, shooter}) {
            assertNotEquals(b + " is also the shoot button", BioBuzzConstants.Controls.SHOOT_BUTTON, b);
            assertNotEquals(b + " is also the HIVE toggle button", BioBuzzConstants.Controls.TOGGLE_HIVE_BUTTON, b);
            assertNotEquals(b + " is also the pose reset button", BioBuzzConstants.Controls.RESET_POSE_BUTTON, b);
        }
    }

    /**
     * Situation: the constants checker that runs when the TeleOp starts.
     * Expected: it reports no error about the jam-clearing settings.
     * Why: the checker is what stops a bad setting on the robot, so the shipped values must pass it.
     */
    @Test
    public void theStartupCheckAcceptsTheShippedJamSettings() {
        ConstantsCheck.Report report = ConstantsCheck.run();
        for (String error : report.errors) {
            assertTrue("unexpected constants error: " + error, !error.contains("UNJAM") && !error.contains("jam-clearing")
                    && !error.contains("FEEDER_REVERSE_POWER"));
        }
    }
}
