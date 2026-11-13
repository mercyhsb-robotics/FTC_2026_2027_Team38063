package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The park assist's settings and its button, as the start-up check ({@link ConstantsCheck}) sees them. Run with
 * {@code ./gradlew :TeamCode:testDebugUnitTest --tests "*ConstantsCheckParkTest"}.
 *
 * <p>The button check takes the buttons as parameters, so these tests can hand it a button that clashes. Without that the only thing a
 * test could do is read the shipped settings, which are fine. Words: <b>clash</b> two jobs on one button, so one press does both.
 */
public class ConstantsCheckParkTest {

    /** Every button the TeleOp already uses (the same list {@code ConstantsCheck.run} builds), and who uses it. */
    private final List<Button> others = new ArrayList<Button>();
    private final List<String> names = new ArrayList<String>();

    public ConstantsCheckParkTest() {
        add(BioBuzzConstants.Controls.SHOOT_BUTTON, "Controls.SHOOT_BUTTON");
        add(BioBuzzConstants.Controls.TOGGLE_HIVE_BUTTON, "Controls.TOGGLE_HIVE_BUTTON");
        add(BioBuzzConstants.Controls.RESET_POSE_BUTTON, "Controls.RESET_POSE_BUTTON");
        add(BioBuzzConstants.Controls.ALLIANCE_RED_BUTTON, "Controls.ALLIANCE_RED_BUTTON");
        add(BioBuzzConstants.Controls.ALLIANCE_BLUE_BUTTON, "Controls.ALLIANCE_BLUE_BUTTON");
        add(BioBuzzConstants.Controls.UNJAM_INDEXER_BUTTON, "Controls.UNJAM_INDEXER_BUTTON");
        add(BioBuzzConstants.Controls.UNJAM_SHOOTER_BUTTON, "Controls.UNJAM_SHOOTER_BUTTON");
        for (Button fixed : BioBuzzConstants.Controls.FIXED_MANUAL_BUTTONS) {
            add(fixed, "a fixed manual control in BioBuzzRobot (" + fixed + ")");
        }
    }

    private void add(Button button, String name) {
        others.add(button);
        names.add(name);
    }

    private ConstantsCheck.Report checkButton(Button park) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkParkButton(r, park, others.toArray(new Button[0]), names.toArray(new String[0]));
        return r;
    }

    /**
     * Situation: the shipped settings.
     * Expected: the park button differs from every other button the TeleOp uses (the shoot, HIVE-toggle, pose-reset, alliance and
     *     jam-clearing buttons and the five fixed manual ones), and the whole start-up check has no error and no warning about parking.
     * Why: pressing park must never also shoot, clear a jam, reset the position or start the intake.
     */
    @Test
    public void theShippedParkButtonClashesWithNothing() {
        Button park = BioBuzzConstants.Controls.PARK_BUTTON;
        for (int i = 0; i < others.size(); i++) {
            assertNotEquals(park + " is also " + names.get(i), others.get(i), park);
        }
        assertTrue(checkButton(park).errors.toString(), checkButton(park).errors.isEmpty());
        ConstantsCheck.Report whole = ConstantsCheck.run();
        for (String e : whole.errors) {
            assertFalse(e, e.contains("PARK") || e.contains("Park"));
        }
        for (String w : whole.warnings) {
            assertFalse(w, w.contains("PARK") || w.contains("Park"));
        }
    }

    /**
     * Situation: the park button is set, one after the other, to EACH button the TeleOp already uses.
     * Expected: every time an ERROR that names the park button and the other job (two errors for the one button that has two jobs on
     *     purpose: the blue-alliance button of INIT and the shooter jam button are both the D-pad right).
     * Why: this is the check that stops a clash at INIT instead of on the field.
     */
    @Test
    public void aClashWithAnyOtherButtonIsAnError() {
        for (int i = 0; i < others.size(); i++) {
            ConstantsCheck.Report r = checkButton(others.get(i));
            int uses = 0;
            for (Button o : others) {
                if (o == others.get(i)) {
                    uses++;
                }
            }
            assertEquals(names.get(i) + ": " + r.errors, uses, r.errors.size());
            boolean named = false;
            for (String e : r.errors) {
                assertTrue(e, e.contains("Controls.PARK_BUTTON"));
                named |= e.contains(names.get(i));
            }
            assertTrue(names.get(i) + " must be named in " + r.errors, named);
        }
    }

    /**
     * Situation: the source text of {@code ConstantsCheck}.
     * Expected: it hands every button of this test's list to the park check (it names each constant) and the park button itself.
     * Why: the test above checks its own copy of the list; this makes sure the real start-up check did not lose a button.
     */
    @Test
    public void theStartUpCheckKnowsEveryButtonThisTestKnows() throws java.io.IOException {
        String rel = "src/main/java/org/firstinspires/ftc/teamcode/autoshoot/core/ConstantsCheck.java";
        java.io.File file = new java.io.File(rel);
        if (!file.exists()) {
            file = new java.io.File("TeamCode/" + rel);
        }
        String src = new String(java.nio.file.Files.readAllBytes(file.toPath()), java.nio.charset.StandardCharsets.UTF_8);
        for (String name : new String[]{"SHOOT_BUTTON", "TOGGLE_HIVE_BUTTON", "RESET_POSE_BUTTON", "ALLIANCE_RED_BUTTON", "ALLIANCE_BLUE_BUTTON",
                "UNJAM_INDEXER_BUTTON", "UNJAM_SHOOTER_BUTTON", "FIXED_MANUAL_BUTTONS", "PARK_BUTTON"}) {
            assertTrue(name + " must be in the check", src.contains("Controls." + name));
        }
    }

    /**
     * Situation: the five fixed manual buttons listed in the constants.
     * Expected: they are the ones the manual controls of the TeleOp read: cross, circle, triangle, square and the right bumper.
     * Why: the clash check is only as good as this list.
     */
    @Test
    public void theFixedManualButtonsAreTheOnesTheTeleOpReads() {
        Set<Button> set = new HashSet<Button>(Arrays.asList(BioBuzzConstants.Controls.FIXED_MANUAL_BUTTONS));
        assertEquals(new HashSet<Button>(Arrays.asList(Button.CROSS, Button.CIRCLE, Button.TRIANGLE, Button.SQUARE, Button.RIGHT_BUMPER)), set);
    }

    private static ConstantsCheck.Report settings(double minOverlap, double safe, double arriveIn, double arriveDeg, double arriveSpeed,
                                                  double grid, double headingStep, int checks, double zoneDepth, double clearance) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkParkSettings(r, minOverlap, safe, arriveIn, arriveDeg, arriveSpeed, grid, headingStep, checks, zoneDepth, clearance);
        return r;
    }

    /** Situation: the shipped park numbers. Expected: no error. */
    @Test
    public void theShippedSettingsPass() {
        ConstantsCheck.Report r = settings(BioBuzzConstants.Park.MIN_OVERLAP_IN, BioBuzzConstants.Auto.PARK_MIN_OVERLAP_IN,
                BioBuzzConstants.Park.ARRIVE_IN, BioBuzzConstants.Park.ARRIVE_DEG, BioBuzzConstants.Park.ARRIVE_SPEED_IN_S,
                BioBuzzConstants.Park.GRID_STEP_IN, BioBuzzConstants.Park.HEADING_STEP_DEG, BioBuzzConstants.Park.MAX_ROUTE_CHECKS,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN,
                BioBuzzConstants.AutoShoot.WALL_CLEARANCE_IN);
        assertTrue(r.errors.toString(), r.errors.isEmpty());
    }

    /**
     * Situation: settings that cannot work: a depth deeper than the 11 in zone, a depth that the arrive tolerance would eat into the safe
     *     minimum, zero steps, no route checks, a zero arrive tolerance.
     * Expected: an ERROR each, naming the constant.
     * Why: each of these would make the assist never find a spot, or show "Parked" for a robot that does not count.
     */
    @Test
    public void settingsThatCannotWorkAreErrors() {
        double depth = 11.0;
        assertContains(settings(10.9, 3.0, 2.0, 10.0, 4.0, 1.0, 90.0, 150, depth, 0.25), "Park.MIN_OVERLAP_IN");
        assertContains(settings(4.0, 3.0, 2.0, 10.0, 4.0, 1.0, 90.0, 150, depth, 0.25), "Park.ARRIVE_IN");
        assertContains(settings(8.0, 3.0, 2.0, 10.0, 4.0, 0.0, 90.0, 150, depth, 0.25), "Park.GRID_STEP_IN");
        assertContains(settings(8.0, 3.0, 2.0, 10.0, 4.0, 1.0, 0.0, 150, depth, 0.25), "HEADING_STEP_DEG");
        assertContains(settings(8.0, 3.0, 2.0, 10.0, 4.0, 1.0, 90.0, 0, depth, 0.25), "MAX_ROUTE_CHECKS");
        assertContains(settings(8.0, 3.0, 0.0, 10.0, 4.0, 1.0, 90.0, 150, depth, 0.25), "ARRIVE_IN");
        assertContains(settings(0.0, 3.0, 2.0, 10.0, 4.0, 1.0, 90.0, 150, depth, 0.25), "MIN_OVERLAP_IN");
    }

    private static void assertContains(ConstantsCheck.Report r, String text) {
        assertFalse("expected an error about " + text, r.errors.isEmpty());
        boolean found = false;
        for (String e : r.errors) {
            found |= e.contains(text);
        }
        assertTrue(text + " must be named in " + r.errors, found);
    }
}
