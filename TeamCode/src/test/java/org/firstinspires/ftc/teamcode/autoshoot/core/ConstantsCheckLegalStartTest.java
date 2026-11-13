package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * {@link ConstantsCheck#checkLegalStart}: the TeleOp's default start pose (where the robot is put when no AUTO has run) must be a start rule G304 allows. The check is
 * the route validator's own start rule, so a pose is judged the same way here and in the autonomous routes. The shipped default is legal; each part of the rule has
 * a pose that breaks it and the check says which.
 */
public class ConstantsCheckLegalStartTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double SIDE = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
    private static final FieldModel FIELD = new FieldModel();

    private static ConstantsCheck.Report check(Pose2 start) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkLegalStart(r, FIELD, "default start", start);
        return r;
    }

    private static boolean says(ConstantsCheck.Report r, String text) {
        for (String e : r.errors) {
            if (e.contains(text)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Situation: the shipped TeleOp default start (front on the audience wall, in the middle of tile C1) run through the check, and through the whole INIT check.
     * Expected: it is a legal start (no error, no warning), for both alliances; INIT has no error about it.
     */
    @Test
    public void theShippedDefaultStartIsALegalG304Start() {
        Pose2 shipped = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN, BioBuzzConstants.Controls.DEFAULT_START_Y_IN,
                BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        ConstantsCheck.Report r = check(shipped);
        assertTrue(r.errors.toString() + r.warnings, r.errors.isEmpty() && r.warnings.isEmpty());
        for (String e : ConstantsCheck.run().errors) {
            assertTrue(e, !e.contains("is not a legal start"));
        }
    }

    /** Situation: other starts the team uses (front or back on the rear wall, a side on the red wall, a corner). Expected: legal. */
    @Test
    public void otherLegalStartsAreAccepted() {
        for (Pose2 p : new Pose2[] {Pose2.ofDegrees(35.0, FRONT, -90), Pose2.ofDegrees(61.5, 141.348 - FRONT, 90), Pose2.ofDegrees(SIDE, 30.0, 90), Pose2.ofDegrees(FRONT, SIDE, 180)}) {
            ConstantsCheck.Report r = check(p);
            assertTrue(p + ": " + r.errors + r.warnings, r.errors.isEmpty() && r.warnings.isEmpty());
        }
    }

    /** Situation: a start that touches no wall (20 in from the audience wall). Expected: an ERROR that names G304.C and says it must touch a wall. */
    @Test
    public void aStartThatTouchesNoWallIsRejected() {
        ConstantsCheck.Report r = check(Pose2.ofDegrees(58.9, 30.0, -90));
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("G304.C") && r.errors.get(0).contains("must touch a perimeter wall"));
    }

    /** Situation: a start whose center of rotation is on the audience wall, so the front pokes through it (the old default, 8.9 in). Expected: an ERROR for G304.A. */
    @Test
    public void aStartThroughAWallIsRejected() {
        ConstantsCheck.Report r = check(Pose2.ofDegrees(58.9, 8.9, -90));
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("G304.A") && r.errors.get(0).contains("sticks out of the field"));
    }

    /** Situation: a start in the blue half (the red robot's default moved over the center line). Expected: an ERROR for G304.A that names the opponent half. */
    @Test
    public void aStartInTheOpponentHalfIsRejected() {
        ConstantsCheck.Report r = check(Pose2.ofDegrees(70.0, FRONT, -90));
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(r.errors.get(0), r.errors.get(0).contains("opponent half"));
    }

    /** Situation: a start in the red LOADING ZONE, and a start on the red wall's FLOWER. Expected: an ERROR for G304.E and one for G304.D. */
    @Test
    public void aStartInTheLoadingZoneOrOnAFlowerIsRejected() {
        ConstantsCheck.Report zone = check(Pose2.ofDegrees(FRONT, 108.0, 180));
        assertEquals(zone.errors.toString(), 1, zone.errors.size());
        assertTrue(zone.errors.get(0), zone.errors.get(0).contains("G304.E") && zone.errors.get(0).contains("LOADING ZONE"));
        ConstantsCheck.Report flower = check(Pose2.ofDegrees(FRONT, 48.0, 180));
        assertEquals(flower.errors.toString(), 1, flower.errors.size());
        assertTrue(flower.errors.get(0), flower.errors.get(0).contains("G304.D") && flower.errors.get(0).contains("FLOWER 1"));
    }

    /** Situation: a start on the HIVE frame (and so touching no wall either). Expected: an ERROR for G304.B among the two. */
    @Test
    public void aStartOnTheHiveIsRejected() {
        ConstantsCheck.Report r = check(Pose2.ofDegrees(60.0, 70.0, 0));
        assertTrue(r.errors.toString(), says(r, "G304.B") && says(r, "HIVE frame"));
    }

    /**
     * Situation: a start 0.18 in over the end of a FLOWER's wall bracket (clear of the plate).
     * Expected: no ERROR, one warning that names the bracket: the rule only warns about it.
     */
    @Test
    public void standingOverABracketIsOnlyAWarning() {
        double bracketEnd = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1] + BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN / 2.0;
        ConstantsCheck.Report r = check(Pose2.ofDegrees(BioBuzzConstants.Robot.REAR_EXTENT_IN, bracketEnd - 0.18 + BioBuzzConstants.Robot.RIGHT_EXTENT_IN, 0));
        assertTrue(r.errors.toString(), r.errors.isEmpty());
        assertEquals(r.warnings.toString(), 1, r.warnings.size());
        assertTrue(r.warnings.get(0), r.warnings.get(0).contains("wall bracket"));
    }

    /**
     * Situation: a fault that both alliances share (touching no wall).
     * Expected: it is reported once, not once for RED and once for BLUE; and the message does not say "BLUE" (it is the shared fault, said for the first alliance checked).
     */
    @Test
    public void aFaultBothAlliancesShareIsReportedOnce() {
        ConstantsCheck.Report r = check(Pose2.ofDegrees(40.0, 40.0, 0));
        assertEquals(r.errors.toString(), 1, r.errors.size());
        assertTrue(!r.errors.get(0).contains("BLUE alliance"));
    }
}
