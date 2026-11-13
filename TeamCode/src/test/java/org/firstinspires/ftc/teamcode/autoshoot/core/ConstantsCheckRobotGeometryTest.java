package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The robot-geometry checks of {@link ConstantsCheck}: the four distances from the center of rotation to the robot's edges
 * must add up to its length and width, the intake wheels and the ball exit must lie on the robot, and the TeleOp start pose
 * must be somewhere the real (off-center) outline can stand.
 *
 * <p>How it works: the shipped constants are checked first (no complaint), then the same checks are run on values written in the
 * test, which are wrong on purpose. Each negative case would pass silently without the check it targets.
 *
 * <p>Words used: <b>center of rotation</b> is the point the robot spins about; <b>extent</b> is the distance from it to one
 * edge of the robot's outline.
 */
public class ConstantsCheckRobotGeometryTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double REAR = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double SIDE = BioBuzzConstants.Robot.LEFT_EXTENT_IN;

    /** Runs the edge check with the shipped values except the ones given (null = keep). */
    private static ConstantsCheck.Report edges(Double front, Double rear, Double left, Double right, Double length, Double width,
                                               Double reach, Double launchForward, Double launchLeft) {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        ConstantsCheck.checkRobotEdges(r,
                front != null ? front : BioBuzzConstants.Robot.FRONT_EXTENT_IN,
                rear != null ? rear : BioBuzzConstants.Robot.REAR_EXTENT_IN,
                left != null ? left : BioBuzzConstants.Robot.LEFT_EXTENT_IN,
                right != null ? right : BioBuzzConstants.Robot.RIGHT_EXTENT_IN,
                length != null ? length : BioBuzzConstants.Robot.LENGTH_IN,
                width != null ? width : BioBuzzConstants.Robot.WIDTH_IN,
                reach != null ? reach : BioBuzzConstants.Robot.INTAKE_REACH_IN,
                BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN, BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN,
                launchForward != null ? launchForward : BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                launchLeft != null ? launchLeft : BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN);
        return r;
    }

    // ---- the shipped numbers --------------------------------------------------------------------------------------

    @Test
    public void theShippedExtentsAddUpToTheRobotAndTheChecksFindNothingToSay() {
        assertEquals(BioBuzzConstants.Robot.LENGTH_IN, FRONT + REAR, 0.005);
        assertEquals(BioBuzzConstants.Robot.WIDTH_IN, BioBuzzConstants.Robot.LEFT_EXTENT_IN + BioBuzzConstants.Robot.RIGHT_EXTENT_IN, 0.005);
        ConstantsCheck.Report r = edges(null, null, null, null, null, null, null, null, null);
        assertTrue(r.errors.toString(), r.errors.isEmpty());
        assertTrue(r.warnings.toString(), r.warnings.isEmpty());
        ConstantsCheck.Report all = ConstantsCheck.run();
        assertTrue(all.errors.toString(), all.ok());
        assertTrue(all.warnings.toString(), all.warnings.isEmpty());
    }

    @Test
    public void theDerivedChassisFrontIsTheFrontExtentMinusTheWheelReach() {
        assertEquals(BioBuzzConstants.Robot.FRONT_EXTENT_IN - BioBuzzConstants.Robot.INTAKE_REACH_IN,
                BioBuzzConstants.Robot.CHASSIS_FRONT_EXTENT_IN, 1e-12);
        // the Footprint body (chassis) is built from the same two numbers
        Pose2 facingX = new Pose2(0.0, 0.0, 0.0);
        assertEquals(BioBuzzConstants.Robot.CHASSIS_FRONT_EXTENT_IN, Footprint.chassis(facingX).bounds().maxX, 1e-9);
        assertEquals(BioBuzzConstants.Robot.FRONT_EXTENT_IN, Footprint.of(facingX).bounds().maxX, 1e-9);
    }

    // ---- the sums ---------------------------------------------------------------------------------------------------

    /**
     * Situation: the front and rear distances do not add up to the robot's length (a typo: 9.77 and 9.03).
     * Expected: an ERROR that names the constants and both numbers, so the team knows what to re-measure.
     * Why: the extents describe one rectangle; if they disagree with the overall length one of them is a typo or a bad measurement.
     */
    @Test
    public void frontPlusRearThatDisagreesWithTheLengthIsAnError() {
        ConstantsCheck.Report r = edges(null, 9.03, null, null, null, null, null, null, null);
        assertEquals(r.errors.toString(), 1, r.errors.size());
        String m = r.errors.get(0);
        assertTrue(m, m.contains("FRONT_EXTENT_IN (9.77)") && m.contains("REAR_EXTENT_IN (9.03)") && m.contains("LENGTH_IN is 17.8"));
        assertTrue(m, m.contains("add up to the robot's length"));
        // a rounding hair (0.004 in) is forgiven, 0.02 in is not
        assertTrue(edges(null, REAR + 0.004, null, null, null, null, null, null, null).errors.isEmpty());
        assertFalse(edges(null, REAR + 0.02, null, null, null, null, null, null, null).errors.isEmpty());
    }

    @Test
    public void leftPlusRightThatDisagreesWithTheWidthIsAnError() {
        ConstantsCheck.Report r = edges(null, null, 8.9, 9.4, null, null, null, null, null);
        assertEquals(r.errors.toString(), 1, r.errors.size());
        String m = r.errors.get(0);
        assertTrue(m, m.contains("LEFT_EXTENT_IN (8.9)") && m.contains("RIGHT_EXTENT_IN (9.4)") && m.contains("WIDTH_IN is 17.8"));
        // an off-center robot that is still 17.8 wide is fine: 9.4 + 8.4
        assertTrue(edges(null, null, 9.4, 8.4, null, null, null, null, null).errors.isEmpty());
    }

    @Test
    public void aChangedOverallSizeMustBeMatchedByTheExtents() {
        // somebody re-measures the length as 18.2 in but forgets the extents
        ConstantsCheck.Report r = edges(null, null, null, null, 18.2, null, null, null, null);
        assertEquals(r.errors.toString(), 1, r.errors.size());
        // and the matching extents (front 10.17, rear 8.03) pass again
        assertTrue(edges(10.17, null, null, null, 18.2, null, null, null, null).errors.isEmpty());
    }

    @Test
    public void everyExtentMustBePositive() {
        assertFalse(edges(0.0, null, null, null, null, null, null, null, null).errors.isEmpty());
        assertFalse(edges(null, -1.0, null, null, null, null, null, null, null).errors.isEmpty());
        assertTrue(edges(null, -1.0, null, null, null, null, null, null, null).errors.get(0).contains("positive"));
    }

    // ---- wheels and ball exit -------------------------------------------------------------------------------------

    @Test
    public void theWheelReachMustBeLessThanTheFrontExtent() {
        // reach 9.77 would put the chassis front exactly on the center of rotation; more would put it behind
        assertFalse(edges(null, null, null, null, null, null, 9.77, null, null).errors.isEmpty());
        assertFalse(edges(null, null, null, null, null, null, 0.0, null, null).errors.isEmpty());
        assertTrue(edges(null, null, null, null, null, null, 2.7, null, null).errors.isEmpty());
    }

    @Test
    public void theBallExitMustLieOnTheRobot() {
        // -8.03 is the rear edge itself: on the robot
        assertTrue(edges(null, null, null, null, null, null, null, -REAR, null).warnings.isEmpty());
        // 0.5 in behind the rear edge: off the robot
        ConstantsCheck.Report off = edges(null, null, null, null, null, null, null, -REAR - 0.5, null);
        assertEquals(off.warnings.toString(), 1, off.warnings.size());
        assertTrue(off.warnings.get(0), off.warnings.get(0).contains("outside the robot's perimeter"));
        // the old value, -8.9 in, is 0.87 in behind where the rear edge is now: the check notices a launch point left over from
        // the centered assumption
        assertFalse(edges(null, null, null, null, null, null, null, -8.9, null).warnings.isEmpty());
        // a mix-up of millimeters and inches, and a sign mistake (the shooter is at the BACK)
        assertFalse(edges(null, null, null, null, null, null, null, -203.0, null).warnings.isEmpty());
        assertFalse(edges(null, null, null, null, null, null, null, null, 25.0).warnings.isEmpty());
    }

    // ---- the TeleOp default start pose ---------------------------------------------------------------------------

    @Test
    public void theDefaultStartIsTheFrontExtentFromTheAudienceWallSoTheFrontEdgeTouchesIt() {
        assertEquals(FRONT, BioBuzzConstants.Controls.DEFAULT_START_Y_IN, 1e-12);
        Pose2 start = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN, BioBuzzConstants.Controls.DEFAULT_START_Y_IN,
                BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        assertEquals("front edge ON the audience wall", 0.0, Footprint.of(start).bounds().minY, 1e-9);
    }

    /**
     * Situation: start poses for the check: the shipped default, the old default (y = 8.9, from the centered-robot days), a
     *     robot facing away from the wall at its back extent, one 0.3 in too close, and one standing on the HIVE frame.
     * Expected: only the first and third pass; the old default is an ERROR that says it pokes 0.87 in through the wall and
     *     names the 9.77 in the front edge needs; the one in the HIVE is an ERROR too.
     */
    @Test
    public void aStartWhoseRealFootprintPokesThroughAWallIsRejected() {
        FieldModel field = new FieldModel();
        ConstantsCheck.Report good = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPose(good, field, "default start", Pose2.ofDegrees(60.0, FRONT, -90.0));
        assertTrue(good.errors.toString(), good.errors.isEmpty());
        // facing away: the back edge is only 8.03 in from the center, so y = 8.03 is a legal place to stand
        ConstantsCheck.Report away = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPose(away, field, "back to the wall", Pose2.ofDegrees(60.0, REAR, 90.0));
        assertTrue(away.errors.toString(), away.errors.isEmpty());
        // the OLD default: 8.9 in from the wall facing it, 0.87 in too close for a front edge that is 9.77 in from the center
        ConstantsCheck.Report old = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPose(old, field, "old default", Pose2.ofDegrees(60.0, 8.9, -90.0));
        assertEquals(old.errors.toString(), 1, old.errors.size());
        assertTrue(old.errors.get(0), old.errors.get(0).contains("old default puts the robot's real footprint 0.87"));
        assertTrue(old.errors.get(0), old.errors.get(0).contains("9.77"));
        // 0.3 in too close facing away
        ConstantsCheck.Report tooClose = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPose(tooClose, field, "too close", Pose2.ofDegrees(60.0, REAR - 0.3, 90.0));
        assertEquals(tooClose.errors.toString(), 1, tooClose.errors.size());
        // a side edge on the red wall
        ConstantsCheck.Report side = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPose(side, field, "side", Pose2.ofDegrees(SIDE - 0.5, 30.0, 90.0));
        assertEquals(side.errors.toString(), 1, side.errors.size());
        // standing on the HIVE frame
        ConstantsCheck.Report hive = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPose(hive, field, "on the HIVE", Pose2.ofDegrees(72.0, 72.0, 0.0));
        assertEquals(hive.errors.toString(), 1, hive.errors.size());
        assertTrue(hive.errors.get(0), hive.errors.get(0).contains("HIVE frame or a FLOWER plate"));
        // the tolerance of noise (0.1 in) is forgiven
        ConstantsCheck.Report noise = new ConstantsCheck.Report();
        ConstantsCheck.checkStartPose(noise, field, "noise", Pose2.ofDegrees(60.0, FRONT - 0.09, -90.0));
        assertTrue(noise.errors.toString(), noise.errors.isEmpty());
    }
}
