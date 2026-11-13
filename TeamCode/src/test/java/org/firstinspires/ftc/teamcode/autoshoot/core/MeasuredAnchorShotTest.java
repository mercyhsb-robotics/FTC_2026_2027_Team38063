package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Locale;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The team's one MEASURED shot: with the robot's FRONT pressed against the audience wall and in line with the HIVE, the ball
 * leaves 37.7 in from the aim point of the UP cell and the POLLEN passes through the center of the opening to slightly above
 * it at about 1200 ticks per second. That is a PHYSICAL measurement of where the robot stood, so the model has to reproduce it
 * whatever the robot's center of rotation turns out to be.
 *
 * <p>How it works: no robot. The pose of that situation is built from the real extents (the center of rotation is the front
 * extent, 9.77 in, from the wall because the FRONT edge touches it), the model computes the range from the ball exit to the aim
 * point, and the test prints it and compares it with the 37.7 in that was measured. The next step of the field-geometry work
 * replaces the field coordinates by the official CAD numbers and will run this comparison again; the printed line is what to
 * compare.
 *
 * <p>Words used: <b>ball exit</b> is where the POLLEN leaves the shooter (the rear edge: 8.03 in behind the center of rotation,
 * which the CAD drawing dimensions at the rear lip); <b>aim point</b> is the center of the UP cell's opening; <b>range</b> is the
 * horizontal distance between the two.
 */
public class MeasuredAnchorShotTest {
    private final FieldModel field = new FieldModel();
    private final ShotTable table = ShotTable.fromConstants();

    /** The audience cell of the red HIVE, the one that is UP at the start of the match. */
    private CellSpec cell() {
        return field.cell(Alliance.RED, Side.AUDIENCE);
    }

    /** The robot with its front edge on the audience wall, in line with the HIVE axis, facing the wall. */
    private Pose2 anchorPose() {
        return new Pose2(cell().axisX, Footprint.frontExtentIn(), -Math.PI / 2.0);
    }

    /**
     * Situation: the measured anchor situation with the new extents (front on the wall, in line with the HIVE).
     * Expected: the model's range is the 37.7 in that was measured (within 0.1 in), the shooter is exactly in line and aimed,
     *     and the shot table gives about 1200 ticks per second there. The test prints the numbers.
     * Why: when the center of rotation moved from the middle of the robot to 9.77 in behind the front edge, the launch point moved
     *     from -8.9 to -8.03 in. The two moves cancel exactly (9.77 + 8.03 = 8.9 + 8.9 = 17.8 in from the wall to the ball
     *     exit), so the physical shot is unchanged. If the next measurement moves one of them without the other, this test shows
     *     by how much the shot moved.
     */
    @Test
    public void theMeasuredWallShotIsStillThirtySevenPointSevenInchesFromTheAimPoint() {
        Pose2 pose = anchorPose();
        CellSpec cell = cell();
        double range = ShotGeometry.rangeTo(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN,
                cell.originX, cell.originY);
        double exitY = ShotGeometry.launchY(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN);
        System.out.println(String.format(Locale.ROOT,
                "ANCHOR SHOT with the corrected extents: front on the audience wall (center of rotation %.2f in from it), in line with the HIVE"
                        + " axis x = %.2f; ball exit %.2f in from the wall at %.2f in behind the center; aim point (%.2f, %.2f);"
                        + " model range = %.2f in (measured 37.7 in); shot table at that range = %.0f ticks per second (measured about 1200)",
                pose.y, pose.x, exitY, -BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, cell.originX, cell.originY, range,
                table.velocityAt(range)));
        assertEquals("range from the ball exit to the aim point", 37.7, range, 0.1);
        assertEquals("the ball exit is a whole robot length from the wall", BioBuzzConstants.Robot.LENGTH_IN, exitY, 1e-9);
        assertEquals("in line with the HIVE", 0.0, Math.toDegrees(ShotGeometry.azimuthOffAxis(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN, cell)), 1e-6);
        assertEquals("aimed", 0.0, Math.toDegrees(ShotGeometry.aimError(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN, cell.originX, cell.originY)), 1e-6);
        assertEquals(1200.0, table.velocityAt(range), 6.0);
    }

    /**
     * Situation: the same situation worked out with the OLD numbers (center of rotation in the middle of the robot: 8.9 in from the
     *     wall, ball exit 8.9 in behind it).
     * Expected: the ball exit is at the same place, so the model's range is the same to the hundredth of an inch.
     * Why: it shows the correction moved the center of rotation and the launch point together and left the measured shot alone.
     */
    @Test
    public void theCorrectionMovesTheCenterAndTheBallExitTogetherSoTheMeasuredShotStaysPut() {
        CellSpec cell = cell();
        Pose2 oldPose = new Pose2(cell.axisX, 8.9, -Math.PI / 2.0);
        double oldRange = ShotGeometry.rangeTo(oldPose, -8.9, 0.0, cell.originX, cell.originY);
        double newRange = ShotGeometry.rangeTo(anchorPose(), BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN,
                cell.originX, cell.originY);
        assertEquals(oldRange, newRange, 0.01);
        assertEquals(37.72, newRange, 0.01);
    }

    /**
     * Situation: the shot table, which carries the anchor row.
     * Expected: the row {37.7, 1200} is still there (it is a physical measurement, kept as it is), and the one measured range is
     *     the table's own anchor and the planner's preferred range.
     */
    @Test
    public void theAnchorRowOfTheShotTableIsKept() {
        boolean found = false;
        for (double[] row : BioBuzzConstants.Shooter.SHOT_TABLE) {
            found |= Math.abs(row[0] - 37.7) < 1e-9 && Math.abs(row[1] - 1200.0) < 1e-9;
        }
        assertTrue("the measured row {37.7, 1200} must stay in the table", found);
        assertEquals(37.7, BioBuzzConstants.AutoShoot.PREFERRED_RANGE_IN, 1e-9);
        assertEquals(37.7, BioBuzzConstants.Shooter.TUNER_START_RANGE_IN, 1e-9);
    }

    /**
     * Situation: the planner is asked, from that very pose, where to stand for the audience cell.
     * Expected: it stays within 2.5 in of where it is (the measured spot) and chooses a range within 1.5 in of the measured one,
     *     never a pose that puts the front edge through the wall.
     */
    @Test
    public void theTeleOpPlannerKeepsTheMeasuredSpotAsItsAnswerWithinTheWallClearance() {
        ScoringPlanner planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(table));
        ScoringPlanner.Solution s = planner.solve(anchorPose(), cell());
        assertTrue(s.toString(), s.ok());
        assertTrue("travel " + s.travelIn, s.travelIn < 2.5);
        assertEquals(37.7, s.rangeIn, 1.5);
        assertTrue("the front edge is not through the wall", Footprint.of(s.pose).bounds().minY >= -1e-9);
    }
}
