package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Locale;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The ANCHOR SHOT: the one place where the team saw the shot work. With the robot's FRONT pressed against the audience wall and in line
 * with the HIVE, the POLLEN passes through the center of the opening to slightly above it at about 1200 ticks per second.
 *
 * <p><b>The distance of that spot was NOT measured with a tape.</b> The model COMPUTES it from the field and the robot: the aim point of the
 * UP cell is 54.156 in from the audience wall (the official CAD), the center of rotation is the front extent (9.77 in) from the wall because the
 * FRONT edge touches it, and the ball leaves the robot 5.38 in behind the center of rotation (measured), so the ball exit is 9.77 + 5.38 = 15.15 in
 * from the wall and the range is 54.156 - 15.15 = 39.006 in. The shot-table row {39.0, 1200}, {@code AutoShoot.PREFERRED_RANGE_IN} and
 * {@code Shooter.TUNER_START_RANGE_IN} all follow that computed range: this test fails if somebody moves the ball exit (a new measurement) and
 * forgets to move them.
 *
 * <p>Because the range is computed, the guide (docs/START_HERE.md) and the Javadoc of the shot table both tell the team to check it: "re-measure the
 * distance from the ball exit to the cell opening center with a tape", and to check the row with the Shooter Tuner. This test makes sure both keep
 * saying so, and that neither calls the number "measured" or quotes the old 37.7 in.
 *
 * <p>Words used: <b>ball exit</b> is where the POLLEN leaves the shooter (5.38 in behind the center of rotation, 0.16 in right of the center line);
 * <b>aim point</b> is the center of the UP cell's opening; <b>range</b> is the horizontal distance between the two.
 */
public class AnchorShotTest {
    /** The wording the checklist and the constants must contain: the team checks the computed anchor distance with a tape. */
    private static final String REMEASURE = "re-measure the distance from the ball exit to the cell opening center with a tape";

    private final FieldModel field = new FieldModel();
    private final ShotTable table = ShotTable.fromConstants();

    /** The audience cell of the red HIVE, the one that is UP at the start of the match. */
    private CellSpec cell() {
        return field.cell(Alliance.RED, Side.AUDIENCE);
    }

    /** The robot with its front edge on the audience wall, its middle over the HIVE axis, facing the wall. */
    private Pose2 anchorPose() {
        return new Pose2(cell().axisX, Footprint.frontExtentIn(), -Math.PI / 2.0);
    }

    private double modelRange() {
        CellSpec cell = cell();
        return ShotGeometry.rangeTo(anchorPose(), BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN,
                cell.originX, cell.originY);
    }

    /** The row of the shot table that is the anchor: the one nearest to the model's range. */
    private double[] anchorRow() {
        double[] best = null;
        for (double[] row : BioBuzzConstants.Shooter.SHOT_TABLE) {
            if (best == null || Math.abs(row[0] - modelRange()) < Math.abs(best[0] - modelRange())) {
                best = row;
            }
        }
        return best;
    }

    /**
     * Situation: the anchor situation (front on the wall, in line with the HIVE), worked out with the CAD field numbers and the measured ball exit.
     * Expected: the ball exit is the front extent plus its distance behind the center of rotation from the wall, the range is the aim point minus
     *     that (39.0 in), the aim is off by the sideways offset of the ball only (a fraction of a degree), and the shot table gives about 1200 ticks
     *     per second there. The test prints the numbers.
     * Why: it is the one place where the model meets what the team saw. If a field or robot number moves, the printed line shows by how much the
     *     model's picture of the shot moved.
     */
    @Test
    public void theModelsRangeForTheAnchorSpotFollowsFromTheFieldAndRobotNumbers() {
        Pose2 pose = anchorPose();
        CellSpec cell = cell();
        double range = modelRange();
        double exitY = ShotGeometry.launchY(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN);
        System.out.println(String.format(Locale.ROOT,
                "ANCHOR SHOT with the CAD field: front on the audience wall (center of rotation %.2f in from it), robot middle over the HIVE"
                        + " axis x = %.3f; ball exit %.2f in from the wall at %.2f in behind the center and %.2f in left of it; aim point (%.3f, %.3f);"
                        + " model range = %.3f in; shot table at that range = %.0f ticks per second (seen: about 1200)",
                pose.y, pose.x, exitY, -BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN,
                cell.originX, cell.originY, range, table.velocityAt(range)));
        assertEquals("the ball exit is the front extent plus its distance behind the center of rotation from the wall",
                BioBuzzConstants.Robot.FRONT_EXTENT_IN - BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, exitY, 1e-9);
        assertEquals("the CAD aim point is 54.156 in from the wall", 54.156, cell.originY, 1e-6);
        assertEquals("range = aim point minus the exit distance (the ball is a hair off the axis: 0.16 in)", cell.originY - exitY, range, 0.01);
        assertEquals("the range the model works out for the anchor spot", 39.0, range, 0.05);
        // the ball leaves a hair to one side of the robot's middle, so the shooter is off the HIVE axis by that much and no more
        double hair = Math.toDegrees(Math.atan2(Math.abs(BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN), range));
        assertEquals("off the HIVE axis only by the sideways offset of the ball", hair,
                Math.abs(Math.toDegrees(ShotGeometry.azimuthOffAxis(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                        BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN, cell))), 1e-3);
        double aim = Math.abs(Math.toDegrees(ShotGeometry.aimError(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN, cell.originX, cell.originY)));
        assertEquals("aimed, but for the sideways offset of the ball", hair, aim, 1e-3);
        assertTrue("that hair is inside the aim tolerance", aim < BioBuzzConstants.Auto.SHOOT_AIM_TOLERANCE_DEG);
        assertEquals(1200.0, table.velocityAt(range), 15.0);
    }

    /**
     * Situation: the same robot pose worked out with the ball exit of the goBILDA drawing (the rear edge of the robot, 8.03 in behind the center of
     *     rotation) and with the ball exit the team measured (5.38 in behind it).
     * Expected: the measured exit is 2.65 in nearer the front of the robot, so for the same robot pose the ball is 2.65 in FARTHER from the HIVE.
     * Why: it shows where the change of range comes from (the real shooter, not the field or the planner), and it is the number by which every
     *     shooting pose of every route had to move closer to the HIVE.
     */
    @Test
    public void theMeasuredExitIsNearerTheFrontThanTheDrawingsAndThatMovesTheRange() {
        CellSpec cell = cell();
        double drawingRange = ShotGeometry.rangeTo(anchorPose(), -BioBuzzConstants.Robot.REAR_EXTENT_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN,
                cell.originX, cell.originY);
        assertEquals(BioBuzzConstants.Robot.REAR_EXTENT_IN + BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, modelRange() - drawingRange, 0.01);
        assertEquals("2.65 in nearer the front", 2.65, modelRange() - drawingRange, 0.01);
    }

    /**
     * Situation: the shot table, which carries the anchor row, and the two other numbers that name the anchor range.
     * Expected: one row of the table is the model's range for the anchor spot (to a twentieth of an inch) at the speed the team saw, 1200 ticks per
     *     second, and the planner's preferred range and the Shooter Tuner's start range are that same row's range.
     */
    @Test
    public void theAnchorRowOfTheShotTableFollowsTheModelsRange() {
        double[] row = anchorRow();
        assertEquals("the anchor row is the model's range for the anchor spot", modelRange(), row[0], 0.05);
        assertEquals("at the speed the team saw work", 1200.0, row[1], 1e-9);
        assertEquals(row[0], BioBuzzConstants.AutoShoot.PREFERRED_RANGE_IN, 1e-9);
        assertEquals(row[0], BioBuzzConstants.Shooter.TUNER_START_RANGE_IN, 1e-9);
    }

    /**
     * Situation: the standoff band of the route validator, and the anchor spot.
     * Expected: the spot is inside the band [SHOOT_STANDOFF_MIN_IN, SHOOT_STANDOFF_MAX_IN].
     * Why: the wall limits how far the robot can stand from the HIVE, so the band must reach the one place where the team saw the shot work, or the
     *     validator would reject the start of every route that shoots from it.
     */
    @Test
    public void theStandoffBandReachesTheAnchorSpot() {
        double range = modelRange();
        assertTrue("range " + range, range >= BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN && range <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN);
    }

    /**
     * Situation: the robot-day guide and the Javadoc of the shot table are read as text.
     * Expected: both say "re-measure the distance from the ball exit to the cell opening center with a tape", and neither quotes the old 37.7 in.
     * Why: the anchor distance is computed, never measured, so the team must be told to check it; a number that the guide still calls
     *     "measured" (37.7 in) would send them to trust it.
     */
    @Test
    public void theChecklistAndTheConstantsTellTheTeamToCheckTheComputedAnchorWithATape() throws IOException {
        String checklist = flat(read("docs/START_HERE.md"));
        assertTrue("docs/START_HERE.md must say '" + REMEASURE + "'", checklist.contains(REMEASURE));
        String constants = flat(read("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzConstants.java"));
        assertTrue("the Javadoc of Shooter.SHOT_TABLE must say '" + REMEASURE + "'", constants.contains(REMEASURE));
        assertFalse("the guide must not quote the old 37.7 in", checklist.contains("37.7"));
        assertFalse("the constants must not quote the old 37.7 in", constants.contains("37.7"));
    }

    /**
     * Situation: the planner is asked, from the anchor pose, where to stand for the audience cell.
     * Expected: it stays within 2.5 in of where it is and chooses a range within 1.5 in of the model's range for the spot, never a pose that puts
     *     the front edge through the wall.
     */
    @Test
    public void theTeleOpPlannerKeepsTheAnchorSpotAsItsAnswerWithinTheWallClearance() {
        ScoringPlanner planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(table));
        ScoringPlanner.Solution s = planner.solve(anchorPose(), cell());
        assertTrue(s.toString(), s.ok());
        assertTrue("travel " + s.travelIn, s.travelIn < 2.5);
        assertEquals(modelRange(), s.rangeIn, 1.5);
        assertTrue("the front edge is not through the wall", Footprint.of(s.pose).bounds().minY >= -1e-9);
    }

    // ---- reading the docs ------------------------------------------------------------------------------------------

    /** A repo file as text: Gradle starts the tests in the TeamCode folder, an IDE may start in the repository root. */
    private static String read(String relativeToRepoRoot) throws IOException {
        File a = new File("../" + relativeToRepoRoot);
        File b = new File(relativeToRepoRoot);
        File use = a.exists() ? a : b;
        if (!use.exists()) {
            throw new AssertionError("Cannot find " + relativeToRepoRoot + " from " + new File(".").getAbsolutePath());
        }
        return new String(Files.readAllBytes(use.toPath()), StandardCharsets.UTF_8);
    }

    /** The text with every run of whitespace, and the comment stars of a Javadoc, turned into single spaces, so a sentence that wraps still matches. */
    private static String flat(String text) {
        return text.replaceAll("(?m)^\\s*\\*", " ").replaceAll("\\s+", " ");
    }
}
