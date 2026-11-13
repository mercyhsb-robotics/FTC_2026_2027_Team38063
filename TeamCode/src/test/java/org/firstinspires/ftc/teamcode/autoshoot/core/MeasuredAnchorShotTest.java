package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Locale;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The team's one MEASURED shot: with the robot's FRONT pressed against the audience wall and in line with the HIVE, the ball
 * leaves 37.7 in from the aim point of the UP cell and the POLLEN passes through the center of the opening to slightly above
 * it at about 1200 ticks per second. That is a PHYSICAL measurement of where the robot stood, so the shot-table row {37.7, 1200}
 * is kept exactly as measured. What can change is the MODEL's idea of the same situation, because the model's range comes from
 * the field numbers: where the aim point is, how far the center of rotation is from the wall and where the ball leaves the robot.
 *
 * <p>How it works: no robot. The pose of that situation is built from the real extents (the center of rotation is the front
 * extent, 9.77 in, from the wall because the FRONT edge touches it), the model computes the range from the ball exit to the aim
 * point, and the test prints it and compares it with the 37.7 in that was measured.
 *
 * <p><b>What the official CAD changed.</b> With the old 144 in field the aim point was 55.52 in from the audience wall and the
 * model agreed with the measurement (55.52 - 17.80 = 37.72 in). The CAD puts the field walls 1.326 in nearer the field center
 * on every side, so the aim point is 54.156 in from the wall, and the model's range for the same physical shot is
 * 54.156 - 17.80 = <b>36.36 in, 1.34 in shorter</b> than the measurement. Either the tape measure measured something slightly
 * different (for example from a different point of the cell), or the ball leaves the robot farther forward than the 8.03 in
 * behind the center of rotation that the constants say: the robot CAD suggests 6.58 in, which would give 37.8 in. The row of the
 * table is not changed. The day-one checklist says to re-measure; this test makes sure that, as long as the two differ by more
 * than 1 in, the checklist and the constants say so.
 *
 * <p>Words used: <b>ball exit</b> is where the POLLEN leaves the shooter (8.03 in behind the center of rotation for now); <b>aim
 * point</b> is the center of the UP cell's opening; <b>range</b> is the horizontal distance between the two.
 */
public class MeasuredAnchorShotTest {
    /** The wording the checklist and the constants must contain while the model and the measurement disagree by more than 1 in. */
    private static final String REMEASURE = "re-measure the distance from the ball exit to the cell opening center with a tape";
    /** How far the model may differ from the measurement before the docs must say so (inches). */
    private static final double DOCUMENT_ABOVE_IN = 1.0;

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

    private double modelRange() {
        CellSpec cell = cell();
        return ShotGeometry.rangeTo(anchorPose(), BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN,
                cell.originX, cell.originY);
    }

    /**
     * Situation: the measured anchor situation (front on the wall, in line with the HIVE), worked out with the CAD field numbers.
     * Expected: the model's range is aim point minus a whole robot length (the ball exit is 9.77 + 8.03 = 17.80 in from the
     *     wall): 54.156 - 17.80 = 36.356 in. The shooter is exactly in line and aimed, and the shot table gives about 1191 ticks
     *     per second there (the row {37.7, 1200} is the measurement). The test prints the numbers.
     * Why: it is the one place where the model meets a physical measurement. If a field or robot number moves, the printed
     *     line shows by how much the model's picture of the shot moved.
     */
    @Test
    public void theModelsRangeForTheMeasuredWallShotFollowsFromTheCadNumbers() {
        Pose2 pose = anchorPose();
        CellSpec cell = cell();
        double range = modelRange();
        double exitY = ShotGeometry.launchY(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN);
        System.out.println(String.format(Locale.ROOT,
                "ANCHOR SHOT with the CAD field: front on the audience wall (center of rotation %.2f in from it), in line with the HIVE"
                        + " axis x = %.3f; ball exit %.2f in from the wall at %.2f in behind the center; aim point (%.3f, %.3f);"
                        + " model range = %.2f in (measured 37.7 in, difference %.2f in); shot table at that range = %.0f ticks per second (measured about 1200)",
                pose.y, pose.x, exitY, -BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, cell.originX, cell.originY, range,
                range - 37.7, table.velocityAt(range)));
        assertEquals("the ball exit is a whole robot length from the wall", BioBuzzConstants.Robot.LENGTH_IN, exitY, 1e-9);
        assertEquals("range = aim point minus the exit distance", cell.originY - exitY, range, 1e-9);
        assertEquals("the CAD aim point is 54.156 in from the wall", 54.156, cell.originY, 1e-6);
        assertEquals("range from the ball exit to the aim point", 36.356, range, 0.001);
        assertEquals("in line with the HIVE", 0.0, Math.toDegrees(ShotGeometry.azimuthOffAxis(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN, cell)), 1e-6);
        assertEquals("aimed", 0.0, Math.toDegrees(ShotGeometry.aimError(pose, BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN, cell.originX, cell.originY)), 1e-6);
        assertEquals(1200.0, table.velocityAt(range), 15.0);
    }

    /**
     * Situation: the same situation worked out with the OLD numbers (the 144 in field with the aim point 55.52 in from the wall
     *     and the center of rotation in the middle of the robot: 8.9 in from the wall, ball exit 8.9 in behind it).
     * Expected: the old model gave 37.72 in, the measurement. The CAD aim point is 1.364 in nearer the wall than the old one,
     *     and that is the whole difference to the new model range.
     * Why: it shows where the 1.34 in comes from: the field frame, not the robot or the planner.
     */
    @Test
    public void theWholeDifferenceToTheOldModelIsTheMovedAimPoint() {
        double oldAimY = 55.52;
        double oldRange = ShotGeometry.rangeTo(new Pose2(cell().axisX, 8.9, -Math.PI / 2.0), -8.9, 0.0, cell().axisX, oldAimY);
        assertEquals("the old model agreed with the measurement", 37.72, oldRange, 0.001);
        assertEquals("the CAD moved the aim point 1.364 in toward the wall", 1.364, oldAimY - cell().originY, 0.001);
        assertEquals(oldRange - (oldAimY - cell().originY), modelRange(), 1e-9);
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
     * Situation: the model's range for the anchor situation compared with the measured 37.7 in.
     * Expected: if they differ by more than 1 in, the day-one checklist AND the Javadoc of the shot table in BioBuzzConstants both
     *     tell the team to "re-measure the distance from the ball exit to the cell opening center with a tape".
     * Why: a disagreement between the model and the one measurement must never be left unsaid, and the table row must not be
     *     changed silently to hide it. (If the launch point is measured and the two agree again, this test asks for nothing.)
     */
    @Test
    public void aDisagreementOfMoreThanAnInchIsSaidInTheChecklistAndTheConstants() throws IOException {
        double difference = Math.abs(modelRange() - 37.7);
        if (difference <= DOCUMENT_ABOVE_IN) {
            return;
        }
        String checklist = read("docs/DAY_ONE_CHECKLIST.md");
        assertTrue("docs/DAY_ONE_CHECKLIST.md must say '" + REMEASURE + "' (the model's range differs by " + difference + " in)",
                flat(checklist).contains(REMEASURE));
        String constants = read("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzConstants.java");
        assertTrue("the Javadoc of Shooter.SHOT_TABLE must say '" + REMEASURE + "'", flat(constants).contains(REMEASURE));
    }

    /**
     * Situation: the planner is asked, from the anchor pose, where to stand for the audience cell.
     * Expected: it stays within 2.5 in of where it is (the measured spot) and chooses a range within 1.5 in of the model's range
     *     for the measured spot, never a pose that puts the front edge through the wall.
     */
    @Test
    public void theTeleOpPlannerKeepsTheMeasuredSpotAsItsAnswerWithinTheWallClearance() {
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
