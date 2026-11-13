package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * The FLOWER numbers that follow from the official CAD, and what they mean for a 12.13 in tall robot: the plate (4.871 deep, 0.162
 * off the wall, 5.95 long), the POLLEN stack 2.634 in out from the wall, the bottom ring hole (3.04 in in the CAD, 2.79 in in
 * the manual: the SMALLER one is used for clearance decisions), the retrieval opening, and the wall bracket that sticks out 1.67 in
 * at 11.44 to 11.84 in above the floor, inside the robot's height.
 *
 * <p>How it works: no robot. The numbers are written out by hand from the CAD report; the tests check that the constants, the
 * boxes the validator uses and the press-in geometry all agree with them, and that a start over the bracket is reported.
 */
public class FlowerCadNumbersTest {
    private static final double EPS = 1e-9;
    private final AllianceField red = new AllianceField(new FieldModel(), Alliance.RED);

    /**
     * Situation: the plate, the POLLEN stack and the press numbers of the FLOWER on the red wall.
     * Expected: the plate's field-side edge is 0.162 + 4.871 = 5.033 in from the wall and it is 5.95 in long; the POLLEN center line
     *     is 2.634 in out, so the bottom POLLEN spans 1.234 to 4.034 in; the wheel tip stops 2.833 in from the wall, 1.201 in into the
     *     POLLEN, and the chassis then stands 5.533 in out, exactly 0.5 in off the plate with the 2.7 in wheel reach.
     * Why: these are the derived numbers the press-in rests on; each one moved when the field frame changed.
     */
    @Test
    public void thePlateAndThePressNumbersFollowFromTheCad() {
        assertEquals(5.033, BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN, 1e-9);
        assertEquals(5.95, BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN, EPS);
        Rect plate = red.flowerPlate(0);                       // on the red wall: x from the wall (0) out to 5.033, 5.95 along y
        assertEquals(0.0, plate.minX, EPS);
        assertEquals(5.033, plate.maxX, 1e-9);
        assertEquals(5.95, plate.maxY - plate.minY, 1e-9);
        assertEquals("the plate is centered on the POLLEN stack", BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1], (plate.minY + plate.maxY) / 2.0, 1e-9);
        assertEquals(2.634, BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN, EPS);
        assertEquals("the FLOWER center is the POLLEN center line", BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN,
                BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][0], EPS);
        double pollenFar = BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN + BioBuzzConstants.Auto.POLLEN_DIAMETER_IN / 2.0;
        assertEquals(4.034, pollenFar, 1e-9);
        assertEquals(2.833, red.flowerPressStopLineIn(), 1e-9);
        assertEquals(1.201, pollenFar - red.flowerPressStopLineIn(), 1e-9);
        assertEquals(2.7, red.flowerRequiredReachIn(), 1e-9);
        assertEquals(5.533, red.flowerPressStopLineIn() + BioBuzzConstants.Robot.INTAKE_REACH_IN, 1e-9);
        assertEquals(0.5, red.flowerPressStopLineIn() + BioBuzzConstants.Robot.INTAKE_REACH_IN
                - BioBuzzConstants.Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN, 1e-9);
        // the wheel never goes closer than the center line of the stack
        assertTrue(red.flowerPressStopLineIn() >= BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN);
    }

    /**
     * Situation: the bottom ring hole, where the CAD (3.04 in) and the manual (2.79 in) disagree by 0.25 in.
     * Expected: both figures are kept, and the one used for clearance decisions is the SMALLER, 2.79 in. A POLLEN (2.8 in) is
     *     then 0.01 in wider than that hole: with the manual's hole the ball is held, not loose.
     * Negative: the planning value is not the CAD one; it would be 0.25 in too generous.
     */
    @Test
    public void theBottomHoleUsesTheSmallerOfTheTwoDisagreeingFigures() {
        assertEquals(3.04, BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_CAD_IN, EPS);
        assertEquals(2.79, BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_MANUAL_IN, EPS);
        assertEquals(2.79, BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_PLANNING_IN, EPS);
        assertTrue(BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_PLANNING_IN <= BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_CAD_IN);
        assertTrue(BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_PLANNING_IN <= BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_MANUAL_IN);
        assertEquals(0.25, BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_CAD_IN - BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_PLANNING_IN, EPS);
        assertTrue("a POLLEN is wider than the manual's hole", BioBuzzConstants.Auto.POLLEN_DIAMETER_IN > BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_PLANNING_IN);
        assertTrue("and narrower than the CAD's", BioBuzzConstants.Auto.POLLEN_DIAMETER_IN < BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_CAD_IN);
    }

    /**
     * Situation: the retrieval opening, the lip and the middle ring.
     * Expected: the opening is 3.55 in tall (a 2.8 in POLLEN rolls out under it, a 3.6 in NECTAR does not fit through the 3.04 / 2.79
     *     in hole above it), 3.58 in deep; the lip is 0.354 in; the middle ring passes even a NECTAR (its hole is at least 3.76 in).
     */
    @Test
    public void theRetrievalOpeningTheLipAndTheMiddleRingMatchTheCad() {
        assertEquals(3.55, BioBuzzConstants.Field.FLOWER_RETRIEVAL_OPENING_HEIGHT_IN, EPS);
        assertEquals(3.58, BioBuzzConstants.Field.FLOWER_RETRIEVAL_OPENING_DEPTH_IN, EPS);
        assertTrue(BioBuzzConstants.Auto.POLLEN_DIAMETER_IN < BioBuzzConstants.Field.FLOWER_RETRIEVAL_OPENING_HEIGHT_IN);
        double nectar = 3.6;
        assertTrue("a NECTAR does not fit the bottom ring hole, whichever figure is right", nectar > BioBuzzConstants.Field.FLOWER_BOTTOM_HOLE_CAD_IN);
        assertTrue("it does fit the middle ring (second derivation: at least 3.76 in), so the bottom hole is what keeps it in",
                nectar < BioBuzzConstants.Field.FLOWER_MIDDLE_RING_HOLE_MIN_IN);
        assertEquals(0.354, BioBuzzConstants.Field.FLOWER_BOTTOM_RING_LIP_CAD_IN, EPS);
    }

    /**
     * Situation: the wall bracket.
     * Expected: it sticks out 1.67 in at 11.437 to 11.837 in above the tile, which is inside the robot's 12.13 in; it is 6.67 in
     *     long, 0.36 in longer than the plate on each side, which the 0.5 in keep-out margin around a plate covers.
     */
    @Test
    public void theWallBracketIsInsideTheRobotsHeightAndItsOverhangIsInsideTheMargin() {
        assertEquals(1.67, BioBuzzConstants.Field.FLOWER_BRACKET_PROTRUSION_IN, EPS);
        assertEquals(11.437, BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN, EPS);
        assertEquals(11.837, BioBuzzConstants.Field.FLOWER_BRACKET_TOP_Z_IN, EPS);
        assertTrue("a part of the robot that is taller than the bottom of the bracket can touch it",
                BioBuzzConstants.Robot.HEIGHT_IN > BioBuzzConstants.Field.FLOWER_BRACKET_BOTTOM_Z_IN);
        Rect bracket = red.flowerBracket(0);
        assertEquals(0.0, bracket.minX, EPS);
        assertEquals(1.67, bracket.maxX, 1e-9);
        assertEquals(6.67, bracket.maxY - bracket.minY, 1e-9);
        Rect plate = red.flowerPlate(0);
        double overhang = (bracket.maxY - bracket.minY - (plate.maxY - plate.minY)) / 2.0;
        assertEquals(0.36, overhang, 1e-9);
        assertTrue("the plate keep-out margin covers the bracket's overhang", BioBuzzConstants.Auto.FLOWER_KEEPOUT_MARGIN_IN >= overhang);
    }

    private static List<Issue> start(double x, double y, double headingDeg) {
        return RouteValidator.check(AutoRoute.builder("x").start(x, y, headingDeg).waitSeconds(1).build());
    }

    private static int count(List<Issue> issues, Severity severity) {
        int n = 0;
        for (Issue i : issues) {
            if (i.rule == Rule.START_G304 && i.severity == severity) {
                n++;
            }
        }
        return n;
    }

    /**
     * Situation: a robot starting on the red wall with its BACK to the wall (heading 0, center 8.03 in out) next to the FLOWER on
     *     that wall, so that its side stands 0.18 in over the end of the bracket but clear of the plate; then 0.2 in farther along
     *     the wall, clear of the bracket too; then 0.3 in over the PLATE.
     * Expected: the first start gets a WARN that names the bracket (and no ERROR), the second is clean, the third is the G304.D
     *     ERROR.
     * Why: the bracket is 0.36 in longer than the plate at each end, and sits at a height the robot reaches.
     */
    @Test
    public void aStartOverTheEndOfTheBracketIsAWarnButOverThePlateIsAnError() {
        double bracketEnd = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1] + BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN / 2.0;
        double side = BioBuzzConstants.Robot.RIGHT_EXTENT_IN;                        // facing +x the robot's right side is toward -y
        List<Issue> over = start(BioBuzzConstants.Robot.REAR_EXTENT_IN, bracketEnd - 0.18 + side, 0);
        assertEquals(over.toString(), 1, count(over, Severity.WARN));
        assertEquals(over.toString(), 0, count(over, Severity.ERROR));
        boolean named = false;
        for (Issue i : over) {
            named |= i.message.contains("wall bracket of FLOWER 1");
        }
        assertTrue(over.toString(), named);
        List<Issue> clear = start(BioBuzzConstants.Robot.REAR_EXTENT_IN, bracketEnd + 0.02 + side, 0);
        assertTrue(clear.toString(), clear.isEmpty());
        double plateEnd = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1] + BioBuzzConstants.Field.FLOWER_PLATE_ALONG_WALL_IN / 2.0;
        List<Issue> onPlate = start(BioBuzzConstants.Robot.REAR_EXTENT_IN, plateEnd - 0.3 + side, 0);
        assertEquals(onPlate.toString(), 1, count(onPlate, Severity.ERROR));
        // the footprint really does sit where the test says
        Footprint fp = Footprint.of(Pose2.ofDegrees(BioBuzzConstants.Robot.REAR_EXTENT_IN, bracketEnd - 0.18 + side, 0));
        assertTrue(fp.overlaps(red.flowerBracket(0)));
        assertFalse(fp.overlaps(red.flowerPlate(0)));
    }
}
