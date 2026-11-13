package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;

/**
 * The BIOBUZZ field is turned half a turn about its center when you swap alliances: blue = (size - x, size - y, heading + 180
 * degrees), with the size and the center of the official CAD (141.348 in and (70.674, 70.674) in the wall-corner frame). The CAD
 * confirms the symmetry to 0.000 in for the walls, tiles, FLOWERS and POLLEN and to 0.017 in for the HIVE parts. These tests pin it
 * for everything the code knows about the field, so that a typo in one red number (or a field size that is not the center
 * times two) shows up as a broken symmetry, not as a robot that drives to the wrong place for one alliance.
 *
 * <p>Words used: <b>half turn</b> is a rotation by 180 degrees about the field center (NOT a mirror image: a mirror would put
 * the blue HIVE's audience cell on the wrong side); <b>image</b> of a thing is where the half turn puts it.
 */
public class FieldSymmetryTest {
    private static final double EPS = 1e-9;
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private final FieldModel model = new FieldModel();
    private final AllianceField red = new AllianceField(model, Alliance.RED);
    private final AllianceField blue = new AllianceField(model, Alliance.BLUE);

    /** The half-turn image of a point. */
    private static Point2 half(double x, double y) {
        return new Point2(SIZE - x, SIZE - y);
    }

    /**
     * Situation: the field center and the field size.
     * Expected: the center is exactly half the size, 70.674 in, and the half turn is the Pedro half turn about that point.
     */
    @Test
    public void theCenterIsHalfTheSizeAndTheHalfTurnGoesAboutIt() {
        assertEquals(70.674, BioBuzzConstants.Field.CENTER_IN, 1e-9);
        assertEquals(2.0 * BioBuzzConstants.Field.CENTER_IN, SIZE, EPS);
        assertEquals(BioBuzzConstants.Field.CENTER_IN, model.center(), EPS);
        Pose2 p = Pose2.ofDegrees(40.0, 20.0, 30.0);
        Pose2 q = Alliance.BLUE.fromRed(p, SIZE);
        // the middle of the two poses is the field center, and they face opposite ways
        assertEquals(BioBuzzConstants.Field.CENTER_IN, (p.x + q.x) / 2.0, EPS);
        assertEquals(BioBuzzConstants.Field.CENTER_IN, (p.y + q.y) / 2.0, EPS);
        assertEquals(Math.PI, Angles.distance(p.heading, q.heading), 1e-9);
        // turning twice gives the pose back
        Pose2 back = Alliance.BLUE.toRed(q, SIZE);
        assertEquals(p.x, back.x, EPS);
        assertEquals(p.y, back.y, EPS);
        assertEquals(0.0, Angles.distance(p.heading, back.heading), 1e-9);
    }

    /**
     * Situation: the four FLOWERS (red wall, rear wall, blue wall, audience wall).
     * Expected: the blue-wall FLOWER is the half-turn image of the red-wall one, and the audience-wall one of the rear-wall one.
     */
    @Test
    public void theFlowersComeInHalfTurnPairs() {
        double[][] f = BioBuzzConstants.Field.FLOWER_CENTERS_IN;
        for (int i = 0; i < 2; i++) {
            Point2 image = half(f[i][0], f[i][1]);
            assertEquals("FLOWER " + (i + 3) + " x", image.x, f[i + 2][0], 1e-9);
            assertEquals("FLOWER " + (i + 3) + " y", image.y, f[i + 2][1], 1e-9);
        }
        // and so are their plates and keep-outs (the plate is a box from the wall, so its image is the other wall's plate)
        for (int i = 0; i < 2; i++) {
            Rect a = red.flowerPlate(i);
            Rect b = red.flowerPlate(i + 2);
            assertEquals(SIZE - a.maxX, b.minX, 1e-9);
            assertEquals(SIZE - a.minX, b.maxX, 1e-9);
            assertEquals(SIZE - a.maxY, b.minY, 1e-9);
            assertEquals(SIZE - a.minY, b.maxY, 1e-9);
            Rect ka = model.flowerKeepOuts().get(i);
            Rect kb = model.flowerKeepOuts().get(i + 2);
            assertEquals(SIZE - ka.maxX, kb.minX, 1e-9);
            assertEquals(SIZE - ka.minY, kb.maxY, 1e-9);
        }
    }

    /**
     * Situation: the zones of the two alliances (LOADING ZONE and GARDEN) and the 4 GARDEN POLLEN of each.
     * Expected: every blue one is the half-turn image of the red one.
     */
    @Test
    public void theZonesAndTheGardenPollenAreHalfTurnImages() {
        for (Rect[] pair : new Rect[][] {{red.loadingZone(Alliance.RED), red.loadingZone(Alliance.BLUE)},
                {red.garden(Alliance.RED), red.garden(Alliance.BLUE)}}) {
            assertEquals(SIZE - pair[0].maxX, pair[1].minX, EPS);
            assertEquals(SIZE - pair[0].minX, pair[1].maxX, EPS);
            assertEquals(SIZE - pair[0].maxY, pair[1].minY, EPS);
            assertEquals(SIZE - pair[0].minY, pair[1].maxY, EPS);
        }
        List<Point2> r = red.gardenPollen(Alliance.RED);
        List<Point2> b = red.gardenPollen(Alliance.BLUE);
        for (int i = 0; i < r.size(); i++) {
            assertEquals(SIZE - r.get(i).x, b.get(i).x, EPS);
            assertEquals(SIZE - r.get(i).y, b.get(i).y, EPS);
        }
        // the zones keep to their own half, and the HIVE frame is its own image
        assertTrue(red.loadingZone(Alliance.RED).maxX < BioBuzzConstants.Field.CENTER_IN);
        assertTrue(red.loadingZone(Alliance.BLUE).minX > BioBuzzConstants.Field.CENTER_IN);
        Rect frame = model.hiveFrame();
        assertEquals(SIZE - frame.maxX, frame.minX, EPS);
        assertEquals(SIZE - frame.maxY, frame.minY, EPS);
    }

    /**
     * Situation: the four HIVE cells.
     * Expected: the blue cell on one side is the half-turn image of the red cell on the OPPOSITE side (aim point, tag row and the
     *     tag centers), and the HIVE axes are 25.5 in apart, 12.75 in either side of the center.
     * Why: this is why a mirror would be wrong. (The CAD's own HIVE parts are symmetric to 0.017 in, see CadTagCentersTest.)
     */
    @Test
    public void theHiveCellsAreHalfTurnImagesOfTheCellsOnTheOtherSide() {
        assertEquals(25.5, model.cell(Alliance.BLUE, Side.REAR).axisX - model.cell(Alliance.RED, Side.AUDIENCE).axisX, EPS);
        assertEquals(BioBuzzConstants.Field.CENTER_IN - 12.75, model.cell(Alliance.RED, Side.AUDIENCE).axisX, EPS);
        for (Side side : Side.values()) {
            CellSpec redCell = model.cell(Alliance.RED, side);
            CellSpec blueCell = model.cell(Alliance.BLUE, side.opposite());
            Point2 aim = half(redCell.originX, redCell.originY);
            assertEquals(aim.x, blueCell.originX, EPS);
            assertEquals(aim.y, blueCell.originY, EPS);
            assertEquals(redCell.originZ, blueCell.originZ, EPS);
            assertEquals(SIZE - redCell.tagRowY, blueCell.tagRowY, EPS);
            assertEquals(SIZE - redCell.downTagRowY, blueCell.downTagRowY, EPS);
            for (int i = 0; i < 4; i++) {
                assertEquals("tag " + i, SIZE - redCell.tagFieldX(i), blueCell.tagFieldX(i), EPS);
            }
            // the opening faces the other way
            assertEquals(Math.PI, Angles.distance(redCell.facingHeading, blueCell.facingHeading), 1e-9);
        }
    }

    /**
     * Situation: the field as each alliance sees it: its own half and the center line.
     * Expected: the blue half is the red half turned; the center line is the same line for both; the same point is "own side" for
     *     exactly one alliance, except on the line.
     */
    @Test
    public void theTwoHalvesAreHalfTurnImagesAndMeetOnTheCenterLine() {
        Rect own = red.ownHalf();
        Rect other = blue.ownHalf();
        assertEquals(0.0, own.minX, EPS);
        assertEquals(BioBuzzConstants.Field.CENTER_IN, own.maxX, EPS);
        assertEquals(SIZE - own.maxX, other.minX, EPS);
        assertEquals(SIZE, other.maxX, EPS);
        for (double x = 0.0; x <= SIZE; x += 7.3) {
            if (Math.abs(x - BioBuzzConstants.Field.CENTER_IN) > 1e-6) {
                assertTrue("x = " + x, red.isOwnSide(x) != blue.isOwnSide(x));
            }
        }
        assertTrue(red.isOwnSide(BioBuzzConstants.Field.CENTER_IN) && blue.isOwnSide(BioBuzzConstants.Field.CENTER_IN));
    }
}
