package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Checks {@link FieldModel}, the robot's map of the BIOBUZZ field: the four HIVE cells with their aim points and
 * AprilTag IDs and positions, the HIVE frame, the four FLOWER keep-out boxes, and the red-to-blue flip. It also checks
 * {@link Rect}, the rectangle helper the path planner uses to ask 'does this line cut through that box?'.
 *
 * <p>How it works: plain unit tests with no robot. Each test builds the model from the numbers in
 * BioBuzzConstants.Field and compares them with figures worked out by hand from the game manual drawings (section 9)
 * and the FTC SDK's AprilTag table. If someone mistypes a field constant or breaks the flip, a test here fails before
 * the robot ever drives.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*FieldModelTest"} to run only this
 * class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Field frame:</b> field coordinates in inches. The origin is the corner where the audience wall meets the
 *       red wall, +x points toward the blue side and +y points away from the audience. The field is 141.348 x 141.348 in
 *       (the official CAD), so the center is (70.674, 70.674).</li>
 *   <li><b>HIVE, CELL, aim point:</b> each alliance has one HIVE with two CELLs (an audience-side cell and a
 *       rear-side cell). Exactly one is tipped UP. The center of an UP cell's opening is the aim point (called the
 *       'origin' in the code).</li>
 *   <li><b>AprilTag:</b> a black-and-white square code the camera can read. Each cell carries four tags with
 *       consecutive IDs: 30-33 red rear, 34-37 red audience, 38-41 blue audience, 42-45 blue rear.</li>
 *   <li><b>Keep-out:</b> a rectangle the robot must not drive into (the HIVE frame, the FLOWERS).</li>
 *   <li><b>Epsilon (EPS) / tolerance:</b> how far two decimal numbers may differ and still count as equal. Here one
 *       millionth of an inch: only rounding, no real measurement error.</li>
 *   <li><b>Fixture:</b> the shared setup for the tests. The {@code field} below is built fresh for every test
 *       (JUnit makes a new copy of this class per test method).</li>
 * </ul>
 */
public class FieldModelTest {
    /** Allowed rounding error when comparing two decimal numbers (inches or degrees): one millionth. */
    private static final double EPS = 1e-6;
    /** The field model built from the real BioBuzzConstants, shared by the tests in this class. */
    private final FieldModel field = new FieldModel();

    /**
     * Situation: the model's two RED cells.
     * Expected: both aim points are on the red HIVE axis x = 57.924 in (12.75 in left of the field center); the audience
     *     cell is at y = 54.156 and the rear cell at y = 87.192 (16.518 in either side of the field center y = 70.674),
     *     both 58.254 in high; their base tag IDs are 34 (audience) and 30 (rear).
     * Why: these are the 'cluster origin' numbers the FTC SDK's AprilTag database implies, placed with the CAD (the
     *     origin of the cluster is 16.518 in out from the pivot plane). A typo here would mis-aim every shot.
     */
    @Test
    public void redCellsSitOnTheRedHiveAxisAtTheSdkClusterOrigin() {
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        assertEquals(57.924, aud.originX, EPS);
        assertEquals(54.156, aud.originY, EPS);
        assertEquals(58.254, aud.originZ, EPS);
        assertEquals(57.924, rear.originX, EPS);
        assertEquals(87.192, rear.originY, EPS);
        assertEquals(34, aud.baseTagId);
        assertEquals(30, rear.baseTagId);
    }

    /**
     * Situation: the model's two BLUE cells.
     * Expected: each is the half-turn image (about the field center (70.674, 70.674)) of the RED cell on the OPPOSITE side:
     *     x = 141.348 - 57.924 = 83.424; BLUE AUDIENCE comes from RED REAR (tags 38 and up) and BLUE REAR from RED
     *     AUDIENCE (tags 42 and up). Height is unchanged.
     * Why: the field is rotationally symmetric, NOT mirror symmetric, so a mirror-image shortcut would put the blue
     *     cells in the wrong place.
     */
    @Test
    public void blueCellsAreTheHalfTurnImagesOfTheRedCellsOnTheOppositeSide() {
        CellSpec bAud = field.cell(Alliance.BLUE, Side.AUDIENCE);
        CellSpec bRear = field.cell(Alliance.BLUE, Side.REAR);
        assertEquals(83.424, bAud.originX, EPS);
        assertEquals(54.156, bAud.originY, EPS);
        assertEquals(83.424, bRear.originX, EPS);
        assertEquals(87.192, bRear.originY, EPS);
        assertEquals(38, bAud.baseTagId);
        assertEquals(42, bRear.baseTagId);
        assertEquals(58.254, bAud.originZ, EPS);
    }

    /**
     * Situation: all four cells (both alliances).
     * Expected: the audience cell's opening faces -90 degrees (toward the audience wall, -y) and sits below the field
     *     center; the rear cell's opening faces +90 degrees and sits above it.
     * Why: the robot must stand outside the opening, on the side it faces, with its back (the shooter) toward the cell.
     */
    @Test
    public void everyOpeningFacesAwayFromTheHiveOnItsOwnSide() {
        for (Alliance a : Alliance.values()) {
            CellSpec aud = field.cell(a, Side.AUDIENCE);
            CellSpec rear = field.cell(a, Side.REAR);
            assertEquals(-90.0, Math.toDegrees(aud.facingHeading), EPS);
            assertEquals(90.0, Math.toDegrees(rear.facingHeading), EPS);
            assertTrue(aud.originY < field.center());
            assertTrue(rear.originY > field.center());
        }
    }

    /**
     * Situation: the field x of all four tags of all four cells.
     * Expected: they match the table in the test (the HIVE axes 57.924 and 83.424 plus the SDK offsets -6.5, -2.75, 2.75, 6.5).
     *     The audience-side tags run left to right with increasing ID, toward +x; a rear cluster's sticker is turned half a
     *     turn, so its IDs run toward -x. (CadTagCentersTest pins them against the official CAD tag centers.)
     * Why: ClusterAnalyzer uses these positions to work out where the robot is, and the ID order to tell an UP cell
     *     from a DOWN one.
     */
    @Test
    public void tagFieldPositionsMatchTheVerifiedClusterTable() {
        // Field x of tags 0 to 3 of each cell: the cell's axis plus the offsets -6.5, -2.75, 2.75 and 6.5 in; the rear
        // cells run the other way.
        double[] redAud = {51.424, 55.174, 60.674, 64.424};
        double[] redRear = {64.424, 60.674, 55.174, 51.424};
        double[] blueAud = {76.924, 80.674, 86.174, 89.924};
        double[] blueRear = {89.924, 86.174, 80.674, 76.924};
        for (int i = 0; i < 4; i++) {
            assertEquals(redAud[i], field.cell(Alliance.RED, Side.AUDIENCE).tagFieldX(i), EPS);
            assertEquals(redRear[i], field.cell(Alliance.RED, Side.REAR).tagFieldX(i), EPS);
            assertEquals(blueAud[i], field.cell(Alliance.BLUE, Side.AUDIENCE).tagFieldX(i), EPS);
            assertEquals(blueRear[i], field.cell(Alliance.BLUE, Side.REAR).tagFieldX(i), EPS);
        }
    }

    /**
     * Situation: tag IDs at the edges of each block of four (30, 33, 34, 41, 45) and just outside the range (29, 46).
     * Expected: cellForTag returns the matching cell (the very same object, checked with assertSame) or null for an ID
     *     that is not a HIVE tag; tagIndex(36) is 2 (the third tag of the red audience cell) and -1 for a tag of
     *     another cell.
     * Why: the camera can see tags that are not ours. We must never treat them as part of our HIVE.
     */
    @Test
    public void tagIdsMapToTheRightCell() {
        assertSame(field.cell(Alliance.RED, Side.REAR), field.cellForTag(30));
        assertSame(field.cell(Alliance.RED, Side.REAR), field.cellForTag(33));
        assertSame(field.cell(Alliance.RED, Side.AUDIENCE), field.cellForTag(34));
        assertSame(field.cell(Alliance.BLUE, Side.AUDIENCE), field.cellForTag(41));
        assertSame(field.cell(Alliance.BLUE, Side.REAR), field.cellForTag(45));
        assertNull(field.cellForTag(29));
        assertNull(field.cellForTag(46));
        assertEquals(2, field.cell(Alliance.RED, Side.AUDIENCE).tagIndex(36));
        assertEquals(-1, field.cell(Alliance.RED, Side.AUDIENCE).tagIndex(30));
    }

    /**
     * Situation: the state of each HIVE when the match starts.
     * Expected: the RED HIVE's audience-side cell is UP and the BLUE HIVE's rear-side cell is UP (manual Fig 10-2).
     * Why: it is the tracker's first guess, used until the camera confirms which cell is really up.
     */
    @Test
    public void initialHiveStateMatchesTheSetupFigure() {
        assertEquals(Side.AUDIENCE, field.initialUpSide(Alliance.RED));
        assertEquals(Side.REAR, field.initialUpSide(Alliance.BLUE));
    }

    /**
     * Situation: the HIVE frame (its feet and base rails) as a rectangle.
     * Expected: it is 49.466 in wide (x) and 38.944 in deep (y) (the two foot bars of the CAD), centered on the field center
     *     (70.674, 70.674): x from 45.941 to 95.407 and y from 51.202 to 90.146, and it contains the field center.
     * Why: the path planner drives around this rectangle, so its size and position must be right.
     */
    @Test
    public void hiveFrameFootprintIsCenteredOnTheField() {
        Rect f = field.hiveFrame();
        assertEquals(45.941, f.minX, EPS);
        assertEquals(95.407, f.maxX, EPS);
        assertEquals(51.202, f.minY, EPS);
        assertEquals(90.146, f.maxY, EPS);
        assertTrue(f.contains(BioBuzzConstants.Field.CENTER_IN, BioBuzzConstants.Field.CENTER_IN));
    }

    /**
     * Situation: the four FLOWER keep-out boxes.
     * Expected: there are four, and each is Field.FLOWER_KEEPOUT_ALONG_WALL_IN long along its wall and
     *     Field.FLOWER_KEEPOUT_DEPTH_IN deep out from it (the red-wall one is long in y and deep in x; the rear-wall one
     *     is long in x and deep in y).
     * Why: the planner keeps the robot out of these boxes. A box turned the wrong way would block the wrong part of the
     *     field.
     */
    @Test
    public void flowerKeepOutsHugTheirWallsWithTheLongSideAlongTheWall() {
        assertEquals(4, field.flowerKeepOuts().size());
        double along = BioBuzzConstants.Field.FLOWER_KEEPOUT_ALONG_WALL_IN;
        double depth = BioBuzzConstants.Field.FLOWER_KEEPOUT_DEPTH_IN;
        Rect left = field.flowerKeepOuts().get(0);     // on the red wall
        assertEquals(along, left.maxY - left.minY, EPS); // long side along y
        assertEquals(depth, left.maxX - left.minX, EPS);
        Rect rear = field.flowerKeepOuts().get(1);     // on the rear wall
        assertEquals(along, rear.maxX - rear.minX, EPS); // long side along x
        assertEquals(depth, rear.maxY - rear.minY, EPS);
    }

    /**
     * Situation: a RED pose (35, 9.2) facing -90 degrees converted to BLUE.
     * Expected: the result is (106.348, 132.148) facing +90 degrees, i.e. blue = (141.348 - x, 141.348 - y, heading + 180 degrees);
     *     converting back gives the original; converting RED to RED returns the very same object.
     * Why: autonomous routines and the planner are written once for RED and flipped for BLUE.
     */
    @Test
    public void allianceFlipIsAHalfTurnAboutTheFieldCenter() {
        Pose2 red = Pose2.ofDegrees(35.0, 9.2, -90);
        Pose2 blue = Alliance.BLUE.fromRed(red, field.size());
        assertEquals(106.348, blue.x, EPS);
        assertEquals(132.148, blue.y, EPS);
        assertEquals(90.0, blue.headingDegrees(), EPS);
        Pose2 back = Alliance.BLUE.toRed(blue, field.size());
        assertEquals(red.x, back.x, EPS);
        assertEquals(red.y, back.y, EPS);
        // 1e-9 degrees: rounding only.
        assertEquals(red.headingDegrees(), back.headingDegrees(), 1e-9);
        assertSame(red, Alliance.RED.fromRed(red, field.size()));
    }

    /**
     * Pins the team's alliance flip to Pedro's own half-turn operation so the two can never disagree.
     *
     * Situation: four sample poses flipped two ways: by our Alliance.fromRed and by Pedro Pathing's own
     *     PoseFactory.mirrorAroundPoint(center, center) about the field center. (Pedro Pathing is the library that follows paths.)
     * Expected: x, y and heading agree to within EPS. Headings are compared with Angles.distance, because +180 and
     *     -180 degrees are the same direction.
     * Why: autonomous paths may use Pedro's flip while the auto-shoot code uses ours. If they ever disagreed, the robot
     *     would believe it stood somewhere it does not.
     */
    @Test
    public void allianceFlipAgreesWithPedroMirrorAroundPoint() {
        PoseFactory half = PoseFactory.degrees().mirrorAroundPoint(BioBuzzConstants.Field.CENTER_IN, BioBuzzConstants.Field.CENTER_IN);
        double[][] samples = {{35.0, 9.2, -90}, {58.0, 9.2, -90}, {12.0, 90.0, 0}, {100.0, 20.0, 135}};
        for (double[] s : samples) {
            Pose pedro = half.of(s[0], s[1], s[2]);
            Pose2 mine = Alliance.BLUE.fromRed(Pose2.ofDegrees(s[0], s[1], s[2]), BioBuzzConstants.Field.SIZE_IN);
            assertEquals(pedro.x(), mine.x, EPS);
            assertEquals(pedro.y(), mine.y, EPS);
            double diff = Angles.distance(pedro.heading(), mine.heading);
            assertTrue("heading differs by " + diff, diff < 1e-6);
        }
    }

    /**
     * Situation: a 10 x 10 in box (x from 10 to 20, y from 10 to 20) and several straight line segments.
     * Expected: a segment through the inside, fully inside, or ending inside counts as a crossing; a segment that only
     *     runs along an edge, touches a corner or misses does not. The last two lines check that inflate(1) returns a
     *     bigger box that now contains the point (9.5, 9.5), just outside the original.
     * Why: the planner may legally drive right along the edge of a keep-out, but must never cut through it.
     */
    @Test
    public void rectSegmentTestIgnoresGrazingAndFindsRealCrossings() {
        Rect r = new Rect(10, 20, 10, 20);
        assertTrue(r.segmentCrossesInterior(0, 15, 30, 15));
        assertTrue(r.segmentCrossesInterior(15, 0, 15, 30));
        assertFalse(r.segmentCrossesInterior(0, 10, 30, 10));    // runs along the bottom edge
        assertFalse(r.segmentCrossesInterior(0, 0, 10, 10));     // touches a corner
        assertFalse(r.segmentCrossesInterior(0, 25, 30, 25));    // misses
        assertTrue(r.segmentCrossesInterior(12, 12, 18, 18));    // fully inside
        assertTrue(r.segmentCrossesInterior(5, 5, 15, 15));      // ends inside
        assertNotNull(r.inflate(1));
        assertTrue(r.inflate(1).contains(9.5, 9.5));
    }
}
