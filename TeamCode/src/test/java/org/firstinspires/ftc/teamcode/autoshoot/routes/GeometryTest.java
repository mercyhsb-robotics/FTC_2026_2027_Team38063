package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.junit.Test;

/** Footprint, shot geometry and the alliance-specific zones. */
public class GeometryTest {
    private static final double EPS = 1e-6;
    /**
     * Where the robot's outline lies around its center of rotation (the pose): 9.77 in ahead, 8.03 in behind, 8.90 in to each
     * side. The robot is NOT centered on its pose.
     */
    private static final double F = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double B = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double S = BioBuzzConstants.Robot.LEFT_EXTENT_IN;

    // ---- Footprint ---------------------------------------------------------------------------------

    @Test
    public void footprintBoundsFollowTheHeading() {
        Rect facingX = Footprint.of(Pose2.ofDegrees(50, 50, 0)).bounds();
        assertEquals("rear edge 8.03 behind the pose", 50 - B, facingX.minX, EPS);
        assertEquals("front edge 9.77 ahead of it", 50 + F, facingX.maxX, EPS);
        assertEquals(50 - S, facingX.minY, EPS);
        assertEquals(50 + S, facingX.maxY, EPS);
        Rect facingDown = Footprint.of(Pose2.ofDegrees(35, F, -90)).bounds();
        assertEquals("front edge ON the audience wall", 0.0, facingDown.minY, EPS);
        assertEquals("rear edge 17.8 in from it", F + B, facingDown.maxY, EPS);
        assertEquals(35 - S, facingDown.minX, EPS);
        // the same pose, facing away: the rear edge is the near one
        Rect facingUp = Footprint.of(Pose2.ofDegrees(35, B, 90)).bounds();
        assertEquals(0.0, facingUp.minY, EPS);
        assertEquals(B + F, facingUp.maxY, EPS);
        // 45 degrees: the corners worked out by hand, a front corner reaches farthest
        Rect diagonal = Footprint.of(Pose2.ofDegrees(50, 50, 45)).bounds();
        double c = Math.sqrt(0.5);
        assertEquals(50 - c * (B + S), diagonal.minX, EPS);       // the rear-left corner (-B, +S): x = -B c - S c
        assertEquals(50 + c * (F + S), diagonal.maxX, EPS);       // the front-right corner (F, -S): x = F c + S c
    }

    @Test
    public void theFrontLeftCornerIsAheadAndToTheLeftOfTheRobot() {
        Footprint fp = Footprint.of(Pose2.ofDegrees(0, 0, 90));
        // heading +y: front is +y (9.77 in), left is -x (8.90 in)
        assertEquals(-S, fp.corner(0).x, EPS);
        assertEquals(F, fp.corner(0).y, EPS);
        // the back-right corner is 8.03 in behind and 8.90 in to the right
        assertEquals(S, fp.corner(2).x, EPS);
        assertEquals(-B, fp.corner(2).y, EPS);
    }

    @Test
    public void touchingEdgesDoNotOverlapButPenetrationDoes() {
        Rect wall = new Rect(0, 100, -10, 0);
        // robot standing exactly on the line y = 0 with its front edge
        assertFalse(Footprint.of(Pose2.ofDegrees(35, F, -90)).overlaps(wall));
        assertTrue(Footprint.of(Pose2.ofDegrees(35, F - 0.01, -90)).overlaps(wall));
        // with its back edge: the pose may be closer
        assertFalse(Footprint.of(Pose2.ofDegrees(35, B, 90)).overlaps(wall));
        assertTrue(Footprint.of(Pose2.ofDegrees(35, B - 0.01, 90)).overlaps(wall));
        // far away never overlaps
        assertFalse(Footprint.of(Pose2.ofDegrees(35, 50, -90)).overlaps(wall));
    }

    @Test
    public void aRotatedCornerThatPokesIntoABoxIsDetectedEvenIfTheBoundingBoxesOverlapAnyway() {
        // box in the upper right; a 45 degree robot whose bounding box overlaps the box but whose body does not
        Rect box = new Rect(60, 70, 60, 70);
        Pose2 p = Pose2.ofDegrees(48, 48, 45);
        Footprint fp = Footprint.of(p);
        assertTrue("premise: the bounding boxes overlap", fp.bounds().maxX > box.minX && fp.bounds().maxY > box.minY);
        assertFalse("but the rotated body stays clear of the corner", fp.overlaps(box));
        // the front edge of a 45 degree robot at (56, 56) lies along x + y = 112 + 9.77 sqrt 2 = 125.8, past the box corner at 120
        assertTrue(Footprint.of(Pose2.ofDegrees(56, 56, 45)).overlaps(box));
        // while at (52, 52) the same edge (x + y = 104 + 13.8 = 117.8) stops short of the corner
        assertFalse(Footprint.of(Pose2.ofDegrees(52, 52, 45)).overlaps(box));
    }

    @Test
    public void overshootAndInsideRectAreMeasuredFromTheBoundingBox() {
        Rect field = new Rect(0, 144, 0, 144);
        Footprint out = Footprint.of(Pose2.ofDegrees(35, 9.2, -115));
        // heading -115 at y = 9.2 (the old B1 shooting pose): the rotated robot reaches 9.77 * cos 25 + 8.90 * sin 25 = 12.62 in
        // toward the audience wall from its center (the front-left corner leads), so 3.42 in through it
        double reach = F * Math.cos(Math.toRadians(25)) + S * Math.sin(Math.toRadians(25));
        assertEquals(3.416, reach - 9.2, 0.001);
        assertEquals(reach - 9.2, out.overshootOutside(field), EPS);
        assertFalse(out.insideRect(field, 0.1));
        assertFalse(out.insideRect(field, 3.3));
        assertTrue(out.insideRect(field, 3.5));
        assertTrue(Footprint.of(Pose2.ofDegrees(72, 72, 33)).insideRect(field, 0));
    }

    @Test
    public void clippedBoundsGivesTheOverlapWithARectangle() {
        Rect zone = new Rect(0, 11, 96, 120);
        // B1 park (12, 90, 0): the robot spans x 3.97..21.77 (rear edge 8.03 behind the pose, front 9.77 ahead) and y 81.1..98.9,
        // so 7.03 in of it is inside the zone in x (0..11) and 2.9 in in y (96..120)
        Rect c = Footprint.of(Pose2.ofDegrees(12, 90, 0)).clippedBounds(zone);
        assertNotNull(c);
        assertEquals(11.0 - (12.0 - B), c.maxX - c.minX, EPS);
        assertEquals(7.03, c.maxX - c.minX, EPS);
        assertEquals(90.0 + S - 96.0, c.maxY - c.minY, EPS);
        assertEquals(2.9, c.maxY - c.minY, EPS);
        // far away -> null
        assertNull(Footprint.of(Pose2.ofDegrees(60, 60, 0)).clippedBounds(zone));
        // merely touching -> null (the rear edge on the zone's far side)
        assertNull(Footprint.of(Pose2.ofDegrees(11 + B, 108, 0)).clippedBounds(zone));
    }

    @Test
    public void clippingARotatedFootprintIsSmallerThanItsBoundingBox() {
        // LeavePark end pose (16, 91.5, 135): only a corner reaches into the zone; the front edge runs along y = x + 89.3 and cuts
        // the zone x 0..11, y 96..120 in a triangle 4.3 in wide and 4.3 in high
        Rect zone = new Rect(0, 11, 96, 120);
        Footprint fp = Footprint.of(Pose2.ofDegrees(16, 91.5, 135));
        assertTrue(fp.overlaps(zone));
        Rect clipped = fp.clippedBounds(zone);
        assertNotNull(clipped);
        assertEquals(4.32, clipped.maxX - clipped.minX, 0.01);
        assertEquals(4.32, clipped.maxY - clipped.minY, 0.01);
    }

    // ---- ShotGeometry ------------------------------------------------------------------------------

    private final FieldModel model = new FieldModel();

    @Test
    public void theLaunchPointIsAtTheBackOfTheRobot() {
        // front edge on the audience wall: the position is 9.77 in from it
        Point2 l = ShotGeometry.launchPoint(Pose2.ofDegrees(58, F, -90));
        // back of the robot faces +y at heading -90: the launch point is 8.03 in toward +y, i.e. the rear edge, 17.8 in from the wall
        assertEquals(58.0, l.x, EPS);
        assertEquals(F + 8.03, l.y, EPS);
        assertEquals("the ball leaves the rear edge, a whole robot length from the wall", BioBuzzConstants.Robot.LENGTH_IN, l.y, EPS);
    }

    @Test
    public void c1StartIsInsideTheStandoffBandAndAimedAtTheUpCell() {
        CellSpec aud = model.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 c1 = Pose2.ofDegrees(58, F, -90);
        double d = ShotGeometry.standoff(c1, aud);
        assertEquals(37.74, d, 0.05);
        assertTrue(d >= BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN && d <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN);
        assertEquals(1.9, Math.toDegrees(ShotGeometry.aimError(c1, aud)), 0.1);
        assertTrue(ShotGeometry.onOpeningSide(c1, aud));
    }

    @Test
    public void b1HoldPoseIsAimedAtTheCellButFartherThanTheStandoffBand() {
        CellSpec aud = model.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 b1 = Pose2.ofDegrees(35, F, -115);
        assertEquals(43.76, ShotGeometry.standoff(b1, aud), 0.05);
        assertTrue(Math.toDegrees(ShotGeometry.aimError(b1, aud)) < 5.0);
    }

    @Test
    public void nominalPoseIsOnTheHiveAxisAtTheNominalStandoffWithTheBackToTheCell() {
        for (Alliance a : Alliance.values()) {
            for (Side s : Side.values()) {
                CellSpec cell = model.cell(a, s);
                Pose2 p = ShotGeometry.nominalPose(cell);
                assertEquals(BioBuzzConstants.Auto.SHOOT_STANDOFF_NOMINAL_IN, ShotGeometry.standoff(p, cell), EPS);
                assertEquals(0.0, ShotGeometry.aimError(p, cell), EPS);
                assertTrue(ShotGeometry.onOpeningSide(p, cell));
                assertEquals(cell.axisX, ShotGeometry.launchPoint(p).x, EPS);
                assertEquals(cell.facingHeading, p.heading, EPS);
            }
        }
        // red audience cell: robot south of the HIVE, heading -90, center of rotation 13.5 in from the wall (the ball exit 8.03 in behind it)
        Pose2 red = ShotGeometry.nominalPose(model.cell(Alliance.RED, Side.AUDIENCE));
        assertEquals(59.25, red.x, EPS);
        assertEquals(55.52 - 34.0 - B, red.y, 1e-6);
        assertEquals(-90.0, red.headingDegrees(), EPS);
    }

    @Test
    public void theBlueNominalPoseIsTheHalfTurnOfTheRedPoseOnTheOppositeSide() {
        Pose2 blueAudience = ShotGeometry.nominalPose(model.cell(Alliance.BLUE, Side.AUDIENCE));
        Pose2 redRear = ShotGeometry.nominalPose(model.cell(Alliance.RED, Side.REAR));
        Pose2 expected = Alliance.BLUE.fromRed(redRear, 144.0);
        assertEquals(expected.x, blueAudience.x, EPS);
        assertEquals(expected.y, blueAudience.y, EPS);
    }

    // ---- AllianceField -----------------------------------------------------------------------------

    @Test
    public void zonesOfTheTwoAlliancesAreHalfTurnImages() {
        AllianceField red = new AllianceField(model, Alliance.RED);
        AllianceField blue = new AllianceField(model, Alliance.BLUE);
        Rect lzRed = red.ownLoadingZone();
        assertEquals(0.0, lzRed.minX, EPS);
        assertEquals(11.0, lzRed.maxX, EPS);
        assertEquals(96.0, lzRed.minY, EPS);
        assertEquals(120.0, lzRed.maxY, EPS);
        Rect lzBlue = blue.ownLoadingZone();
        assertEquals(133.0, lzBlue.minX, EPS);
        assertEquals(144.0, lzBlue.maxX, EPS);
        assertEquals(24.0, lzBlue.minY, EPS);
        assertEquals(48.0, lzBlue.maxY, EPS);
        assertEquals("the blue robot's view of the red zone is the same rectangle", lzRed.minX,
                blue.loadingZone(Alliance.RED).minX, EPS);
        Rect gBlue = blue.ownGarden();
        assertEquals(121.0, gBlue.minX, EPS);
        assertEquals(142.0, gBlue.minY, EPS);
    }

    @Test
    public void gardenPollenLiesAlongTheWallInsideTheGarden() {
        AllianceField red = new AllianceField(model, Alliance.RED);
        List<Point2> red4 = red.gardenPollen(Alliance.RED);
        assertEquals(4, red4.size());
        for (Point2 p : red4) {
            assertTrue(red.ownGarden().contains(p.x, p.y));
        }
        AllianceField blue = new AllianceField(model, Alliance.BLUE);
        for (Point2 p : blue.gardenPollen(Alliance.BLUE)) {
            assertTrue(blue.ownGarden().contains(p.x, p.y));
        }
    }

    @Test
    public void sidesAndFlowers() {
        AllianceField red = new AllianceField(model, Alliance.RED);
        AllianceField blue = new AllianceField(model, Alliance.BLUE);
        assertTrue(red.isOwnSide(72.0));
        assertFalse(red.isOwnSide(72.5));
        assertTrue(blue.isOwnSide(72.0));
        assertFalse(blue.isOwnSide(71.5));
        assertEquals(2.0, red.pastCenterLine(74.0), EPS);
        assertEquals(2.0, blue.pastCenterLine(70.0), EPS);
        // FLOWERS 1 and 2 are red's, 3 and 4 are blue's
        assertTrue(red.isOwnFlower(0));
        assertTrue(red.isOwnFlower(1));
        assertFalse(red.isOwnFlower(2));
        assertFalse(red.isOwnFlower(3));
        assertFalse(blue.isOwnFlower(0));
        assertTrue(blue.isOwnFlower(2));
        assertEquals(0, red.nearestFlower(new Point2(6, 50)));
        assertEquals(-1, red.nearestFlower(new Point2(60, 30)));
        assertEquals(Alliance.RED, red.nearestGarden(new Point2(6, 8)));
        assertEquals(Alliance.BLUE, red.nearestGarden(new Point2(135, 138)));
        assertNull(red.nearestGarden(new Point2(72, 72)));
    }

    @Test
    public void cellPlansMatchTheMeasuredFootprints() {
        AllianceField red = new AllianceField(model, Alliance.RED);
        Rect up = red.cellPlan(Alliance.RED, Side.AUDIENCE, true);
        assertEquals(49.25, up.minX, EPS);
        assertEquals(69.25, up.maxX, EPS);
        assertEquals(52.3, up.minY, 0.01);
        assertEquals(69.9, up.maxY, 0.01);
        Rect down = red.cellPlan(Alliance.RED, Side.REAR, false);
        assertEquals(79.4, down.minY, 0.01);
        assertEquals(96.9, down.maxY, 0.01);
        // blue UP cell is the rear one at match start: y [74.1, 91.7], x [74.75, 94.75]
        Rect blueUp = red.cellPlan(Alliance.BLUE, model.initialUpSide(Alliance.BLUE), true);
        assertEquals(74.75, blueUp.minX, EPS);
        assertEquals(94.75, blueUp.maxX, EPS);
        assertEquals(74.1, blueUp.minY, 0.01);
        assertEquals(91.7, blueUp.maxY, 0.01);
    }
}
