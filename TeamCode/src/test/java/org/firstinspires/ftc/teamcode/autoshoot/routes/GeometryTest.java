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
import org.junit.Test;

/** Footprint, shot geometry and the alliance-specific zones. */
public class GeometryTest {
    private static final double EPS = 1e-6;
    private static final double HL = BioBuzzConstants.Robot.LENGTH_IN / 2.0;
    private static final double HW = BioBuzzConstants.Robot.WIDTH_IN / 2.0;

    // ---- Footprint ---------------------------------------------------------------------------------

    @Test
    public void footprintBoundsFollowTheHeading() {
        Rect facingX = Footprint.of(Pose2.ofDegrees(50, 50, 0)).bounds();
        assertEquals(50 - HL, facingX.minX, EPS);
        assertEquals(50 + HL, facingX.maxX, EPS);
        assertEquals(50 - HW, facingX.minY, EPS);
        Rect facingDown = Footprint.of(Pose2.ofDegrees(35, 9.2, -90)).bounds();
        assertEquals("front edge 0.3 in from the audience wall", 0.3, facingDown.minY, EPS);
        assertEquals(35 - HW, facingDown.minX, EPS);
        Rect diagonal = Footprint.of(Pose2.ofDegrees(50, 50, 45)).bounds();
        double half = (HL + HW) / Math.sqrt(2.0);
        assertEquals(50 - half, diagonal.minX, EPS);
        assertEquals(50 + half, diagonal.maxX, EPS);
    }

    @Test
    public void theFrontLeftCornerIsAheadAndToTheLeftOfTheRobot() {
        Footprint fp = Footprint.of(Pose2.ofDegrees(0, 0, 90));
        // heading +y: front is +y, left is -x
        assertEquals(-HW, fp.corner(0).x, EPS);
        assertEquals(HL, fp.corner(0).y, EPS);
    }

    @Test
    public void touchingEdgesDoNotOverlapButPenetrationDoes() {
        Rect wall = new Rect(0, 100, -10, 0);
        // robot standing exactly on the line y = 0
        assertFalse(Footprint.of(Pose2.ofDegrees(35, HL, -90)).overlaps(wall));
        assertTrue(Footprint.of(Pose2.ofDegrees(35, HL - 0.01, -90)).overlaps(wall));
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
        // the front edge of a 45 degree robot at (56, 56) lies along x + y = 124.6, past the box corner at 120
        assertTrue(Footprint.of(Pose2.ofDegrees(56, 56, 45)).overlaps(box));
        // while at (52, 52) the same edge (x + y = 116.6) stops short of the corner
        assertFalse(Footprint.of(Pose2.ofDegrees(52, 52, 45)).overlaps(box));
    }

    @Test
    public void overshootAndInsideRectAreMeasuredFromTheBoundingBox() {
        Rect field = new Rect(0, 144, 0, 144);
        Footprint out = Footprint.of(Pose2.ofDegrees(35, 9.2, -115));
        // heading -115 at y = 9.2 sticks 2.4 in through the audience wall (the old B1 shooting pose)
        assertEquals(2.4, out.overshootOutside(field), 0.05);
        assertFalse(out.insideRect(field, 0.1));
        assertTrue(out.insideRect(field, 2.5));
        assertTrue(Footprint.of(Pose2.ofDegrees(72, 72, 33)).insideRect(field, 0));
    }

    @Test
    public void clippedBoundsGivesTheOverlapWithARectangle() {
        Rect zone = new Rect(0, 11, 96, 120);
        // B1 park (12, 90, 0): 7.9 in in x, 2.385 in in y
        Rect c = Footprint.of(Pose2.ofDegrees(12, 90, 0)).clippedBounds(zone);
        assertNotNull(c);
        assertEquals(7.9, c.maxX - c.minX, EPS);
        assertEquals(98.385 - 96.0, c.maxY - c.minY, EPS);
        // far away -> null
        assertNull(Footprint.of(Pose2.ofDegrees(60, 60, 0)).clippedBounds(zone));
        // merely touching -> null
        assertNull(Footprint.of(Pose2.ofDegrees(11 + HL, 108, 0)).clippedBounds(zone));
    }

    @Test
    public void clippingARotatedFootprintIsSmallerThanItsBoundingBox() {
        // LeavePark end pose: only a corner reaches into the zone
        Rect zone = new Rect(0, 11, 96, 120);
        Footprint fp = Footprint.of(Pose2.ofDegrees(16, 91.5, 135));
        assertTrue(fp.overlaps(zone));
        Rect clipped = fp.clippedBounds(zone);
        assertNotNull(clipped);
        assertEquals(3.08, clipped.maxX - clipped.minX, 0.05);
        assertEquals(3.08, clipped.maxY - clipped.minY, 0.05);
    }

    // ---- ShotGeometry ------------------------------------------------------------------------------

    private final FieldModel model = new FieldModel();

    @Test
    public void theLaunchPointIsAtTheBackOfTheRobot() {
        Point2 l = ShotGeometry.launchPoint(Pose2.ofDegrees(58, 9.2, -90));
        // back of the robot faces +y at heading -90: the launch point is 8.9 in toward +y
        assertEquals(58.0, l.x, EPS);
        assertEquals(9.2 + 8.9, l.y, EPS);
    }

    @Test
    public void c1StartIsInsideTheStandoffBandAndAimedAtTheUpCell() {
        CellSpec aud = model.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 c1 = Pose2.ofDegrees(58, 9.2, -90);
        double d = ShotGeometry.standoff(c1, aud);
        assertEquals(37.4, d, 0.05);
        assertTrue(d >= BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN && d <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN);
        assertEquals(1.9, Math.toDegrees(ShotGeometry.aimError(c1, aud)), 0.1);
        assertTrue(ShotGeometry.onOpeningSide(c1, aud));
    }

    @Test
    public void b1HoldPoseIsAimedAtTheCellButFartherThanTheStandoffBand() {
        CellSpec aud = model.cell(Alliance.RED, Side.AUDIENCE);
        Pose2 b1 = Pose2.ofDegrees(35, 9.2, -115);
        assertEquals(43.4, ShotGeometry.standoff(b1, aud), 0.1);
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
        // red audience cell: robot south of the HIVE, heading -90, center about 12.6 in from the wall
        Pose2 red = ShotGeometry.nominalPose(model.cell(Alliance.RED, Side.AUDIENCE));
        assertEquals(59.25, red.x, EPS);
        assertEquals(55.52 - 34.0 - 8.9, red.y, 1e-6);
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
