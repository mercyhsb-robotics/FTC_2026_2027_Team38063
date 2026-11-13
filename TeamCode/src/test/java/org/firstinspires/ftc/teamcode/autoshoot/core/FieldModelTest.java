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

public class FieldModelTest {
    private static final double EPS = 1e-6;
    private final FieldModel field = new FieldModel();

    @Test
    public void redCellsSitOnTheRedHiveAxisAtTheSdkClusterOrigin() {
        CellSpec aud = field.cell(Alliance.RED, Side.AUDIENCE);
        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        assertEquals(59.25, aud.originX, EPS);
        assertEquals(55.52, aud.originY, EPS);
        assertEquals(58.31, aud.originZ, EPS);
        assertEquals(59.25, rear.originX, EPS);
        assertEquals(88.48, rear.originY, EPS);
        assertEquals(34, aud.baseTagId);
        assertEquals(30, rear.baseTagId);
    }

    @Test
    public void blueCellsAreTheHalfTurnImagesOfTheRedCellsOnTheOppositeSide() {
        CellSpec bAud = field.cell(Alliance.BLUE, Side.AUDIENCE);
        CellSpec bRear = field.cell(Alliance.BLUE, Side.REAR);
        assertEquals(84.75, bAud.originX, EPS);
        assertEquals(55.52, bAud.originY, EPS);
        assertEquals(84.75, bRear.originX, EPS);
        assertEquals(88.48, bRear.originY, EPS);
        assertEquals(38, bAud.baseTagId);
        assertEquals(42, bRear.baseTagId);
        assertEquals(58.31, bAud.originZ, EPS);
    }

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

    @Test
    public void tagFieldPositionsMatchTheVerifiedClusterTable() {
        double[] redAud = {52.75, 56.50, 62.00, 65.75};
        double[] redRear = {65.75, 62.00, 56.50, 52.75};
        double[] blueAud = {78.25, 82.00, 87.50, 91.25};
        double[] blueRear = {91.25, 87.50, 82.00, 78.25};
        for (int i = 0; i < 4; i++) {
            assertEquals(redAud[i], field.cell(Alliance.RED, Side.AUDIENCE).tagFieldX(i), EPS);
            assertEquals(redRear[i], field.cell(Alliance.RED, Side.REAR).tagFieldX(i), EPS);
            assertEquals(blueAud[i], field.cell(Alliance.BLUE, Side.AUDIENCE).tagFieldX(i), EPS);
            assertEquals(blueRear[i], field.cell(Alliance.BLUE, Side.REAR).tagFieldX(i), EPS);
        }
    }

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

    @Test
    public void initialHiveStateMatchesTheSetupFigure() {
        assertEquals(Side.AUDIENCE, field.initialUpSide(Alliance.RED));
        assertEquals(Side.REAR, field.initialUpSide(Alliance.BLUE));
    }

    @Test
    public void hiveFrameFootprintIsCenteredOnTheField() {
        Rect f = field.hiveFrame();
        assertEquals(47.27, f.minX, EPS);
        assertEquals(96.73, f.maxX, EPS);
        assertEquals(52.525, f.minY, EPS);
        assertEquals(91.475, f.maxY, EPS);
        assertTrue(f.contains(72, 72));
    }

    @Test
    public void flowerKeepOutsHugTheirWallsWithTheLongSideAlongTheWall() {
        assertEquals(4, field.flowerKeepOuts().size());
        Rect left = field.flowerKeepOuts().get(0);     // (2.4, 48) on the red wall
        assertEquals(6.0, left.maxY - left.minY, EPS); // long side along y
        assertEquals(5.0, left.maxX - left.minX, EPS);
        Rect rear = field.flowerKeepOuts().get(1);     // (48, 141.6) on the rear wall
        assertEquals(6.0, rear.maxX - rear.minX, EPS); // long side along x
        assertEquals(5.0, rear.maxY - rear.minY, EPS);
    }

    @Test
    public void allianceFlipIsAHalfTurnAboutTheFieldCenter() {
        Pose2 red = Pose2.ofDegrees(35.0, 9.2, -90);
        Pose2 blue = Alliance.BLUE.fromRed(red, field.size());
        assertEquals(109.0, blue.x, EPS);
        assertEquals(134.8, blue.y, EPS);
        assertEquals(90.0, blue.headingDegrees(), EPS);
        Pose2 back = Alliance.BLUE.toRed(blue, field.size());
        assertEquals(red.x, back.x, EPS);
        assertEquals(red.y, back.y, EPS);
        assertEquals(red.headingDegrees(), back.headingDegrees(), 1e-9);
        assertSame(red, Alliance.RED.fromRed(red, field.size()));
    }

    /** Pins the team's alliance flip to Pedro's own half-turn operation so the two can never disagree. */
    @Test
    public void allianceFlipAgreesWithPedroMirrorAroundPoint() {
        PoseFactory half = PoseFactory.degrees().mirrorAroundPoint(72, 72);
        double[][] samples = {{35.0, 9.2, -90}, {58.0, 9.2, -90}, {12.0, 90.0, 0}, {100.0, 20.0, 135}};
        for (double[] s : samples) {
            Pose pedro = half.of(s[0], s[1], s[2]);
            Pose2 mine = Alliance.BLUE.fromRed(Pose2.ofDegrees(s[0], s[1], s[2]), 144.0);
            assertEquals(pedro.x(), mine.x, EPS);
            assertEquals(pedro.y(), mine.y, EPS);
            double diff = Angles.distance(pedro.heading(), mine.heading);
            assertTrue("heading differs by " + diff, diff < 1e-6);
        }
    }

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
