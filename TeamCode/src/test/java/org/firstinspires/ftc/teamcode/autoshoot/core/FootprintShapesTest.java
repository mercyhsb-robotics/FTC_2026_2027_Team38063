package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The three shapes of the robot that {@link Footprint} makes: the whole perimeter, the chassis (the body without the front
 * {@code Robot.INTAKE_REACH_IN}) and the two flat intake wheel discs. The route validator needs them to let ONE wheel press
 * into a FLOWER while the body must stay out of its plate.
 *
 * <p>Words: the "pose point" is the point the robot spins about, the one a pose describes; "forward" and "left" are measured
 * from it along the robot's own axes.
 */
public class FootprintShapesTest {
    private static final double EPS = 1e-9;
    private static final double REACH = BioBuzzConstants.Robot.INTAKE_REACH_IN;
    private static final double OFFSET = BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN;
    private static final double DIAMETER = BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN;

    /** Robot-frame (forward, left) of corner i of a footprint at a pose. */
    private static double[] local(Footprint fp, int i) {
        Pose2 p = fp.pose();
        return new double[] {p.forwardOf(fp.corner(i).x, fp.corner(i).y), p.leftOf(fp.corner(i).x, fp.corner(i).y)};
    }

    /** The reach of a footprint in the robot frame: {minForward, maxForward, minLeft, maxLeft}. */
    private static double[] extents(Footprint fp) {
        double[] e = {Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (int i = 0; i < 4; i++) {
            double[] c = local(fp, i);
            e[0] = Math.min(e[0], c[0]);
            e[1] = Math.max(e[1], c[0]);
            e[2] = Math.min(e[2], c[1]);
            e[3] = Math.max(e[3], c[1]);
        }
        return e;
    }

    // ---- the one place that knows how the perimeter sits around the pose point ---------------------------------

    @Test
    public void thePerimeterIsBuiltFromTheFourExtentsAndTheExtentsAddUpToTheRobotSize() {
        double[] e = extents(Footprint.of(Pose2.ofDegrees(40, 50, 33)));
        assertEquals(-Footprint.rearExtentIn(), e[0], 1e-9);
        assertEquals(Footprint.frontExtentIn(), e[1], 1e-9);
        assertEquals(-Footprint.rightExtentIn(), e[2], 1e-9);
        assertEquals(Footprint.leftExtentIn(), e[3], 1e-9);
        // front + rear is the robot's length and left + right its width, whatever the pose point
        assertEquals(BioBuzzConstants.Robot.LENGTH_IN, Footprint.frontExtentIn() + Footprint.rearExtentIn(), EPS);
        assertEquals(BioBuzzConstants.Robot.WIDTH_IN, Footprint.leftExtentIn() + Footprint.rightExtentIn(), EPS);
    }

    @Test
    public void theCircumscribedRadiusIsTheFarthestCornerFromThePoseAndHoldsForEveryPart() {
        Pose2 p = Pose2.ofDegrees(20, 20, 71);
        double far = 0.0;
        for (int i = 0; i < 4; i++) {
            far = Math.max(far, Footprint.of(p).corner(i).distanceTo(new Point2(p.x, p.y)));
        }
        assertEquals(far, Footprint.circumscribedRadiusIn(), 1e-9);
        // the position is NOT the middle of the robot: the farthest corner is a FRONT one, hypot(front, side) = 13.16 in, not the
        // 12.52 in of a rectangle of the same size centered on the position
        double side = Math.max(Footprint.leftExtentIn(), Footprint.rightExtentIn());
        assertTrue("the front is the long end", Footprint.frontExtentIn() > Footprint.rearExtentIn());
        assertEquals(Math.hypot(Footprint.frontExtentIn(), side), far, 1e-9);
        assertTrue(far > Math.hypot(BioBuzzConstants.Robot.LENGTH_IN / 2.0, BioBuzzConstants.Robot.WIDTH_IN / 2.0) + 0.5);
        Footprint[] parts = {Footprint.chassis(p), Footprint.wheelDisc(p, IntakeWheel.LEFT), Footprint.wheelDisc(p, IntakeWheel.RIGHT)};
        for (Footprint part : parts) {
            for (int i = 0; i < 4; i++) {
                assertTrue(part.corner(i).distanceTo(new Point2(p.x, p.y)) <= Footprint.circumscribedRadiusIn() + EPS);
            }
        }
    }

    // ---- the chassis ---------------------------------------------------------------------------------------

    @Test
    public void theChassisIsThePerimeterWithTheFrontReachCutOff() {
        double[] whole = extents(Footprint.of(Pose2.ofDegrees(30, 40, 25)));
        double[] body = extents(Footprint.chassis(Pose2.ofDegrees(30, 40, 25)));
        assertEquals("front edge is REACH behind the perimeter's", whole[1] - REACH, body[1], 1e-9);
        assertEquals("back edge unchanged", whole[0], body[0], 1e-9);
        assertEquals("left edge unchanged", whole[3], body[3], 1e-9);
        assertEquals("right edge unchanged", whole[2], body[2], 1e-9);
        // an explicit reach moves only the front edge
        assertEquals(whole[1] - 5.0, extents(Footprint.chassis(Pose2.ofDegrees(30, 40, 25), 5.0))[1], 1e-9);
    }

    // ---- the two wheel discs ----------------------------------------------------------------------------------

    @Test
    public void theLeftWheelDiscIsASquareAtTheFrontLeftAndTheRightOneIsItsMirrorImage() {
        Pose2 p = Pose2.ofDegrees(30, 40, 0);   // facing +x: forward = +x, left = +y
        double[] left = extents(Footprint.wheelDisc(p, IntakeWheel.LEFT));
        assertEquals(Footprint.frontExtentIn() - DIAMETER, left[0], 1e-9);
        assertEquals("front edge on the perimeter front", Footprint.frontExtentIn(), left[1], 1e-9);
        assertEquals(OFFSET - DIAMETER / 2.0, left[2], 1e-9);
        assertEquals(OFFSET + DIAMETER / 2.0, left[3], 1e-9);
        double[] right = extents(Footprint.wheelDisc(p, IntakeWheel.RIGHT));
        assertEquals(left[0], right[0], 1e-9);
        assertEquals(left[1], right[1], 1e-9);
        assertEquals(-left[3], right[2], 1e-9);
        assertEquals(-left[2], right[3], 1e-9);
        // in field terms at heading 0 the left wheel is on the +y side
        Point2 c = Footprint.wheelCenter(p, IntakeWheel.LEFT);
        assertEquals(30 + Footprint.frontExtentIn() - DIAMETER / 2.0, c.x, 1e-9);
        assertEquals(40 + OFFSET, c.y, 1e-9);
        assertEquals(40 - OFFSET, Footprint.wheelCenter(p, IntakeWheel.RIGHT).y, 1e-9);
    }

    @Test
    public void theTwoDiscsAreTwelvePointTwoInchesApartAndFitInsideThePerimeterAtEveryHeading() {
        // a gap of 12.2 - 2.8 = 9.4 in between the discs, as on the drawing (9.45 in)
        Footprint a = Footprint.wheelDisc(Pose2.ofDegrees(0, 0, 0), IntakeWheel.LEFT);
        Footprint b = Footprint.wheelDisc(Pose2.ofDegrees(0, 0, 0), IntakeWheel.RIGHT);
        assertEquals(2.0 * OFFSET, Footprint.wheelCenter(a.pose(), IntakeWheel.LEFT).distanceTo(Footprint.wheelCenter(b.pose(), IntakeWheel.RIGHT)), 1e-9);
        for (int deg = -180; deg < 180; deg += 7) {
            Pose2 p = Pose2.ofDegrees(60, 70, deg);
            Footprint whole = Footprint.of(p);
            for (IntakeWheel w : IntakeWheel.physical()) {
                Footprint disc = Footprint.wheelDisc(p, w);
                for (int i = 0; i < 4; i++) {
                    double[] c = local(disc, i);
                    assertTrue("inside front/rear", c[0] <= Footprint.frontExtentIn() + EPS && c[0] >= -Footprint.rearExtentIn() - EPS);
                    assertTrue("inside left/right", c[1] <= Footprint.leftExtentIn() + EPS && c[1] >= -Footprint.rightExtentIn() - EPS);
                }
                // and a pose that fits the whole perimeter in a box fits the disc too
                Rect box = whole.bounds();
                assertTrue("the disc is inside the perimeter's bounding box", disc.insideRect(box, EPS));
            }
        }
    }

    @Test
    public void theWheelsStickOutInFrontOfTheChassisByTheReach() {
        Pose2 p = Pose2.ofDegrees(30, 40, -90);
        double chassisFront = extents(Footprint.chassis(p))[1];
        double wheelFront = extents(Footprint.wheelDisc(p, IntakeWheel.LEFT))[1];
        assertEquals(REACH, wheelFront - chassisFront, 1e-9);
        // and only the wheels: the middle of the front edge of the perimeter is NOT a wheel (nothing is there, the gap is 9.4 in)
        Rect middleOfTheFront = new Rect(Footprint.frontExtentIn() - 1.0, Footprint.frontExtentIn(), -1.0, 1.0);
        Pose2 origin = Pose2.ofDegrees(0, 0, 0);
        assertFalse(Footprint.wheelDisc(origin, IntakeWheel.LEFT).overlaps(middleOfTheFront));
        assertFalse(Footprint.wheelDisc(origin, IntakeWheel.RIGHT).overlaps(middleOfTheFront));
        assertTrue("but the chassis frame is there, just behind", Footprint.of(origin).overlaps(middleOfTheFront));
    }

    // ---- distances from a shape that is not centered on the pose point ---------------------------------------

    @Test
    public void aWheelDiscMeasuresItsOwnGapToAWallNotThePerimetersGap() {
        // robot facing the audience wall (y = 0, the wall is the strip y < 0), center 20 in from it. The perimeter front
        // is 20 - front from the wall; the wheel tip is at the same place; the chassis front is REACH farther.
        Rect wall = new Rect(-50, 200, -10, 0);
        Pose2 p = Pose2.ofDegrees(70, 20, -90);
        double front = 20.0 - Footprint.frontExtentIn();
        assertEquals(front, Footprint.of(p).signedGap(wall), 1e-9);
        assertEquals(front, Footprint.wheelDisc(p, IntakeWheel.LEFT).signedGap(wall), 1e-9);
        assertEquals(front + REACH, Footprint.chassis(p).signedGap(wall), 1e-9);
    }

    @Test
    public void theSignedGapOfAnOffCenterShapeIsTheTrueDistanceWhenApartAndTheDepthWhenOverlapping() {
        // brute force: the distance between two convex shapes that do not overlap is the smallest distance from a point of
        // one outline to the other shape; check it on random poses for the discs, whose center is far from the pose point
        Random rnd = new Random(42);
        int apart = 0;
        int overlapping = 0;
        for (int k = 0; k < 600; k++) {
            Pose2 p = new Pose2(50 + rnd.nextGaussian() * 15, 50 + rnd.nextGaussian() * 15, rnd.nextDouble() * 6.2 - 3.1);
            IntakeWheel w = rnd.nextBoolean() ? IntakeWheel.LEFT : IntakeWheel.RIGHT;
            Footprint disc = Footprint.wheelDisc(p, w);
            Point2 wc = Footprint.wheelCenter(p, w);       // boxes are dropped around the wheel so both cases are common
            Rect box = Rect.centered(wc.x + rnd.nextGaussian() * 4, wc.y + rnd.nextGaussian() * 4, 1 + rnd.nextDouble() * 12, 1 + rnd.nextDouble() * 12);
            double gap = disc.signedGap(box);
            if (disc.overlaps(box)) {
                overlapping++;
                assertTrue("overlapping shapes have a negative gap: " + gap, gap < 0.0);
            } else {
                apart++;
                assertTrue(gap >= 0.0);
                double brute = Double.POSITIVE_INFINITY;
                for (int i = 0; i < 4; i++) {
                    Point2 a = disc.corner(i);
                    Point2 b = disc.corner((i + 1) % 4);
                    for (int s = 0; s <= 400; s++) {
                        double t = s / 400.0;
                        double x = a.x + (b.x - a.x) * t;
                        double y = a.y + (b.y - a.y) * t;
                        double dx = Math.max(Math.max(box.minX - x, 0.0), x - box.maxX);
                        double dy = Math.max(Math.max(box.minY - y, 0.0), y - box.maxY);
                        brute = Math.min(brute, Math.hypot(dx, dy));
                    }
                }
                assertEquals("true distance", brute, gap, 0.03);
                assertTrue("never larger than the true distance", gap <= brute + 1e-9);
            }
        }
        assertTrue("the random poses covered both cases: " + apart + " apart, " + overlapping + " overlapping", apart > 50 && overlapping > 50);
    }

    @Test
    public void aDiscThatPressesIntoAPlateOverlapsItButTheChassisBehindItDoesNot() {
        // the FLOWER situation in miniature: a 4.75 in deep slab along the wall x = 0, robot facing the wall (heading 180)
        Rect plate = new Rect(0, 4.75, 40, 46);
        double chassisFrontX = 4.75 + 0.5;                       // the chassis stops 0.5 in off the slab
        double centerX = chassisFrontX + (Footprint.frontExtentIn() - REACH);
        Pose2 p = Pose2.ofDegrees(centerX, 43 + OFFSET, 180);    // the left wheel is 43 + OFFSET - OFFSET = 43 along the wall
        assertTrue("the left wheel reaches into the slab", Footprint.wheelDisc(p, IntakeWheel.LEFT).overlaps(plate));
        assertFalse("the right wheel is 12.2 in away along the wall, nowhere near it", Footprint.wheelDisc(p, IntakeWheel.RIGHT).overlaps(plate));
        assertFalse("the chassis stays off it", Footprint.chassis(p, REACH).overlaps(plate));
        assertFalse("so does the whole body with a longer reach assumed", Footprint.chassis(p, REACH + 1.0).overlaps(plate));
        assertTrue("but a chassis assumed to be 1.5 in shorter would hit", Footprint.chassis(p, REACH - 1.5).overlaps(plate));
    }
}
