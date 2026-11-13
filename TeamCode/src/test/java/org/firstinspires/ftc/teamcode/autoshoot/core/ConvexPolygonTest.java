package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

/**
 * {@link ConvexPolygon}: the shape the exact HIVE solid is made of. These tests check that a polygon is built right (corner order,
 * dents refused), that the convex hull holds every point, and above all that {@link ConvexPolygon#signedGap} is a true distance: it
 * matches the distance worked out by brute force (many points along both boundaries), it is the minus the overlap depth when the shapes overlap,
 * and moving one shape by m inches never changes it by more than m (the property the route check's proof stands on).
 *
 * <p>How it works: no robot. Small shapes with obvious answers (unit squares two inches apart) and a seeded random test (the same shapes on
 * every run) compared with a slow referee.
 */
public class ConvexPolygonTest {
    private static final double EPS = 1e-9;

    private static ConvexPolygon square(double x, double y, double side) {
        return ConvexPolygon.of(x, y, x + side, y, x + side, y + side, x, y + side);
    }

    /** Situation: a unit square given clockwise. Expected: it is stored counter-clockwise, with area +1 and the right bounds. */
    @Test
    public void cornersAreKeptCounterClockwiseWhicheverWayTheyAreGiven() {
        ConvexPolygon cw = ConvexPolygon.of(0, 0, 0, 1, 1, 1, 1, 0);
        assertEquals(1.0, cw.area(), EPS);
        assertEquals(4, cw.size());
        Rect b = cw.bounds();
        assertEquals(0.0, b.minX, EPS);
        assertEquals(1.0, b.maxX, EPS);
        assertEquals(1.0, ConvexPolygon.of(new Rect(0, 1, 0, 1)).area(), EPS);
        // a repeated closing corner is ignored
        assertEquals(3, ConvexPolygon.of(0, 0, 2, 0, 0, 2, 0, 0).size());
    }

    /**
     * Situation: an L shape (a dent), two points, a line of three points.
     * Expected: each is refused with an IllegalArgumentException that says why.
     */
    @Test
    public void shapesThatAreNotConvexAreRefused() {
        double[][] bad = {{0, 0, 2, 0, 2, 1, 1, 1, 1, 2, 0, 2}, {0, 0, 1, 1}, {0, 0, 1, 1, 2, 2}};
        String[] why = {"dent", "at least 3", "no area"};
        for (int i = 0; i < bad.length; i++) {
            try {
                ConvexPolygon.of(bad[i]);
                fail("shape " + i + " is not a convex polygon");
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(why[i]));
            }
        }
    }

    /**
     * Situation: the hull of a square's four corners plus points inside and on its sides, and the hull of 200 random points.
     * Expected: the first is the square (4 corners, area 4); the second contains every one of its points.
     */
    @Test
    public void theHullHoldsEveryPoint() {
        List<Point2> pts = new ArrayList<Point2>();
        double[][] raw = {{0, 0}, {2, 0}, {2, 2}, {0, 2}, {1, 1}, {1, 0}, {0.5, 1.5}, {2, 1}};
        for (double[] p : raw) {
            pts.add(new Point2(p[0], p[1]));
        }
        ConvexPolygon hull = ConvexPolygon.hull(pts);
        assertEquals(4, hull.size());
        assertEquals(4.0, hull.area(), EPS);
        Random rng = new Random(7);
        List<Point2> many = new ArrayList<Point2>();
        for (int i = 0; i < 200; i++) {
            many.add(new Point2(rng.nextGaussian() * 5.0, rng.nextGaussian() * 3.0));
        }
        ConvexPolygon big = ConvexPolygon.hull(many);
        for (Point2 p : many) {
            assertTrue("hull holds " + p, big.contains(p.x, p.y));
        }
        try {
            ConvexPolygon.hull(pts.subList(0, 2));
            fail("two points are no polygon");
        } catch (IllegalArgumentException expected) {
            // good
        }
    }

    /**
     * Situation: two unit squares: 2 in apart along x, diagonal (2 in apart in x and y), touching along an edge, touching at a corner, overlapping
     *     by 0.5 in in x and 0.25 in in y.
     * Expected: the gaps are 2, 2.828 (the diagonal), 0, 0 and -0.25 (the SMALLER overlap: the shortest slide that frees them).
     */
    @Test
    public void theGapIsTheDistanceWhenApartZeroWhenTouchingAndMinusTheDepthWhenOverlapping() {
        ConvexPolygon a = square(0, 0, 1);
        assertEquals(2.0, a.signedGap(square(3, 0, 1)), EPS);
        assertEquals(2.0 * Math.sqrt(2.0), a.signedGap(square(3, 3, 1)), EPS);
        assertEquals(0.0, a.signedGap(square(1, 0, 1)), EPS);
        assertEquals(0.0, a.signedGap(square(1, 1, 1)), EPS);
        assertEquals(-0.25, a.signedGap(square(0.5, 0.75, 1)), EPS);
        assertEquals("symmetric", square(0.5, 0.75, 1).signedGap(a), a.signedGap(square(0.5, 0.75, 1)), EPS);
        // a triangle with a slanted side against a square: the nearest points are a corner and the middle of the slanted side
        ConvexPolygon triangle = ConvexPolygon.of(3, 0, 5, 0, 3, 2);
        assertEquals(Math.hypot(1.0, 0.0) + 0.0, a.signedGap(ConvexPolygon.of(2, 0, 4, 0, 2, 2)), EPS);
        assertTrue(a.signedGap(triangle) > 1.9);
    }

    /** A slow referee: the distance between two shapes by sampling points along both boundaries. */
    private static double refereeDistance(ConvexPolygon a, ConvexPolygon b) {
        double best = Double.POSITIVE_INFINITY;
        List<Point2> pa = boundary(a);
        List<Point2> pb = boundary(b);
        for (Point2 p : pa) {
            for (Point2 q : pb) {
                best = Math.min(best, p.distanceTo(q));
            }
        }
        return best;
    }

    private static List<Point2> boundary(ConvexPolygon p) {
        List<Point2> out = new ArrayList<Point2>();
        for (int i = 0; i < p.size(); i++) {
            Point2 a = p.corner(i);
            Point2 b = p.corner((i + 1) % p.size());
            for (int k = 0; k <= 400; k++) {
                double t = k / 400.0;
                out.add(new Point2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
            }
        }
        return out;
    }

    private static ConvexPolygon randomPolygon(Random rng, double cx, double cy) {
        List<Point2> pts = new ArrayList<Point2>();
        for (int i = 0; i < 6; i++) {
            pts.add(new Point2(cx + rng.nextDouble() * 4.0 - 2.0, cy + rng.nextDouble() * 4.0 - 2.0));
        }
        return ConvexPolygon.hull(pts);
    }

    /**
     * Situation: 150 pairs of random convex polygons (seeded), placed 0 to 7 in apart.
     * Expected: when apart, the gap equals the referee's sampled distance to within the sampling error (0.02 in); when overlapping it is
     *     negative, and sliding one polygon by its own depth along the shortest way makes the gap about zero.
     * Why: the route check relies on the gap being a TRUE distance, not a lower bound.
     */
    @Test
    public void theGapMatchesABruteForceDistanceForRandomShapes() {
        Random rng = new Random(2027);
        int apart = 0;
        int overlapping = 0;
        for (int i = 0; i < 150; i++) {
            ConvexPolygon a = randomPolygon(rng, 0.0, 0.0);
            ConvexPolygon b = randomPolygon(rng, rng.nextDouble() * 7.0, rng.nextDouble() * 6.0 - 3.0);
            double gap = a.signedGap(b);
            if (gap > 0.0) {
                apart++;
                assertEquals("pair " + i, refereeDistance(a, b), gap, 0.02);
            } else {
                overlapping++;
                assertTrue("pair " + i + " overlap depth " + gap, gap <= 0.0);
            }
        }
        assertTrue("both cases occurred: " + apart + " apart, " + overlapping + " overlapping", apart > 20 && overlapping > 20);
    }

    /**
     * Situation: a polygon moved by a random step of m inches against a fixed one, 300 times, apart and overlapping.
     * Expected: the gap never changes by more than m (it is 1-Lipschitz).
     * Why: {@code Sweep} proves that no pose between two samples breaks a rule by this property of the gap.
     */
    @Test
    public void movingAShapeByMInchesChangesTheGapByAtMostM() {
        Random rng = new Random(11);
        ConvexPolygon fixed = randomPolygon(rng, 0.0, 0.0);
        for (int i = 0; i < 300; i++) {
            double x = rng.nextDouble() * 10.0 - 5.0;
            double y = rng.nextDouble() * 10.0 - 5.0;
            double dx = rng.nextDouble() * 1.0 - 0.5;
            double dy = rng.nextDouble() * 1.0 - 0.5;
            ConvexPolygon moving = square(x, y, 1.5);
            ConvexPolygon moved = square(x + dx, y + dy, 1.5);
            double change = Math.abs(fixed.signedGap(moving) - fixed.signedGap(moved));
            assertTrue("change " + change + " for a move of " + Math.hypot(dx, dy), change <= Math.hypot(dx, dy) + 1e-9);
        }
    }

    /**
     * Situation: robot footprints at random poses against a box, compared two ways: the footprint's own gap to the box ({@link Footprint#signedGap(Rect)})
     *     and its gap to the same box as a polygon.
     * Expected: the two agree to 1e-9 for every pose, apart or overlapping.
     * Why: the lane check replaces the box by polygons; for a plain box it must give exactly the answer the old check gave.
     */
    @Test
    public void aBoxAsAPolygonGivesTheSameGapAsTheBoxItself() {
        Random rng = new Random(5);
        Rect box = new Rect(30.0, 50.0, 40.0, 60.0);
        ConvexPolygon polygon = ConvexPolygon.of(box);
        for (int i = 0; i < 400; i++) {
            Pose2 pose = new Pose2(rng.nextDouble() * 100.0, rng.nextDouble() * 100.0, rng.nextDouble() * 2.0 * Math.PI);
            Footprint fp = Footprint.of(pose);
            assertEquals("pose " + pose, fp.signedGap(box), fp.signedGap(polygon), 1e-9);
        }
    }

    /**
     * Situation: where a polygon is and is not, the x range it covers in a row, and its half-turn image.
     * Expected: points inside and on the edge are contained, points outside are not; a trapezoid's x range is wider at its base than at its top;
     *     a row that misses it gives null; the half turn about (5, 5) maps the corners to (10 - x, 10 - y).
     */
    @Test
    public void containsRowRangeAndHalfTurn() {
        ConvexPolygon trapezoid = ConvexPolygon.of(0, 0, 4, 0, 3, 2, 1, 2);
        assertTrue(trapezoid.contains(2, 1));
        assertTrue(trapezoid.contains(0, 0));
        assertTrue(trapezoid.contains(2, 2));
        assertFalse(trapezoid.contains(4, 2));
        assertFalse(trapezoid.contains(-0.01, 0));
        double[] base = trapezoid.xRangeAt(0.0);
        double[] top = trapezoid.xRangeAt(2.0);
        double[] middle = trapezoid.xRangeAt(1.0);
        assertEquals(0.0, base[0], EPS);
        assertEquals(4.0, base[1], EPS);
        assertEquals(1.0, top[0], EPS);
        assertEquals(3.0, top[1], EPS);
        assertEquals(0.5, middle[0], EPS);
        assertEquals(3.5, middle[1], EPS);
        assertNull(trapezoid.xRangeAt(2.5));
        assertNotNull(trapezoid.xRangeAt(-0.0));
        ConvexPolygon turned = trapezoid.rotated180(5.0, 5.0);
        assertEquals(trapezoid.area(), turned.area(), EPS);
        for (int i = 0; i < trapezoid.size(); i++) {
            assertEquals(10.0 - trapezoid.corner(i).x, turned.corner(i).x, EPS);
            assertEquals(10.0 - trapezoid.corner(i).y, turned.corner(i).y, EPS);
        }
    }
}
