package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotShape;
import org.junit.Test;

/**
 * The legal starts of rule G304 ({@link StartPoseSearch}): every place on the own walls, every few inches and degrees, that {@link StartRules} finds nothing
 * wrong with, and the same region as stretches of wall for the diagrams.
 *
 * <p>How it is checked: the list is compared with an independent brute force that places the robot with the bounding box of {@link Footprint}, not with
 * the search's own code, so a mistake in how the robot is pushed against a wall cannot hide itself.
 */
public class StartPoseSearchTest {
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final double CENTER = BioBuzzConstants.Field.CENTER_IN;
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double REAR = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double SIDE = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
    private static final double TOL = 1e-6;
    /** The rule forgives this much overlap (measurement noise): it applies to the center line as it does to a wall. */
    private static final double OVER = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
    private static final FieldModel MODEL = new FieldModel();
    private static final AllianceField RED = new AllianceField(MODEL, Alliance.RED);
    private static final AllianceField BLUE = new AllianceField(MODEL, Alliance.BLUE);

    /** A coarse grid that keeps the tests quick: a place every inch, a heading every 15 degrees (the full grid is checked once, below). */
    private static final double COARSE_IN = 1.0;
    private static final double COARSE_DEG = 15.0;

    private static AllianceField field(Alliance a) {
        return a == Alliance.RED ? RED : BLUE;
    }

    /** A text that is the same for two poses that are the same place and way of facing (the heading by its cosine and sine, so that 180 and -180 agree). */
    private static String key(Pose2 p) {
        return String.format(Locale.ROOT, "%.5f/%.5f/%.5f/%.5f", p.x, p.y, Math.cos(p.heading), Math.sin(p.heading));
    }

    private static Set<String> keys(List<Pose2> poses) {
        Set<String> out = new HashSet<String>();
        for (Pose2 p : poses) {
            out.add(key(p));
        }
        return out;
    }

    private static AllianceField fieldOfFirstAlliance() {
        return RED;
    }

    // ---- the robot pushed against a wall -------------------------------------------------------------------------

    /**
     * Situation: the robot pushed against the audience wall facing it, facing away, and sideways.
     * Expected: its pose point is 9.77 in from the wall front-first, 8.03 in back-first and 8.90 in sideways, and at 45 degrees it matches the formula of
     *     {@code RobotShape}; the footprint touches the wall and does not pass it, and the coordinate along the wall is the one asked for.
     * Why: the robot is not centered on its pose point, so "touching the wall" puts the pose a different distance from it at every heading.
     */
    @Test
    public void aSeatedRobotTouchesTheWallAtTheDistanceItsOwnEdgeNeeds() {
        assertEquals(FRONT, StartPoseSearch.seatedAt(Wall.AUDIENCE, 40.0, -90.0).y, TOL);
        assertEquals(REAR, StartPoseSearch.seatedAt(Wall.AUDIENCE, 40.0, 90.0).y, TOL);
        assertEquals(SIDE, StartPoseSearch.seatedAt(Wall.AUDIENCE, 40.0, 0.0).y, TOL);
        assertEquals(SIDE, StartPoseSearch.seatedAt(Wall.AUDIENCE, 40.0, 180.0).y, TOL);
        assertEquals(FRONT, StartPoseSearch.seatedAt(Wall.RED, 40.0, 180.0).x, TOL);
        assertEquals(SIZE - FRONT, StartPoseSearch.seatedAt(Wall.REAR, 40.0, 90.0).y, TOL);
        assertEquals(SIZE - FRONT, StartPoseSearch.seatedAt(Wall.BLUE, 100.0, 0.0).x, TOL);
        RobotShape shape = RobotShape.fromConstants();
        for (Wall wall : Wall.values()) {
            for (int deg = -180; deg < 180; deg += 15) {
                Pose2 p = StartPoseSearch.seatedAt(wall, 70.0, deg);
                Rect b = Footprint.of(p).bounds();
                assertEquals(wall + " " + deg + " touches", 0.0, wall.gapIn(b, SIZE), TOL);
                assertEquals(wall + " " + deg + " along", 70.0, wall.along(p), TOL);
                // the distance from the wall, by the formula of RobotShape (the farthest the outline reaches toward the wall)
                double distance = wall == Wall.RED ? p.x : wall == Wall.BLUE ? SIZE - p.x : wall == Wall.AUDIENCE ? p.y : SIZE - p.y;
                assertEquals(wall + " " + deg + " distance", shape.reachTowardIn(p.heading, wall.outwardRad()), distance, TOL);
            }
        }
        assertTrue("a robot turned 45 degrees needs more than any straight edge", StartPoseSearch.seatedAt(Wall.AUDIENCE, 40.0, 45.0).y > FRONT + 1.0);
    }

    // ---- the list of legal starts -----------------------------------------------------------------------------------

    /**
     * Situation: every start of the coarse list, for both alliances.
     * Expected: each is legal and clear by {@link StartRules}, touches a wall of its OWN half exactly (gap 0, never overlapping it), and is listed once.
     */
    @Test
    public void everyListedStartIsLegalAndTouchesAnOwnWallExactly() {
        for (Alliance alliance : Alliance.values()) {
            List<Pose2> list = StartPoseSearch.legalStarts(alliance, COARSE_IN, COARSE_DEG);
            assertTrue(alliance + " has starts", list.size() > 1000);
            assertEquals("each start is listed once", list.size(), keys(list).size());
            for (Pose2 p : list) {
                assertTrue(alliance + " " + p + ": " + StartRules.check(field(alliance), p), StartRules.isLegalAndClear(field(alliance), p));
                Rect b = Footprint.of(p).bounds();
                boolean touchesOwn = false;
                for (Wall wall : Wall.values()) {
                    double gap = wall.gapIn(b, SIZE);
                    assertTrue(alliance + " " + p + " overlaps the " + wall + " by " + gap, gap > -TOL);
                    if (gap < TOL) {
                        touchesOwn |= Wall.own(alliance).contains(wall);
                    }
                }
                assertTrue(alliance + " " + p + " touches none of its own walls", touchesOwn);
            }
        }
    }

    /**
     * Situation: an independent brute force for the red alliance (bounding box of the footprint, every heading of the grid, every inch along the three own
     * walls, plus the corner places).
     * Expected: every place it finds legal and clear is in the list, and nothing else of its grid is.
     * Why: it proves the list is complete (no gap in the walls, the corners included) and that the rule is applied to every place, not to a sample.
     */
    @Test
    public void theListHoldsEverythingABruteForceFindsLegalAndNothingElse() {
        List<Pose2> list = StartPoseSearch.legalStarts(Alliance.RED, COARSE_IN, COARSE_DEG);
        Set<String> listed = keys(list);
        Set<String> expected = new HashSet<String>();
        for (int deg = -165; deg <= 180; deg += (int) COARSE_DEG) {
            double h = Math.toRadians(deg);
            Rect box = Footprint.of(new Pose2(0.0, 0.0, h)).bounds();
            for (int wall = 0; wall < 3; wall++) {            // 0 = red wall, 1 = audience wall, 2 = rear wall
                double lo = wall == 0 ? -box.minY : -box.minX;
                double hi = wall == 0 ? SIZE - box.maxY : SIZE - box.maxX;
                for (double along = Math.ceil(lo); along <= hi; along += 1.0) {
                    addIfLegal(expected, wall, along, box, h);
                }
                addIfLegal(expected, wall, lo, box, h);       // the corner places, which are not on the inch grid
                addIfLegal(expected, wall, hi, box, h);
            }
        }
        assertTrue("the brute force found something", expected.size() > 1000);
        Set<String> missing = new HashSet<String>(expected);
        missing.removeAll(listed);
        assertTrue("legal places the list misses (" + missing.size() + "): " + first(missing), missing.isEmpty());
        Set<String> extra = new HashSet<String>(listed);
        extra.removeAll(expected);
        assertTrue("places in the list that the brute force does not find (" + extra.size() + "): " + first(extra), extra.isEmpty());
    }

    private static void addIfLegal(Set<String> out, int wall, double along, Rect box, double h) {
        double x = wall == 0 ? -box.minX : along;
        double y = wall == 0 ? along : wall == 1 ? -box.minY : SIZE - box.maxY;
        Pose2 p = new Pose2(x, y, h);
        if (StartRules.isLegalAndClear(RED, p)) {
            out.add(key(p));
        }
    }

    private static String first(Set<String> keys) {
        return keys.isEmpty() ? "none" : keys.iterator().next();
    }

    /**
     * Situation: the full standard grid (a place every 0.25 in, a heading every 5 degrees) for the red alliance.
     * Expected: tens of thousands of starts (about 37 thousand), all 72 headings present (0, 90, 180 and -90 among them), and the team's own starts are in it.
     */
    @Test
    public void theStandardGridHasEveryHeadingAndTheTeamsOwnStarts() {
        List<Pose2> list = StartPoseSearch.legalStarts(Alliance.RED);
        assertTrue("size " + list.size(), list.size() > 30000);
        Set<String> headings = new HashSet<String>();
        for (Pose2 p : list) {
            headings.add(String.format(Locale.ROOT, "%.4f/%.4f", Math.cos(p.heading), Math.sin(p.heading)));
        }
        assertEquals(72, headings.size());
        Set<String> listed = keys(list);
        assertTrue("front on the audience wall in line with the HIVE", listed.contains(key(Pose2.ofDegrees(58.0, FRONT, -90))));
        assertTrue("front on the rear wall, the partner route's start", listed.contains(key(Pose2.ofDegrees(61.5, SIZE - FRONT, 90))));
        assertTrue("back on the red wall", listed.contains(key(Pose2.ofDegrees(REAR, 30.0, 0))));
        assertFalse("not on the FLOWER: the old A3 start", listed.contains(key(Pose2.ofDegrees(FRONT, 48.0, 180))));
        assertFalse("not in the LOADING ZONE", listed.contains(key(Pose2.ofDegrees(FRONT, 108.0, 180))));
        assertFalse("not through the wall: the old OpMode's 9.2 in", listed.contains(key(Pose2.ofDegrees(35.0, 9.2, -90))));
    }

    /**
     * Situation: the robot in the audience/red corner at 30 degrees, turned as in the manual's Figure 11-1, touching both walls with two corners.
     * Expected: it is in the list, once (a corner is reached from both walls).
     */
    @Test
    public void aRobotTurnedInACornerIsListedOnce() {
        Rect box = Footprint.of(Pose2.ofDegrees(0.0, 0.0, 30.0)).bounds();
        Pose2 corner = new Pose2(-box.minX, -box.minY, Math.toRadians(30.0));
        assertTrue(StartRules.isLegalAndClear(RED, corner));
        int times = 0;
        for (Pose2 p : StartPoseSearch.legalStarts(Alliance.RED, COARSE_IN, COARSE_DEG)) {
            if (key(p).equals(key(corner))) {
                times++;
            }
        }
        assertEquals(1, times);
    }

    /**
     * Situation: the blue list.
     * Expected: it is the half turn of the red list about the middle of the field, place for place: the same code and the same rule give the same
     *     starts for the other alliance, on the blue wall instead of the red one.
     */
    @Test
    public void theBlueStartsAreTheHalfTurnOfTheRedOnes() {
        Set<String> red = new HashSet<String>();
        for (Pose2 p : StartPoseSearch.legalStarts(Alliance.RED, COARSE_IN, COARSE_DEG)) {
            red.add(key(Alliance.BLUE.fromRed(p, SIZE)));
        }
        Set<String> blue = keys(StartPoseSearch.legalStarts(Alliance.BLUE, COARSE_IN, COARSE_DEG));
        assertEquals(red.size(), blue.size());
        assertTrue("blue misses a half-turned red start", blue.containsAll(red));
        assertTrue("blue has a start the red list does not", red.containsAll(blue));
    }

    /** Situation: a step of zero. Expected: an IllegalArgumentException (the search would never end). */
    @Test
    public void aStepThatCannotMoveOnIsRefused() {
        try {
            StartPoseSearch.legalStarts(Alliance.RED, 0.0, 5.0);
            fail("a step of 0 in must be refused");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("positive"));
        }
        try {
            StartPoseSearch.legalStarts(Alliance.RED, 0.25, 0.0);
            fail("a heading step of 0 must be refused");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("positive"));
        }
    }

    // ---- the legal region as stretches of wall --------------------------------------------------------------------------------

    private static void assertStretch(StartPoseSearch.Interval in, Wall wall, double from, double to) {
        assertEquals(in.toString(), wall, in.wall);
        assertEquals(in + " from", from, in.fromIn, 1e-4);
        assertEquals(in + " to", to, in.toIn, 1e-4);
    }

    /**
     * Situation: red, facing the red wall front-first (heading 180), so the robot is 17.80 in wide along the wall (8.90 in either side of its pose).
     * Expected: three stretches of the red wall are legal: from the audience corner to the FLOWER, from the FLOWER to the LOADING ZONE and from the
     *     LOADING ZONE to the rear corner. Each end is where the footprint just clears the bracket of the FLOWER (6.67 in long, centered on the FLOWER) or the
     *     LOADING ZONE (y 94.584 to 117.274), or the corner.
     * Why: this is the picture the diagram tints, and it shows the three things a start must avoid on that wall.
     */
    @Test
    public void facingTheRedWallTheRedWallHasThreeLegalStretches() {
        List<StartPoseSearch.Interval> all = StartPoseSearch.legalIntervals(Alliance.RED, 180.0);
        double flowerY = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1];
        double half = BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN / 2.0;
        int onRedWall = 0;
        for (StartPoseSearch.Interval in : all) {
            if (in.wall == Wall.RED) {
                onRedWall++;
            }
        }
        assertEquals(all.toString(), 3, onRedWall);
        assertStretch(all.get(0), Wall.RED, SIDE, flowerY - half - SIDE);
        assertStretch(all.get(1), Wall.RED, flowerY + half + SIDE, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN - SIDE);
        assertStretch(all.get(2), Wall.RED, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MAX_IN + SIDE, SIZE - SIDE);
    }

    /**
     * Situation: red, facing the audience wall (heading -90) and the rear wall (heading 90).
     * Expected: the audience wall is legal from the corner to the center line (the robot is 17.80 in wide, so its pose runs from 8.90 in to the middle minus 8.90 in);
     *     the rear wall is legal in two stretches, the corner to the FLOWER and from past the FLOWER to the center line.
     */
    @Test
    public void facingTheAudienceWallAndTheRearWallTheStretchesAreTheOnesBetweenTheCornersTheCenterLineAndTheFlower() {
        List<StartPoseSearch.Interval> facingAudience = StartPoseSearch.legalIntervals(Alliance.RED, -90.0);
        StartPoseSearch.Interval audience = null;
        for (StartPoseSearch.Interval in : facingAudience) {
            if (in.wall == Wall.AUDIENCE) {
                assertTrue("one stretch on the audience wall", audience == null);
                audience = in;
            }
        }
        assertStretch(audience, Wall.AUDIENCE, SIDE, CENTER - SIDE + OVER);
        double flowerX = BioBuzzConstants.Field.FLOWER_CENTERS_IN[1][0];
        double half = BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN / 2.0;
        int onRear = 0;
        for (StartPoseSearch.Interval in : StartPoseSearch.legalIntervals(Alliance.RED, 90.0)) {
            if (in.wall == Wall.REAR) {
                if (onRear == 0) {
                    assertStretch(in, Wall.REAR, SIDE, flowerX - half - SIDE);
                } else {
                    assertStretch(in, Wall.REAR, flowerX + half + SIDE, CENTER - SIDE + OVER);
                }
                onRear++;
            }
        }
        assertEquals(2, onRear);
    }

    /**
     * Situation: the stretches of the red alliance for several headings, and those of the blue alliance for the half-turned headings.
     * Expected: each blue stretch is the half turn of a red one: the red wall becomes the blue wall and the audience wall the rear wall (and back), and
     *     a position along a wall x becomes the field size minus x.
     */
    @Test
    public void blueStretchesAreTheHalfTurnOfRedOnes() {
        for (double deg : new double[] {180.0, -90.0, 90.0, 0.0, 35.0, -110.0}) {
            List<StartPoseSearch.Interval> red = StartPoseSearch.legalIntervals(Alliance.RED, deg);
            List<StartPoseSearch.Interval> blue = StartPoseSearch.legalIntervals(Alliance.BLUE, deg + 180.0);
            assertEquals(deg + ": " + red + " / " + blue, red.size(), blue.size());
            for (StartPoseSearch.Interval r : red) {
                Wall turned = r.wall == Wall.RED ? Wall.BLUE : r.wall == Wall.AUDIENCE ? Wall.REAR : Wall.AUDIENCE;
                boolean found = false;
                for (StartPoseSearch.Interval b : blue) {
                    found |= b.wall == turned && Math.abs(b.fromIn - (SIZE - r.toIn)) < 1e-6 && Math.abs(b.toIn - (SIZE - r.fromIn)) < 1e-6;
                }
                assertTrue(deg + ": no blue stretch is the half turn of " + r + " in " + blue, found);
            }
        }
    }

    /**
     * Situation: the stretches whatever the heading, on the audience wall.
     * Expected: they run from 8.03 in (the robot facing along the wall, its back edge toward the corner) to the center line minus 8.03 in; and every stretch of
     *     one heading lies inside them.
     */
    @Test
    public void theLegalRegionWhateverTheHeadingContainsTheStretchOfEveryHeading() {
        List<StartPoseSearch.Interval> any = StartPoseSearch.legalIntervalsAnyHeading(Alliance.RED);
        StartPoseSearch.Interval audience = null;
        for (StartPoseSearch.Interval in : any) {
            if (in.wall == Wall.AUDIENCE) {
                assertTrue("the audience wall has no gap whatever the heading", audience == null);
                audience = in;
            }
        }
        assertStretch(audience, Wall.AUDIENCE, REAR, CENTER - REAR + OVER);
        for (double deg : new double[] {-90.0, 0.0, 35.0, 180.0}) {
            for (StartPoseSearch.Interval one : StartPoseSearch.legalIntervals(Alliance.RED, deg)) {
                boolean inside = false;
                for (StartPoseSearch.Interval in : any) {
                    inside |= in.wall == one.wall && in.fromIn <= one.fromIn + 1e-6 && in.toIn >= one.toIn - 1e-6;
                }
                assertTrue(deg + " degrees: " + one + " is not inside " + any, inside);
            }
        }
    }

    /**
     * Situation: the middle of a stretch and a point just inside its ends, then just outside, as starts.
     * Expected: inside is legal and clear, outside is not: the stretch ends are the real boundary of the rule.
     */
    @Test
    public void theEndsOfAStretchAreTheEdgeOfTheRule() {
        for (StartPoseSearch.Interval in : StartPoseSearch.legalIntervals(Alliance.RED, 180.0)) {
            Pose2 inside = StartPoseSearch.seatedAt(in.wall, (in.fromIn + in.toIn) / 2.0, 180.0);
            assertTrue(in + " middle", StartRules.isLegalAndClear(fieldOfFirstAlliance(), inside));
            Pose2 justInsideStart = StartPoseSearch.seatedAt(in.wall, in.fromIn + 0.01, 180.0);
            assertTrue(in + " just inside its start", StartRules.isLegalAndClear(RED, justInsideStart));
            Pose2 justInsideEnd = StartPoseSearch.seatedAt(in.wall, in.toIn - 0.01, 180.0);
            assertTrue(in + " just inside its end", StartRules.isLegalAndClear(RED, justInsideEnd));
        }
        // outside: just past the FLOWER end of the first red-wall stretch
        StartPoseSearch.Interval first = StartPoseSearch.legalIntervals(Alliance.RED, 180.0).get(0);
        assertFalse(StartRules.isLegalAndClear(RED, StartPoseSearch.seatedAt(first.wall, first.toIn + 0.02, 180.0)));
    }
}
