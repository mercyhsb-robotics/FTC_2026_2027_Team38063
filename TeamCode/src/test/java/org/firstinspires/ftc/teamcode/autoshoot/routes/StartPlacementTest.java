package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.junit.Test;

/**
 * The words that tell a student where to put the robot ({@link StartPlacement}): exact sentences for the kinds of start, the same words for the other alliance
 * (the half turn), and a test that FOLLOWING the words gives back the start.
 */
public class StartPlacementTest {
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double REAR = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double SIDE = BioBuzzConstants.Robot.LEFT_EXTENT_IN;

    private static String red(Pose2 p) {
        return StartPlacement.placementText(p, Alliance.RED);
    }

    // ---- straight at a wall, one wall ------------------------------------------------------------------------------

    /**
     * Situation: the team's usual start: front on the audience wall, the center of rotation at x = 60.3 (so the robot spans x 51.4 to 69.2).
     * Expected: the words of the manual's example: which side is on the wall, how far the edge nearest the red wall is from it, which way it faces. The edge is
     *     named by the robot's own right and left (a robot facing the audience wall has its right side toward the red wall).
     */
    @Test
    public void frontOnTheAudienceWall() {
        assertEquals("Front on the audience wall. Right edge 51.4 in from the red wall. Facing the audience wall.", red(Pose2.ofDegrees(60.3, FRONT, -90)));
        // an old OpMode number: the words describe where the robot IS (its front on the wall), whatever the validator says about the pose
        assertEquals("Front on the audience wall. Right edge 26.1 in from the red wall. Facing the audience wall.", red(Pose2.ofDegrees(35.0, 9.2, -90)));
    }

    /** Situation: back on the audience wall facing the rear wall; a side on the audience wall facing the blue wall; front on the red wall; front on the rear wall. Expected: the matching words. */
    @Test
    public void everySideCanBeOnTheWall() {
        assertEquals("Back on the audience wall. Left edge 26.1 in from the red wall. Facing the rear wall.", red(Pose2.ofDegrees(35.0, REAR, 90)));
        assertEquals("Right side on the audience wall. Back edge 27.0 in from the red wall. Facing the blue wall.", red(Pose2.ofDegrees(35.0, SIDE, 0)));
        assertEquals("Left side on the audience wall. Front edge 25.2 in from the red wall. Facing the red wall.", red(Pose2.ofDegrees(35.0, SIDE, 180)));
        assertEquals("Front on the red wall. Left edge 61.4 in from the audience wall. Facing the red wall.", red(Pose2.ofDegrees(FRONT, 70.27, 180)));
        assertEquals("Front on the rear wall. Left edge 52.6 in from the red wall. Facing the rear wall.", red(Pose2.ofDegrees(61.5, SIZE - FRONT, 90)));
    }

    /**
     * Situation: the same starts for the BLUE alliance (the half turn of the red ones about the middle of the field).
     * Expected: the same words with the walls turned the same way: red wall to blue wall, audience wall to rear wall and back. The numbers are the same, because
     *     the distance is measured from the alliance's own side wall.
     */
    @Test
    public void theBlueWordsAreTheRedWordsTurnedHalfWayRound() {
        Pose2 start = Pose2.ofDegrees(60.3, FRONT, -90);
        assertEquals("Front on the rear wall. Right edge 51.4 in from the blue wall. Facing the rear wall.",
                StartPlacement.placementText(Alliance.BLUE.fromRed(start, SIZE), Alliance.BLUE));
        // a start on the side wall is measured from the AUDIENCE wall for both alliances (the half turn of the red wall is the blue wall, facing +x)
        Pose2 onSide = Pose2.ofDegrees(FRONT, 70.27, 180);
        assertEquals("Front on the blue wall. Right edge 62.2 in from the audience wall. Facing the blue wall.",
                StartPlacement.placementText(Alliance.BLUE.fromRed(onSide, SIZE), Alliance.BLUE));
    }

    // ---- corners, turned robots, not touching ----------------------------------------------------------------------------------

    /** Situation: the robot straight in the audience/red corner, front on the red wall. Expected: both walls are named, nothing to measure. */
    @Test
    public void aStraightRobotInACornerNamesBothWalls() {
        assertEquals("Front on the red wall and left side on the audience wall. Facing the red wall.", red(Pose2.ofDegrees(FRONT, SIDE, 180)));
        assertEquals("Back on the red wall and right side on the audience wall. Facing the blue wall.", red(Pose2.ofDegrees(REAR, SIDE, 0)));
    }

    /**
     * Situation: the robot turned 40 degrees clockwise from facing the audience wall, one corner on the wall at x about 35.
     * Expected: the corner that touches the wall (by its name on the robot), how far along the wall it is from the red wall, and the turn, clockwise as seen from above
     *     (the heading goes down by 40: from -90 to -130 degrees). The numbers are those of the real outline, worked out here from the corners by hand.
     */
    @Test
    public void aTurnedRobotIsDescribedByItsCornerAndTheAngle() {
        double heading = Math.toRadians(-130.0);
        double[][] corners = {{FRONT, SIDE}, {-REAR, SIDE}, {-REAR, -SIDE}, {FRONT, -SIDE}};         // forward, left of the four corners, front-left first
        String[] names = {"Front-left", "Back-left", "Back-right", "Front-right"};
        double bestY = Double.POSITIVE_INFINITY;
        int index = 0;
        for (int i = 0; i < 4; i++) {
            double y = corners[i][0] * Math.sin(heading) + corners[i][1] * Math.cos(heading);
            if (y < bestY) {
                bestY = y;
                index = i;
            }
        }
        double cornerX = 35.25 + corners[index][0] * Math.cos(heading) - corners[index][1] * Math.sin(heading);
        Pose2 start = new Pose2(35.25, -bestY, heading);
        assertEquals(names[index] + String.format(Locale.ROOT, " corner on the audience wall, %.1f in from the red wall. Turned 40 degrees clockwise (seen from above, as in the diagram)"
                + " from facing the audience wall.", cornerX), red(start));
        // the other way: 25 degrees counter-clockwise, and a heading that is not a whole number of degrees
        assertTrue(red(Pose2.ofDegrees(40.0, 12.0, -65.0)).contains("Turned 25 degrees counter-clockwise"));
        assertTrue(red(Pose2.ofDegrees(40.0, 12.0, -77.5)).contains("Turned 12.5 degrees counter-clockwise"));
    }

    /** Situation: a robot turned 30 degrees in the audience/red corner (as in the manual's Figure 11-1). Expected: a sentence for each wall. */
    @Test
    public void aTurnedRobotInACornerGivesAFixForEachWall() {
        double heading = Math.toRadians(150.0);
        org.firstinspires.ftc.teamcode.autoshoot.core.Rect box = Footprint.of(new Pose2(0.0, 0.0, heading)).bounds();
        String text = red(new Pose2(-box.minX, -box.minY, heading));
        assertTrue(text, text.contains("corner on the red wall,") && text.contains("corner on the audience wall,"));
        assertTrue(text, text.contains("in from the audience wall.") && text.contains("in from the red wall."));
        assertTrue(text, text.contains("Turned 30 degrees clockwise") && text.endsWith("from facing the red wall."));
    }

    /** Situation: a robot 10.2 in from the audience wall. Expected: it says this is not a legal start and how far from the wall it is. */
    @Test
    public void aStartThatTouchesNoWallSaysSo() {
        String text = red(Pose2.ofDegrees(35.0, 20.0, -90));
        assertEquals("This is not a legal start: the robot touches no wall (rule G304.C). Its nearest edge is 10.2 in from the audience wall.", text);
    }

    /**
     * Situation: a robot 0.8 degrees and then 2 degrees off straight at the audience wall.
     * Expected: 0.8 is straight (the tolerance is 1 degree); 2 is described as a turn.
     */
    @Test
    public void straightMeansWithinTheTolerance() {
        assertTrue(red(Pose2.ofDegrees(60.0, FRONT, -90.8)).endsWith("Facing the audience wall."));
        assertFalse(red(Pose2.ofDegrees(60.0, 10.0, -92.0)).contains("Facing the audience wall."));
        assertTrue(red(Pose2.ofDegrees(60.0, 10.0, -92.0)).contains("Turned 2 degrees clockwise"));
    }

    // ---- routes ---------------------------------------------------------------------------------------------------------------

    /** Situation: a route (RED) and its BLUE version. Expected: the words are those of the route's start pose in the route's own alliance. */
    @Test
    public void aRouteGetsTheWordsOfItsStart() {
        AutoRoute route = AutoRoute.builder("x").start(60.3, FRONT, -90).waitSeconds(1).build();
        assertEquals(red(route.startPose()), StartPlacement.placementText(route));
        assertEquals(StartPlacement.placementText(route.forAlliance(Alliance.BLUE).startPose(), Alliance.BLUE), StartPlacement.placementText(route.forAlliance(Alliance.BLUE)));
        assertTrue(StartPlacement.placementText(route.forAlliance(Alliance.BLUE)).contains("blue wall"));
    }

    /** Situation: every route of the catalog, both alliances. Expected: words for each; printed so the team can read what the diagrams will say. */
    @Test
    public void everyRouteOfTheCatalogHasWords() {
        StringBuilder all = new StringBuilder("\nWHERE TO PUT THE ROBOT, route by route\n");
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                String text = StartPlacement.placementText(r.forAlliance(a));
                assertFalse(r.id() + " " + a, text.isEmpty());
                all.append(String.format(Locale.ROOT, "  %-26s %-4s %s%n", r.id(), a, text));
            }
        }
        System.out.println(all);
    }

    // ---- following the words gives the start back -------------------------------------------------------------------------------

    private static final Pattern STRAIGHT_ONE_WALL = Pattern.compile("^(Front|Back|Left side|Right side) on the (\\w+) wall\\. (Front|Back|Left|Right) edge ([0-9.]+) in from the (\\w+) wall\\. Facing the (\\w+) wall\\.$");

    /** The direction (radians) a robot faces when it faces the named wall. */
    private static double direction(String wall) {
        for (Wall w : Wall.values()) {
            if (w.plainName().equals(wall + " wall")) {
                return w.outwardRad();
            }
        }
        throw new AssertionError("no wall " + wall);
    }

    /**
     * Situation: every legal start of the coarse grid (both alliances) that is straight at a wall and touches only one.
     * Expected: the words parse, and FOLLOWING them (put this side on that wall, put the edge nearest the other wall this far from it, face that wall) lands the
     *     robot on the start, to within the 0.05 in that the one decimal of the text loses.
     * Why: it proves that the sentence holds the whole start and that its numbers and names are right for every heading and wall, not just for the examples.
     */
    @Test
    public void followingTheWordsPutsTheRobotOnTheStart() {
        int checked = 0;
        for (Alliance alliance : Alliance.values()) {
            for (Pose2 p : StartPoseSearch.legalStarts(alliance, 1.0, 90.0)) {
                String text = StartPlacement.placementText(p, alliance);
                Matcher m = STRAIGHT_ONE_WALL.matcher(text);
                if (!m.matches()) {
                    continue;               // a corner start: nothing to measure
                }
                double facing = direction(m.group(6));
                double along = Double.parseDouble(m.group(4));
                // rebuild the pose from the words: the heading is the way it faces; the edge named is that edge of the robot; the wall sets the other coordinate
                Pose2 turned = new Pose2(0.0, 0.0, facing);
                double extent = edgeExtent(m.group(3));
                String reference = m.group(5);
                // the edge is 'along' inches from the reference wall; the pose point is 'extent' further in (the edge is the part of the robot nearest that wall)
                double alongCoordinate;
                switch (reference) {
                    case "red":
                        alongCoordinate = along + extent;
                        break;
                    case "blue":
                        alongCoordinate = SIZE - along - extent;
                        break;
                    default:                                  // the audience wall, for a start on the red or blue wall
                        alongCoordinate = along + extent;
                        break;
                }
                Wall on = Wall.AUDIENCE;
                for (Wall w : Wall.values()) {
                    if (w.plainName().equals(m.group(2) + " wall")) {
                        on = w;
                    }
                }
                Pose2 rebuilt = StartPoseSearch.seatedAt(on, alongCoordinate, turned.heading, SIZE);
                assertEquals(text + " -> x", p.x, rebuilt.x, 0.06);
                assertEquals(text + " -> y", p.y, rebuilt.y, 0.06);
                assertEquals(text + " -> heading", 0.0, Math.abs(org.firstinspires.ftc.teamcode.autoshoot.core.Angles.error(p.heading, rebuilt.heading)), 1e-6);
                checked++;
            }
        }
        assertTrue("checked " + checked + " starts", checked > 100);
    }

    /** How far the named edge of the robot is from the pose point, inches. */
    private static double edgeExtent(String edge) {
        switch (edge) {
            case "Front":
                return FRONT;
            case "Back":
                return REAR;
            case "Left":
                return SIDE;
            default:
                return BioBuzzConstants.Robot.RIGHT_EXTENT_IN;
        }
    }

    /** Situation: the corner a turned start names. Expected: it is a real corner of the outline, and it is on the wall (touching it, to within the touching tolerance). */
    @Test
    public void theCornerNamedIsTheOneOnTheWall() {
        List<Pose2> turned = StartPoseSearch.legalStarts(Alliance.RED, 4.0, 15.0);
        int checked = 0;
        for (Pose2 p : turned) {
            String text = StartPlacement.placementText(p, Alliance.RED);
            if (!text.contains("Turned")) {
                continue;
            }
            Matcher m = Pattern.compile("(Front-left|Back-left|Back-right|Front-right) corner on the (\\w+) wall, ([0-9.]+) in from the (\\w+) wall").matcher(text);
            assertTrue(text, m.find());
            Footprint fp = Footprint.of(p);
            String[] names = {"Front-left", "Back-left", "Back-right", "Front-right"};
            int index = java.util.Arrays.asList(names).indexOf(m.group(1));
            Point2 c = fp.corner(index);
            Wall wall = Wall.AUDIENCE;
            for (Wall w : Wall.values()) {
                if (w.plainName().equals(m.group(2) + " wall")) {
                    wall = w;
                }
            }
            // 'on the wall' means within the touching tolerance: a robot turned in a corner touches one wall exactly and the other to within a fraction of an inch
            double gap = wall.gapIn(new org.firstinspires.ftc.teamcode.autoshoot.core.Rect(c.x, c.x, c.y, c.y), SIZE);
            assertTrue(text + " gap " + gap, gap > -1e-6 && gap <= BioBuzzConstants.Auto.START_WALL_CONTACT_TOLERANCE_IN);
            checked++;
        }
        assertTrue("checked " + checked, checked > 50);
    }
}
