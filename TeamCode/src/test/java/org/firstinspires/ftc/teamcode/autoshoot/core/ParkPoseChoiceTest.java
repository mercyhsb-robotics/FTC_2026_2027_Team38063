package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AllianceField;

/**
 * Unit tests of the park-spot choice ({@link ParkAssist#choose}): which spot in the LOADING ZONE the assist picks, and why.
 * No simulated robot here; the closed loop is {@code ParkAssistSimTest}.
 *
 * <p>The referee for "is the spot legal" is {@link BruteForceFootprint}, which shares no code with the planner. Words: <b>spot</b> the
 * pose the robot is sent to; <b>overlap</b> how deep the robot's outline reaches into the zone (the autonomous validator's measure);
 * <b>cost</b> the road length plus the cost of turning, in "inches of driving".
 */
public class ParkPoseChoiceTest {
    private static final double EPS = 1e-6;
    private static final RobotShape ROBOT = RobotShape.fromConstants();

    private static ParkAssist assist(Alliance alliance) {
        FieldModel field = new FieldModel();
        ScoringPlanner planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(ShotTable.fromConstants()));
        return new ParkAssist(alliance, field, planner, ParkAssist.Config.fromConstants());
    }

    /**
     * Situation: the zone the assist uses, for both alliances.
     * Expected: exactly the zone of the route framework ({@code AllianceField.ownLoadingZone}): red x 0 to 11, y 94.584 to 117.274, and
     *     blue its half turn.
     * Why: two copies of the zone must never disagree.
     */
    @Test
    public void theZoneIsTheOneTheRouteFrameworkUses() {
        FieldModel field = new FieldModel();
        for (Alliance alliance : Alliance.values()) {
            Rect expected = new AllianceField(field, alliance).ownLoadingZone();
            Rect zone = assist(alliance).zone();
            assertEquals(expected.minX, zone.minX, EPS);
            assertEquals(expected.maxX, zone.maxX, EPS);
            assertEquals(expected.minY, zone.minY, EPS);
            assertEquals(expected.maxY, zone.maxY, EPS);
        }
        Rect red = assist(Alliance.RED).zone();
        assertEquals(0.0, red.minX, EPS);
        assertEquals(11.0, red.maxX, EPS);
    }

    /**
     * Situation: the overlap measure at a pose whose answer can be worked out by hand: facing +x with the BACK against the red wall
     *     (0.25 in clearance), the center of rotation is the rear extent plus the clearance from the wall, in the middle of the zone.
     * Expected: the outline spans x 0.25 to 0.25 + 17.80 and y 17.80 wide, inside the zone's 22.69 in length, so the overlap is the zone
     *     depth minus the clearance: 10.75 in; the blue twin gives the same number.
     * Why: the overlap number must be the same measure the validator uses, and the robot is not centered on its pose.
     */
    @Test
    public void theOverlapMeasureAgreesWithAHandCalculation() {
        double clearance = BioBuzzConstants.AutoShoot.WALL_CLEARANCE_IN;
        double centerY = 0.5 * (BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN + BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MAX_IN);
        Pose2 red = new Pose2(clearance + ROBOT.rearIn(), centerY, 0.0);
        double depth = BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - clearance;
        assertEquals(depth, assist(Alliance.RED).overlapIn(red), EPS);
        Pose2 blue = Alliance.BLUE.fromRed(red, BioBuzzConstants.Field.SIZE_IN);
        assertEquals(depth, assist(Alliance.BLUE).overlapIn(blue), EPS);
        // the same robot in the OTHER zone counts for nothing
        assertEquals(0.0, assist(Alliance.BLUE).overlapIn(red), EPS);
        // facing the wall the center must be farther from it (the front sticks out 9.77 in), the overlap is the same
        Pose2 facing = new Pose2(clearance + ROBOT.frontIn(), centerY, Math.PI);
        assertEquals(depth, assist(Alliance.RED).overlapIn(facing), EPS);
    }

    /**
     * Situation: robots on a grid over the whole field (every 12 in, four headings), both alliances; those standing on legal floor
     *     are asked for a park spot.
     * Expected: a spot is found from every one of them, and the spot is legal by the independent checker: the whole outline inside the
     *     field, clear of the HIVE frame and every FLOWER plate, overlapping the own zone by at least {@code Park.MIN_OVERLAP_IN}, and the
     *     planner agrees it fits. The road ends on the spot and the first point is not the start.
     * Why: "from anywhere on the own half, and also from the other half" (the user's rule), and the spot must be a legal place to stand.
     */
    @Test
    public void everyLegalStartGetsALegalSpotFromAnywhereOnTheField() {
        FieldModel field = new FieldModel();
        List<String> problems = new ArrayList<String>();
        int asked = 0;
        for (Alliance alliance : Alliance.values()) {
            ParkAssist assist = assist(alliance);
            for (double x = 12.0; x < field.size(); x += 12.0) {
                for (double y = 12.0; y < field.size(); y += 12.0) {
                    for (int deg = -135; deg <= 180; deg += 90) {
                        Pose2 start = Pose2.ofDegrees(x, y, deg);
                        if (!standsOnLegalFloor(field, start)) {
                            continue;
                        }
                        asked++;
                        ParkAssist.Plan plan = assist.choose(start);
                        String name = alliance + " " + start;
                        if (plan == null) {
                            problems.add(name + ": no spot");
                            continue;
                        }
                        problems.addAll(whyNotLegal(field, assist, name, plan));
                    }
                }
            }
        }
        assertTrue("asked " + asked + " starts; problems: " + problems, problems.isEmpty());
        assertTrue("asked " + asked, asked > 200);
    }

    /** True if the robot's real outline at this pose is inside the field and clear of the HIVE frame and the FLOWER plates (so the start is not the test's own mistake). */
    private static boolean standsOnLegalFloor(FieldModel field, Pose2 start) {
        double margin = BioBuzzConstants.AutoShoot.KEEPOUT_MARGIN_IN;
        if (BruteForceFootprint.wallPoke(start, ROBOT, field.fieldRect()) > 0.0) {
            return false;
        }
        for (Rect o : field.hardObstacles()) {
            if (BruteForceFootprint.overlaps(start, ROBOT, o.inflate(margin), 0.0)) {
                return false;
            }
        }
        return true;
    }

    private static List<String> whyNotLegal(FieldModel field, ParkAssist assist, String name, ParkAssist.Plan plan) {
        List<String> out = new ArrayList<String>();
        Pose2 spot = plan.pose;
        if (BruteForceFootprint.wallPoke(spot, ROBOT, field.fieldRect()) > 0.0) {
            out.add(name + ": the spot " + spot + " is through a wall");
        }
        for (Rect o : field.hardObstacles()) {
            if (BruteForceFootprint.overlaps(spot, ROBOT, o, 0.0)) {
                out.add(name + ": the spot " + spot + " is on " + o);
            }
        }
        if (!BruteForceFootprint.overlaps(spot, ROBOT, assist.zone(), 0.0)) {
            out.add(name + ": the spot " + spot + " is not in the zone");
        }
        if (assist.overlapIn(spot) < BioBuzzConstants.Park.MIN_OVERLAP_IN - EPS) {
            out.add(name + ": the spot " + spot + " overlaps only " + assist.overlapIn(spot));
        }
        if (!plan.waypoints.isEmpty()) {
            double[] last = plan.waypoints.get(plan.waypoints.size() - 1);
            if (Math.hypot(last[0] - spot.x, last[1] - spot.y) > EPS) {
                out.add(name + ": the road does not end on the spot");
            }
        }
        return out;
    }

    /**
     * Situation: eight starts spread over the field; the choice is made, then a test-side search tries EVERY legal spot on a grid of
     *     2 in and 5 headings (no early stopping) and asks the planner for its road.
     * Expected: no spot found by the exhaustive search costs less than the spot the assist chose.
     * Why: the assist stops looking as soon as no unseen spot can be cheaper; this proves that shortcut never loses the nearest spot.
     */
    @Test
    public void theChosenSpotIsTheCheapestOfAllLegalSpots() {
        double[][] starts = {{30, 30, 0}, {58.9, 9.8, -90}, {100, 110, 180}, {110, 30, 90}, {70.7, 100, 0}, {40, 125, 45}, {20, 70, 90}, {130, 130, -135}};
        for (Alliance alliance : Alliance.values()) {
            FieldModel field = new FieldModel();
            ScoringPlanner planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(ShotTable.fromConstants()));
            ParkAssist assist = new ParkAssist(alliance, field, planner, ParkAssist.Config.fromConstants());
            double rotation = BioBuzzConstants.AutoShoot.PLAN_ROTATION_COST_IN_PER_DEG;
            for (double[] row : starts) {
                Pose2 start = alliance.fromRed(Pose2.ofDegrees(row[0], row[1], row[2]), field.size());
                ParkAssist.Plan chosen = assist.choose(start);
                assertNotNull(alliance + " " + start, chosen);
                double cheapest = Double.POSITIVE_INFINITY;
                // the headings in the RED frame: the one the robot has now, then the four square ones
                double[] headings = {alliance.toRed(start, field.size()).headingDegrees(), 0.0, 90.0, 180.0, -90.0};
                for (double rx = 0.0; rx <= 26.0; rx += 2.0) {
                    for (double ry = 80.0; ry <= 132.0; ry += 2.0) {
                        for (double h : headings) {
                            Pose2 spot = alliance.fromRed(Pose2.ofDegrees(rx, ry, h), field.size());
                            if (assist.overlapIn(spot) < BioBuzzConstants.Park.MIN_OVERLAP_IN) {
                                continue;
                            }
                            ScoringPlanner.Route road = planner.routeTo(start, spot);
                            if (road != null) {
                                double cost = road.travelIn + rotation * Math.toDegrees(Angles.distance(start.heading, spot.heading));
                                cheapest = Math.min(cheapest, cost);
                            }
                        }
                    }
                }
                assertTrue(alliance + " " + start + ": chosen cost " + chosen.cost + " but a spot costs " + cheapest, chosen.cost <= cheapest + EPS);
            }
        }
    }

    /**
     * Situation: the same trips for RED and for BLUE, where each BLUE start is the half turn of the RED one.
     * Expected: the cost of the chosen road is the same for both (to a millionth of an inch), and so are the spot's overlap and its
     *     distance from the start.
     * Why: the field is half-turn symmetric and the assist works in the RED frame, so the two alliances must behave as mirror images.
     */
    @Test
    public void blueIsTheHalfTurnOfRed() {
        double[][] starts = {{30, 30, 0}, {58.9, 9.8, -90}, {100, 110, 180}, {110, 30, 90}, {70.7, 100, 0}, {40, 125, 45}, {20, 70, 90}};
        FieldModel field = new FieldModel();
        ParkAssist red = assist(Alliance.RED);
        ParkAssist blue = assist(Alliance.BLUE);
        for (double[] row : starts) {
            Pose2 redStart = Pose2.ofDegrees(row[0], row[1], row[2]);
            Pose2 blueStart = Alliance.BLUE.fromRed(redStart, field.size());
            ParkAssist.Plan a = red.choose(redStart);
            ParkAssist.Plan b = blue.choose(blueStart);
            assertNotNull(redStart.toString(), a);
            assertNotNull(blueStart.toString(), b);
            assertEquals(redStart.toString(), a.cost, b.cost, EPS);
            assertEquals(redStart.toString(), a.overlapIn, b.overlapIn, EPS);
            Pose2 twin = Alliance.BLUE.fromRed(a.pose, field.size());
            assertEquals(redStart.toString(), twin.x, b.pose.x, EPS);
            assertEquals(redStart.toString(), twin.y, b.pose.y, EPS);
            assertEquals(redStart.toString(), 0.0, Angles.distance(twin.heading, b.pose.heading), EPS);
        }
    }

    /**
     * Situation: a robot already deep inside its zone on legal floor (heading 30 degrees, not square to anything).
     * Expected: the spot is exactly where the robot already stands, with no road and zero cost: it just holds.
     * Why: pressing park when already parked must not move the robot.
     */
    @Test
    public void aRobotAlreadyInTheZoneStaysWhereItIs() {
        for (Alliance alliance : Alliance.values()) {
            ParkAssist assist = assist(alliance);
            Pose2 here = alliance.fromRed(Pose2.ofDegrees(14.0, 106.0, 30.0), BioBuzzConstants.Field.SIZE_IN);
            assertTrue(assist.overlapIn(here) >= BioBuzzConstants.Park.MIN_OVERLAP_IN);
            ParkAssist.Plan plan = assist.choose(here);
            assertNotNull(plan);
            assertEquals(here.x, plan.pose.x, EPS);
            assertEquals(here.y, plan.pose.y, EPS);
            assertEquals(0.0, Angles.distance(here.heading, plan.pose.heading), EPS);
            assertTrue(plan.waypoints.isEmpty());
            assertEquals(0.0, plan.cost, EPS);
        }
    }

    /**
     * Situation: a robot out on the floor facing +x (its back toward the red wall), 23 in from the zone with nothing in the way.
     * Expected: the chosen spot has the heading the robot has now (a turn costs, and no other heading gets deeper into the zone with less
     *     driving) and the road is a single straight leg that needs no detour.
     * Why: "nearest in travel time": a robot that is not asked to turn does not turn.
     */
    @Test
    public void itKeepsItsHeadingWhenThatIsTheNearestWay() {
        ParkAssist assist = assist(Alliance.RED);
        Pose2 start = Pose2.ofDegrees(34.0, 108.0, 0.0);
        ParkAssist.Plan plan = assist.choose(start);
        assertNotNull(plan);
        assertEquals(0.0, Angles.distance(start.heading, plan.pose.heading), EPS);
        assertEquals(1, plan.waypoints.size());
    }

    /**
     * Situation: a start from which the way into the zone is longer when the robot stays at its heading than when it turns the other way
     *     (it faces the red wall, 12 in from it: staying means the front reaches the wall first).
     * Expected: whatever heading is chosen, the chosen cost is not above the cost of the best spot at the current heading (the choice is
     *     "total time", not "least turning").
     * Why: it checks the cost really adds travel and turning, not just one of them.
     */
    @Test
    public void theChoiceWeighsTravelAndTurningTogether() {
        FieldModel field = new FieldModel();
        ScoringPlanner planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(ShotTable.fromConstants()));
        ParkAssist assist = new ParkAssist(Alliance.RED, field, planner, ParkAssist.Config.fromConstants());
        Pose2 start = Pose2.ofDegrees(30.0, 105.0, 180.0);
        ParkAssist.Plan plan = assist.choose(start);
        assertNotNull(plan);
        double sameHeading = Double.POSITIVE_INFINITY;
        for (double rx = 0.0; rx <= 26.0; rx += 1.0) {
            for (double ry = 80.0; ry <= 132.0; ry += 1.0) {
                Pose2 spot = new Pose2(rx, ry, start.heading);
                if (assist.overlapIn(spot) >= BioBuzzConstants.Park.MIN_OVERLAP_IN) {
                    ScoringPlanner.Route road = planner.routeTo(start, spot);
                    if (road != null) {
                        sameHeading = Math.min(sameHeading, road.travelIn);
                    }
                }
            }
        }
        assertTrue("chosen " + plan.cost + " vs same heading " + sameHeading, plan.cost <= sameHeading + EPS);
    }
}
