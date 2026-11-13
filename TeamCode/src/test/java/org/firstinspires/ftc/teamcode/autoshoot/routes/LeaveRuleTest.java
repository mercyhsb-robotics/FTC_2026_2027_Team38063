package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The LEAVE rule as FIRST's Q&amp;A QA21 settles it: LEAVE is judged at the END of AUTO and is lost by a robot that touches ANY perimeter
 * wall then, also the wall inside the LOADING ZONE. A robot that parks in the LOADING ZONE touching its wall keeps its PARK (5) but not
 * its LEAVE (3); to claim both it must end standing off the wall (the zone is 11 in deep, so there is room). A POLLEN squeezed between
 * the robot and the wall does not count as the robot touching the wall (QA47).
 *
 * <p>How it works: every test builds a small route that ends in the red LOADING ZONE (or at the GARDEN) at a chosen distance from the
 * wall and reads the analysis (the same simulation the validator, the points and the gallery use), for RED and for the BLUE half turn.
 */
public class LeaveRuleTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double REAR = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double MARGIN = BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN;
    /** y of the middle of the red LOADING ZONE (y 94.6 to 117.3). */
    private static final double ZONE_Y = 106.0;

    /**
     * A route that starts legally on the red wall and drives to the LOADING ZONE, facing +x (the shooter toward the wall) with the
     * BACK edge {@code backEdgeOffWallIn} inches off the red wall: 0 is touching the wall.
     */
    private static AutoRoute parkWithBackEdgeOffWall(double backEdgeOffWallIn) {
        double x = REAR + backEdgeOffWallIn;
        return AutoRoute.builder("park").start(FRONT, 70.2702, 180).driveLinear(x, ZONE_Y, 0).park(x, ZONE_Y, 0).build();
    }

    private static int count(List<Issue> issues, Rule rule, Severity severity) {
        int n = 0;
        for (Issue i : issues) {
            if (i.rule == rule && i.severity == severity) {
                n++;
            }
        }
        return n;
    }

    // ---- QA21: touching the wall in the LOADING ZONE loses LEAVE, not PARK ---------------------------------

    @Test
    public void aRobotParkedInTheZoneTouchingTheWallKeepsParkButLosesLeave() {
        for (Alliance alliance : Alliance.values()) {
            AutoRoute route = parkWithBackEdgeOffWall(0.0).forAlliance(alliance);
            RouteAnalysis a = route.analyze();
            assertTrue(alliance + ": the robot is in the LOADING ZONE", a.park());
            assertFalse(alliance + ": it touches the wall of the zone, so no LEAVE", a.leave());
            assertEquals("only the PARK is scored", BioBuzzConstants.Auto.PARK_POINTS, a.points());
            assertEquals(0.0, a.endWallGapIn(), 1e-9);
            assertEquals(alliance == Alliance.RED ? "red wall (x = 0)" : "blue wall (x = " + String.format(java.util.Locale.ROOT, "%.1f", BioBuzzConstants.Field.SIZE_IN) + ")",
                    a.endNearestWall());
        }
    }

    @Test
    public void theValidatorWarnsThatLeaveIsLostAndSaysHowToClaimBoth() {
        for (Alliance alliance : Alliance.values()) {
            List<Issue> l = RouteValidator.check(parkWithBackEdgeOffWall(0.0).forAlliance(alliance));
            assertEquals(alliance + ": " + l, 1, count(l, Rule.LEAVE_QA21, Severity.WARN));
            assertEquals("a WARN, not an ERROR: the PARK is still worth having", 0, count(l, Rule.LEAVE_QA21, Severity.ERROR));
            Issue issue = null;
            for (Issue i : l) {
                if (i.rule == Rule.LEAVE_QA21) {
                    issue = i;
                }
            }
            assertEquals("on the last step", 2, issue.stepIndex);
            assertTrue(issue.message, issue.message.contains("in the LOADING ZONE but only 0.0 in from the"));
            assertTrue(issue.message, issue.message.contains("PARK (5) still counts"));
            assertTrue(issue.message, issue.message.contains("loses its LEAVE (3"));
            assertTrue(issue.message, issue.message.contains("QA21"));
            assertTrue(issue.message, issue.message.contains("the one inside the LOADING ZONE too"));
            assertTrue(issue.message, issue.message.contains("end 1.5 in or more off every wall"));
            assertTrue("the zone is 11 in deep, so 9.5 in of it can still be reached", issue.message.contains("reaches 9.5 in into it"));
        }
    }

    @Test
    public void aRobotParkedInTheZoneStandingOffTheWallClaimsBothAndGetsNoWarning() {
        for (Alliance alliance : Alliance.values()) {
            AutoRoute route = parkWithBackEdgeOffWall(MARGIN + 0.1).forAlliance(alliance);
            RouteAnalysis a = route.analyze();
            assertTrue(a.park());
            assertTrue("1.6 in off the wall is clear of it", a.leave());
            assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.PARK_POINTS, a.points());
            assertEquals(MARGIN + 0.1, a.endWallGapIn(), 1e-9);
            assertEquals(alliance + ": " + RouteValidator.check(route), 0, count(RouteValidator.check(route), Rule.LEAVE_QA21, Severity.WARN));
            assertTrue("it still overlaps the zone by a lot: " + a.parkOverlapIn(), a.parkOverlapIn() > BioBuzzConstants.Auto.PARK_MIN_OVERLAP_IN);
        }
    }

    @Test
    public void theMarginIsTheLineBetweenLosingAndKeepingLeave() {
        // just inside the margin (1.4 in off the wall) loses LEAVE, just outside (1.6 in) keeps it; the margin itself is not a touch
        assertFalse(parkWithBackEdgeOffWall(MARGIN - 0.1).analyze().leave());
        assertTrue(parkWithBackEdgeOffWall(MARGIN + 0.1).analyze().leave());
        assertEquals(1, count(RouteValidator.check(parkWithBackEdgeOffWall(MARGIN - 0.1)), Rule.LEAVE_QA21, Severity.WARN));
        assertEquals(0, count(RouteValidator.check(parkWithBackEdgeOffWall(MARGIN + 0.1)), Rule.LEAVE_QA21, Severity.WARN));
    }

    @Test
    public void everyWallCountsNotOnlyTheOneOfTheZone() {
        // The audience wall: a robot that ends facing it with its front edge on it touches it and loses LEAVE, whatever else it does.
        RouteAnalysis onAudienceWall = AutoRoute.builder("audience").start(35, FRONT, -90).waitSeconds(1).build().analyze();
        assertFalse(onAudienceWall.leave());
        assertEquals("audience wall (y = 0)", onAudienceWall.endNearestWall());
        // ... but it is not in the zone, so the WARN about a PARK that loses LEAVE does not apply
        assertEquals(0, count(RouteValidator.check(AutoRoute.builder("audience").start(35, FRONT, -90).waitSeconds(1).build()), Rule.LEAVE_QA21, Severity.WARN));
        // a robot 2 in off the audience wall keeps LEAVE
        assertTrue(AutoRoute.builder("off").start(35, FRONT, -90).driveConstant(35, FRONT + 2.0, -90).build().analyze().leave());
    }

    // ---- QA47: a POLLEN between the robot and the wall is not the robot touching it ----------------------------

    @Test
    public void aPollenSqueezedBetweenTheRobotAndTheWallDoesNotCostLeave() {
        // The GARDEN POLLEN lie along the audience wall (centers 1.4 in off it, 2.8 in across). A robot whose front edge is 1.6 in off the
        // wall, over a POLLEN, is not touching the wall: LEAVE stays, with the POLLEN taken or not.
        double y = FRONT + MARGIN + 0.1;
        double x = BioBuzzConstants.Robot.LEFT_EXTENT_IN + MARGIN + 0.1;          // also off the red wall, which the GARDEN corner is next to
        AutoRoute withPollen = AutoRoute.builder("pollen").start(35, FRONT, -90).driveConstant(x, y, -90)
                .intakeGarden(5.7, 1.4, x, y, -90, 3).build();
        AutoRoute without = AutoRoute.builder("without").start(35, FRONT, -90).driveConstant(x, y, -90).build();
        assertTrue("POLLEN between the robot and the wall do not make it touch the wall (QA47)", withPollen.analyze().leave());
        assertEquals("LEAVE depends on the robot's footprint only", without.analyze().leave(), withPollen.analyze().leave());
        assertEquals(without.analyze().endWallGapIn(), withPollen.analyze().endWallGapIn(), 1e-9);
        // the same robot 1.0 in off the wall is inside the margin whatever lies there
        double closer = FRONT + 1.0;
        AutoRoute tooClose = AutoRoute.builder("close").start(35, FRONT, -90).driveConstant(x, closer, -90)
                .intakeGarden(5.7, 1.4, x, closer, -90, 3).build();
        assertFalse(tooClose.analyze().leave());
    }

    // ---- the manual does not call a FLOWER or the HIVE "the wall" -------------------------------------------------

    @Test
    public void touchingAFlowerDoesNotCostLeaveBecauseItIsNotThePerimeterWall() {
        // face the red-wall FLOWER with the front edge on its plate: the robot is 5 in off the wall, so LEAVE stays
        AllianceField red = new AllianceField(new org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel(), Alliance.RED);
        Rect plate = red.flowerPlates().get(0);
        double x = plate.maxX + FRONT;
        AutoRoute route = AutoRoute.builder("flower").start(FRONT, 70.2702, 180).driveLinear(x, 48.0, 180).build();
        RouteAnalysis a = route.analyze();
        assertEquals("the front edge touches the plate", plate.maxX, Footprint.of(a.endPose()).bounds().minX, 1e-9);
        assertTrue(a.leave());
        assertTrue(a.endWallGapIn() > MARGIN);
    }

    // ---- the routes of the catalog ------------------------------------------------------------------------------

    @Test
    public void noRouteOfTheCatalogParksInTheZoneAndLosesItsLeave() {
        List<String> bad = new ArrayList<String>();
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance alliance : Alliance.values()) {
                RouteAnalysis a = r.forAlliance(alliance).analyze();
                if (a.park() && !a.leave()) {
                    bad.add(r.id() + " " + alliance);
                }
            }
        }
        assertTrue("these routes park touching a wall and so lose LEAVE (QA21): " + bad, bad.isEmpty());
    }

    @Test
    public void theCatalogDescriptionOfTheDemonstrationParkSaysWhyItKeepsLeave() {
        AutoRoute r = RouteCatalog.byId(RouteCatalog.WALL_SHOOT_PARK);
        assertTrue(r.analyze().park() && r.analyze().leave());
        assertTrue(r.description(), r.description().contains("QA21"));
        assertTrue("the description's 4 in is the real gap: " + r.analyze().endWallGapIn(), Math.abs(r.analyze().endWallGapIn() - 4.0) < 0.1);
    }
}
