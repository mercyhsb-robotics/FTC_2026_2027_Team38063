package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/** The route catalog: ids, both alliances, the faithful conversions of the old autos and their validator findings. */
public class RouteCatalogTest {
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;

    private static List<Issue> issues(String id, Alliance a) {
        return RouteValidator.check(RouteCatalog.byId(id).forAlliance(a));
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

    @Test
    public void idsAreUniqueStableAndFindable() {
        Set<String> seen = new HashSet<String>();
        for (AutoRoute r : RouteCatalog.all()) {
            assertTrue("duplicate id " + r.id(), seen.add(r.id()));
            assertSame(r, RouteCatalog.byId(r.id()));
        }
        assertTrue(seen.contains("red_b1"));
        assertTrue(seen.contains("red_c1"));
        assertTrue(seen.contains("red_leave_park"));
        assertTrue(seen.contains("wall_shoot_park"));
        assertTrue(seen.contains("garden_cycle"));
        assertEquals(5, RouteCatalog.all().size());
    }

    @Test
    public void anUnknownIdNamesTheKnownOnes() {
        try {
            RouteCatalog.byId("nope");
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("red_b1"));
        }
    }

    @Test
    public void theCatalogListIsReadOnly() {
        try {
            RouteCatalog.all().clear();
            fail("all() must be unmodifiable");
        } catch (UnsupportedOperationException expected) {
            // good
        }
    }

    @Test
    public void everyRouteHasItsMetadataAndAStartTileThatMatchesItsStartPose() {
        for (AutoRoute r : RouteCatalog.all()) {
            assertFalse(r.id(), r.title().isEmpty());
            assertFalse(r.id(), r.description().isEmpty());
            Pose2 s = r.startPose();
            String tile = "" + (char) ('A' + (int) (s.x / 24.0)) + (1 + (int) (s.y / 24.0));
            assertEquals("start tile of " + r.id(), tile, r.startTile());
            assertEquals(Alliance.RED, r.alliance());
            assertEquals(StepKind.START, r.step(0).kind());
        }
    }

    @Test
    public void everyRouteBuildsAnalysesAndStaysWithinTheAutoTimeForBothAlliances() {
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                AutoRoute x = r.forAlliance(a);
                assertEquals(a, x.alliance());
                RouteAnalysis an = x.analyze();
                assertEquals(r.size(), an.steps().size());
                assertTrue(r.id() + " " + a + " time " + an.totalSeconds(),
                        an.totalSeconds() > 0 && an.totalSeconds() <= BioBuzzConstants.Auto.AUTO_DURATION_SEC);
                assertNotNull(RouteValidator.check(x));
            }
        }
    }

    @Test
    public void blueIsTheHalfTurnOfRedForEveryStepOfEveryRoute() {
        for (AutoRoute red : RouteCatalog.all()) {
            AutoRoute blue = red.forAlliance(Alliance.BLUE);
            List<Pose2> re = red.endPoses();
            List<Pose2> be = blue.endPoses();
            assertEquals(re.size(), be.size());
            for (int i = 0; i < re.size(); i++) {
                Pose2 expected = Alliance.BLUE.fromRed(re.get(i), SIZE);
                String where = red.id() + " step " + (i + 1);
                assertEquals(where, expected.x, be.get(i).x, 1e-6);
                assertEquals(where, expected.y, be.get(i).y, 1e-6);
                assertEquals(where, 0.0, Angles.distance(expected.heading, be.get(i).heading), 1e-6);
            }
            assertEquals(red.estimatedSeconds(), blue.estimatedSeconds(), 1e-9);
            assertEquals(red.claimablePoints(), blue.claimablePoints());
        }
    }

    @Test
    public void redAndBlueGiveTheSameValidatorFindings() {
        for (AutoRoute red : RouteCatalog.all()) {
            List<Issue> a = issues(red.id(), Alliance.RED);
            List<Issue> b = issues(red.id(), Alliance.BLUE);
            assertEquals(red.id(), a.size(), b.size());
            for (int i = 0; i < a.size(); i++) {
                assertEquals(red.id(), a.get(i).rule, b.get(i).rule);
                assertEquals(red.id(), a.get(i).severity, b.get(i).severity);
                assertEquals(red.id(), a.get(i).stepIndex, b.get(i).stepIndex);
            }
        }
    }

    // ---- faithful conversion of the old OpModes -----------------------------------------------------

    @Test
    public void redB1MatchesTheOpModeSource() {
        AutoRoute r = RouteCatalog.byId("red_b1");
        assertEquals("START, SHOOT, WAIT, DRIVE, DRIVE, PARK", 6, r.size());
        Pose2 start = r.startPose();
        assertEquals(35.0, start.x, 1e-9);
        assertEquals(9.2, start.y, 1e-9);
        assertEquals(-90.0, start.headingDegrees(), 1e-9);
        Step.Shoot shoot = (Step.Shoot) r.step(1);
        assertEquals(-115.0, shoot.poseOrNull().headingDegrees(), 1e-9);
        assertEquals(35.0, shoot.poseOrNull().x, 1e-9);
        assertEquals(5.0, shoot.fixedSeconds(), 1e-9);
        assertEquals(4, shoot.count());
        assertEquals(4.5, ((Step.Wait) r.step(2)).seconds(), 1e-9);
        Step.Drive p1 = (Step.Drive) r.step(3);
        Step.Drive p2 = (Step.Drive) r.step(4);
        assertEquals(HeadingMode.REVERSE_TANGENT, p1.mode());
        assertEquals(HeadingMode.REVERSE_TANGENT, p2.mode());
        assertEquals(35.8, p1.target().x, 1e-9);
        assertEquals(90.0, p1.target().y, 1e-9);
        assertEquals(12.0, p2.target().x, 1e-9);
        assertEquals(90.0, p2.target().y, 1e-9);
        assertTrue(p1.via().isEmpty() && p2.via().isEmpty());
        // the launcher stays on for 5.0 + 4.5 s as in the source
        assertEquals(9.5, shoot.fixedSeconds() + ((Step.Wait) r.step(2)).seconds(), 1e-9);
    }

    @Test
    public void redC1MatchesTheOpModeSource() {
        AutoRoute r = RouteCatalog.byId("red_c1");
        assertEquals(58.0, r.startPose().x, 1e-9);
        Step.Shoot shoot = (Step.Shoot) r.step(1);
        assertEquals(-90.0, shoot.poseOrNull().headingDegrees(), 1e-9);
        assertEquals(5.0, shoot.fixedSeconds(), 1e-9);
        double[][] targets = {{62.0, 118.1}, {58.0, 118.1}, {12.0, 118.1}};
        for (int i = 0; i < 3; i++) {
            Step.Drive d = (Step.Drive) r.step(3 + i);
            assertEquals(HeadingMode.REVERSE_TANGENT, d.mode());
            assertEquals(targets[i][0], d.target().x, 1e-9);
            assertEquals(targets[i][1], d.target().y, 1e-9);
            assertEquals("path" + (i + 1), d.label());
        }
        // Pedro ignores the -90 deg of the pose for reverseTangent: the robot really ends facing +x
        assertEquals(0.0, r.analyze().endPose().headingDegrees(), 1e-6);
    }

    @Test
    public void redLeaveParkMatchesTheOpModeSource() {
        AutoRoute r = RouteCatalog.byId("red_leave_park");
        assertEquals(9.2112, r.startPose().x, 1e-9);
        assertEquals(70.2702, r.startPose().y, 1e-9);
        assertEquals(180.0, r.startPose().headingDegrees(), 1e-9);
        Step.Drive p1 = (Step.Drive) r.step(1);
        Step.Drive p2 = (Step.Drive) r.step(2);
        assertEquals(HeadingMode.LINEAR, p1.mode());
        assertEquals(24.8608, p1.target().x, 1e-9);
        assertEquals(83.6152, p1.target().y, 1e-9);
        assertEquals(135.0, p1.target().headingDegrees(), 1e-9);
        assertEquals(16.0, p2.target().x, 1e-9);
        assertEquals(91.5, p2.target().y, 1e-9);
        for (Step s : r.steps()) {
            assertFalse("the old LeavePark never shoots", s.kind() == StepKind.SHOOT);
        }
    }

    /**
     * The notes of the converted autos quote how far the final footprint reaches into the LOADING ZONE. Those numbers
     * depend on the robot size, so they went stale once already: this keeps them honest.
     */
    @Test
    public void theParkNotesQuoteTheOverlapsTheGeometryGives() {
        for (String id : new String[] {RouteCatalog.RED_B1, RouteCatalog.RED_C1, RouteCatalog.RED_LEAVE_PARK}) {
            AutoRoute r = RouteCatalog.byId(id);
            RouteAnalysis a = r.analyze();
            Rect c = Footprint.of(a.endPose()).clippedBounds(a.field().ownLoadingZone());
            String w = String.format(java.util.Locale.ROOT, "%.1f", c.maxX - c.minX);
            String h = String.format(java.util.Locale.ROOT, "%.1f", c.maxY - c.minY);
            String shallow = String.format(java.util.Locale.ROOT, "%.1f", Math.min(c.maxX - c.minX, c.maxY - c.minY));
            String note = r.step(r.size() - 1).note();
            if (id.equals(RouteCatalog.RED_C1)) {
                assertTrue(note, note.contains(w + " x " + h + " in"));
            } else {
                assertTrue(id + ": " + note, note.contains(shallow + " in"));
            }
        }
    }

    // ---- what the validator honestly says about them -------------------------------------------------

    @Test
    public void redC1DrivesStraightThroughTheHiveFootprintAndTheValidatorSaysSo() {
        List<Issue> l = issues("red_c1", Alliance.RED);
        assertEquals(1, count(l, Rule.HIVE_KEEPOUT, Severity.ERROR));
        for (Issue i : l) {
            if (i.rule == Rule.HIVE_KEEPOUT) {
                assertEquals("path1", 3, i.stepIndex);
            }
        }
        // the shot itself is fine (37.4 in standoff, aimed) and the park is deep
        assertEquals(0, count(l, Rule.SHOOT_POSE, Severity.WARN));
        assertEquals(0, count(l, Rule.PARK_ZONE, Severity.WARN));
        assertEquals(1, l.size());
        assertFalse(RouteValidator.isRunnable(l));
    }

    @Test
    public void redB1TurnsThroughTheWallShootsFromTooFarAndParksShallowly() {
        List<Issue> l = issues("red_b1", Alliance.RED);
        // the -115 deg hold swings a corner 2.6 in through the audience wall, and so does turning back for path1
        assertEquals(2, count(l, Rule.FIELD_BOUNDS, Severity.ERROR));
        assertEquals(1, count(l, Rule.SHOOT_POSE, Severity.WARN));
        assertEquals(1, count(l, Rule.PARK_ZONE, Severity.WARN));
        // The old code drove up x = 35..35.8 for a 16.77 in wide robot. The real robot is 17.80 in wide, so its
        // right edge is at 35.8 + 8.9 = 44.7 in, 0.43 in inside the keep-out (the HIVE frame at x = 47.27 minus the
        // 3 in margin): path1 (step 4) enters it for the last 49 in, and path2 (step 5) turns on the spot at
        // (35.8, 90) with a corner even closer to the frame.
        assertEquals(2, count(l, Rule.HIVE_KEEPOUT, Severity.ERROR));
        java.util.List<Integer> keepOutSteps = new java.util.ArrayList<Integer>();
        for (Issue i : l) {
            if (i.rule == Rule.HIVE_KEEPOUT) {
                keepOutSteps.add(i.stepIndex);
            }
        }
        assertEquals(java.util.Arrays.asList(3, 4), keepOutSteps);
        assertFalse(RouteValidator.isRunnable(l));
    }

    @Test
    public void redLeaveParkPassesAndClaimsLeaveAndPark() {
        assertTrue("findings: " + issues("red_leave_park", Alliance.RED), issues("red_leave_park", Alliance.RED).isEmpty());
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.PARK_POINTS,
                RouteCatalog.byId("red_leave_park").claimablePoints());
    }

    // ---- the two demonstration routes --------------------------------------------------------------------

    @Test
    public void theDemonstrationRoutesPassTheValidatorCleanlyForBothAlliances() {
        for (String id : new String[] {"wall_shoot_park", "garden_cycle"}) {
            for (Alliance a : Alliance.values()) {
                List<Issue> l = issues(id, a);
                assertTrue(id + " " + a + ": " + l, l.isEmpty());
            }
        }
    }

    @Test
    public void wallShootParkShootsThePreloadsThenParksInTheCornerOfTheZone() {
        AutoRoute r = RouteCatalog.byId("wall_shoot_park");
        assertEquals("C1", r.startTile());
        RouteAnalysis a = r.analyze();
        assertEquals(4, a.pollenShot());
        assertEquals(0, a.tips());
        assertTrue(a.park());
        assertTrue(a.leave());
        assertEquals(8, r.claimablePoints());
        assertTrue(a.parkOverlapIn() > 7.0);
        // it uses the curved leg and the left corridor
        Step.Drive curve = (Step.Drive) r.step(2);
        assertEquals(1, curve.via().size());
        for (RouteAnalysis.StepInfo s : a.steps()) {
            assertTrue("never right of x = 47 in the corridor part", s.step.kind() != StepKind.DRIVE || s.to.x < 47.0);
        }
    }

    @Test
    public void gardenCycleReachesTheTipThresholdWithTheGardenPollen() {
        AutoRoute r = RouteCatalog.byId("garden_cycle");
        assertEquals("B1", r.startTile());
        RouteAnalysis a = r.analyze();
        assertEquals(8, a.pollenShot());
        assertEquals(4, a.pollenGained());
        assertEquals(BioBuzzConstants.Auto.TIP_POLLEN_THRESHOLD, a.pollenShot());
        assertEquals(1, a.tips());
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.TIP_POINTS, r.claimablePoints());
        assertEquals(0, a.pollenEnd());
        assertEquals("own GARDEN", a.step(4).sourceName);
        assertTrue(a.totalSeconds() < BioBuzzConstants.Auto.AUTO_DURATION_SEC - BioBuzzConstants.Auto.TIME_SAFETY_MARGIN_SEC);
    }
}
