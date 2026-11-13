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
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
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
        assertEquals("the five routes of the catalog itself plus whatever the two family files hold",
                5 + SelfTipRoutes.routes().size() + PartnerTipRoutes.routes().size(), RouteCatalog.all().size());
    }

    @Test
    public void theCatalogIsItsOwnRoutesThenTheSelfFamilyThenThePartnerFamily() {
        List<String> expected = new java.util.ArrayList<String>(java.util.Arrays.asList(RouteCatalog.RED_B1, RouteCatalog.RED_C1,
                RouteCatalog.RED_LEAVE_PARK, RouteCatalog.WALL_SHOOT_PARK, RouteCatalog.GARDEN_CYCLE));
        for (AutoRoute r : SelfTipRoutes.routes()) {
            expected.add(r.id());
        }
        for (AutoRoute r : PartnerTipRoutes.routes()) {
            expected.add(r.id());
        }
        List<String> actual = new java.util.ArrayList<String>();
        for (AutoRoute r : RouteCatalog.all()) {
            actual.add(r.id());
        }
        assertEquals(expected, actual);
        // and all of them have different ids, across the three places
        assertEquals(actual.size(), new HashSet<String>(actual).size());
    }

    @Test
    public void everyRouteSaysWhoShootsTheFirstTipAndEachFamilyFileKeepsItsOwnAssumption() {
        for (AutoRoute r : RouteCatalog.all()) {
            assertNotNull(r.id(), r.firstTip());
            assertFalse(r.id(), r.firstTipSummary().isEmpty());
        }
        // the routes written in the catalog itself shoot the first tip themselves
        for (String id : new String[] {RouteCatalog.RED_B1, RouteCatalog.RED_C1, RouteCatalog.RED_LEAVE_PARK,
                RouteCatalog.WALL_SHOOT_PARK, RouteCatalog.GARDEN_CYCLE}) {
            assertEquals(id, FirstTip.SELF, RouteCatalog.byId(id).firstTip());
        }
        assertEquals(java.util.Collections.<String>emptyList(),
                RouteCatalog.familyProblems("SelfTipRoutes", SelfTipRoutes.routes(), FirstTip.SELF));
        assertEquals(java.util.Collections.<String>emptyList(),
                RouteCatalog.familyProblems("PartnerTipRoutes", PartnerTipRoutes.routes(), FirstTip.PARTNER));
    }

    @Test
    public void noRouteOfTheCatalogBreaksTheTipRulesForEitherAlliance() {
        // G417 (a shot at a cell that is not UP, a volley that goes on after the tip, a shot at the partner's cell without
        // waiting for the camera to see it up) and G409 (standing where the tipped cell spills): an ERROR of these rules is
        // a route that cannot be run as written. The routes of the two family files are checked the day they are added.
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                List<Issue> l = RouteValidator.check(r.forAlliance(a));
                assertEquals(r.id() + " " + a + ": " + l, 0, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
                assertEquals(r.id() + " " + a + ": " + l, 0, count(l, Rule.SPILL_G409, Severity.ERROR));
            }
        }
    }

    @Test
    public void everyPartnerRouteOfTheCatalogHasAMeasurableSlackAndWaitsForTheTipBeforeShootingAtIt() {
        for (AutoRoute r : RouteCatalog.all()) {
            RouteAnalysis a = r.analyze();
            if (r.firstTip() != FirstTip.PARTNER) {
                assertTrue(r.id(), Double.isNaN(a.latestPartnerTipSec()));
                continue;
            }
            assertFalse(r.id() + " has a latest partner tip", Double.isNaN(a.latestPartnerTipSec()));
            // a partner route that fires at the cell that comes up must hold a WAIT_FOR_TIP (the validator says so too)
            boolean waits = false;
            for (int i = 0; i < r.size(); i++) {
                waits |= r.step(i).kind() == StepKind.WAIT_FOR_TIP;
            }
            boolean dependsOnTip = !Double.isInfinite(a.latestPartnerTipSec());
            assertTrue(r.id() + ": a route that depends on the partner's tip waits for it", !dependsOnTip || waits);
        }
    }

    @Test
    public void aRouteInTheWrongFamilyFileIsReportedByName() {
        AutoRoute self = AutoRoute.builder("shoots_itself").start(58, 9.2, -90).waitSeconds(1).build();
        AutoRoute partner = AutoRoute.builder("waits_for_partner").start(61.5, 135.1, 90).partnerTipsFirst().waitSeconds(1).build();
        List<String> inPartnerFile = RouteCatalog.familyProblems("PartnerTipRoutes", java.util.Arrays.asList(self, partner), FirstTip.PARTNER);
        assertEquals(inPartnerFile.toString(), 1, inPartnerFile.size());
        assertTrue(inPartnerFile.get(0), inPartnerFile.get(0).contains("shoots_itself"));
        assertTrue(inPartnerFile.get(0), inPartnerFile.get(0).contains("PartnerTipRoutes"));
        assertTrue(inPartnerFile.get(0), inPartnerFile.get(0).contains("partnerTipsFirst()"));
        List<String> inSelfFile = RouteCatalog.familyProblems("SelfTipRoutes", java.util.Arrays.asList(self, partner), FirstTip.SELF);
        assertEquals(inSelfFile.toString(), 1, inSelfFile.size());
        assertTrue(inSelfFile.get(0), inSelfFile.get(0).contains("waits_for_partner"));
    }

    @Test
    public void theFamilyFilesHandOutAFreshListEveryTime() {
        // a route list that somebody changes must not change the catalog
        List<AutoRoute> a = SelfTipRoutes.routes();
        a.add(AutoRoute.builder("stray").start(58, 9.2, -90).waitSeconds(1).build());
        assertEquals(a.size() - 1, SelfTipRoutes.routes().size());
        List<AutoRoute> b = PartnerTipRoutes.routes();
        b.add(AutoRoute.builder("stray2").start(58, 9.2, -90).waitSeconds(1).build());
        assertEquals(b.size() - 1, PartnerTipRoutes.routes().size());
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
            // the tiles have a 23.5 in pitch and their own seams (TileGrid), not 24 in
            assertEquals("start tile of " + r.id(), TileGrid.nameAt(s.x, s.y), r.startTile());
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
        // the park is deep, and the shot is aimed and inside the 30 to 38 in band: with the CAD aim point (54.156 in from the wall)
        // the real ball exit (8.03 in behind the center of rotation, at y = 9.2 + 8.03) is 36.9 in from it. (With the old 144 in field
        // the aim point was 1.4 in farther out and this was a WARN at 38.3 in.)
        assertEquals(0, count(l, Rule.PARK_ZONE, Severity.WARN));
        assertEquals(0, count(l, Rule.SHOOT_POSE, Severity.WARN));
        // and the old start y = 9.2 puts the front edge 0.57 in through the audience wall (the start, and the first leg that
        // turns toward the HIVE a couple of degrees: 0.9 in)
        assertEquals(1, count(l, Rule.START_G304, Severity.ERROR));
        assertEquals(1, count(l, Rule.FIELD_BOUNDS, Severity.ERROR));
        // and the old OpMode feeds for a fixed 5 s: the HIVE tips on the 3rd of its 4 POLLEN and the 4th goes into a cell that is
        // already tipping down (a WARN: QA12 allows it, G417 does not like it)
        assertEquals(1, count(l, Rule.SHOOT_TARGET, Severity.WARN));
        assertEquals(4, l.size());
        assertFalse(RouteValidator.isRunnable(l));
    }

    @Test
    public void theFaithfulConversionsSayThatTheRealRobotMustBePlacedWithItsFrontOnTheWall() {
        // each old OpMode start (y 9.2, 9.2 and x 9.21) is written for a robot centered 9.2 in from its wall; the notes of the
        // START step say so, and say where the real robot goes
        for (String id : new String[] {"red_b1", "red_c1", "red_leave_park"}) {
            String note = RouteCatalog.byId(id).step(0).note();
            assertTrue(id + ": " + note, note.contains("FRONT ON the wall"));
            assertTrue(id + ": " + note, note.contains("9.77"));
        }
    }

    @Test
    public void everyStartOfTheDemonstrationRoutesHasItsFrontEdgeOnTheWall() {
        // the new demonstration routes are placed the right way: the center of rotation is the front extent from the audience wall,
        // so the footprint touches the wall (and no more)
        for (String id : new String[] {"wall_shoot_park", "garden_cycle"}) {
            Pose2 start = RouteCatalog.byId(id).startPose();
            assertEquals(id, BioBuzzConstants.Robot.FRONT_EXTENT_IN, start.y, 1e-9);
            assertEquals(id, 0.0, Footprint.of(start).bounds().minY, 1e-9);
        }
    }

    @Test
    public void redB1TurnsThroughTheWallAndShootsFromTooFar() {
        List<Issue> l = issues("red_b1", Alliance.RED);
        // the -115 deg hold swings a corner 3.4 in through the audience wall (9.2 in out, and it needs 12.6), and so does turning back for path1
        assertEquals(2, count(l, Rule.FIELD_BOUNDS, Severity.ERROR));
        assertEquals(1, count(l, Rule.SHOOT_POSE, Severity.WARN));
        // the park used to be a shallow WARN (2.9 in into a zone that started at y 96); the CAD tape starts the zone at y 94.58, so the
        // same pose is now 4.3 in deep, more than the 3 in the field tolerance asks for
        assertEquals(0, count(l, Rule.PARK_ZONE, Severity.WARN));
        // The old code drove up x = 35..35.8 for a 16.77 in wide robot. The real robot is 17.80 in wide, so its
        // right edge is at 35.8 + 8.9 = 44.7 in, 1.76 in inside the keep-out (the HIVE frame at x = 45.94 minus the
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
    public void redLeaveParkClipsTheRedWallByItsOldStartNumberButStillClaimsLeaveAndPark() {
        // the OpMode starts at x = 9.2112 facing the red wall (180 deg); the real front edge is 9.77 in from the center of
        // rotation, so the robot would reach 0.56 in through the wall at the start and along the first leg. Nothing else is wrong.
        List<Issue> l = issues("red_leave_park", Alliance.RED);
        assertEquals("findings: " + l, 2, l.size());
        assertEquals(Rule.START_G304, l.get(0).rule);
        assertEquals(0, l.get(0).stepIndex);
        assertTrue(l.get(0).message, l.get(0).message.contains("sticks out of the field by 0.6 in"));
        assertEquals(Rule.FIELD_BOUNDS, l.get(1).rule);
        assertEquals(1, l.get(1).stepIndex);
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.PARK_POINTS,
                RouteCatalog.byId("red_leave_park").claimablePoints());
        // the same route placed with the front ON the wall (x = 9.77) is clean
        AutoRoute placed = AutoRoute.builder("placed").start(BioBuzzConstants.Robot.FRONT_EXTENT_IN, 70.2702, 180)
                .driveLinear(24.8608, 83.6152, 135).driveLinear(16.0, 91.5, 135).park(16.0, 91.5, 135).build();
        assertTrue(RouteValidator.check(placed).toString(), RouteValidator.check(placed).isEmpty());
    }

    // ---- the two demonstration routes --------------------------------------------------------------------

    /**
     * The demonstration routes have no ERROR for either alliance. Their ONE finding is the WARN that says the 4th pre-load, the spare of the first
     * volley, is probably launched into the cell while it swings down (the camera stops the feeder only about 0.85 s after the tipping POLLEN
     * lands, see Auto.TIP_STOP_LATENCY_SEC, a number to be measured on day one). With a feeder that stops in time they are clean.
     */
    @Test
    public void theDemonstrationRoutesHaveNoErrorAndOnlyTheSpareWarningForBothAlliances() {
        for (String id : new String[] {"wall_shoot_park", "garden_cycle"}) {
            for (Alliance a : Alliance.values()) {
                List<Issue> l = issues(id, a);
                assertEquals(id + " " + a + ": " + l, 1, l.size());
                assertEquals(l.toString(), Rule.SHOOT_TARGET, l.get(0).rule);
                assertEquals(l.toString(), Severity.WARN, l.get(0).severity);
                assertEquals("on the volley", 1, l.get(0).stepIndex);
                assertTrue(l.get(0).message, l.get(0).message.contains("the camera cannot stop the feeder at once"));
                assertTrue(l.get(0).message, l.get(0).message.contains("Auto.TIP_STOP_LATENCY_SEC"));
                // the same route for a robot whose feeder stops in time is clean
                assertTrue(id + " " + a, FastFeederRobot.check(RouteCatalog.byId(id).forAlliance(a)).isEmpty());
            }
        }
    }

    @Test
    public void wallShootParkTipsTheHiveWithThePreloadsThenParksInTheCornerOfTheZone() {
        AutoRoute r = RouteCatalog.byId("wall_shoot_park");
        assertEquals("C1", r.startTile());
        RouteAnalysis a = r.analyze();
        assertEquals("the volley tips on the 3rd POLLEN and the 4th is launched before the feeder can stop", 4, a.pollenShot());
        assertEquals("3 staged NECTAR + the 3rd of the 4 pre-loads: one volley is the first tip", 1, a.tips());
        assertEquals("the 4th pre-load is the planned spare", 1, a.minimumTipSpare());
        assertEquals("a feeder that stops in time would keep it", 3, FastFeederRobot.analyze(r).pollenShot());
        assertTrue(a.park());
        assertTrue(a.leave());
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.PARK_POINTS + BioBuzzConstants.Auto.TIP_POINTS,
                r.claimablePoints());
        assertTrue(a.parkOverlapIn() > 7.0);
        // it slides along the wall first, STAYING LOW so that the rear edge is out of the spill zone of the tipped cell (G409), then goes up the left corridor
        Step.Drive slide = (Step.Drive) r.step(2);
        assertEquals("a straight slide at the shooting pose's height", 0, slide.via().size());
        assertEquals(14.8, slide.target().y, 1e-9);
        double rearEdgeOutIn = BioBuzzConstants.Field.CENTER_IN - (slide.target().y + BioBuzzConstants.Robot.REAR_EXTENT_IN);
        assertTrue("the rear edge stays beyond the spill reach while the robot is beside the cell: " + rearEdgeOutIn,
                rearEdgeOutIn > BioBuzzConstants.Auto.TIP_SPILL_REACH_IN);
        for (RouteAnalysis.StepInfo s : a.steps()) {
            double keepOutMinX = a.field().hiveKeepOut().minX;
            assertTrue("never right of the HIVE keep-out (x = " + keepOutMinX + ") in the corridor part",
                    s.step.kind() != StepKind.DRIVE || s.to.x < keepOutMinX);
        }
    }

    @Test
    public void gardenCycleTipsWithThePreloadsThenCarriesTheGardenPollenAndStepsOffTheWall() {
        AutoRoute r = RouteCatalog.byId("garden_cycle");
        assertEquals("B1", r.startTile());
        RouteAnalysis a = r.analyze();
        // one volley only: after the tip the UP cell is on the far side of the HIVE and holds no NECTAR, so a second
        // volley of 4 there would not tip it (it needs 8). The first volley tips on its 3rd POLLEN and the 4th is probably launched before the
        // feeder can stop, so the robot holds 0 and the 3 GARDEN POLLEN are what it carries (3 with the spare spent, 4 if it was kept).
        assertEquals(4, a.pollenShot());
        assertEquals(3, a.pollenGained());
        assertEquals(1, a.tips());
        assertEquals(Side.REAR, a.step(1).upSideAfter);
        assertEquals(BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.TIP_POINTS, r.claimablePoints());
        assertEquals("the GARDEN POLLEN are carried into TELEOP (the spare was probably spent)", BioBuzzConstants.Auto.GARDEN_POLLEN - 1, a.pollenEnd());
        assertEquals("with a feeder that stops in time the robot carries the most it may hold (G407)", BioBuzzConstants.Auto.POLLEN_CAPACITY,
                FastFeederRobot.analyze(r).pollenEnd());
        assertEquals("own GARDEN", a.step(4).sourceName);
        assertTrue(a.leave());
        assertTrue(a.totalSeconds() < BioBuzzConstants.Auto.AUTO_DURATION_SEC - BioBuzzConstants.Auto.TIME_SAFETY_MARGIN_SEC);
    }

    @Test
    public void gardenCycleTouchesTheWallsButNeverClipsThemAndTheSameSweepADeeperIsStillAnError() {
        AutoRoute r = RouteCatalog.byId("garden_cycle");
        Step.Intake garden = (Step.Intake) r.step(4);
        Rect b = Footprint.of(garden.robotPose()).bounds();
        double tol = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
        assertTrue("the left edge is on or inside the red wall: " + b, b.minX >= -tol);
        assertTrue("the front edge is on or inside the audience wall: " + b, b.minY >= -tol);
        assertTrue("but it really is at the wall, on the row of GARDEN POLLEN: " + b, b.minX < 1.5 && b.minY < 1.5);
        // wallContact() is only a note: the same sweep with the robot 0.5 in closer to the audience wall pushes the front edge
        // 0.2 in into it, and that is an ERROR on the drive...
        AutoRoute deeper = AutoRoute.builder("deeper").start(35.0, 9.2, -90).shootUp(4)
                .driveConstant(9.0, 18.0, -90).driveConstant(9.0, 8.7, -90).wallContact()
                .intakeGarden(5.9, 1.4, 9.0, 8.7, -90, 4).build();
        assertTrue(wallErrorAt(RouteValidator.check(deeper), 3));
        // ...and on an INTAKE that makes the move itself (here the drive stops short and the INTAKE steps into the wall)
        AutoRoute intakeDeeper = AutoRoute.builder("intake_deeper").start(35.0, 9.2, -90).shootUp(4)
                .driveConstant(9.0, 18.0, -90).intakeGarden(5.9, 1.4, 9.0, 8.7, -90, 4).build();
        assertTrue(wallErrorAt(RouteValidator.check(intakeDeeper), 3));
        // the real one is clean
        assertFalse(wallErrorAt(RouteValidator.check(r), 3) || wallErrorAt(RouteValidator.check(r), 4));
    }

    // ---- the two first-tip families -------------------------------------------------------------------

    /**
     * The catalog must hold at least one route of EACH family. The tests that loop over "every partner route of the catalog" pass on an empty list, so without
     * this one an empty partner family (it was: "No route is in this file yet") leaves the whole suite green while the third requirement ("both first-tip
     * families must exist") is unmet.
     */
    @Test
    public void bothFirstTipFamiliesHaveARouteInTheCatalog() {
        int self = 0;
        int partner = 0;
        for (AutoRoute r : RouteCatalog.all()) {
            if (r.firstTip() == FirstTip.PARTNER) {
                partner++;
            } else {
                self++;
            }
        }
        assertTrue("the SELF family has routes: " + self, self >= 1);
        assertTrue("the PARTNER family must not be empty: " + partner, partner >= 1);
        assertFalse("PartnerTipRoutes holds the partner routes", PartnerTipRoutes.routes().isEmpty());
    }

    /**
     * Situation: the partner route of the catalog, for both alliances.
     * Expected: it validates with no finding at all; it waits for the tip BEFORE it shoots; it earns LEAVE and PARK (8 points, the partner's 20 are not
     *     counted); our own volley of 4 into the empty cell that comes up does not tip it (it needs 8); the partner's assumed tip is in the plan; and the route
     *     has at least the minimum slack for a slower partner.
     */
    @Test
    public void thePartnerRouteWaitsForTheCameraThenShootsParksAndHasRoomToSpare() {
        AutoRoute r = RouteCatalog.byId(PartnerTipRoutes.PARTNER_REAR_SHOOT_PARK);
        assertEquals(FirstTip.PARTNER, r.firstTip());
        assertEquals(StepKind.WAIT_FOR_TIP, r.step(1).kind());
        assertEquals(StepKind.SHOOT, r.step(2).kind());
        for (Alliance a : Alliance.values()) {
            AutoRoute x = r.forAlliance(a);
            List<Issue> l = RouteValidator.check(x);
            assertTrue(a + ": " + l, l.isEmpty());
            RouteAnalysis an = x.analyze();
            assertTrue(an.partnerTipAssumed());
            assertTrue(an.leave());
            assertTrue(an.park());
            assertEquals("LEAVE 3 + PARK 5; the partner's tip is not ours", BioBuzzConstants.Auto.LEAVE_POINTS + BioBuzzConstants.Auto.PARK_POINTS, x.claimablePoints());
            assertEquals("4 POLLEN into the empty cell do not tip it", 0, an.tips());
            assertEquals(4, an.pollenShot());
            assertTrue("slack " + an.partnerTipSlackSec(), an.partnerTipSlackSec() >= BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC);
            assertTrue(an.totalSeconds() <= RouteAnalysis.timeBudgetSec());
        }
    }

    /**
     * Situation: the route the partner family was asked for: the partner tips (assumed at the shipped 12 s), we wait, shoot 4 into the empty cell that comes
     *     up, take 4 from the rear FLOWER and shoot 4 again for the SECOND tip.
     * Expected: with the shipped numbers it does not fit: the plan ends past the 30 s AUTO limit and the validator says so (G401), and even the sum of the
     *     waits and the work alone, with NO driving, is over 30 s. It fits only for a much earlier partner tip.
     * Why: this pins the numbers behind the decision NOT to put that route in the catalog yet (see PartnerTipRoutes and docs/AUTOS.md): 12 s is the team's own
     *     guess and is about twice what our own first volley takes. The test says what has to change (a partner that tips by about 6 s, and wheels that reach)
     *     instead of leaving the team to find out from a red route.
     */
    @Test
    public void theSecondTipOfThePartnerFamilyDoesNotFitWithTheShippedNumbers() {
        double[] rearFlower = BioBuzzConstants.Field.FLOWER_CENTERS_IN[1];
        AutoRoute r = AutoRoute.builder("second_tip").partnerTipsFirst().start(61.5, SIZE - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90)
                .waitForTip(20).shootUp(61.5, 130.0, 90, 4).intakeFlower(rearFlower[0], rearFlower[1], 4).shootUp(61.5, 130.0, 90, 4).build();
        RouteAnalysis a = r.analyze();
        assertTrue("ends at " + a.totalSeconds(), a.totalSeconds() > BioBuzzConstants.Auto.AUTO_DURATION_SEC);
        assertEquals(1, count(RouteValidator.check(r), Rule.TIME_G401, Severity.ERROR));
        double withoutDriving = BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC + BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC
                + 2.0 * (BioBuzzConstants.Auto.AIM_SPINUP_SEC + 4.0 * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC) + a.step(3).pickupSec;
        assertTrue("the waits and the work alone: " + withoutDriving, withoutDriving > BioBuzzConstants.Auto.AUTO_DURATION_SEC);
        // it works for a partner that tips early: the latest tip it can use is far below the assumed one
        assertTrue("latest partner tip " + a.latestPartnerTipSec(), a.latestPartnerTipSec() < BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC / 1.5);
        assertTrue(RouteAnalysis.ofWithPartnerTipAt(r, 4.0).totalSeconds() <= RouteAnalysis.timeBudgetSec());
    }

    private static boolean wallErrorAt(List<Issue> issues, int stepIndex) {
        for (Issue i : issues) {
            if (i.rule == Rule.FIELD_BOUNDS && i.severity == Severity.ERROR && i.stepIndex == stepIndex) {
                return true;
            }
        }
        return false;
    }
}
