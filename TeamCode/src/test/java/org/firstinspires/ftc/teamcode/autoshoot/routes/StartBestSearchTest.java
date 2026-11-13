package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.junit.Test;

/**
 * The search for the best start ({@link StartPoseSearch#bestStart}) and the shortcuts it is asked about ({@link StartShortcuts}).
 *
 * <p>How it is checked: the search runs on a coarse grid (a place every inch, a heading every 5 degrees: about 9 thousand starts instead of 37 thousand, the
 * same code) so the tests are quick; {@code tools/start-shortcuts.sh} runs the full grid. The answer is checked against the rules (legal, valid, keeps its
 * margin), against the standard starts, and against a brute force that looks at the starts the search skipped.
 */
public class StartBestSearchTest {
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double AXIS = BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN;
    private static final double M_IN = BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN;
    private static final double M_DEG = BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_DEG;
    private static final FieldModel MODEL = new FieldModel();
    private static final AllianceField RED = new AllianceField(MODEL, Alliance.RED);

    private static StartShortcuts.Shortcut family(String id) {
        for (StartShortcuts.Shortcut s : StartShortcuts.all()) {
            if (s.id.equals(id)) {
                return s;
            }
        }
        throw new AssertionError("no family " + id);
    }

    /**
     * The grid the tests search on: a place every inch and a heading every 5 degrees (about 9 thousand starts instead of 37 thousand); the partner family on
     * half-inch steps, because the places that keep the full margin there are only about 2 in wide and an inch grid can miss them.
     */
    private static StartPoseSearch.Settings coarse(StartShortcuts.Shortcut s) {
        return StartPoseSearch.Settings.standard().withTiming(s.timing).withResolution(s.id.startsWith("partner") ? 0.5 : 1.0, 5.0);
    }

    /** The searches already made: a search takes a few seconds and several tests ask the same question. */
    private static final java.util.Map<String, StartPoseSearch.Best> MADE = new java.util.HashMap<String, StartPoseSearch.Best>();

    private static StartPoseSearch.Best best(StartShortcuts.Shortcut s, boolean withMargin) {
        String key = s.id + (withMargin ? "/margin" : "/none");
        if (!MADE.containsKey(key)) {
            StartPoseSearch.Settings st = withMargin ? coarse(s) : coarse(s).withMargin(0.0, 0.0);
            MADE.put(key, StartPoseSearch.bestStart(s.template, s.standardStarts, st));
        }
        return MADE.get(key);
    }

    // ---- the answer is a start the rules accept ---------------------------------------------------------------------

    /**
     * Situation: the best start for four families (shoot and park; shoot and take GARDEN POLLEN; partner tips, shoot and park; leave and park only).
     * Expected: a start is found; it is legal and clear by {@link StartRules}; its route has no validator ERROR; it keeps the placement margin (checked again with
     *     the slow full report); its time is what the analysis says; it is never slower than the fastest valid standard start; and the same route for the
     *     BLUE alliance validates the same way with the same time.
     */
    @Test
    public void theBestStartIsLegalValidAndKeepsItsMarginAndIsNeverSlowerThanTheStandardStart() {
        for (String id : new String[] {"shoot_park", "shoot_garden", "partner_park", "leave_park"}) {
            StartShortcuts.Shortcut s = family(id);
            StartPoseSearch.Best best = best(s, true);
            assertTrue(id + ": " + best.reason, best.found);
            assertNotNull(best.start);
            assertTrue(id + " legal and clear: " + StartRules.check(RED, best.start), StartRules.isLegalAndClear(RED, best.start));
            RouteValidator validator = new RouteValidator(MODEL, s.timing);
            for (Issue i : validator.validate(best.route)) {
                assertFalse(id + " has an ERROR: " + i, i.isError());
            }
            assertTrue(id + " placement: " + best.placement.summary(), best.placement.placementSafe());
            assertTrue(id + " placement, the full report again", StartRobustness.check(s.template, best.start, Alliance.RED, s.timing, M_IN, M_DEG).placementSafe());
            assertEquals(id + " time", RouteAnalysis.of(best.route, MODEL, s.timing).totalSeconds(), best.seconds, 1e-9);
            if (!Double.isNaN(best.standardSeconds)) {
                assertTrue(id + " best " + best.seconds + " standard " + best.standardSeconds, best.seconds <= best.standardSeconds + 1e-9);
                assertTrue(best.secondsSaved() >= -1e-9);
            }
            // the blue alliance gets the same route turned half a turn: the same findings and the same time
            AutoRoute blue = best.route.forAlliance(Alliance.BLUE);
            for (Issue i : new RouteValidator(MODEL, s.timing).validate(blue)) {
                assertFalse(id + " blue has an ERROR: " + i, i.isError());
            }
            assertEquals(id + " blue time", best.seconds, RouteAnalysis.of(blue, MODEL, s.timing).totalSeconds(), 1e-9);
            assertTrue(id + " counts: " + best.legalStarts + " legal, " + best.keepingTheMargin + " keep the margin", best.keepingTheMargin <= best.legalStarts && best.validated >= 1);
        }
    }

    private static final class Timed {
        final Pose2 start;
        final double seconds;

        Timed(Pose2 start, double seconds) {
            this.start = start;
            this.seconds = seconds;
        }
    }

    /**
     * Situation: every legal start of the coarse grid that gives a FASTER route than the best start the search returned (the search looked at them first and
     *     threw each away).
     * Expected: each one really is unusable: its route has a validator ERROR or it does not keep its placement margin (checked with the slow full report, not with the
     *     search's own quick check). The first 40 faster starts are looked at.
     * Why: it proves that nothing better was skipped, and that the quick check the search uses agrees with the full one.
     */
    @Test
    public void everyStartThatIsFasterThanTheBestOneReallyIsUnusable() {
        for (String id : new String[] {"shoot_park", "partner_park", "leave_park"}) {
            StartShortcuts.Shortcut s = family(id);
            StartPoseSearch.Settings st = coarse(s);
            StartPoseSearch.Best best = best(s, true);
            assertTrue(id, best.found);
            List<Timed> faster = new ArrayList<Timed>();
            for (Pose2 p : StartPoseSearch.legalStarts(Alliance.RED, st.stepIn, st.headingStepDeg)) {
                double sec = RouteAnalysis.of(s.template.build(p), MODEL, s.timing).totalSeconds();
                if (sec < best.seconds - BioBuzzConstants.Auto.START_SEARCH_TIE_SEC * 2.0) {
                    faster.add(new Timed(p, sec));
                }
            }
            Collections.sort(faster, new Comparator<Timed>() {
                @Override
                public int compare(Timed a, Timed b) {
                    return Double.compare(a.seconds, b.seconds);
                }
            });
            for (int i = 0; i < Math.min(40, faster.size()); i++) {
                Timed t = faster.get(i);
                StartRobustness.Report r = StartRobustness.check(s.template, t.start, Alliance.RED, s.timing, M_IN, M_DEG);
                assertFalse(id + ": the start " + t.start + " (" + t.seconds + " s) is faster than " + best.start + " (" + best.seconds
                        + " s) and keeps its margin: " + r.summary(), r.placementSafe() && StartRobustness.startsStayLegal(RED, t.start, M_IN, M_DEG));
            }
        }
    }

    /**
     * Situation: the quick yes or no ({@link StartRobustness#passes}) and the full report, for the best start of a family, the old partner start (which fails its
     *     margin) and a start that is not even valid.
     * Expected: they agree every time.
     */
    @Test
    public void theQuickPlacementCheckAgreesWithTheFullReport() {
        StartShortcuts.Shortcut partner = family("partner_park");
        List<Pose2> starts = new ArrayList<Pose2>();
        starts.add(Pose2.ofDegrees(61.5, SIZE - FRONT, 90));                     // legal, fails the margin (the center line)
        starts.add(Pose2.ofDegrees(60.75, SIZE - FRONT, 90));                    // legal, fails the full margin, passes a smaller one
        starts.add(Pose2.ofDegrees(36.25, 128.24, 125));                         // the best start of the family
        starts.add(Pose2.ofDegrees(20.0, 20.0, -90));                            // touches no wall
        starts.add(Pose2.ofDegrees(58.0, FRONT, -90));                           // the audience wall: the route is not meant for it
        for (Pose2 p : starts) {
            for (double scale : new double[] {1.0, 0.5, 0.0}) {
                boolean quick = StartRobustness.passes(partner.template, p, Alliance.RED, partner.timing, M_IN * scale, M_DEG * scale);
                StartRobustness.Report full = StartRobustness.check(partner.template, p, Alliance.RED, partner.timing, M_IN * scale, M_DEG * scale);
                assertEquals(p + " at " + scale + " of the margin: " + full.summary(), full.placementSafe(), quick);
            }
        }
    }

    // ---- the margin changes the answer -----------------------------------------------------------------------------------

    /**
     * Situation: the partner route from the rear wall, searched with the placement margin and with none.
     * Expected: with no margin the search may use any legal start and is at least as fast; the catalog's own partner start (x = 61.5, 0.27 in from the center line)
     *     fails the margin, so the answer with the margin is another start and still saves time; and the answer with no margin keeps a margin of nothing.
     */
    @Test
    public void theMarginCostsSomeTimeButNeverMakesTheAnswerIllegal() {
        StartShortcuts.Shortcut s = family("partner_park");
        StartPoseSearch.Best with = best(s, true);
        StartPoseSearch.Best without = best(s, false);
        assertTrue(with.found && without.found);
        assertTrue("no margin is never slower: " + without.seconds + " vs " + with.seconds, without.seconds <= with.seconds + 1e-9);
        assertFalse("the catalog's partner start does not keep the margin", StartRobustness.passes(s.template, s.standardStarts.get(0), Alliance.RED, s.timing, M_IN, M_DEG));
        assertTrue("so the standard start is not the answer", with.start.distanceTo(s.standardStarts.get(0)) > 1.0);
        assertTrue("and the answer saves time", with.secondsSaved() > 0.0);
        assertTrue(StartRules.isLegalAndClear(RED, without.start));
    }

    /**
     * Situation: a template whose route is so long that it can never fit in the 30 s (a wait of 40 s).
     * Expected: no start is found, with a reason that names the validator's finding; the search ends (it does not run for minutes) after the limit of checks.
     */
    @Test
    public void aRouteThatCanNeverFitHasNoBestStartAndSaysWhy() {
        RouteTemplate tooLong = new RouteTemplate() {
            @Override
            public AutoRoute build(Pose2 start) {
                return AutoRoute.builder("long").start(start.x, start.y, start.headingDegrees()).waitSeconds(40.0).build();
            }
        };
        StartPoseSearch.Best best = StartPoseSearch.bestStart(tooLong, Collections.<Pose2>emptyList(),
                StartPoseSearch.Settings.standard().withResolution(2.0, 30.0).withMaxValidations(25));
        assertFalse(best.found);
        assertNull(best.start);
        assertTrue(best.reason, best.reason.contains("25 fastest starts") && best.reason.contains("longer than"));
        assertEquals(25, best.validated);
        assertTrue(Double.isNaN(best.secondsSaved()));
    }

    /**
     * Situation: a template that is only valid from a start close to one spot: it parks at a pose in the LOADING ZONE by a straight drive, so only starts near
     *     the zone keep clear of the HIVE.
     * Expected: the search finds a start from which the drive is short, and it is faster than a start on the audience wall (which is not even valid).
     */
    @Test
    public void aShortDriveBeatsALongOneAndAnInvalidStandardStartIsLeftOut() {
        StartShortcuts.Shortcut s = family("leave_park");
        StartPoseSearch.Best best = best(s, true);
        assertTrue(best.reason, best.found);
        assertTrue("C1 is not valid for this route (the straight drive crosses the HIVE): the standard start is B1", best.standardStart != null
                && Math.abs(best.standardStart.x - s.standardStarts.get(0).x) < 1e-9);
        assertTrue("it saves seconds against B1: " + best.secondsSaved(), best.secondsSaved() > 1.0);
        assertTrue("and its drive is short: the start is near the LOADING ZONE", best.start.distanceTo(15.0, 108.0) < 30.0);
    }

    // ---- the shortcuts of rule G304 ---------------------------------------------------------------------------------------------

    /**
     * Situation: the two starts the team's notes call the shooting-pose shortcuts, found by {@link StartPoseSearch#nearestSafeStart}: front on the audience wall in line
     *     with the HIVE, and front on the rear wall in line with the rear cell.
     * Expected: the audience start is on the axis (within the 0.25 in grid), 9.77 in from the wall, and keeps the full margin; the ball leaves 36.4 in from the aim
     *     point, in the shooting band, and the volley is fired without a move first. On the rear wall NO start keeps the full margin (the gap between the rear FLOWER
     *     and the center line is 20.42 in and the robot with its margin needs 20.8 in), a start does at 0.7 of the margin, close to the axis and 36.4 in out as well.
     */
    @Test
    public void theTwoShootingPoseShortcutsAreConfirmedAndTheRearOneNeedsASmallerMargin() {
        Pose2 audience = StartPoseSearch.nearestSafeStart(Alliance.RED, Wall.AUDIENCE, -90.0, AXIS, M_IN, M_DEG);
        assertNotNull(audience);
        assertEquals(AXIS, audience.x, 0.25);
        assertEquals(FRONT, audience.y, 1e-9);
        CellSpecHolder cells = new CellSpecHolder();
        assertEquals("the ball exit is 36.4 in from the aim point", 36.36, ShotGeometry.standoff(audience, cells.audience), 0.1);
        assertTrue(StartShortcuts.isShootingPose(audience, Side.AUDIENCE));
        RouteAnalysis a = RouteAnalysis.of(family("shoot_park").template.build(audience), MODEL, new TimingModel());
        assertNull("no move before the volley", a.step(1).approach);
        assertFalse("the volley is fired from the start itself, not from the solver's pose", a.step(1).shootPoseIsNominal);
        assertEquals(0.0, a.step(1).startSec - a.step(0).endSec(), 1e-9);

        assertNull("the full margin does not fit between the rear FLOWER and the center line, near the axis",
                nearInTheSlot(StartPoseSearch.nearestSafeStart(Alliance.RED, Wall.REAR, 90.0, AXIS, M_IN, M_DEG)));
        Pose2 rear = StartPoseSearch.nearestSafeStart(Alliance.RED, Wall.REAR, 90.0, AXIS, 0.7 * M_IN, 0.7 * M_DEG);
        assertNotNull(rear);
        assertTrue("clear of the rear FLOWER plate: the robot's left edge is more than the margin past it: x " + rear.x,
                rear.x - BioBuzzConstants.Robot.LEFT_EXTENT_IN > RED.flowerPlate(1).maxX + 0.7 * M_IN);
        assertEquals(SIZE - FRONT, rear.y, 1e-9);
        assertEquals("the same distance from the aim point, 36.4 in", 36.4, ShotGeometry.standoff(rear, cells.rear), 0.15);
        assertTrue(StartShortcuts.isShootingPose(rear, Side.REAR));
    }

    /** A start in the narrow slot next to the axis (null if the search had to go far from it). */
    private static Pose2 nearInTheSlot(Pose2 p) {
        return p != null && Math.abs(p.x - AXIS) < 6.0 ? p : null;
    }

    /** The two cells of the red HIVE, to measure a shot from. */
    private static final class CellSpecHolder {
        final org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec audience = RED.cell(Side.AUDIENCE);
        final org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec rear = RED.cell(Side.REAR);
    }

    /**
     * Situation: the fastest start for the shoot-and-park family on the coarse grid.
     * Expected: it is better than the in-line start by at least 0.3 s, and it is not straight at the audience wall: the robot is turned (its back toward the cell)
     *     and standing on the wall with a corner. This is the "rotated start" shortcut.
     */
    @Test
    public void aTurnedStartBeatsTheStraightOneForShootAndPark() {
        StartShortcuts.Shortcut s = family("shoot_park");
        StartPoseSearch.Best best = best(s, true);
        assertTrue(best.found);
        assertTrue("saves at least 0.3 s: " + best.secondsSaved(), best.secondsSaved() >= 0.3);
        double offStraight = Math.abs(Math.IEEEremainder(best.start.headingDegrees(), 90.0));
        assertTrue("turned off the straight headings by " + offStraight + " degrees: " + best.start, offStraight > 10.0);
    }

    // ---- the settings and the table ---------------------------------------------------------------------------------------------

    /** Situation: settings changed one thing at a time. Expected: each change gives a new object and leaves the original and the other values alone. */
    @Test
    public void settingsAreChangedByCopying() {
        StartPoseSearch.Settings a = StartPoseSearch.Settings.standard();
        StartPoseSearch.Settings b = a.withResolution(2.0, 10.0).withMargin(0.5, 1.0).withMaxValidations(7);
        assertEquals(BioBuzzConstants.Auto.START_SEARCH_STEP_IN, a.stepIn, 0.0);
        assertEquals(BioBuzzConstants.Auto.START_PLACEMENT_MARGIN_IN, a.marginIn, 0.0);
        assertEquals(BioBuzzConstants.Auto.START_SEARCH_MAX_VALIDATIONS, a.maxValidations);
        assertEquals(2.0, b.stepIn, 0.0);
        assertEquals(10.0, b.headingStepDeg, 0.0);
        assertEquals(0.5, b.marginIn, 0.0);
        assertEquals(1.0, b.marginDeg, 0.0);
        assertEquals(7, b.maxValidations);
        assertEquals(a.timing, b.timing);
    }

    /** Situation: a wall that is not one of the alliance's own. Expected: an IllegalArgumentException that names the wall. */
    @Test
    public void aWallOfTheOtherAllianceIsRefused() {
        try {
            StartPoseSearch.nearestSafeStart(Alliance.RED, Wall.BLUE, 0.0, 100.0, M_IN, M_DEG);
            fail("a red robot cannot start against the blue wall");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("blue wall"));
        }
    }

    /**
     * Situation: the table of shortcuts for two families (the leave-and-park one and the partner one), on the coarse grid, printed.
     * Expected: every family has its standard starts, a best start with and without the margin, and its named starts; the text is plain words with units. (The
     *     full table, on the grid of 0.25 in and 5 degrees, is {@code tools/start-shortcuts.sh}.)
     */
    @Test
    public void theTablePrintsTheStandardStartsTheBestStartsAndTheNamedOnes() {
        StringBuilder all = new StringBuilder();
        for (String id : new String[] {"leave_park", "partner_park"}) {
            StartPoseSearch.Settings st = StartPoseSearch.Settings.standard().withResolution(id.startsWith("partner") ? 0.5 : 1.0, 5.0);
            String text = StartShortcuts.table(st, id);
            assertTrue(text, text.contains("standard, "));
            assertTrue(text, text.contains("best start: "));
            assertTrue(text, text.contains("best start with NO margin: "));
            assertTrue(text, text.contains("named, "));
            assertTrue(text, text.contains(" s"));
            all.append(text).append("\n");
        }
        assertTrue("the rear shortcut says plainly that the full margin does not fit", all.toString().contains("NO start there keeps the margin"));
        assertTrue("and how much of the margin does", all.toString().contains("it works with 70% of it"));
        System.out.println(all);
    }

    /** Situation: a family id nobody has. Expected: a table with the header and no family (so a typo shows an empty table, not a crash). */
    @Test
    public void anUnknownFamilyGivesAnEmptyTable() {
        String text = StartShortcuts.table(StartPoseSearch.Settings.standard().withResolution(5.0, 45.0), "no_such_family");
        assertTrue(text, text.contains("WHERE TO SET THE ROBOT UP"));
        assertFalse(text, text.contains("best start"));
    }
}
