package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.junit.Test;

/**
 * The numbers of the catalog's partner route that its text and its start depend on. They are checked here against what the code computes, because the first
 * version of this route said "1.7 in clear", "about 3 in from the start" and "later than 16 s would not fit", and all three were wrong.
 */
public class PartnerRouteDataTest {
    private static AutoRoute route() {
        return RouteCatalog.byId(PartnerTipRoutes.PARTNER_REAR_SHOOT_PARK);
    }

    private static CellSpec rearCell() {
        return new AllianceField(new FieldModel(), Alliance.RED).cell(Side.REAR);
    }

    /**
     * Situation: the partner route, set down by hand 1 in and 3 degrees off, for both alliances.
     * Expected: every placement is still legal and still valid. The first version started straight on the rear wall at x = 61.5, 0.27 in from the center line,
     *     and 5 of the 8 hand placements put the robot into the other half (G304.A).
     */
    @Test
    public void theStartKeepsThePlacementMarginForBothAlliances() {
        for (Alliance a : Alliance.values()) {
            StartRobustness.Report r = StartRobustness.check(route().forAlliance(a));
            assertTrue(a + ": " + r.summary(), r.placementSafe());
            assertTrue(a + " has no ERROR of its own", r.nominalErrors.isEmpty());
        }
    }

    /**
     * Situation: the wait for the partner's tip.
     * Expected: the give-up time is the latest the rest of the route allows, rounded down to a tenth of a second: not later (the route would overrun the AUTO
     *     limit) and not earlier by more than that (a partner that tips at 16 to 18 s would be turned away although the route still finishes). The first
     *     version gave up at 16.0 s and its note said a later time "would not fit in the 30 s", but the latest was 18.3 s.
     */
    @Test
    public void theGiveUpTimeIsTheLatestTheRouteAllows() {
        AutoRoute r = route();
        RouteAnalysis a = r.analyze();
        RouteAnalysis.StepInfo wait = a.step(1);
        assertEquals(StepKind.WAIT_FOR_TIP, wait.step.kind());
        double giveUp = ((Step.WaitForTip) wait.step).giveUpSec();
        double latest = a.latestGiveUpSec(wait);
        assertTrue("give-up " + giveUp + " must not be later than " + latest, giveUp <= latest + 1e-9);
        assertTrue("give-up " + giveUp + " is more than 0.1 s earlier than the latest " + latest, giveUp > latest - 0.1 - 1e-9);
        assertTrue("and the slack for a slow partner is at least the minimum", a.partnerTipSlackSec() >= BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC);
        assertTrue("the text says the give-up time", r.description().contains(String.format(Locale.ROOT, "%.1f s", giveUp)));
    }

    /**
     * Situation: where the robot shoots from.
     * Expected: the move from the start is about 1 in (the note says so); the launch point is 37.5 in from the aim point, inside the validator's band and within
     *     0.5 in of the measured shot (37.7 in); the start itself is just outside the band (so the move has a reason).
     */
    @Test
    public void theShotIsFiredFromOneInchCloserThanTheStartAndNearTheMeasuredShot() {
        AutoRoute r = route();
        RouteAnalysis.StepInfo shoot = r.analyze().step(2);
        Pose2 start = r.startPose();
        double move = start.distanceTo(shoot.shootPose);
        assertEquals("the note says about 1 in", 1.0, move, 0.1);
        double fromShot = ShotGeometry.standoff(shoot.shootPose, rearCell());
        double fromStart = ShotGeometry.standoff(start, rearCell());
        assertEquals("the note says 37.5 in", 37.5, fromShot, 0.05);
        assertEquals("the measured shot", 37.7, fromShot, 0.5);
        assertTrue("inside the band: " + fromShot, fromShot <= BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN);
        assertEquals("the note says 0.5 in too far", BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN + 0.5, fromStart, 0.05);
    }

    /**
     * Situation: the park note quotes how far the robot overlaps the LOADING ZONE and how far off the wall it stands.
     * Expected: the quoted numbers are the real ones (the first version said 9.5 x 17.8 in, the old centered-robot figure; the real overlap is 7.0 in).
     */
    @Test
    public void theParkNoteQuotesTheRealOverlap() {
        AutoRoute r = route();
        RouteAnalysis a = r.analyze();
        RouteAnalysis.StepInfo park = a.step(r.size() - 1);
        String note = park.step.note();
        assertTrue(note, note.contains(String.format(Locale.ROOT, "%.1f in (the zone is 11 in deep)", a.parkOverlapIn())));
        double wallGap = Footprint.of(((Step.Park) park.step).pose()).bounds().minX;
        assertTrue(note, note.contains(String.format(Locale.ROOT, "standing %.1f in off the wall", wallGap)));
    }
}
