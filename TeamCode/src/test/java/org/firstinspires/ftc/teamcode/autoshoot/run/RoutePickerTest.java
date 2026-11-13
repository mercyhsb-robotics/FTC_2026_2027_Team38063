package org.firstinspires.ftc.teamcode.autoshoot.run;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.ConstantsCheck;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;

/**
 * The INIT screen of the Auto Route Picker, without a robot: the four buttons choose a route and an alliance, the screen says what is known about
 * the choice, and a choice that cannot be run is REFUSED with a reason (a route with an ERROR; a route that waits for the HIVE to tip while the
 * Limelight is not working).
 */
public class RoutePickerTest {
    private static final VisionFrame.Status OK = VisionFrame.Status.OK;

    private static RoutePicker picker() {
        return new RoutePicker(RouteCatalog.all(), Alliance.RED);
    }

    private static int indexOf(String id) {
        List<AutoRoute> all = RouteCatalog.all();
        for (int i = 0; i < all.size(); i++) {
            if (all.get(i).id().equals(id)) {
                return i;
            }
        }
        throw new AssertionError(id);
    }

    /** Presses a button once: down, then up. */
    private static void press(RoutePicker p, boolean previous, boolean next, boolean red, boolean blue) {
        p.buttons(previous, next, red, blue);
        p.buttons(false, false, false, false);
    }

    private static String joined(List<String> lines) {
        StringBuilder sb = new StringBuilder();
        for (String l : lines) {
            sb.append(l).append('\n');
        }
        return sb.toString();
    }

    // ---- choosing -----------------------------------------------------------------------------------------------------------

    @Test
    public void theButtonsGoThroughTheListOfRoutesAndWrapRoundAndChooseTheAlliance() {
        RoutePicker p = picker();
        assertEquals(0, p.routeIndex());
        assertEquals(Alliance.RED, p.alliance());
        press(p, false, true, false, false);
        assertEquals(1, p.routeIndex());
        press(p, true, false, false, false);
        assertEquals(0, p.routeIndex());
        press(p, true, false, false, false);
        assertEquals("up from the first route wraps to the last", p.routeCount() - 1, p.routeIndex());
        press(p, false, true, false, false);
        assertEquals("down from the last wraps to the first", 0, p.routeIndex());
        press(p, false, false, false, true);
        assertEquals(Alliance.BLUE, p.alliance());
        press(p, false, false, true, false);
        assertEquals(Alliance.RED, p.alliance());
    }

    @Test
    public void aButtonThatIsHeldCountsOnceNotOnEveryLoop() {
        RoutePicker p = picker();
        for (int loop = 0; loop < 20; loop++) {
            p.buttons(false, true, false, false);
        }
        assertEquals(1, p.routeIndex());
        p.buttons(false, false, false, false);
        p.buttons(false, true, false, false);
        assertEquals(2, p.routeIndex());
    }

    @Test
    public void theBlueChoiceIsTheHalfTurnOfTheRedRoute() {
        RoutePicker p = picker();
        press(p, false, false, false, true);
        AutoRoute blue = p.choice().route;
        assertEquals(Alliance.BLUE, blue.alliance());
        assertEquals(RouteCatalog.all().get(0).forAlliance(Alliance.BLUE).startPose().x, blue.startPose().x, 1e-9);
    }

    @Test
    public void theAnalysisIsWorkedOutOnceAndKept() {
        RoutePicker p = picker();
        assertTrue("the same object every time", p.choice() == p.choice());
        RoutePicker.Choice red = p.choice();
        press(p, false, false, false, true);
        press(p, false, false, true, false);
        assertTrue("back to RED: the kept choice, not a new one", red == p.choice());
    }

    // ---- what the screen says -------------------------------------------------------------------------------------------------

    @Test
    public void theScreenShowsTheRouteTheAssumptionTheTimeThePointsThePlacementTheFindingsAndWhatIsStillToMeasure() {
        RoutePicker p = picker();
        while (!p.choice().route.id().equals(RouteCatalog.WALL_SHOOT_PARK)) {
            press(p, false, true, false, false);
        }
        List<String> placeholders = new ConstantsCheck.Report().placeholders;
        placeholders.addAll(Arrays.asList(BioBuzzConstants.PLACEHOLDERS));
        String screen = joined(p.screen(OK, placeholders));
        RoutePicker.Choice c = p.choice();
        assertTrue(screen, screen.contains(BioBuzzConstants.Run.MENU_NEXT_ROUTE_BUTTON.name()));
        assertTrue(screen, screen.contains("ALLIANCE: RED"));
        assertTrue(screen, screen.contains(c.route.title()));
        assertTrue(screen, screen.contains("FIRST TIP: " + c.route.firstTipSummary()));
        assertTrue(screen, screen.contains(String.format(java.util.Locale.ROOT, "%.1f", c.analysis.totalSeconds())));
        assertTrue(screen, screen.contains("OUR POINTS " + c.analysis.points()));
        assertTrue(screen, screen.contains("PLACE THE ROBOT: " + c.placement));
        assertTrue(screen, c.placement.toLowerCase().contains("front") && c.placement.contains("wall"));
        assertTrue(screen, screen.contains("ROUTE CHECK: " + c.errors + " ERROR, " + c.warnings + " WARN"));
        assertTrue(screen, screen.contains("STILL TO MEASURE (" + placeholders.size() + ")"));
        assertTrue("only the first few names are spelled out", screen.contains("more"));
        assertTrue(screen, screen.contains("READY"));
        // a WARN of the checker is listed with its step
        assertTrue(screen, c.warnings == 0 || screen.contains("WARN step"));
    }

    @Test
    public void aPartnerRouteSaysWhenTheLatestUsefulPartnerTipIs() {
        RoutePicker p = picker();
        while (!p.choice().route.id().equals("partner_rear_shoot_park")) {
            press(p, false, true, false, false);
        }
        String screen = joined(p.screen(OK, new ArrayList<String>()));
        assertTrue(screen, screen.contains("LATEST PARTNER TIP"));
        assertTrue(screen, screen.contains("partner tips first"));
    }

    @Test
    public void theLongListOfFindingsIsCutAndTheRestIsCounted() {
        RoutePicker p = picker();
        while (!p.choice().route.id().equals(RouteCatalog.RED_B1)) {
            press(p, false, true, false, false);
        }
        RoutePicker.Choice c = p.choice();
        List<String> lines = p.screen(OK, new ArrayList<String>());
        int listed = 0;
        for (String l : lines) {
            if (l.startsWith("  ERROR") || l.startsWith("  WARN")) {
                listed++;
            }
        }
        assertEquals(Math.min(c.issues.size(), BioBuzzConstants.Run.MENU_MAX_ISSUE_LINES), listed);
        // ERRORs come before WARNs
        boolean seenWarn = false;
        for (String l : lines) {
            if (l.startsWith("  WARN")) {
                seenWarn = true;
            }
            assertFalse("an ERROR after a WARN: " + l, seenWarn && l.startsWith("  ERROR"));
        }
    }

    // ---- refusing -------------------------------------------------------------------------------------------------------------------

    @Test
    public void aRouteWithAnErrorRefusesToStartAndSaysWhy() {
        RoutePicker p = picker();
        p.buttons(false, false, false, false);
        int b1 = indexOf(RouteCatalog.RED_B1);
        for (int i = 0; i < b1; i++) {
            press(p, false, true, false, false);
        }
        assertEquals(RouteCatalog.RED_B1, p.choice().route.id());
        assertTrue("the route of the old OpMode has ERRORs", p.choice().errors > 0);
        RoutePicker.Readiness r = p.readiness(OK);
        assertFalse(r.canStart());
        assertNotNull(r.refusal);
        assertTrue(r.refusal, r.refusal.contains("ERROR") && r.refusal.contains("step"));
        String screen = joined(p.screen(OK, new ArrayList<String>()));
        assertTrue(screen, screen.contains("REFUSED"));
        // the same for blue
        press(p, false, false, false, true);
        assertFalse(p.readiness(OK).canStart());
    }

    @Test
    public void everyRouteWithoutAnErrorMayStartWithAWorkingCamera() {
        RoutePicker p = picker();
        for (int i = 0; i < p.routeCount(); i++) {
            for (Alliance a : Alliance.values()) {
                press(p, false, false, a == Alliance.RED, a == Alliance.BLUE);
                RoutePicker.Readiness r = p.readiness(OK);
                assertEquals(p.choice().route.id() + "/" + a + ": " + r.refusal, p.choice().errors == 0, r.canStart());
            }
            press(p, false, true, false, false);
        }
    }

    @Test
    public void aRouteThatWaitsForTheTipIsRefusedWhenTheLimelightIsNotWorkingWhateverTheStatus() {
        RoutePicker p = picker();
        while (!p.choice().route.id().equals("partner_rear_shoot_park")) {
            press(p, false, true, false, false);
        }
        assertEquals("the partner route has no ERROR in the catalog", 0, p.choice().errors);
        assertTrue(p.choice().waitsForTip);
        assertTrue(p.readiness(OK).canStart());
        for (VisionFrame.Status status : VisionFrame.Status.values()) {
            if (status == OK) {
                continue;
            }
            RoutePicker.Readiness r = p.readiness(status);
            assertFalse(status + " must refuse", r.canStart());
            assertTrue(status + ": " + r.refusal, r.refusal.toLowerCase().contains("limelight") || r.refusal.toLowerCase().contains("camera"));
            assertTrue("the screen says so too", joined(p.screen(status, new ArrayList<String>())).contains("REFUSED"));
        }
        // not connected: the plain words say the cable and the name in the configuration
        assertTrue(p.readiness(VisionFrame.Status.NOT_CONNECTED).refusal.contains("cable"));
        // the same for blue
        press(p, false, false, false, true);
        assertFalse(p.readiness(VisionFrame.Status.NOT_CONNECTED).canStart());
        assertTrue(p.readiness(OK).canStart());
    }

    @Test
    public void aRouteThatNeedsNoCameraIsNotRefusedWithoutIt() {
        RoutePicker p = picker();
        while (!p.choice().route.id().equals(RouteCatalog.WALL_SHOOT_PARK)) {
            press(p, false, true, false, false);
        }
        assertFalse(p.choice().waitsForTip);
        for (VisionFrame.Status status : VisionFrame.Status.values()) {
            assertTrue(status + ": " + p.readiness(status).refusal, p.readiness(status).canStart());
        }
    }

    @Test
    public void aRouteThatHasAnErrorAndNeedsTheCameraGivesBothReasons() {
        // the start is 5 in from the wall, so the front edge (9.77 in) would be through it: an ERROR; and the route waits for the tip
        RoutePicker p = new RoutePicker(Arrays.asList(AutoRoute.builder("bad_partner").partnerTipsFirst()
                .start(35, 5.0, -90).waitForTip(20.0).shootUp(4).build()), Alliance.RED);
        assertTrue(p.choice().errors > 0);
        RoutePicker.Readiness r = p.readiness(VisionFrame.Status.NOT_CONNECTED);
        assertFalse(r.canStart());
        assertTrue(r.refusal, r.refusal.contains("ERROR") && r.refusal.contains("ALSO") && r.refusal.toLowerCase().contains("limelight"));
        // with a working camera only the ERROR is left
        RoutePicker.Readiness withCamera = p.readiness(OK);
        assertFalse(withCamera.canStart());
        assertFalse(withCamera.refusal, withCamera.refusal.contains("ALSO"));
    }
}
