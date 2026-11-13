package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.junit.Test;

/**
 * What the Auto Route Picker says at INIT when the Limelight is not working ({@link TipCameraPolicy}): a route that depends on a HIVE tip
 * is refused (partner family) or warned about (our own later volleys), with the reason in plain words; a route that does not depend on
 * a tip is fine. Every camera status is tried, for both alliances.
 */
public class TipCameraPolicyTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;

    /** The partner family: wait for the partner's tip at the rear wall, fire at the cell that comes up, then leave the wall. */
    private static AutoRoute partnerRoute() {
        return AutoRoute.builder("partner").start(61.5, BioBuzzConstants.Field.SIZE_IN - FRONT, 90).partnerTipsFirst().waitForTip(20)
                .shootUp(61.5, 130.0, 90, 4).driveConstant(61.5, 112.0, 90).build();
    }

    /** Our own tip first, then a second volley at the cell that came up with it. */
    private static AutoRoute selfTwoTipsRoute() {
        return AutoRoute.builder("self").start(58, FRONT, -90).shootUp(58, 30, -90, 4).shootSide(Side.REAR, 58, 30, -90, 2)
                .driveConstant(58, 20, -90).build();
    }

    @Test
    public void aWorkingCameraIsFineForEveryRoute() {
        for (AutoRoute r : new AutoRoute[] {partnerRoute(), selfTwoTipsRoute()}) {
            TipCameraPolicy.Finding f = TipCameraPolicy.atInit(r, VisionFrame.Status.OK);
            assertEquals(r.id(), TipCameraPolicy.Verdict.OK, f.verdict);
            assertTrue(f.message, f.message.contains("never for a time"));
        }
    }

    @Test
    public void aPartnerRouteIsRefusedWithoutAWorkingCameraWhateverIsWrongWithIt() {
        for (VisionFrame.Status status : VisionFrame.Status.values()) {
            if (status == VisionFrame.Status.OK) {
                continue;
            }
            for (Alliance alliance : Alliance.values()) {
                TipCameraPolicy.Finding f = TipCameraPolicy.atInit(partnerRoute().forAlliance(alliance), status);
                assertEquals(status + " " + alliance, TipCameraPolicy.Verdict.REFUSE, f.verdict);
                assertTrue(f.message, f.message.startsWith("do not run this route: it shoots after the alliance partner's tip"));
                assertTrue(f.message, f.message.contains("no camera, no shot"));
                assertTrue(f.message, f.message.contains("leave 1 volley out"));
                assertTrue(f.message, f.message.contains("carry on with the safe steps (LEAVE, PARK)"));
                assertTrue(f.message, f.message.contains("Pick a route that does not depend on the camera, or fix the Limelight first"));
            }
        }
    }

    @Test
    public void theReasonSaysWhatIsWrongWithTheCamera() {
        String[][] expected = {
                {"NOT_CONNECTED", "the Control Hub cannot reach the Limelight (check its cable and power, and the name in the robot configuration)"},
                {"NO_RESULT", "connected but has not produced a result yet"},
                {"STALE", "newest picture is too old to trust"},
                {"STALLED", "no new picture has arrived for a while"},
                {"WRONG_PIPELINE", "running a pipeline other than the AprilTag one"}};
        for (String[] e : expected) {
            TipCameraPolicy.Finding f = TipCameraPolicy.atInit(partnerRoute(), VisionFrame.Status.valueOf(e[0]));
            assertTrue(e[0] + ": " + f.message, f.message.contains(e[1]));
        }
    }

    @Test
    public void aRouteWhoseLaterVolleysDependOnItsOwnTipIsOnlyWarnedAbout() {
        TipCameraPolicy.Finding f = TipCameraPolicy.atInit(selfTwoTipsRoute(), VisionFrame.Status.NOT_CONNECTED);
        assertEquals(TipCameraPolicy.Verdict.WARN, f.verdict);
        assertTrue(f.message, f.message.contains("the Limelight is not working"));
        assertTrue(f.message, f.message.contains("leave 1 volley out"));
        assertTrue(f.message, f.message.contains("its first tip does not need the camera"));
        assertFalse("nothing of the route's own points is lost here, so no loss is claimed: " + f.message, f.message.contains("claim only"));
        // it still earns its first tip: the give-up plan keeps the 20 points
        RouteAnalysis a = selfTwoTipsRoute().analyze();
        assertEquals(BioBuzzConstants.Auto.TIP_POINTS, a.giveUpPlan().tipPoints());
    }

    @Test
    public void theMessageSaysHowManyPointsAreLostWhenSomeAre() {
        // our own second tip (8 POLLEN into the cell that comes up) is worth 20 points that a robot without a camera does not get
        AutoRoute twoTips = AutoRoute.builder("two_tips").start(58, FRONT, -90).shootUp(58, 30, -90, 4)
                .shootSide(Side.REAR, 58, 30, -90, 4).shootSide(Side.REAR, 58, 30, -90, 4).driveConstant(58, 20, -90).build();
        RouteAnalysis a = twoTips.analyze();
        assertEquals("two tips of ours", 2, a.tips());
        TipCameraPolicy.Finding f = TipCameraPolicy.atInit(twoTips, VisionFrame.Status.STALLED);
        assertEquals(TipCameraPolicy.Verdict.WARN, f.verdict);
        assertTrue(f.message, f.message.contains("leave 2 volleys out"));
        assertTrue(f.message, f.message.contains("claim only " + (a.points() - BioBuzzConstants.Auto.TIP_POINTS) + " of the " + a.points() + " points"));
    }

    @Test
    public void aRouteThatDependsOnNoTipNeedsNoCamera() {
        AutoRoute first = AutoRoute.builder("first").start(58, FRONT, -90).shootUp(4).driveConstant(58, 30, -90).build();
        for (VisionFrame.Status status : VisionFrame.Status.values()) {
            TipCameraPolicy.Finding f = TipCameraPolicy.atInit(first, status);
            assertEquals(status.name(), TipCameraPolicy.Verdict.OK, f.verdict);
            assertTrue(f.message, f.message.contains("needs no camera for any tip"));
        }
        // no route of the catalog depends on a tip yet, so the picker has nothing to say about any of them, with or without the camera
        for (AutoRoute r : RouteCatalog.all()) {
            if (!r.analyze().dependsOnTipConfirmation()) {
                assertEquals(r.id(), TipCameraPolicy.Verdict.OK, TipCameraPolicy.atInit(r, VisionFrame.Status.NOT_CONNECTED).verdict);
            } else {
                assertFalse(r.id(), TipCameraPolicy.atInit(r, VisionFrame.Status.NOT_CONNECTED).verdict == TipCameraPolicy.Verdict.OK);
            }
        }
    }
}
