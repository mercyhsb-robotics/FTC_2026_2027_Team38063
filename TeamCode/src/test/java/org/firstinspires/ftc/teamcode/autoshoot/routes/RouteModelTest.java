package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.junit.Test;

/** The route data model: builder behaviour and the red -> blue half turn of every step kind. */
public class RouteModelTest {
    private static final double EPS = 1e-9;
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;

    /** One step of every kind and every heading mode, so the mirror test touches every field. */
    private static AutoRoute everything() {
        return AutoRoute.builder("everything").title("All step kinds").description("d").startTile("B1")
                .start(35.0, 9.2, -90)
                .shootUp(35.0, 9.2, -115, 4).launchSeconds(5.0).note("fixed volley")
                .waitSeconds(1.5)
                .driveReverseTangent(35.8, 90, -90).label("p1").via(30, 40).via(31, 60).speed(0.5).wallContact()
                .driveTangent(20, 95)
                .driveLinear(24.0, 83.0, 135)
                .driveConstant(30, 30, 45)
                .driveFacing(40, 20, 57.924, 54.156)
                .driveBackFacing(41, 21, 57.924, 54.156)
                .intakeFlower(2.634, 47.281, 13.5, 47.281, 180, 4)
                .intakeGarden(5.9, 1.4, 9.0, 9.7, -90, 4)
                .intakeFloor(40, 30, 38, 30, 0, 1)
                .shootSide(Side.AUDIENCE, 2)
                .waitForTip(8.0).note("for the partner")
                .shootUp(1)
                .park(12, 108, 0).note("end")
                .partnerTipsFirst()
                .build();
    }

    @Test
    public void builderRecordsStepsInOrderWithTheirModifiers() {
        AutoRoute r = everything();
        assertEquals("everything", r.id());
        assertEquals("All step kinds", r.title());
        assertEquals("B1", r.startTile());
        assertEquals(Alliance.RED, r.alliance());
        assertEquals(StepKind.START, r.step(0).kind());
        assertEquals(StepKind.SHOOT, r.step(1).kind());
        assertEquals(StepKind.WAIT, r.step(2).kind());

        Step.Shoot shoot = (Step.Shoot) r.step(1);
        assertTrue(shoot.hasFixedSeconds());
        assertEquals(5.0, shoot.fixedSeconds(), EPS);
        assertEquals("fixed volley", shoot.note());
        assertEquals(4, shoot.count());
        assertTrue(shoot.target().isUpCell());
        assertFalse(shoot.isAutoPose());

        Step.Drive drive = (Step.Drive) r.step(3);
        assertEquals("p1", drive.label());
        assertEquals(2, drive.via().size());
        assertEquals(0.5, drive.speedFraction(), EPS);
        assertTrue(drive.isWallContact());
        assertEquals(HeadingMode.REVERSE_TANGENT, drive.mode());
        // The (ignored) heading the old code wrote is kept as data.
        assertEquals(-90.0, drive.target().headingDegrees(), 1e-9);

        Step.Shoot auto = (Step.Shoot) r.step(r.size() - 2);
        assertTrue("shootUp(count) leaves the pose to the solver", auto.isAutoPose());
        assertFalse(auto.hasFixedSeconds());
        assertEquals("end", r.step(r.size() - 1).note());
    }

    @Test
    public void intakeDwellDefaultsComeFromTheConstants() {
        AutoRoute r = AutoRoute.builder("dwell").start(35, 9.2, -90)
                .intakeFlower(2.4, 48, 13.5, 48, 180, 4)
                .intakeGarden(5.9, 1.4, 9, 9.7, -90, 4)
                .intakeFloor(40, 30, 38, 30, 0, 1)
                .build();
        assertEquals(BioBuzzConstants.Auto.INTAKE_DWELL_FLOWER_SEC, ((Step.Intake) r.step(1)).dwellSec(), EPS);
        assertEquals(BioBuzzConstants.Auto.INTAKE_DWELL_GARDEN_SEC, ((Step.Intake) r.step(2)).dwellSec(), EPS);
        assertEquals(BioBuzzConstants.Auto.INTAKE_DWELL_FLOOR_SEC, ((Step.Intake) r.step(3)).dwellSec(), EPS);
        assertTrue(((Step.Intake) r.step(1)).source().isWallContact());
        assertFalse(((Step.Intake) r.step(3)).source().isWallContact());
    }

    @Test
    public void stepsAreImmutableAndTheListIsReadOnly() {
        AutoRoute r = everything();
        try {
            r.steps().add(r.step(0));
            fail("steps() must be unmodifiable");
        } catch (UnsupportedOperationException expected) {
            // good
        }
        Step.Drive d = (Step.Drive) r.step(3);
        try {
            d.via().add(new Point2(0, 0));
            fail("via() must be unmodifiable");
        } catch (UnsupportedOperationException expected) {
            // good
        }
        Step.Drive faster = d.withSpeed(1.0);
        assertEquals("withX returns a copy", 0.5, d.speedFraction(), EPS);
        assertEquals(1.0, faster.speedFraction(), EPS);
    }

    @Test
    public void buildRejectsStructurallyBrokenRoutes() {
        expectIllegalState(AutoRoute.builder("empty"));
        expectIllegalState(AutoRoute.builder("no_start").waitSeconds(1));
        expectIllegalState(AutoRoute.builder("two_starts").start(35, 9.2, -90).start(58, 9.2, -90));
        try {
            AutoRoute.builder(" ");
            fail("blank id");
        } catch (IllegalArgumentException expected) {
            // good
        }
    }

    private static void expectIllegalState(AutoRoute.Builder b) {
        try {
            b.build();
            fail("build() should have thrown");
        } catch (IllegalStateException expected) {
            // good
        }
    }

    @Test
    public void modifiersOnTheWrongStepKindFailLoudly() {
        try {
            AutoRoute.builder("x").start(35, 9.2, -90).via(1, 1);
            fail("via() after START");
        } catch (IllegalStateException expected) {
            // good
        }
        try {
            AutoRoute.builder("x").start(35, 9.2, -90).waitSeconds(1).launchSeconds(2);
            fail("launchSeconds() after WAIT");
        } catch (IllegalStateException expected) {
            // good
        }
        try {
            AutoRoute.builder("x").note("nothing yet");
            fail("note() with no step");
        } catch (IllegalStateException expected) {
            // good
        }
    }

    @Test
    public void insaneNumbersAreRejectedAtTheCallThatSuppliesThem() {
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).driveTangent(1, 1).speed(0.0));
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).driveTangent(1, 1).speed(1.5));
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).waitSeconds(-1));
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).shootUp(0));
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).intakeGarden(1, 1, 9, 9, 0, 0));
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).shootUp(4).launchSeconds(-1));
    }

    private static void expectIllegalArgument(Runnable action) {
        try {
            action.run();
            fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // good
        }
    }

    @Test
    public void redRouteForRedIsTheSameInstance() {
        AutoRoute r = everything();
        assertSame(r, r.forAlliance(Alliance.RED));
    }

    @Test
    public void blueIsTheHalfTurnOfRedForEveryStep() {
        AutoRoute red = everything();
        AutoRoute blue = red.forAlliance(Alliance.BLUE);
        assertEquals(Alliance.BLUE, blue.alliance());
        assertEquals(red.size(), blue.size());
        for (int i = 0; i < red.size(); i++) {
            Step r = red.step(i);
            Step b = blue.step(i);
            assertEquals("kind of step " + i, r.kind(), b.kind());
            assertEquals("note of step " + i, r.note(), b.note());
            switch (r.kind()) {
                case START:
                    assertHalfTurn(((Step.Start) r).pose(), ((Step.Start) b).pose());
                    break;
                case DRIVE: {
                    Step.Drive rd = (Step.Drive) r;
                    Step.Drive bd = (Step.Drive) b;
                    assertHalfTurn(rd.target(), bd.target());
                    assertEquals(rd.mode(), bd.mode());
                    assertEquals(rd.via().size(), bd.via().size());
                    for (int k = 0; k < rd.via().size(); k++) {
                        assertEquals(SIZE - rd.via().get(k).x, bd.via().get(k).x, EPS);
                        assertEquals(SIZE - rd.via().get(k).y, bd.via().get(k).y, EPS);
                    }
                    if (rd.facePoint() != null) {
                        assertEquals(SIZE - rd.facePoint().x, bd.facePoint().x, EPS);
                        assertEquals(SIZE - rd.facePoint().y, bd.facePoint().y, EPS);
                        assertEquals(rd.faceWithBack(), bd.faceWithBack());
                    }
                    assertEquals(rd.speedFraction(), bd.speedFraction(), EPS);
                    assertEquals(rd.label(), bd.label());
                    assertEquals(rd.isWallContact(), bd.isWallContact());
                    break;
                }
                case INTAKE: {
                    Step.Intake ri = (Step.Intake) r;
                    Step.Intake bi = (Step.Intake) b;
                    assertHalfTurn(ri.robotPose(), bi.robotPose());
                    assertEquals(SIZE - ri.target().x, bi.target().x, EPS);
                    assertEquals(SIZE - ri.target().y, bi.target().y, EPS);
                    assertEquals(ri.source(), bi.source());
                    assertEquals(ri.dwellSec(), bi.dwellSec(), EPS);
                    assertEquals(ri.expectedCount(), bi.expectedCount());
                    break;
                }
                case SHOOT: {
                    Step.Shoot rs = (Step.Shoot) r;
                    Step.Shoot bs = (Step.Shoot) b;
                    assertEquals(rs.isAutoPose(), bs.isAutoPose());
                    if (!rs.isAutoPose()) {
                        assertHalfTurn(rs.poseOrNull(), bs.poseOrNull());
                    }
                    assertEquals(rs.count(), bs.count());
                    assertEquals(rs.hasFixedSeconds(), bs.hasFixedSeconds());
                    break;
                }
                case WAIT:
                    assertEquals(((Step.Wait) r).seconds(), ((Step.Wait) b).seconds(), EPS);
                    break;
                case WAIT_FOR_TIP:
                    assertEquals(((Step.WaitForTip) r).giveUpSec(), ((Step.WaitForTip) b).giveUpSec(), EPS);
                    break;
                case PARK:
                    assertHalfTurn(((Step.Park) r).pose(), ((Step.Park) b).pose());
                    break;
                default:
                    fail("unhandled kind " + r.kind());
            }
        }
    }

    private static void assertHalfTurn(Pose2 red, Pose2 blue) {
        Pose2 expected = Alliance.BLUE.fromRed(red, SIZE);
        assertEquals(expected.x, blue.x, EPS);
        assertEquals(expected.y, blue.y, EPS);
        assertEquals(0.0, Math.abs(org.firstinspires.ftc.teamcode.autoshoot.core.Angles.error(expected.heading,
                blue.heading)), EPS);
        // and it really is a half turn, not a mirror: the heading changes by exactly 180 degrees
        double turned = Math.abs(org.firstinspires.ftc.teamcode.autoshoot.core.Angles.error(red.heading, blue.heading));
        assertEquals(Math.PI, turned, 1e-9);
    }

    @Test
    public void theFirstTipAssumptionDefaultsToSelfAndSurvivesTheHalfTurn() {
        AutoRoute plain = AutoRoute.builder("p").start(35, 9.2, -90).waitSeconds(1).build();
        assertEquals(FirstTip.SELF, plain.firstTip());
        assertTrue(Double.isNaN(plain.partnerTipSec()));
        assertEquals("we shoot the first tip", plain.firstTipSummary());

        AutoRoute partner = AutoRoute.builder("q").start(35, 9.2, -90).partnerTipsFirst().waitSeconds(1).build();
        assertEquals(FirstTip.PARTNER, partner.firstTip());
        assertEquals(BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC, partner.partnerTipSec(), EPS);
        assertEquals("partner tips first, assumed at 12.0 s", partner.firstTipSummary());
        // the same assumption in the blue half turn, and back again
        assertEquals(FirstTip.PARTNER, partner.forAlliance(Alliance.BLUE).firstTip());
        assertEquals(FirstTip.PARTNER, partner.forAlliance(Alliance.BLUE).forAlliance(Alliance.RED).firstTip());
        assertEquals(FirstTip.SELF, plain.forAlliance(Alliance.BLUE).firstTip());
        // the explicit form is the same thing
        assertEquals(FirstTip.PARTNER, AutoRoute.builder("r").start(35, 9.2, -90).firstTip(FirstTip.PARTNER).build().firstTip());
        expectIllegalArgument(() -> AutoRoute.builder("s").firstTip(null));
    }

    @Test
    public void waitForTipTakesAGiveUpTimeAboveZeroAndKeepsItsNote() {
        AutoRoute r = AutoRoute.builder("w").start(35, 9.2, -90).waitForTip(6.5).note("until the rear cell is up").build();
        Step.WaitForTip w = (Step.WaitForTip) r.step(1);
        assertEquals(StepKind.WAIT_FOR_TIP, w.kind());
        assertEquals("a point on the route clock", 6.5, w.giveUpSec(), EPS);
        assertEquals("until the rear cell is up", w.note());
        assertTrue(w.describe(), w.describe().startsWith("WAIT_FOR_TIP"));
        assertTrue(w.describe(), w.describe().contains("6.5"));
        assertTrue("the words say it is the camera that decides: " + w.describe(), w.describe().contains("camera"));
        assertTrue(w.describe(), w.describe().contains("route clock"));
        assertSame("a wait has no pose, so the half turn leaves it as it is", w, w.halfTurn(SIZE));
        // a wait that never waits is a mistake, not a feature
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).waitForTip(0));
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).waitForTip(-3));
        expectIllegalArgument(() -> AutoRoute.builder("a").start(35, 9.2, -90).waitForTip(Double.NaN));
    }

    @Test
    public void aFixedHiveSideFlipsUnderTheHalfTurnButTheUpCellDoesNot() {
        AutoRoute red = AutoRoute.builder("sides").start(58, 9.2, -90)
                .shootSide(Side.AUDIENCE, 4).shootUp(1).build();
        AutoRoute blue = red.forAlliance(Alliance.BLUE);
        assertEquals(Side.REAR, ((Step.Shoot) blue.step(1)).target().fixedSideOrNull());
        assertTrue(((Step.Shoot) blue.step(2)).target().isUpCell());
        // the half turn is its own inverse
        AutoRoute back = blue.forAlliance(Alliance.RED);
        assertEquals(Side.AUDIENCE, ((Step.Shoot) back.step(1)).target().fixedSideOrNull());
    }

    @Test
    public void halfTurnTwiceGivesTheOriginalGeometry() {
        AutoRoute red = everything();
        AutoRoute back = red.forAlliance(Alliance.BLUE).forAlliance(Alliance.RED);
        assertEquals(Alliance.RED, back.alliance());
        List<Step> a = red.steps();
        List<Step> b = back.steps();
        for (int i = 0; i < a.size(); i++) {
            assertEquals(a.get(i).describe(), b.get(i).describe());
        }
    }
}
