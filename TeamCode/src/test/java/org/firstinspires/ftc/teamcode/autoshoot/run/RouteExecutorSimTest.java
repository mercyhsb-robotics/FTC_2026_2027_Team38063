package org.firstinspires.ftc.teamcode.autoshoot.run;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.PoseStore;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.FirstTip;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteValidator;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Step;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StepKind;
import org.firstinspires.ftc.teamcode.autoshoot.routes.WiggleTestRobot;

/**
 * Closed-loop tests of the ROUTE RUNNER: every route of the catalog that the route checker accepts is driven, for the red and the blue
 * alliance, by the REAL {@link RouteExecutor} against a simulated robot, HIVE, camera, flywheel and POLLEN ({@link RouteWorld}, built on the
 * simulator the hold-to-shoot tests use). Only the world is fake.
 *
 * <p>What is asserted for every route (the user's fourth constraint and the game rules):
 * <ul>
 *   <li>it finishes, or is stopped by the AUTO clock, within the 30 s; it never holds more than 4 POLLEN (G407) and never runs the intake
 *       while holding 4; the runner's own count of POLLEN agrees with the simulation's;</li>
 *   <li>no POLLEN goes into a cell that is not UP (a ball is allowed into the cell that is swinging down, and only the spare of a volley that
 *       tips it, as the plan says);</li>
 *   <li>the partner tips the HIVE at 4 s, 12 s, 20 s or never: the robot shoots only after its camera has seen the cell UP and steady for the settle
 *       time, skips the volley and still parks when the tip comes too late or never, and shoots nothing when the camera is dead;</li>
 *   <li>the robot's true rectangle never leaves the field, enters the HIVE frame or puts its chassis over a FLOWER plate (checked at every
 *       20 ms tick by code that shares nothing with the route checker);</li>
 *   <li>the final pose, the alliance and the HIVE belief are saved for the TeleOp.</li>
 * </ul>
 * Negative tests: a robot that is stuck, a heading controller that is dead, a route that would take a fifth POLLEN, a route that is longer than
 * the AUTO.
 */
public class RouteExecutorSimTest {
    private static final double SETTLE = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
    private static final double LIMIT = BioBuzzConstants.Auto.AUTO_DURATION_SEC;
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;

    @After
    public void clearTheStore() {
        PoseStore.clear();
    }

    // ---- the routes under test --------------------------------------------------------------------------------------------

    private static boolean hasError(AutoRoute route) {
        for (Issue i : RouteValidator.check(route)) {
            if (i.isError()) {
                return true;
            }
        }
        return false;
    }

    /** Every route of the catalog, in both alliances' frames, that has no ERROR from the route checker. */
    private static List<AutoRoute> runnableRoutes() {
        List<AutoRoute> out = new ArrayList<AutoRoute>();
        for (AutoRoute red : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                AutoRoute r = red.forAlliance(a);
                if (!hasError(r)) {
                    out.add(r);
                }
            }
        }
        return out;
    }

    private static RouteWorld worldFor(AutoRoute route) {
        return new RouteWorld(route.analyze());
    }

    private static String name(AutoRoute r) {
        return r.id() + "/" + r.alliance();
    }

    // ---- the whole catalog -------------------------------------------------------------------------------------------------

    @Test
    public void theCatalogHasRunnableRoutesOfBothFamiliesForBothAlliances() {
        boolean self = false;
        boolean partner = false;
        int red = 0;
        int blue = 0;
        for (AutoRoute r : runnableRoutes()) {
            self |= r.firstTip() == FirstTip.SELF;
            partner |= r.firstTip() == FirstTip.PARTNER;
            red += r.alliance() == Alliance.RED ? 1 : 0;
            blue += r.alliance() == Alliance.BLUE ? 1 : 0;
        }
        assertTrue("a route that we tip first with, and one that the partner tips first with, both without ERROR", self && partner);
        assertTrue(red + " red and " + blue + " blue routes", red >= 3 && red == blue);
    }

    /** What every run must satisfy, whatever the route and whatever the partner does. */
    private static void assertSafeRun(String what, RouteWorld w) {
        assertTrue(what + ": the runner is over (finished or stopped by the clock) at " + w.t + " s: " + w.lastOut.status.line(), w.executor.finished());
        assertTrue(what + ": inside the AUTO limit, ended at " + w.t, w.t <= LIMIT + 2 * RouteWorld.DT);
        assertTrue(what + ": never more than 4 POLLEN held (G407), the most was " + w.maxHeld, w.maxHeld <= BioBuzzConstants.Auto.POLLEN_CAPACITY);
        assertEquals(what + ": the intake never ran while the robot held 4", 0.0, w.intakeOnWhenFullSec, 1e-9);
        assertEquals(what + ": the robot stayed on the field and out of the HIVE and the FLOWER plates; first clip " + w.firstClip, 0, w.clipTicks);
        assertEquals(what + ": no POLLEN into a cell that faces down", 0, w.shotsAtDownCell);
        assertEquals(what + ": no ball was badly aimed or too slow", 0, w.badShots);
        // one spare POLLEN of a volley that tips the HIVE may go into the cell while it swings down (the plan counts it as spent); never more
        assertTrue(what + ": " + w.shotsIntoSwingingCell + " balls into a swinging cell, the plan allows one per tip of ours", w.shotsIntoSwingingCell <= w.plan.tips());
        assertEquals(what + ": the runner's POLLEN count is the simulation's", w.balls, w.executor.pollenHeld());
    }

    @Test
    public void everyRunnableCatalogRouteFinishesSafelyForBothAlliances() {
        for (AutoRoute r : runnableRoutes()) {
            RouteWorld w = worldFor(r);
            // a partner route is run with the partner tipping at the time the route assumes
            w.partnerTipAt = r.firstTip() == FirstTip.PARTNER ? r.partnerTipSec() : Double.POSITIVE_INFINITY;
            w.runUntilFinished(LIMIT + 1.0);
            assertSafeRun(name(r), w);
            assertEquals(name(r) + ": the clock did not have to stop it", "", w.executor.stoppedBy());
            assertTrue(name(r) + ": no problem was recorded: " + w.executor.failures(), w.executor.failures().isEmpty());
            RouteAnalysis a = w.plan;
            // it ends where the plan says (a few inches: the simulated follower stops short), parked if the plan parks
            Pose2 planned = a.endPose();
            assertTrue(name(r) + ": ends within 5 in of the planned end " + planned + ", at " + w.pose(), w.pose().distanceTo(planned) < 5.0);
            // it did fire what the plan fires, and only that
            assertTrue(name(r) + ": fired " + w.shotsFired + ", the plan fires " + a.pollenShot(), w.shotsFired <= a.pollenShot());
            assertTrue(name(r) + ": the volleys it planned happened", a.pollenShot() == 0 || w.shotsFired >= 3);
        }
    }

    @Test
    public void theRunTakesTheTimeThePlanSaysWithinTheSlowFollowerOfTheSimulation() {
        for (AutoRoute r : runnableRoutes()) {
            RouteWorld w = worldFor(r);
            w.partnerTipAt = r.firstTip() == FirstTip.PARTNER ? r.partnerTipSec() : Double.POSITIVE_INFINITY;
            w.runUntilFinished(LIMIT + 1.0);
            double planned = r.analyze().totalSeconds();
            double took = w.t;
            assertTrue(name(r) + ": planned " + planned + " s, took " + took + " s", took < planned * 1.6 + 2.0);
        }
    }

    // ---- the partner tips the HIVE at 4 s, 12 s, 20 s, never ------------------------------------------------------------------

    private static List<AutoRoute> partnerRoutes() {
        List<AutoRoute> out = new ArrayList<AutoRoute>();
        for (AutoRoute r : runnableRoutes()) {
            if (r.firstTip() == FirstTip.PARTNER) {
                out.add(r);
            }
        }
        return out;
    }

    @Test
    public void thePartnerRoutesShootOnlyAfterTheCameraHasSeenTheCellUpForTheSettleTime() {
        assertFalse(partnerRoutes().isEmpty());
        for (AutoRoute r : partnerRoutes()) {
            // the latest tip the plan promises still works, 4 s and 12 s are well inside it
            for (double tipAt : new double[] {4.0, 12.0}) {
                RouteWorld w = worldFor(r);
                w.partnerTipAt = tipAt;
                w.runUntilFinished(LIMIT + 1.0);
                String what = name(r) + ", tip at " + tipAt;
                assertSafeRun(what, w);
                assertEquals(what + ": the four POLLEN were fired", 4, w.shotTimes.size());
                for (double shot : w.shotTimes) {
                    assertTrue(what + ": a POLLEN left at " + shot + " s, before the camera could have seen the cell UP for " + SETTLE + " s after " + tipAt,
                            shot >= tipAt + SETTLE);
                }
                assertEquals(what + ": every ball into the cell that came up", 4, w.shotsIntoUpCell);
                assertFalse(what, w.executor.tipNeverConfirmed());
                assertTrue(what + ": it parked in the zone and kept its LEAVE", w.executor.failures().isEmpty());
                assertParkedInTheZone(what, w);
            }
        }
    }

    @Test
    public void aTipThatComesAfterTheGiveUpDeadlineMakesTheRobotSkipTheVolleyAndStillPark() {
        for (AutoRoute r : partnerRoutes()) {
            // a tip one second after the latest one the route can use (each route has its own give-up deadline: 20 s was late for every route of the first lane,
            // but the aggressive routes wait longer), and a tip that never comes
            for (double tipAt : new double[] {r.analyze().latestPartnerTipSec() + 1.0, Double.POSITIVE_INFINITY}) {
                RouteWorld w = worldFor(r);
                w.partnerTipAt = tipAt;
                w.runUntilFinished(LIMIT + 1.0);
                String what = name(r) + ", tip at " + tipAt;
                assertSafeRun(what, w);
                assertEquals(what + ": no POLLEN left the robot", 0, w.shotTimes.size());
                assertTrue(what + ": the runner knows the tip was never confirmed", w.executor.tipNeverConfirmed());
                assertEquals(what + ": it still holds its 4 POLLEN (the volley was left out)", 4, w.balls);
                assertTrue(what + ": the reason is recorded: " + w.executor.failures(), w.executor.failures().toString().contains("gave up waiting"));
                assertEquals(what + ": it did not wait past the end", "", w.executor.stoppedBy());
                assertParkedInTheZone(what, w);
            }
        }
    }

    @Test
    public void withADeadCameraTheRobotShootsNothingAtACellThatCameUpEvenThoughItReallyIsUp() {
        for (AutoRoute r : partnerRoutes()) {
            RouteWorld w = worldFor(r);
            w.partnerTipAt = 4.0;
            w.cameraAlive = false;
            w.runUntilFinished(LIMIT + 1.0);
            String what = name(r) + ", dead camera";
            assertSafeRun(what, w);
            assertEquals(what + ": no camera, no shot", 0, w.shotTimes.size());
            assertTrue(what, w.executor.tipNeverConfirmed());
            assertParkedInTheZone(what, w);
        }
    }

    @Test
    public void aCameraThatDiesAfterTheTipStopsTheVolleyAtOnce() {
        for (AutoRoute r : partnerRoutes()) {
            RouteWorld w = worldFor(r);
            w.partnerTipAt = 4.0;
            w.cameraDeadFrom = 6.0;
            w.runUntilFinished(LIMIT + 1.0);
            String what = name(r) + ", camera lost at 6 s";
            assertSafeRun(what, w);
            for (double shot : w.shotTimes) {
                assertTrue(what + ": a ball left at " + shot + " s, after the camera was gone", shot < 6.0 + RouteWorld.DT);
            }
            assertParkedInTheZone(what, w);
        }
    }

    @Test
    public void whateverTheTimeOfThePartnersTipNoPollenIsFiredBeforeTheCameraConfirmsAndTheRobotAlwaysParks() {
        for (AutoRoute r : partnerRoutes()) {
            for (double tipAt = 0.5; tipAt <= 26.0; tipAt += 2.3) {
                RouteWorld w = worldFor(r);
                w.partnerTipAt = tipAt;
                w.runUntilFinished(LIMIT + 1.0);
                String what = name(r) + ", tip at " + tipAt;
                assertSafeRun(what, w);
                for (double shot : w.shotTimes) {
                    assertTrue(what + ": a ball at " + shot, shot >= tipAt + SETTLE);
                }
                assertTrue(what + ": " + w.shotTimes.size() + " balls", w.shotTimes.size() == 0 || w.shotTimes.size() == 4);
                assertParkedInTheZone(what, w);
            }
        }
    }

    /** The robot ended off every wall and overlapping its own LOADING ZONE: the plan earns LEAVE and PARK. */
    private static void assertParkedInTheZone(String what, RouteWorld w) {
        Pose2 p = w.pose();
        org.firstinspires.ftc.teamcode.autoshoot.core.Footprint f = org.firstinspires.ftc.teamcode.autoshoot.core.Footprint.of(p);
        Rect zone = w.plan.field().ownLoadingZone();
        assertNotNull(what + ": in the LOADING ZONE at " + p, f.clippedBounds(zone));
        Rect b = f.bounds();
        double m = BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN - 0.5;      // half an inch for the simulated follower's stopping error
        double s = w.field.size();
        assertTrue(what + ": off every wall (LEAVE) at " + p, b.minX >= m && b.minY >= m && b.maxX <= s - m && b.maxY <= s - m);
    }

    // ---- the first volley without the partner -------------------------------------------------------------------------------------

    @Test
    public void aRouteWeTipFirstWithShootsItsVolleyAndTheHiveTipsOnTheThirdPollen() {
        for (AutoRoute r : runnableRoutes()) {
            if (r.firstTip() != FirstTip.SELF || r.analyze().tips() == 0) {
                continue;
            }
            RouteWorld w = worldFor(r);
            w.runUntilFinished(LIMIT + 1.0);
            String what = name(r);
            assertSafeRun(what, w);
            // A route that ends with its volley (nothing to drive afterwards) is "finished" the moment the last POLLEN leaves; the ones in the air
            // still land and the HIVE still swings while the robot stands there (AUTO goes on until the buzzer), so let them.
            w.runFor(w.flightSec + w.swingSec + 0.5);
            // the first volley fires 3 or 4 (the 4th is the spare); a later volley at the cell that came up with this tip (the rear bank) fires its own count
            int most = 4;
            for (RouteAnalysis.StepInfo info : r.analyze().steps()) {
                if (info.step.kind() == StepKind.SHOOT && info.tipDependent) {
                    most += ((Step.Shoot) info.step).count();
                }
            }
            assertTrue(what + ": the volley happened: " + w.shotsFired, w.shotsFired >= 3 && w.shotsFired <= most);
            assertTrue(what + ": the 3rd POLLEN tipped the HIVE, so the other cell is up", w.trueUp != w.field.initialUpSide(w.alliance));
        }
    }

    @Test
    public void aTopUpVolleyFiresOnlyWhenTheFirstVolleyDidNotTipAndTheCameraStillSeesTheCellUp() {
        int topUps = 0;
        for (AutoRoute r : runnableRoutes()) {
            int wanted = 0;
            for (RouteAnalysis.StepInfo info : r.analyze().steps()) {
                if (info.step.kind() == StepKind.SHOOT && !info.tipDependent && !info.targetWasUp) {
                    wanted += ((Step.Shoot) info.step).count();
                }
            }
            if (r.firstTip() != FirstTip.SELF || wanted == 0) {
                continue;
            }
            topUps++;
            // the first volley misses the tip (the HIVE never tips): the camera still sees the cell UP, so the second volley goes into it
            RouteWorld miss = worldFor(r);
            miss.tipAfterHits = 1000;
            miss.runUntilFinished(LIMIT + 1.0);
            String what = name(r) + ", the first volley did not tip";
            assertSafeRun(what, miss);
            assertEquals(what + ": the second volley fired as well", 4 + wanted, miss.shotsFired);
            assertEquals(what + ": every ball into the cell that is up", 4 + wanted, miss.shotsIntoUpCell);
            // the first volley tips (the normal case): the camera sees the cell fall, the second volley is left out, and nothing is recorded as wrong
            RouteWorld tipped = worldFor(r);
            tipped.runUntilFinished(LIMIT + 1.0);
            assertSafeRun(name(r), tipped);
            assertTrue(name(r) + ": the second volley was left out: " + tipped.shotsFired, tipped.shotsFired <= 4);
            assertTrue(name(r) + ": nothing went wrong: " + tipped.executor.failures(), tipped.executor.failures().isEmpty());
            // the camera dies before the second volley: it is left out, whatever the HIVE did
            RouteWorld blind = worldFor(r);
            blind.tipAfterHits = 1000;
            blind.cameraDeadFrom = 1.0;
            blind.runUntilFinished(LIMIT + 1.0);
            assertSafeRun(name(r) + ", the camera lost at 1 s", blind);
            assertTrue(name(r) + ": no second volley without a camera: " + blind.shotsFired, blind.shotsFired <= 4);
        }
        assertTrue("the catalog has self routes with a top-up", topUps >= 2);
    }

    @Test
    public void ifThePartnerTipsBeforeOurVolleyTheRobotDoesNotFireAtTheCellThatNowFacesDown() {
        for (AutoRoute r : runnableRoutes()) {
            if (r.firstTip() != FirstTip.SELF || r.analyze().tips() == 0) {
                continue;
            }
            RouteWorld w = worldFor(r);
            w.partnerTipAt = 0.8;                       // long before the first POLLEN could leave (aiming and spinning up take seconds)
            w.runUntilFinished(LIMIT + 1.0);
            String what = name(r) + ", the partner tipped at 0.8 s";
            assertTrue(what + ": finished", w.executor.finished());
            assertEquals(what + ": nothing fired at a cell that faces down", 0, w.shotsAtDownCell);
            // The first volley wants the cell that was UP at the start; the partner has just tipped it, so it faces down and the volley must be skipped.
            // Only a LATER volley of the plan at the OTHER cell (the one that came up, like the rear bank) may fire, once the camera has seen that cell UP.
            Side wanted = w.field.initialUpSide(w.alliance);
            int atTheCellThatIsDown = 0;
            for (Side side : w.shotSides) {
                atTheCellThatIsDown += side == wanted ? 1 : 0;
            }
            assertEquals(what + ": no POLLEN at the cell the route wants, it is down", 0, atTheCellThatIsDown);
            int allowed = 0;
            boolean first = true;
            for (RouteAnalysis.StepInfo info : w.plan.steps()) {
                if (info.step.kind() != StepKind.SHOOT) {
                    continue;
                }
                if (!first && info.targetSide == wanted.opposite()) {
                    allowed += ((Step.Shoot) info.step).count();
                }
                first = false;
            }
            assertTrue(what + ": " + w.shotTimes.size() + " POLLEN fired, the plan has " + allowed + " for a later volley at the cell that comes up", w.shotTimes.size() <= allowed);
            assertTrue(what + ": inside the field", w.clipTicks == 0);
        }
    }

    // ---- the pose is saved for the TeleOp -------------------------------------------------------------------------------------------

    @Test
    public void theFinalPoseTheAllianceAndTheHiveBeliefAreSavedForTheTeleOp() {
        for (AutoRoute r : runnableRoutes()) {
            PoseStore.clear();
            RouteWorld w = worldFor(r);
            w.partnerTipAt = r.firstTip() == FirstTip.PARTNER ? 4.0 : Double.POSITIVE_INFINITY;
            w.runUntilFinished(LIMIT + 1.0);
            long now = System.nanoTime();
            w.executor.saveForTeleOp(w.pose(), now);
            PoseStore.Snapshot s = PoseStore.load(now, BioBuzzConstants.Controls.POSE_CARRYOVER_MAX_AGE_SEC);
            assertNotNull(name(r), s);
            assertEquals(name(r), r.alliance(), s.alliance);
            assertEquals(name(r), w.pose().x, s.pose.x, 1e-9);
            assertEquals(name(r), w.pose().y, s.pose.y, 1e-9);
            assertEquals(name(r) + ": the belief the TeleOp starts with is the runner's", w.executor.tracker().upSide(), s.hiveUpSide);
        }
    }

    // ---- every step kind and the blue half turn --------------------------------------------------------------------------------------

    @Test
    public void theBlueRunIsTheHalfTurnOfTheRedRun() {
        for (AutoRoute red : RouteCatalog.all()) {
            AutoRoute r = red.forAlliance(Alliance.RED);
            AutoRoute b = red.forAlliance(Alliance.BLUE);
            if (hasError(r) || hasError(b)) {
                continue;
            }
            RouteWorld wr = worldFor(r);
            RouteWorld wb = worldFor(b);
            wr.partnerTipAt = r.firstTip() == FirstTip.PARTNER ? 4.0 : Double.POSITIVE_INFINITY;
            wb.partnerTipAt = wr.partnerTipAt;
            wr.runUntilFinished(LIMIT + 1.0);
            wb.runUntilFinished(LIMIT + 1.0);
            Pose2 turned = Alliance.BLUE.fromRed(wr.pose(), BioBuzzConstants.Field.SIZE_IN);
            assertEquals(red.id(), turned.x, wb.x, 1.0);
            assertEquals(red.id(), turned.y, wb.y, 1.0);
            assertEquals(red.id(), wr.shotTimes.size(), wb.shotTimes.size());
        }
    }

    // ---- the FLOWER wiggle -----------------------------------------------------------------------------------------------------------

    private static final double[] FLOWER = {BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1]};

    /** Shoot 4 (tips the HIVE), let what the tipped cell drops land, then take 3 POLLEN from the red-wall FLOWER (wheels that reach 4 in: see WiggleTestRobot). */
    private static AutoRoute flowerRoute() {
        return AutoRoute.builder("flower_run").start(35, FRONT, -90).shootUp(4).waitSeconds(5).intakeFlower(FLOWER[0], FLOWER[1], 3).build();
    }

    private static RouteWorld flowerWorld(AutoRoute route) {
        RouteWorld w = new RouteWorld(WiggleTestRobot.analyze(route));
        w.setIntakeReach(WiggleTestRobot.REACH_IN);
        return w;
    }

    @Test
    public void theFlowerPickUpIsExecutedAsAWiggleWithTheChosenWheelAndTakesThePollenThePlanSays() {
        for (Alliance a : Alliance.values()) {
            AutoRoute route = flowerRoute().forAlliance(a);
            assertTrue("the route is valid for the robot with the longer wheels", RouteValidator.isRunnable(WiggleTestRobot.check(route)));
            RouteWorld w = flowerWorld(route);
            w.runUntilFinished(LIMIT + 1.0);
            String what = "flower/" + a;
            assertTrue(what + ": finished", w.executor.finished());
            assertEquals(what + ": no clip: " + w.firstClip, 0, w.clipTicks);
            assertEquals(what + ": 3 POLLEN came out of the FLOWER", 3, w.takenFromFlowers);
            assertTrue(what + ": never more than 4: " + w.maxHeld, w.maxHeld <= 4);
            assertEquals(what + ": the runner counts what the simulation holds", w.balls, w.executor.pollenHeld());
            // the phases of the pick-up, in order
            List<RouteExecutor.Phase> p = w.phases;
            int lineUp = p.indexOf(RouteExecutor.Phase.FLOWER_LINE_UP);
            int approach = p.indexOf(RouteExecutor.Phase.FLOWER_APPROACH);
            int wiggle = p.indexOf(RouteExecutor.Phase.FLOWER_WIGGLE);
            int retreat = p.indexOf(RouteExecutor.Phase.FLOWER_RETREAT);
            assertTrue(what + ": " + p, lineUp >= 0 && lineUp < approach && approach < wiggle && wiggle < retreat);
            // the heading target ALTERNATES between straight at the wall and turned toward the ball, once per POLLEN
            RouteAnalysis.StepInfo info = w.plan.step(3);
            double straight = info.press.start.heading;
            double pushed = info.press.pushed.heading;
            int runs = 0;
            Boolean previousWasPushed = null;
            for (double target : w.wiggleTargets) {
                boolean isPushed = Math.abs(org.firstinspires.ftc.teamcode.autoshoot.core.Angles.error(target, pushed)) < 1e-9;
                boolean isStraight = Math.abs(org.firstinspires.ftc.teamcode.autoshoot.core.Angles.error(target, straight)) < 1e-9;
                assertTrue(what + ": a target that is neither straight nor pushed: " + target, isPushed || isStraight);
                if (previousWasPushed == null || previousWasPushed != isPushed) {
                    runs++;
                    if (previousWasPushed == null) {
                        assertTrue(what + ": the first target is the push", isPushed);
                    }
                }
                previousWasPushed = isPushed;
            }
            assertEquals(what + ": 3 cycles = 3 pushes and 3 releases, alternating", 2 * info.press.cycles, runs);
            // it backed out the way it came, straight, and ended at the retreat pose
            assertTrue(what + ": ends at the retreat pose " + info.press.retreat + ", is at " + w.pose(), w.pose().distanceTo(info.press.retreat) < 3.0);
            assertEquals(what + ": the wheel is the one the shared function chose", info.choice.wheel(), info.press.wheel);
        }
    }

    @Test
    public void aHeadingControllerThatNeverGetsThereMakesTheWiggleGiveUpBackOutAndCarryOn() {
        AutoRoute route = flowerRoute();
        RouteWorld w = flowerWorld(route);
        // run until the wiggle starts, then kill the heading controller: the turns never reach their heading
        while (w.t < LIMIT && w.executor.phase() != RouteExecutor.Phase.FLOWER_WIGGLE) {
            w.tick();
        }
        assertEquals(RouteExecutor.Phase.FLOWER_WIGGLE, w.executor.phase());
        w.headingFrozen = true;
        w.runUntilFinished(LIMIT + 1.0);
        assertTrue("finished", w.executor.finished());
        assertEquals("no POLLEN were taken: the first turn never got there", 0, w.takenFromFlowers);
        assertTrue("the failure is recorded: " + w.executor.failures(), w.executor.failures().toString().contains("never reached its heading"));
        assertTrue("it backed out and the route went on", w.phases.contains(RouteExecutor.Phase.FLOWER_RETREAT));
        assertEquals(4 - w.shotsFired, w.executor.pollenHeld());
    }

    // ---- fail closed ----------------------------------------------------------------------------------------------------------------------

    @Test
    public void aRobotThatIsStuckGivesUpOnEachMoveRecordsWhyAndIsStoppedByTheClockNeverFiringBlind() {
        for (AutoRoute r : runnableRoutes()) {
            if (r.alliance() != Alliance.RED) {
                continue;
            }
            RouteWorld w = worldFor(r);
            w.partnerTipAt = r.firstTip() == FirstTip.PARTNER ? 4.0 : Double.POSITIVE_INFINITY;
            w.frozen = true;
            w.runUntilFinished(LIMIT + 1.0);
            String what = name(r) + ", stuck";
            assertTrue(what + ": over (finished or stopped by the clock)", w.executor.finished());
            assertTrue(what + ": inside the AUTO limit " + w.t, w.t <= LIMIT + 2 * RouteWorld.DT);
            assertEquals(what, 0, w.shotsAtDownCell);
            assertTrue(what + ": never more than 4", w.maxHeld <= 4);
            // a robot that cannot move never arrives, and every give-up is recorded with its reason
            assertFalse(what + ": the problems are recorded", w.executor.failures().isEmpty());
        }
    }

    @Test
    public void aRouteThatWouldTakeAFifthPollenTakesNone() {
        // the robot holds 4 and nothing is shot: the GARDEN must not be touched (G407). The checker calls this route an ERROR; the runner is the last net.
        AutoRoute r = AutoRoute.builder("fifth").start(35, FRONT, -90)
                .intakeGarden(5.7, 1.4, 9.0, FRONT + 0.8, -90, 3).build();
        RouteWorld w = worldFor(r);
        w.runUntilFinished(LIMIT + 1.0);
        assertTrue(w.executor.finished());
        assertEquals(0, w.takenFromGarden);
        assertEquals(4, w.maxHeld);
        assertEquals(4, w.executor.pollenHeld());
        assertTrue(w.executor.failures().toString(), w.executor.failures().toString().contains("holds 4"));
    }

    @Test
    public void aRouteLongerThanTheAutoIsStoppedByTheClockAndEverythingIsSwitchedOff() {
        AutoRoute r = AutoRoute.builder("long").start(35, FRONT, -90).shootUp(4).waitSeconds(60).build();
        RouteWorld w = worldFor(r);
        w.runUntilFinished(LIMIT + 5.0);
        assertTrue(w.executor.finished());
        assertTrue("stopped by the clock: " + w.executor.stoppedBy(), w.executor.stoppedBy().contains("clock"));
        assertEquals("it stopped at the AUTO limit, not later", LIMIT, w.t, 2 * RouteWorld.DT);
        RouteExecutor.Outputs out = w.lastOut;
        assertEquals(0.0, out.flywheelTps, 0.0);
        assertFalse(out.feederOn);
        assertFalse(out.intakeOn);
        assertTrue(out.finished);
        assertEquals("the drive only holds", DriveCommand.Kind.HOLD, out.drive.kind);
        // and it keeps holding if the OpMode goes on calling it
        w.runFor(1.0);
        assertEquals(0.0, w.lastOut.flywheelTps, 0.0);
        assertFalse(w.lastOut.feederOn);
    }

    @Test
    public void theFirstLoopOnlySetsThePoseAndTheStartStepIsNeverRunTwice() {
        AutoRoute r = RouteCatalog.byId(RouteCatalog.WALL_SHOOT_PARK);
        RouteAnalysis plan = r.analyze();
        RouteExecutor ex = new RouteExecutor(plan);
        RouteExecutor.Inputs in = new RouteExecutor.Inputs();
        in.pose = new Pose2(0, 0, 0);               // garbage: the odometry has not been told yet
        in.nowSec = 0.02;
        RouteExecutor.Outputs first = ex.update(in);
        assertNotNull(first.setPose);
        assertEquals(r.startPose().x, first.setPose.x, 1e-9);
        assertEquals(DriveCommand.Kind.HOLD, first.drive.kind);
        assertFalse(first.finished);
        assertEquals("the first loop is the START step", 1, first.status.stepNumber);
        assertEquals(4, first.status.pollenHeld);
        in.pose = r.startPose();
        in.nowSec = 0.04;
        RouteExecutor.Outputs second = ex.update(in);
        assertNull(second.setPose);
        assertTrue(second.status.stepNumber >= 2);
    }

    @Test
    public void theStatusSaysWhichStepWhatItDoesHowManyPollenAndHowMuchTimeIsLeft() {
        AutoRoute r = RouteCatalog.byId(RouteCatalog.WALL_SHOOT_PARK);
        RouteWorld w = worldFor(r);
        w.runFor(3.0);
        RouteExecutor.Status s = w.lastOut.status;
        assertEquals(r.size(), s.stepCount);
        assertTrue(s.stepNumber >= 2 && s.stepNumber <= s.stepCount);
        assertTrue(s.pollenHeld >= 0 && s.pollenHeld <= 4);
        assertEquals(LIMIT - w.t, s.timeLeftSec, 2 * RouteWorld.DT);
        assertFalse(s.doing.isEmpty());
        assertTrue(s.line(), s.line().contains("step " + s.stepNumber + "/" + s.stepCount) && s.line().contains("POLLEN") && s.line().contains("s left"));
    }

    @Test
    public void aRouteThatNeedsNoCameraStillShootsItsFirstVolleyAndParksWithoutOne() {
        for (AutoRoute r : runnableRoutes()) {
            if (r.firstTip() != FirstTip.SELF || r.analyze().pollenShot() == 0) {
                continue;
            }
            RouteWorld w = worldFor(r);
            w.cameraAlive = false;
            w.runUntilFinished(LIMIT + 1.0);
            String what = name(r) + ", dead camera";
            assertSafeRun(what, w);
            assertTrue(what + ": the volley at the cell that is up from the start needs no camera: " + w.shotsFired, w.shotsFired >= 3);
            assertEquals(what, "", w.executor.stoppedBy());
            if (r.analyze().dependsOnTipConfirmation()) {
                // a later volley at the cell that comes up after our own tip (the rear bank) needs the camera: with none it is left out, and the first volley is all that was fired
                assertTrue(what + ": the volley that needs a camera was left out", w.executor.tipNeverConfirmed());
                assertTrue(what + ": only the first volley was fired: " + w.shotsFired, w.shotsFired <= 4);
            } else {
                assertFalse(what + ": nothing was left out for want of a tip", w.executor.tipNeverConfirmed());
            }
        }
    }

    @Test
    public void aRunnerThatLosesTheOdometryStopsEverythingAndSaysWhy() {
        RouteExecutor ex = new RouteExecutor(RouteCatalog.byId(RouteCatalog.WALL_SHOOT_PARK).analyze());
        RouteExecutor.Inputs in = new RouteExecutor.Inputs();
        in.pose = RouteCatalog.byId(RouteCatalog.WALL_SHOOT_PARK).startPose();
        in.nowSec = 0.02;
        ex.update(in);
        in.nowSec = 0.04;
        in.pose = null;
        RouteExecutor.Outputs out = ex.update(in);
        assertTrue(out.finished);
        assertTrue(ex.stoppedBy(), ex.stoppedBy().contains("odometry"));
        assertEquals(0.0, out.flywheelTps, 0.0);
        assertFalse(out.feederOn || out.intakeOn);
    }

    // ---- the DRIVE legs: the turn first, then the leg the validator swept ---------------------------------------------------------------

    @Test
    public void everyHeadingModeOfADriveIsHandedToTheDriveAsTheLegTheValidatorSweepsAfterATurnOnTheSpot() {
        AutoRoute r = AutoRoute.builder("modes").start(58, 22, -90)
                .driveConstant(58, 30, 0)                 // CONSTANT: turn 90 degrees first, then strafe
                .driveTangent(40, 30)                     // TANGENT: faces the way it drives (180 degrees: a 180 degree turn first)
                .driveReverseTangent(40, 20)              // REVERSE_TANGENT: drives backwards down the field
                .driveLinear(55, 20, 0)                   // LINEAR: turns evenly while it drives
                .driveFacing(55, 28, 70.0, 70.0)          // FACE_POINT: keeps its front toward the HIVE
                .build();
        RouteWorld w = worldFor(r);
        w.runUntilFinished(LIMIT + 1.0);
        assertTrue(w.executor.finished());
        assertTrue("no problem: " + w.executor.failures(), w.executor.failures().isEmpty());
        assertEquals("no clip: " + w.firstClip, 0, w.clipTicks);
        List<org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode> modes = new ArrayList<org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode>();
        for (org.firstinspires.ftc.teamcode.autoshoot.routes.DrivePath leg : w.legsFollowed) {
            modes.add(leg.mode());
        }
        assertEquals(java.util.Arrays.asList(org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode.CONSTANT,
                org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode.TANGENT, org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode.REVERSE_TANGENT,
                org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode.LINEAR, org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode.FACE_POINT), modes);
        assertTrue("it turned on the spot before a leg that starts with a big turn: " + w.phases, w.phases.contains(RouteExecutor.Phase.TURNING));
        double towardThePoint = Math.atan2(70.0 - w.y, 70.0 - w.x);
        assertEquals("it ends facing the point", 0.0, Math.toDegrees(org.firstinspires.ftc.teamcode.autoshoot.core.Angles.error(w.heading, towardThePoint)), 6.0);
        assertEquals(55.0, w.x, 2.0);
        assertEquals(28.0, w.y, 2.0);
    }

    // ---- a volley at the cell that came up with OUR OWN tip -----------------------------------------------------------------------------

    @Test
    public void aVolleyAtTheCellThatCameUpWithOurOwnTipWaitsAtItsPoseForTheCameraAndFiresOnlyAfterTheSettleTime() {
        // 3 POLLEN tip the HIVE (3 NECTAR + the 3rd), the robot takes 3 from the GARDEN, and then shoots 4 into the cell on the far side that came up
        AutoRoute r = AutoRoute.builder("own_tip").start(35, FRONT, -90).shootUp(3)
                .driveConstant(9.0, 18.0, -90).driveConstant(9.0, FRONT + 0.8, -90).intakeGarden(5.7, 1.4, 9.0, FRONT + 0.8, -90, 3)
                .shootUp(4).build();
        RouteAnalysis plan = r.analyze();
        assertTrue("the second volley is the one that depends on our own tip", plan.step(plan.steps().size() - 1).tipDependent);
        RouteWorld w = worldFor(r);
        w.runUntilFinished(LIMIT + 1.0);
        assertTrue("finished: " + w.lastOut.status.line(), w.executor.finished());
        assertEquals("no clip: " + w.firstClip, 0, w.clipTicks);
        assertEquals("no ball at a cell that faces down", 0, w.shotsAtDownCell);
        assertTrue("the HIVE tipped once from our 3 POLLEN: " + w.flipTimes, w.flipTimes.size() >= 1);
        double tipped = w.flipTimes.get(0);
        int second = 0;
        for (double shot : w.shotTimes) {
            if (shot > tipped) {
                second++;
                assertTrue("a ball at " + shot + " s before the camera could have seen the new cell steady (tip done at " + tipped + " s)", shot >= tipped + SETTLE);
            }
        }
        assertEquals("the volley at the cell that came up fired its 4", 4, second);
        assertTrue("it stood at the far side to wait: " + w.phases, w.phases.contains(RouteExecutor.Phase.CONFIRMING_TIP) || w.executor.events().toString().contains("up and steady"));
        assertTrue("the far side of the HIVE: y " + w.y, w.y > w.field.center());
        assertEquals(w.balls, w.executor.pollenHeld());
        assertTrue(w.maxHeld <= 4);
    }
}
