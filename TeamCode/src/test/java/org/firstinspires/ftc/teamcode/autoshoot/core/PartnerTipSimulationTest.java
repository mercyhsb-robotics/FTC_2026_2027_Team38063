package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;
import org.firstinspires.ftc.teamcode.autoshoot.routes.ShotGeometry;

/**
 * A closed-loop simulation of a partner-tips-first autonomous route: our robot waits at the rear of the HIVE (WAIT_FOR_TIP), the
 * alliance partner tips the HIVE at a time the robot does not know (4 s, 12 s, 20 s, or never), and the robot may fire its volley at
 * the cell that comes up ONLY after its camera has seen that cell's tags UP, steady, for the settle time.
 *
 * <p>What is real: the {@link HiveTracker}, the {@link TipConfirmation}, the {@link TipWait}, the {@link ClusterAnalyzer} and the
 * camera model, and the route analysis that gives the give-up deadline ({@code RouteAnalysis.effectiveGiveUpSec}) and the time the
 * safe steps take. What is simulated: the HIVE (the rear cluster comes into view while the HIVE swings up, with its tags blinking
 * in and out of view, and is steady only once the tip is complete), the camera (25 frames per second, the real geometry, optional dropouts
 * or a dead link) and the volley (aim and spin-up, then one POLLEN per feed time). The robot's driving is a delay taken from the
 * route analysis, not a physics model: the camera sees the cluster from where the robot stands, which is a simplification.
 *
 * <p>What the tests assert, for every tip time (the user's fourth constraint, point 7):
 * <ul>
 *   <li>no POLLEN leaves before the simulated camera has confirmed the tip (the cell seen UP for the settle time), so none is ever
 *       fired into a cell that still faces down (a foul, G417);</li>
 *   <li>a tip that comes too late, or never, makes the robot skip the volley, and it still finishes its safe step inside the AUTO limit;</li>
 *   <li>a dead camera means no shot at all, even though the cell really is up;</li>
 *   <li>the latest partner tip the route analysis promises ({@code latestPartnerTipSec}) really is the last one the closed loop can use.</li>
 * </ul>
 */
public class PartnerTipSimulationTest {
    private static final double SETTLE = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
    private static final double DT = 0.02;
    private static final double FRAME_PERIOD = 0.04;
    private static final double BUDGET = BioBuzzConstants.Auto.AUTO_DURATION_SEC - BioBuzzConstants.Auto.TIME_SAFETY_MARGIN_SEC;
    /** How long the HIVE takes to swing up before it is at rest: its tags blink in and out of view meanwhile. */
    private static final double SWING_SEC = 1.2;

    /** The route under test: wait for the partner's tip (giving up at {@code giveUp}), fire 4 at the cell that comes up, then leave the wall. */
    private static AutoRoute route(double giveUp) {
        return AutoRoute.builder("partner_rear").start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90)
                .partnerTipsFirst().waitForTip(giveUp).shootUp(4).driveConstant(61.5, 112.0, 90).build();
    }

    /** What a run produced. */
    private static final class Result {
        /** Times (route clock) at which a POLLEN left the robot. */
        final List<Double> shotTimes = new ArrayList<Double>();
        /** POLLEN fired while the HIVE was not yet at rest (the cell still facing down or still swinging): a foul. */
        int firedBeforeTheTipWasDone;
        /** The route time at which the camera confirmed the tip; NaN if it never did. */
        double confirmedAt = Double.NaN;
        TipWait.Outcome outcome;
        /** The route time at which the safe step (the drive off the wall) was finished. */
        double safeStepDoneAt;
    }

    /** One simulated match from the start of the wait to the end of the safe step (the dropouts, if any, are drawn with a fixed seed). */
    private static Result run(double tipDoneAt, boolean cameraDead, double dropoutRate, double giveUp) {
        return run(tipDoneAt, cameraDead, dropoutRate, giveUp, 5);
    }

    /** The same with the seed of the random dropouts given, so that a test can try many different flaky cameras. */
    private static Result run(double tipDoneAt, boolean cameraDead, double dropoutRate, double giveUp, long seed) {
        FieldModel field = new FieldModel();
        CameraModel camera = CameraModel.fromConstants();
        ClusterAnalyzer analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());
        HiveTracker tracker = new HiveTracker(field, Alliance.RED, HiveTracker.Config.fromConstants());
        TipConfirmation confirmation = new TipConfirmation(tracker, Side.REAR, TipConfirmation.Config.fromConstants());

        // what the route analysis says: the deadline, the move to the volley pose, how long the safe step takes with and without the volley
        AutoRoute route = route(giveUp);
        RouteAnalysis plan = route.analyze();
        double deadline = plan.effectiveGiveUpSec(plan.step(1));
        RouteAnalysis.StepInfo volleyStep = plan.step(2);
        double approachSec = volleyStep.launchSec - volleyStep.startSec;
        double safeAfterVolley = plan.step(3).durationSec;
        double safeAfterGiveUp = plan.giveUpPlan().step(3).durationSec;
        TipWait wait = new TipWait(confirmation, deadline);

        CellSpec rear = field.cell(Alliance.RED, Side.REAR);
        Pose2 pose = ShotGeometry.nominalPose(rear);              // where the camera looks at the cluster from
        Random rng = new Random(seed);
        Result out = new Result();

        double t = 0.0;
        double lastFrameAt = -1.0;
        int frameIndex = 0;
        double volleyStartsAt = Double.NaN;
        int fired = 0;
        while (t < 60.0) {
            t += DT;
            // ---- the camera
            VisionFrame frame;
            if (cameraDead) {
                frame = VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, t);
            } else {
                boolean newFrame = t - lastFrameAt >= FRAME_PERIOD - 1e-9;
                List<TagObservation> tags = new ArrayList<TagObservation>();
                if (newFrame) {
                    lastFrameAt = t;
                    frameIndex++;
                    boolean atRest = t >= tipDoneAt;
                    // while it swings up, the cluster is in view for two frames of three, and not at all in the last frame before it is at
                    // rest (so a run of frames that shows it cannot start before the tip is done); at rest it is in view all the time
                    boolean swinging = !atRest && t >= tipDoneAt - SWING_SEC && t < tipDoneAt - FRAME_PERIOD;
                    boolean visible = atRest || (swinging && frameIndex % 3 != 0);
                    if (visible && rng.nextDouble() >= dropoutRate) {
                        for (int i = 0; i < rear.tagCount(); i++) {
                            double wx = rear.tagFieldX(i);
                            double f = pose.forwardOf(wx, rear.tagRowY);
                            double l = pose.leftOf(wx, rear.tagRowY);
                            double u = rear.tagRowZ;
                            double[] tt = camera.txTyOf(f, l, u);
                            tags.add(TagObservation.withPosition(rear.baseTagId + i, tt[0], tt[1], 0.4, camera.azimuthOf(tt[0], tt[1]), f, l, u));
                        }
                    }
                }
                frame = new VisionFrame(VisionFrame.Status.OK, newFrame, 0.05, t, tags);
            }
            List<ClusterView> views = analyzer.analyze(Alliance.RED, pose.heading, frame.tags);
            tracker.update(t, frame, views, Side.REAR);

            // ---- the robot: wait, then (only if the camera confirmed the tip in time) the volley, then the safe step
            if (wait.outcome() == TipWait.Outcome.WAITING) {
                wait.update(t, frame, views);
                if (wait.volleyMayStart()) {
                    out.confirmedAt = confirmation.confirmedAtSec();
                    volleyStartsAt = t + approachSec;
                } else if (wait.gaveUp()) {
                    out.safeStepDoneAt = t + safeAfterGiveUp;
                }
            }
            if (wait.volleyMayStart() && !Double.isNaN(volleyStartsAt)) {
                // aim and spin-up, then one POLLEN per feed time
                double next = volleyStartsAt + BioBuzzConstants.Auto.AIM_SPINUP_SEC + (fired + 1) * BioBuzzConstants.Auto.TIME_PER_SHOT_SEC;
                if (fired < 4 && t >= next - 1e-9) {
                    fired++;
                    out.shotTimes.add(t);
                    if (t < tipDoneAt) {
                        out.firedBeforeTheTipWasDone++;
                    }
                    if (fired == 4) {
                        out.safeStepDoneAt = t + safeAfterVolley;
                    }
                }
            }
            if (out.safeStepDoneAt > 0.0 && t >= out.safeStepDoneAt) {
                break;
            }
        }
        out.outcome = wait.outcome();
        return out;
    }

    // ---- the partner tips at 4 s, 12 s and 20 s, or never --------------------------------------------------------------

    @Test
    public void aTipAt4SecondsIsConfirmedAfterTheSettleTimeAndOnlyThenTheVolleyStarts() {
        double tipAt = 4.0;
        Result r = run(tipAt, false, 0.0, 20.0);
        assertEquals(TipWait.Outcome.CONFIRMED, r.outcome);
        assertEquals("four POLLEN", 4, r.shotTimes.size());
        assertTrue("confirmed no earlier than the tip is done plus the settle time: " + r.confirmedAt, r.confirmedAt >= tipAt + SETTLE - 1e-9);
        assertTrue("and within a few camera frames of it: " + r.confirmedAt, r.confirmedAt <= tipAt + SETTLE + 0.2);
        assertEquals("not one POLLEN before the camera had confirmed it", 0, r.firedBeforeTheTipWasDone);
        for (double shot : r.shotTimes) {
            assertTrue("a POLLEN left at " + shot + " s, before the confirmation at " + r.confirmedAt, shot > r.confirmedAt);
        }
        assertTrue(r.safeStepDoneAt <= BUDGET);
    }

    @Test
    public void aTipAt12SecondsIsHandledTheSameWay() {
        Result r = run(12.0, false, 0.0, 20.0);
        assertEquals(TipWait.Outcome.CONFIRMED, r.outcome);
        assertEquals(4, r.shotTimes.size());
        assertTrue(r.confirmedAt >= 12.0 + SETTLE - 1e-9);
        assertEquals(0, r.firedBeforeTheTipWasDone);
        assertTrue(r.safeStepDoneAt <= BUDGET);
    }

    @Test
    public void aTipAt20SecondsComesTooLateSoTheRobotSkipsTheVolleyAndStillFinishesItsSafeStep() {
        // the deadline of the wait is a little before 20 s (the rest of the route needs time before the AUTO limit)
        Result r = run(20.0, false, 0.0, 20.0);
        assertEquals("gave up before the late tip", TipWait.Outcome.GAVE_UP, r.outcome);
        assertEquals("no POLLEN left the robot", 0, r.shotTimes.size());
        assertTrue("the safe step (leaving the wall) was done inside the AUTO limit: " + r.safeStepDoneAt, r.safeStepDoneAt > 0.0 && r.safeStepDoneAt <= BUDGET);
    }

    @Test
    public void whenThePartnerNeverTipsTheRobotSkipsTheVolleyAndStillFinishesItsSafeStep() {
        Result r = run(Double.POSITIVE_INFINITY, false, 0.0, 20.0);
        assertEquals(TipWait.Outcome.GAVE_UP, r.outcome);
        assertEquals(0, r.shotTimes.size());
        assertTrue(r.confirmedAt != r.confirmedAt);                 // NaN: never confirmed
        assertTrue("safe step done at " + r.safeStepDoneAt, r.safeStepDoneAt > 0.0 && r.safeStepDoneAt <= BUDGET);
    }

    // ---- no camera, no shot --------------------------------------------------------------------------------------------

    @Test
    public void withADeadCameraTheRobotDoesNotShootEvenThoughTheCellReallyIsUp() {
        // the partner tips at 4 s, the cell is up from then on and nothing is wrong with the HIVE; the robot has no camera
        Result r = run(4.0, true, 0.0, 20.0);
        assertEquals(TipWait.Outcome.GAVE_UP, r.outcome);
        assertEquals("no camera, no shot", 0, r.shotTimes.size());
        assertTrue(r.confirmedAt != r.confirmedAt);
        assertTrue(r.safeStepDoneAt > 0.0 && r.safeStepDoneAt <= BUDGET);
    }

    // ---- a flaky camera: whatever happens, nothing is fired before the confirmation ----------------------------------------

    @Test
    public void aCameraThatDropsFramesNeverMakesTheRobotShootBeforeTheTipIsConfirmed() {
        int confirmed = 0;
        int gaveUp = 0;
        for (int seed = 0; seed < 12; seed++) {
            for (double dropout : new double[] {0.1, 0.3, 0.6}) {
                for (double tipAt : new double[] {3.0, 10.0}) {
                    Result r = run(tipAt, false, dropout, 20.0, seed);
                    assertEquals("dropout " + dropout + ", tip at " + tipAt, 0, r.firedBeforeTheTipWasDone);
                    for (double shot : r.shotTimes) {
                        assertTrue(shot > tipAt + SETTLE);
                    }
                    if (r.outcome == TipWait.Outcome.CONFIRMED) {
                        confirmed++;
                        assertEquals(4, r.shotTimes.size());
                    } else {
                        gaveUp++;
                        assertEquals(0, r.shotTimes.size());
                    }
                    assertTrue(r.safeStepDoneAt > 0.0 && r.safeStepDoneAt <= BUDGET);
                }
            }
        }
        assertTrue("some runs confirm and some give up: " + confirmed + " / " + gaveUp, confirmed > 0 && gaveUp > 0);
    }

    // ---- the route analysis against the closed loop --------------------------------------------------------------------

    @Test
    public void theLatestPartnerTipTheAnalysisPromisesIsReallyTheLastOneTheClosedLoopCanUse() {
        for (double giveUp : new double[] {16.0, 20.0, 28.0}) {
            RouteAnalysis plan = route(giveUp).analyze();
            double latest = plan.latestPartnerTipSec();
            assertTrue(giveUp + ": " + latest, latest > 0.0 && latest < giveUp);
            // a tip a little before the latest one: confirmed, volley fired, safe step done inside the budget
            Result before = run(latest - 0.15, false, 0.0, giveUp);
            assertEquals("giveUp " + giveUp + ", tip " + (latest - 0.15), TipWait.Outcome.CONFIRMED, before.outcome);
            assertEquals(4, before.shotTimes.size());
            assertTrue("done at " + before.safeStepDoneAt + " of " + BUDGET, before.safeStepDoneAt <= BUDGET + 0.2);
            // the analysis agrees with the loop on when the route ends (within the camera's frame period and the loop's step)
            double planned = RouteAnalysis.ofWithPartnerTipAt(route(giveUp), latest - 0.15).totalSeconds();
            assertEquals("the plan and the closed loop agree on the end of the route", planned, before.safeStepDoneAt, 0.15);
            // a tip a little after it: the robot gives up and leaves the volley out
            Result after = run(latest + 0.15, false, 0.0, giveUp);
            assertEquals("giveUp " + giveUp + ", tip " + (latest + 0.15), TipWait.Outcome.GAVE_UP, after.outcome);
            assertEquals(0, after.shotTimes.size());
            assertTrue(after.safeStepDoneAt <= BUDGET + 0.2);
        }
    }

    @Test
    public void aSweepOfTipTimesNeverFiresIntoACellThatIsNotAtRestAndAlwaysFinishesInTime() {
        for (double tipAt = 0.5; tipAt <= 40.0; tipAt += 1.7) {
            Result r = run(tipAt, false, 0.0, 20.0);
            assertEquals("tip at " + tipAt, 0, r.firedBeforeTheTipWasDone);
            assertTrue("tip at " + tipAt + ": done at " + r.safeStepDoneAt, r.safeStepDoneAt > 0.0 && r.safeStepDoneAt <= BUDGET + 0.2);
            assertEquals("tip at " + tipAt, r.outcome == TipWait.Outcome.CONFIRMED ? 4 : 0, r.shotTimes.size());
            assertFalse(r.outcome == TipWait.Outcome.WAITING);
        }
    }
}
