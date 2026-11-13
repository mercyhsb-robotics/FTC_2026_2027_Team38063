package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The camera-only confirmation that a HIVE has FULLY tipped ({@link TipConfirmation}) and the deadline wrapper the WAIT_FOR_TIP step
 * uses ({@link TipWait}): the robot shoots at the cell that comes up after a tip only when its camera has seen that cell's tags UP,
 * steady, for the settle time, and never because a time has passed. No camera means no confirmation, and then the volley is skipped.
 *
 * <p>How it works: no robot and no physics. A tiny driver feeds hand-made camera frames (25 per second, like the Limelight) and
 * hand-made cluster views into the REAL {@link HiveTracker}, {@link TipConfirmation} and {@link TipWait}. The red alliance's HIVE
 * starts with its AUDIENCE cell up, so the cell that comes up with the partner's tip, and the one these tests wait for, is REAR.
 * The closed-loop version with a simulated HIVE and the real camera model is {@code PartnerTipSimulationTest}.
 */
public class TipConfirmationTest {
    private static final double SETTLE = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
    /** Time between two new camera frames: 25 frames per second. */
    private static final double FRAME = 0.04;

    private final FieldModel field = new FieldModel();

    /** The tracker, the confirmation of the REAR cell and a clock, wired the way the autonomous loop wires them. */
    private final class Rig {
        final HiveTracker tracker = new HiveTracker(field, Alliance.RED, HiveTracker.Config.fromConstants());
        final TipConfirmation confirmation = new TipConfirmation(tracker, Side.REAR, TipConfirmation.Config.fromConstants());
        double t;

        /** One loop with a usable camera frame that is new and shows the given views. */
        TipConfirmation.State frame(List<ClusterView> views) {
            t += FRAME;
            VisionFrame f = new VisionFrame(VisionFrame.Status.OK, true, 0.05, t, Collections.<TagObservation>emptyList());
            tracker.update(t, f, views, Side.REAR);
            return confirmation.update(t, f, views);
        }

        /** One loop in which the camera delivers the given status instead of a picture. */
        TipConfirmation.State camera(VisionFrame.Status status) {
            t += FRAME;
            VisionFrame f = VisionFrame.unavailable(status, t);
            tracker.update(t, f, Collections.<ClusterView>emptyList(), Side.REAR);
            return confirmation.update(t, f, Collections.<ClusterView>emptyList());
        }

        /** Runs {@code seconds} of frames that all show the same views; returns the state after the last one. */
        TipConfirmation.State run(double seconds, List<ClusterView> views) {
            TipConfirmation.State s = confirmation.state();
            for (double end = t + seconds; t < end - 1e-9; ) {
                s = frame(views);
            }
            return s;
        }
    }

    private ClusterView cluster(Side side, ClusterView.Orientation orientation, boolean geometry) {
        CellSpec cell = field.cell(Alliance.RED, side);
        return new ClusterView(cell, 4, orientation, geometry, 10.0, 0.0, 34.0, 0.0, geometry, 0.0, 0.0);
    }

    private List<ClusterView> rearUp() {
        return Collections.singletonList(cluster(Side.REAR, ClusterView.Orientation.UP, true));
    }

    // ---- the settle time -------------------------------------------------------------------------------------------

    @Test
    public void upEvidenceForLessThanTheSettleTimeIsNotEnough() {
        Rig r = new Rig();
        // just under the settle time (0.5 s) of a steady cluster: not confirmed, however clear the picture is
        assertEquals(TipConfirmation.State.WAITING, r.run(SETTLE - 2 * FRAME, rearUp()));
        assertFalse(r.confirmation.confirmed());
        assertTrue(Double.isNaN(r.confirmation.confirmedAtSec()));
        // a few frames later the settle time is over and the tip counts as complete
        assertEquals(TipConfirmation.State.CONFIRMED, r.run(3 * FRAME, rearUp()));
        assertTrue(r.confirmation.confirmed());
        // and not before: the confirmation came SETTLE after the first frame of the run (one frame of slack for the 25 Hz clock)
        assertEquals(SETTLE, r.confirmation.confirmedAtSec() - FRAME, FRAME + 1e-9);
        assertTrue("confirmed no earlier than the settle time after the first sighting", r.confirmation.confirmedAtSec() - FRAME >= SETTLE - 1e-9);
    }

    @Test
    public void aLongerSettleTimeWaitsLonger() {
        TipConfirmation.Config slow = new TipConfirmation.Config();
        slow.settleSec = 2.0;
        HiveTracker tracker = new HiveTracker(field, Alliance.RED, HiveTracker.Config.fromConstants());
        TipConfirmation c = new TipConfirmation(tracker, Side.REAR, slow);
        double t = 0;
        for (int i = 0; i < 40; i++) {                                  // 1.6 s
            t += FRAME;
            VisionFrame f = new VisionFrame(VisionFrame.Status.OK, true, 0.05, t, Collections.<TagObservation>emptyList());
            tracker.update(t, f, rearUp(), Side.REAR);
            assertEquals(TipConfirmation.State.WAITING, c.update(t, f, rearUp()));
        }
        assertTrue(c.why(t).contains("steady for"));
    }

    @Test
    public void aSingleMissingFrameStartsTheSettleTimeAgain() {
        Rig r = new Rig();
        assertEquals(TipConfirmation.State.WAITING, r.run(SETTLE - 3 * FRAME, rearUp()));
        // the HIVE rocks: one frame without the cluster
        assertEquals(TipConfirmation.State.WAITING, r.frame(Collections.<ClusterView>emptyList()));
        assertEquals("the run is broken", 0.0, r.confirmation.seenSteadySec(r.t), 1e-9);
        // another stretch just under the settle time is still not enough, although the total since the first sighting is more than 0.5 s
        assertEquals(TipConfirmation.State.WAITING, r.run(SETTLE - 3 * FRAME, rearUp()));
        assertFalse(r.confirmation.confirmed());
        // a full settle time after the blink it is
        assertEquals(TipConfirmation.State.CONFIRMED, r.run(4 * FRAME, rearUp()));
    }

    @Test
    public void aRepeatedFrameNeitherStartsNorBreaksTheRun() {
        Rig r = new Rig();
        r.run(0.2, rearUp());
        double steady = r.confirmation.seenSteadySec(r.t);
        // the loop runs faster than the camera: a repeat of the old frame (newFrame false) changes nothing
        r.t += 0.01;
        VisionFrame repeat = new VisionFrame(VisionFrame.Status.OK, false, 0.06, r.t, Collections.<TagObservation>emptyList());
        r.tracker.update(r.t, repeat, Collections.<ClusterView>emptyList(), Side.REAR);
        assertEquals(TipConfirmation.State.WAITING, r.confirmation.update(r.t, repeat, Collections.<ClusterView>emptyList()));
        assertEquals(steady + 0.01, r.confirmation.seenSteadySec(r.t), 1e-9);
    }

    // ---- what counts as seeing the cell UP -----------------------------------------------------------------------

    @Test
    public void aClusterThatReadsDownOrHasNoGeometryNeverConfirms() {
        Rig down = new Rig();
        assertEquals(TipConfirmation.State.WAITING, down.run(3.0, Collections.singletonList(cluster(Side.REAR, ClusterView.Orientation.DOWN, true))));
        Rig unknown = new Rig();
        assertEquals(TipConfirmation.State.WAITING, unknown.run(3.0, Collections.singletonList(cluster(Side.REAR, ClusterView.Orientation.UNKNOWN, false))));
        // UP but only a bearing, no range or position (a cluster seen from too far or with too few tags): not "a full geometry"
        Rig partial = new Rig();
        assertEquals(TipConfirmation.State.WAITING, partial.run(3.0, Collections.singletonList(cluster(Side.REAR, ClusterView.Orientation.UP, false))));
        assertTrue(partial.confirmation.why(partial.t), partial.confirmation.why(partial.t).contains("has not yet seen"));
    }

    @Test
    public void theOtherCellsClusterDoesNotCount() {
        // the old cell (AUDIENCE) still reads UP: that is the cell that is about to tip, not the one the robot waits for
        Rig r = new Rig();
        assertEquals(TipConfirmation.State.WAITING, r.run(3.0, Collections.singletonList(cluster(Side.AUDIENCE, ClusterView.Orientation.UP, true))));
    }

    // ---- no camera, no confirmation ---------------------------------------------------------------------------

    @Test
    public void everyKindOfDeadCameraNeverConfirmsAndSaysSo() {
        for (VisionFrame.Status status : VisionFrame.Status.values()) {
            if (status == VisionFrame.Status.OK) {
                continue;
            }
            Rig r = new Rig();
            TipConfirmation.State s = TipConfirmation.State.WAITING;
            for (int i = 0; i < 1500; i++) {                                // 60 s
                s = r.camera(status);
            }
            assertEquals(status.name(), TipConfirmation.State.NO_CAMERA, s);
            assertFalse(status.name(), r.confirmation.confirmed());
            assertTrue(status.name(), r.confirmation.why(r.t).contains("no camera"));
        }
    }

    @Test
    public void aCameraThatDiesLosesTheSettleClockAndNeedsAFreshStretch() {
        Rig r = new Rig();
        r.run(SETTLE - 3 * FRAME, rearUp());
        assertEquals(TipConfirmation.State.NO_CAMERA, r.camera(VisionFrame.Status.STALLED));
        assertEquals(0.0, r.confirmation.seenSteadySec(r.t), 1e-9);
        assertTrue(r.confirmation.why(r.t), r.confirmation.why(r.t).contains("stopped delivering frames"));
        // it comes back: the old sighting does not count towards the settle time
        assertEquals(TipConfirmation.State.WAITING, r.run(SETTLE - 3 * FRAME, rearUp()));
        assertEquals(TipConfirmation.State.CONFIRMED, r.run(4 * FRAME, rearUp()));
    }

    // ---- the driver and a guess do not confirm ---------------------------------------------------------------------

    @Test
    public void theDriversButtonAndTheAssumeTheOtherSideFlipDoNotCountInAuto() {
        // the driver says REAR is up: the tracker is "confirmed" for the TeleOp, but not by the camera
        Rig driver = new Rig();
        driver.tracker.forceUp(Side.REAR);
        assertTrue(driver.tracker.confirmed());
        assertFalse(driver.tracker.cameraConfirmed());
        // the camera sees nothing at all for a long time: nothing is confirmed
        assertEquals(TipConfirmation.State.WAITING, driver.run(10.0, Collections.<ClusterView>emptyList()));
        assertFalse(driver.confirmation.confirmed());
        assertFalse(driver.tracker.cameraConfirmed());
        // "the tags were never seen, assume the other side": a guess, unconfirmed
        Rig guess = new Rig();
        guess.tracker.flipBecauseUnseen();
        assertEquals(Side.REAR, guess.tracker.upSide());
        assertFalse(guess.tracker.confirmed());
        assertFalse(guess.tracker.cameraConfirmed());
        assertEquals(TipConfirmation.State.WAITING, guess.run(10.0, Collections.<ClusterView>emptyList()));
        assertFalse(guess.confirmation.confirmed());
        assertTrue(guess.confirmation.why(guess.t), guess.confirmation.why(guess.t).contains("has not yet seen"));
    }

    @Test
    public void theCameraAgreeingWithTheDriverIsConfirmationByTheCamera() {
        // the driver forced the right side AND the camera then sees the cluster steady: that is camera evidence
        Rig r = new Rig();
        r.tracker.forceUp(Side.REAR);
        assertEquals(TipConfirmation.State.CONFIRMED, r.run(SETTLE + 3 * FRAME, rearUp()));
        assertTrue(r.tracker.cameraConfirmed());
    }

    @Test
    public void aDriversWordPlusAContradictoryPictureIsNotCameraEvidence() {
        // The driver forced REAR, and the camera shows the REAR cluster UP but ALSO the AUDIENCE cluster UP: both cells cannot be up,
        // so the tracker gets no usable evidence and the belief stays the driver's word, not the camera's. The REAR cluster alone
        // reads UP with a full geometry for as long as you like, and still nothing is confirmed: without the "from the camera"
        // condition this robot would shoot on the driver's word (nobody drives in AUTO, and a guess is not a tip).
        List<ClusterView> contradictory = new ArrayList<ClusterView>();
        contradictory.add(cluster(Side.REAR, ClusterView.Orientation.UP, true));
        contradictory.add(cluster(Side.AUDIENCE, ClusterView.Orientation.UP, true));
        Rig r = new Rig();
        r.tracker.forceUp(Side.REAR);
        assertEquals(TipConfirmation.State.WAITING, r.run(10.0, contradictory));
        assertTrue("the tracker still believes the driver", r.tracker.confirmed() && r.tracker.upSide() == Side.REAR);
        assertFalse(r.tracker.cameraConfirmed());
        assertFalse(r.confirmation.confirmed());
        assertTrue(r.confirmation.why(r.t), r.confirmation.why(r.t).contains("does not yet believe, from the camera"));
        // as soon as the picture is clear the camera agrees and the settle time starts counting
        assertEquals(TipConfirmation.State.CONFIRMED, r.run(SETTLE + 3 * FRAME, rearUp()));
    }

    @Test
    public void theTrackerKnowsWhenItsBeliefCameFromTheCamera() {
        Rig r = new Rig();
        assertFalse("the match-start belief has not been seen yet", r.tracker.cameraConfirmed());
        // the camera shows the REAR cluster UP: after a few frames the belief changes by vision and is camera-confirmed
        r.run(0.5, rearUp());
        assertEquals(Side.REAR, r.tracker.upSide());
        assertTrue(r.tracker.cameraConfirmed());
        assertTrue(r.tracker.confirmed());
        // the driver's toggle clears it again, the camera has to see it again
        r.tracker.toggle();
        assertFalse(r.tracker.cameraConfirmed());
        // and a belief restored from a saved state (reset unconfirmed) is not camera-confirmed either
        r.tracker.reset(Side.REAR, true);
        assertFalse(r.tracker.cameraConfirmed());
    }

    @Test
    public void tagsThatVanishAfterASteadySightingClearTheCameraConfirmation() {
        // the cluster is seen steady and then all its tags vanish with a healthy camera: the tracker suspects a tip and flips, unconfirmed
        Rig r = new Rig();
        r.run(1.0, rearUp());
        assertTrue(r.tracker.cameraConfirmed());
        r.run(BioBuzzConstants.Vision.HIVE_LOST_SEC + 0.2, Collections.<ClusterView>emptyList());
        assertEquals("suspected tip: the other cell is believed up now", Side.AUDIENCE, r.tracker.upSide());
        assertFalse(r.tracker.cameraConfirmed());
        assertFalse(r.tracker.confirmed());
    }

    // ---- latching and the spill rule ---------------------------------------------------------------------------

    @Test
    public void onceConfirmedItStaysConfirmed() {
        Rig r = new Rig();
        r.run(SETTLE + 3 * FRAME, rearUp());
        double at = r.confirmation.confirmedAtSec();
        assertTrue(r.confirmation.confirmed());
        // the volley's own shot gate watches the cluster while it fires; this class does not un-confirm
        r.run(2.0, Collections.<ClusterView>emptyList());
        r.camera(VisionFrame.Status.NOT_CONNECTED);
        assertTrue(r.confirmation.confirmed());
        assertEquals(at, r.confirmation.confirmedAtSec(), 0.0);
        assertEquals("", r.confirmation.why(r.t));
    }

    @Test
    public void theSpillZoneIsForbiddenUntilTheConfirmedTimePlusTheClearTime() {
        Rig r = new Rig();
        assertTrue("not confirmed: the robot must stay out", Double.isNaN(r.confirmation.spillClearAtSec()));
        r.run(SETTLE + 3 * FRAME, rearUp());
        assertEquals(r.confirmation.confirmedAtSec() + BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC, r.confirmation.spillClearAtSec(), 1e-9);
    }

    // ---- the WAIT_FOR_TIP wrapper: a deadline can only END the wait, never allow a shot ----------------------------------

    private TipWait wait(Rig r, double giveUpSec) {
        return new TipWait(r.confirmation, giveUpSec);
    }

    private TipWait.Outcome loop(Rig r, TipWait w, List<ClusterView> views) {
        r.t += FRAME;
        VisionFrame f = new VisionFrame(VisionFrame.Status.OK, true, 0.05, r.t, Collections.<TagObservation>emptyList());
        r.tracker.update(r.t, f, views, Side.REAR);
        return w.update(r.t, f, views);
    }

    @Test
    public void theWaitEndsConfirmedWhenTheCameraConfirmsInTime() {
        Rig r = new Rig();
        TipWait w = wait(r, 20.0);
        for (int i = 0; i < 200; i++) {                                    // 8 s of nothing: the partner has not tipped yet
            assertEquals(TipWait.Outcome.WAITING, loop(r, w, Collections.<ClusterView>emptyList()));
            assertFalse(w.volleyMayStart());
        }
        TipWait.Outcome o = TipWait.Outcome.WAITING;
        for (int i = 0; i < 40 && o == TipWait.Outcome.WAITING; i++) {     // the cell comes up
            o = loop(r, w, rearUp());
        }
        assertEquals(TipWait.Outcome.CONFIRMED, o);
        assertTrue(w.volleyMayStart());
        assertFalse(w.gaveUp());
        assertEquals(w.endedAtSec(), r.confirmation.confirmedAtSec(), 1e-9);
        assertEquals("", w.why(r.t));
        // latched
        assertEquals(TipWait.Outcome.CONFIRMED, loop(r, w, Collections.<ClusterView>emptyList()));
    }

    @Test
    public void theWaitGivesUpAtItsDeadlineWhenTheTipNeverComes() {
        Rig r = new Rig();
        TipWait w = wait(r, 20.0);
        TipWait.Outcome o = TipWait.Outcome.WAITING;
        while (r.t < 25.0) {
            o = loop(r, w, Collections.<ClusterView>emptyList());
            if (r.t < 20.0 - FRAME) {
                assertEquals("waits until the deadline on the route clock", TipWait.Outcome.WAITING, o);
            }
        }
        assertEquals(TipWait.Outcome.GAVE_UP, o);
        assertTrue(w.gaveUp());
        assertFalse("a time never lets the robot shoot, it only ends the wait", w.volleyMayStart());
        assertEquals(20.0, w.endedAtSec(), FRAME);
        assertTrue(w.endedAtSec() >= 20.0);
        assertTrue(w.why(r.t), w.why(r.t).contains("gave up at 20."));
        assertTrue(w.why(r.t), w.why(r.t).contains("has not yet seen"));
    }

    @Test
    public void aTipConfirmedAfterTheDeadlineDoesNotCount() {
        // the cluster only appears at 12 s but the robot has to give up at 12.2 s: the settle time does not fit, so it gives up
        Rig r = new Rig();
        TipWait w = wait(r, 12.2);
        while (r.t < 12.0) {
            loop(r, w, Collections.<ClusterView>emptyList());
        }
        TipWait.Outcome o = TipWait.Outcome.WAITING;
        while (r.t < 14.0) {
            o = loop(r, w, rearUp());
        }
        assertEquals(TipWait.Outcome.GAVE_UP, o);
        assertFalse(w.volleyMayStart());
        assertEquals("a later confirmation does not undo it", TipWait.Outcome.GAVE_UP, loop(r, w, rearUp()));
    }

    @Test
    public void withNoCameraTheWaitRunsOutAtTheDeadlineAndTheVolleyIsSkipped() {
        Rig r = new Rig();
        TipWait w = wait(r, 15.0);
        TipWait.Outcome o = TipWait.Outcome.WAITING;
        while (r.t < 20.0) {
            r.t += FRAME;
            VisionFrame dead = VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, r.t);
            r.tracker.update(r.t, dead, Collections.<ClusterView>emptyList(), Side.REAR);
            o = w.update(r.t, dead, Collections.<ClusterView>emptyList());
        }
        assertEquals(TipWait.Outcome.GAVE_UP, o);
        assertFalse(w.volleyMayStart());
        assertEquals(TipConfirmation.State.NO_CAMERA, r.confirmation.state());
        assertTrue(w.why(r.t), w.why(r.t).contains("no camera"));
    }

    @Test
    public void aDeadlineThatHasPassedAlreadyMeansNoWaitingAtAll() {
        Rig r = new Rig();
        TipWait w = wait(r, 0.0);
        assertEquals(TipWait.Outcome.GAVE_UP, loop(r, w, rearUp()));
        assertFalse(w.volleyMayStart());
    }

    @Test
    public void everyOutcomeIsReportedOnceAndNeverChanges() {
        List<TipWait.Outcome> seen = new ArrayList<TipWait.Outcome>();
        Rig r = new Rig();
        TipWait w = wait(r, 2.0);
        for (int i = 0; i < 100; i++) {
            TipWait.Outcome o = loop(r, w, Collections.<ClusterView>emptyList());
            if (seen.isEmpty() || seen.get(seen.size() - 1) != o) {
                seen.add(o);
            }
        }
        assertEquals(java.util.Arrays.asList(TipWait.Outcome.WAITING, TipWait.Outcome.GAVE_UP), seen);
    }
}
