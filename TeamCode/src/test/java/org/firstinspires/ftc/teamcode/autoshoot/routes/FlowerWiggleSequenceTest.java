package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.routes.FlowerWiggleSequence.Phase;

/**
 * The robot-side script of the wiggle ({@link FlowerWiggleSequence}) run in a CLOSED LOOP against a simulated heading controller, the way the follower
 * would turn the robot: every simulated 10 ms the sequence is told the clock and the heading and answers with the pose to hold, and the simulated
 * robot turns toward the heading it was given. The real footprint is measured at every step and checked against the rules of the pick-up, so a script
 * (or a controller) that swings the robot into the FLOWER fails here, on a PC, and not on the field.
 *
 * <p>Three controllers are tried: a critically damped one (arrives without overshoot, the best a heading loop can do), an underdamped one (overshoots by
 * about 5 % of the turn, what a hot gain gives) and ones that are stuck (a blocked robot, or a controller that does nothing).
 */
public class FlowerWiggleSequenceTest {
    private static final double DT = 0.01;
    private static final double WN = 14.0;                 // natural frequency of the simulated heading loop, rad/s
    private static final AllianceField RED = new AllianceField(new FieldModel(), Alliance.RED);
    private static final FlowerWiggle.Spec SPEC = WiggleTestRobot.spec();

    private static FlowerPress press(IntakeWheel wheel, int count) {
        return FlowerPress.plan(RED, SPEC, 0, wheel, wheel, count, BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN);
    }

    /** A simulated heading loop: a second-order system with damping ratio zeta, which may be told to freeze (a blocked robot). */
    private static final class Heading {
        double h;
        double v;
        final double zeta;
        /** If not null, the heading stops moving when it has turned this far (radians, from the start heading). */
        Double blockedAfter;
        final double origin;
        final boolean dead;

        Heading(double h0, double zeta, boolean dead) {
            this.h = h0;
            this.origin = h0;
            this.zeta = zeta;
            this.dead = dead;
        }

        void step(double target) {
            if (dead) {
                return;
            }
            double e = Angles.error(h, target);
            v += (WN * WN * e - 2.0 * zeta * WN * v) * DT;
            double next = h + v * DT;
            if (blockedAfter != null && Math.abs(Angles.error(origin, next)) > blockedAfter) {
                v = 0.0;
                return;
            }
            h = next;
        }
    }

    /** What happened in one simulated run. */
    private static final class Run {
        double seconds;
        double worstChassisGapToPlate = Double.POSITIVE_INFINITY;
        double worstWheelGapToStopLine = Double.POSITIVE_INFINITY;
        double worstOtherWheelGap = Double.POSITIVE_INFINITY;
        double worstWallGap = Double.POSITIVE_INFINITY;
        double minTurnedAmount = Double.POSITIVE_INFINITY;     // signed amount turned from the start toward the ball (negative = past upright)
        double maxTurnedAmount = Double.NEGATIVE_INFINITY;
        boolean intakeOnWhileFull;
        boolean timedOut;
        FlowerWiggleSequence sequence;
    }

    private static Run run(FlowerPress p, int heldAtStart, Heading heading) {
        FlowerWiggleSequence seq = new FlowerWiggleSequence(p, heldAtStart, 0.0);
        Run r = new Run();
        r.sequence = seq;
        Rect plate = RED.flowerPlate(p.flower);
        Rect stopZone = RED.flowerPressZone(p.flower);
        Rect plateKeepOut = RED.flowerKeepOut(p.flower);
        double dir = Math.signum(p.turnRad());
        double t = 0.0;
        while (!seq.finished() && t < 30.0) {
            FlowerWiggleSequence.Command c = seq.update(t, heading.h);
            // the pose point is held where it is: only the heading changes
            if (c.phase != Phase.RETREAT) {
                assertEquals("the pose point never moves during the wiggle", 0.0, p.start.distanceTo(c.hold), 1e-12);
            }
            heading.step(c.hold.heading);
            Pose2 now = new Pose2(p.start.x, p.start.y, heading.h);
            r.worstChassisGapToPlate = Math.min(r.worstChassisGapToPlate, Footprint.chassis(now, p.spec.intakeReachIn).signedGap(plate));
            r.worstWheelGapToStopLine = Math.min(r.worstWheelGapToStopLine, Footprint.wheelDiscGap(now, p.wheel, stopZone));
            r.worstOtherWheelGap = Math.min(r.worstOtherWheelGap, Footprint.wheelDiscGap(now, p.wheel.other(), plateKeepOut));
            r.worstWallGap = Math.min(r.worstWallGap, -Footprint.of(now).signedOvershoot(RED.fieldRect()));
            double turned = dir * Angles.error(p.start.heading, heading.h);
            r.minTurnedAmount = Math.min(r.minTurnedAmount, turned);
            r.maxTurnedAmount = Math.max(r.maxTurnedAmount, turned);
            r.intakeOnWhileFull |= c.intakeOn && heldAtStart + seq.pollenTaken() >= BioBuzzConstants.Auto.POLLEN_CAPACITY;
            t += DT;
        }
        r.seconds = t;
        r.timedOut = !seq.finished();
        return r;
    }

    // ---- a good controller ---------------------------------------------------------------------------------------------

    @Test
    public void aCriticallyDampedHeadingLoopRunsEveryCycleWithoutBreakingARule() {
        for (IntakeWheel w : IntakeWheel.physical()) {
            FlowerPress p = press(w, 3);
            Run r = run(p, 1, new Heading(p.start.heading, 1.0, false));
            String where = w.toString();
            assertFalse(where, r.timedOut);
            assertFalse(where, r.sequence.failed());
            assertEquals(where + " every cycle done", 3, r.sequence.cyclesDone());
            assertEquals(where + " every POLLEN counted", 3, r.sequence.pollenTaken());
            assertEquals(where, Phase.RETREAT, r.sequence.phase());
            // the real footprint, measured at every step of the loop: never in the plate, never nearer the wall than the ball's center line, never through the wall
            assertTrue(where + " chassis off the plate itself: " + r.worstChassisGapToPlate, r.worstChassisGapToPlate >= BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN - 1e-6);
            assertTrue(where + " wheel short of the stop line: " + r.worstWheelGapToStopLine, r.worstWheelGapToStopLine >= -1e-3);
            assertTrue(where + " the other wheel off the plate: " + r.worstOtherWheelGap, r.worstOtherWheelGap >= 0.0);
            assertTrue(where + " clear of the wall: " + r.worstWallGap, r.worstWallGap >= 0.0);
            // the heading stays between straight and turned-in: it is never turned past upright on the way back
            assertTrue(where + " never past upright: " + Math.toDegrees(r.minTurnedAmount), r.minTurnedAmount >= -1e-4);
            assertTrue(where + " never farther in than the derived turn: " + Math.toDegrees(r.maxTurnedAmount), r.maxTurnedAmount <= Math.abs(p.turnRad()) + 1e-4);
            // and it did turn all the way in (it really pressed)
            assertTrue(where, r.maxTurnedAmount >= Math.abs(p.turnRad()) - Math.toRadians(BioBuzzConstants.Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG) - 1e-9);
        }
    }

    @Test
    public void theSimulatedTimeIsNoLongerThanTheEstimateTheRouteIsPlannedWith() {
        FlowerPress p = press(IntakeWheel.LEFT, 4);
        Run r = run(p, 0, new Heading(p.start.heading, 1.0, false));
        double estimate = WiggleTestRobot.timing().wiggleSeconds(p);
        assertEquals(4, r.sequence.pollenTaken());
        assertTrue("simulated " + r.seconds + " s against the estimate " + estimate + " s", r.seconds <= estimate + 1e-9);
        // the holds alone are a floor: the turns take time too
        double holds = 4 * (BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC + BioBuzzConstants.Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC);
        assertTrue(r.seconds + " vs " + holds, r.seconds > holds);
    }

    @Test
    public void anOvershootingControllerStillKeepsTheChassisOffThePlateItself() {
        // underdamped (damping ratio 0.7): it overshoots the pushed heading and the straight heading by about 5 % of the turn. The clearance (0.5 in) is
        // what absorbs that.
        for (IntakeWheel w : IntakeWheel.physical()) {
            FlowerPress p = press(w, 3);
            Run r = run(p, 1, new Heading(p.start.heading, 0.7, false));
            assertFalse(w.toString(), r.timedOut);
            assertEquals(3, r.sequence.cyclesDone());
            assertTrue(w + " the chassis never touches the plate: " + r.worstChassisGapToPlate, r.worstChassisGapToPlate > 0.0);
            assertTrue(w + " nothing through the wall: " + r.worstWallGap, r.worstWallGap >= 0.0);
            // the overshoot is real (so this test is not the critically damped one in disguise): the heading does go past upright a little
            assertTrue(w + " it overshoots past straight: " + Math.toDegrees(r.minTurnedAmount), r.minTurnedAmount < -Math.toRadians(0.2));
            // but by a few percent of the turn, not more
            assertTrue(w + " " + Math.toDegrees(r.minTurnedAmount), r.minTurnedAmount > -0.15 * Math.abs(p.turnRad()));
        }
    }

    // ---- the robot is full: G407 -----------------------------------------------------------------------------------------

    @Test
    public void theSequenceStopsAndSwitchesTheIntakeOffWhenTheRobotIsFull() {
        FlowerPress p = press(IntakeWheel.LEFT, 3);
        // the robot already holds 3 POLLEN: one cycle fills it, and the other two are never made
        Run r = run(p, 3, new Heading(p.start.heading, 1.0, false));
        assertFalse(r.timedOut);
        assertEquals("only one cycle", 1, r.sequence.cyclesDone());
        assertEquals(1, r.sequence.pollenTaken());
        assertFalse("the intake is off as soon as the robot is full", r.intakeOnWhileFull);
        assertFalse(r.sequence.failed());
        // a robot that holds 4 already makes... no POLLEN count at all and takes none
        FlowerPress q = press(IntakeWheel.LEFT, 3);
        FlowerWiggleSequence full = new FlowerWiggleSequence(q, 4, 0.0);
        assertFalse("nothing to take, so the intake never runs", full.command().intakeOn);
        assertEquals(0, full.pollenTaken());
    }

    @Test
    public void theCountNeverExceedsWhatWasAskedFor() {
        FlowerPress p = press(IntakeWheel.LEFT, 2);
        Run r = run(p, 0, new Heading(p.start.heading, 1.0, false));
        assertEquals(2, r.sequence.cyclesDone());
        assertEquals(2, r.sequence.pollenTaken());
    }

    // ---- fail closed ---------------------------------------------------------------------------------------------------------

    @Test
    public void aHeadingLoopThatDoesNothingGivesUpAfterTheTimeoutAndRetreatsWithNoPollenCounted() {
        FlowerPress p = press(IntakeWheel.LEFT, 3);
        Run r = run(p, 1, new Heading(p.start.heading, 1.0, true));
        assertFalse(r.timedOut);
        assertTrue("it gave up", r.sequence.failed());
        assertEquals(Phase.RETREAT, r.sequence.phase());
        assertEquals("no cycle was completed", 0, r.sequence.cyclesDone());
        assertEquals(0, r.sequence.pollenTaken());
        assertEquals("it gave up after the timeout and not before", BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC, r.seconds, 0.05);
        assertFalse("the intake is off once it has given up", r.sequence.command().intakeOn);
        assertEquals("and the command is the retreat pose", 0.0, p.retreat.distanceTo(r.sequence.command().hold), 1e-12);
    }

    @Test
    public void aRobotBlockedWhileTurningInGivesUpStraightensUpAndRetreats() {
        FlowerPress p = press(IntakeWheel.LEFT, 3);
        Heading stuck = new Heading(p.start.heading, 1.0, false);
        stuck.blockedAfter = 0.0;                                   // blocked at once: it cannot turn in at all, the pushed heading is never reached
        Run r = run(p, 1, stuck);
        assertTrue(r.sequence.failed());
        assertEquals(0, r.sequence.pollenTaken());
        assertTrue("two timeouts at most (turn in, turn back)", r.seconds <= 2.0 * BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC + 0.1);
        // blocked half way in: it never reaches the pushed heading, gives up on the turn in after the timeout, straightens up and retreats
        Heading half = new Heading(p.start.heading, 1.0, false);
        half.blockedAfter = Math.abs(p.turnRad()) / 2.0;
        Run r2 = run(p, 1, half);
        assertTrue(r2.sequence.failed());
        assertEquals(Phase.RETREAT, r2.sequence.phase());
        assertTrue(r2.seconds > BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC);
        assertEquals(0, r2.sequence.pollenTaken());
        assertTrue("it never turned farther than where it was blocked", r2.maxTurnedAmount <= Math.abs(p.turnRad()) / 2.0 + 1e-6);
    }

    // ---- the script itself --------------------------------------------------------------------------------------------------

    @Test
    public void theCommandsAlternateBetweenStraightAndPushedAndEndWithTheRetreat() {
        FlowerPress p = press(IntakeWheel.RIGHT, 1);
        FlowerWiggleSequence seq = new FlowerWiggleSequence(p, 0, 0.0);
        // at the start the robot is straight: the first command is to turn in
        FlowerWiggleSequence.Command c = seq.update(0.0, p.start.heading);
        assertEquals(Phase.TURN_IN, c.phase);
        assertEquals(p.pushed.heading, c.hold.heading, 1e-12);
        assertTrue(c.intakeOn);
        // once the heading is within the tolerance of the pushed heading it holds the push for the push hold
        c = seq.update(0.2, p.pushed.heading);
        assertEquals(Phase.HOLD_PUSH, c.phase);
        assertEquals(p.pushed.heading, c.hold.heading, 1e-12);
        c = seq.update(0.2 + BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC - 0.01, p.pushed.heading);
        assertEquals("still holding", Phase.HOLD_PUSH, c.phase);
        c = seq.update(0.2 + BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC + 1e-9, p.pushed.heading);
        assertEquals(Phase.TURN_BACK, c.phase);
        assertEquals("back to straight, the same angle, not past it", p.start.heading, c.hold.heading, 1e-12);
        // a heading just outside the tolerance is not yet there
        double almost = p.start.heading + Math.signum(p.turnRad()) * Math.toRadians(BioBuzzConstants.Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG + 0.5);
        assertEquals(Phase.TURN_BACK, seq.update(1.0, almost).phase);
        c = seq.update(1.1, p.start.heading);
        assertEquals(Phase.HOLD_RELEASE, c.phase);
        assertEquals("one cycle counts one POLLEN", 1, seq.pollenTaken());
        c = seq.update(1.1 + BioBuzzConstants.Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC + 1e-9, p.start.heading);
        assertEquals("that was the only cycle asked for: retreat", Phase.RETREAT, c.phase);
        assertTrue(seq.finished());
        assertEquals(p.retreat.x, c.hold.x, 1e-12);
        assertEquals(p.retreat.y, c.hold.y, 1e-12);
        assertFalse(c.intakeOn);
        assertFalse(seq.failed());
    }
}
