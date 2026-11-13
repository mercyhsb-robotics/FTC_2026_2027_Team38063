package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * What the ROBOT does during the wiggle of a FLOWER pick-up, one loop at a time: the script the route runner follows once the robot has driven the
 * straight approach and stands at {@link FlowerPress#start}. It is a plain state machine with no FTC or Pedro in it, so it can be tested on a PC in a
 * closed loop against a simulated heading controller (see {@code FlowerWiggleSequenceTest}) before it ever moves a real robot.
 *
 * <p><b>How it drives.</b> The robot's POSITION never changes during the wiggle: the follower is told to HOLD the pose point where it is and only the TARGET
 * HEADING alternates between "straight at the wall" ({@link FlowerPress#start}) and "turned toward the ball" ({@link FlowerPress#pushed}). Every
 * loop the runner calls {@link #update} with the clock and the robot's heading and drives the follower to the {@link Command#hold} pose it gets back. The
 * intake motor and the two corner servos run the whole time ({@link Command#intakeOn}).
 *
 * <p>One cycle, repeated {@code press.cycles} times:
 * <ol>
 *   <li>{@link Phase#TURN_IN}: the target heading is the pushed one, until the heading is within
 *       {@code Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG} of it;</li>
 *   <li>{@link Phase#HOLD_PUSH}: stay turned in for {@code Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC}, the wheel pressed on the ball;</li>
 *   <li>{@link Phase#TURN_BACK}: the target heading is the straight one again (the same angle back, never past it), until it is within the tolerance;</li>
 *   <li>{@link Phase#HOLD_RELEASE}: stay straight for {@code Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC} so the next POLLEN drops into the ring hole.
 *       That is one POLLEN counted ({@link #pollenTaken()}).</li>
 * </ol>
 * Then {@link Phase#RETREAT}: the hold pose is the retreat pose, straight back out.
 *
 * <p><b>It stops when it should.</b> The robot may hold at most {@code Auto.POLLEN_CAPACITY} POLLEN (rule G407): when {@code pollenHeld} plus the POLLEN
 * counted reaches it, the sequence stops after that cycle (and the intake is switched off, so it cannot take a fifth). It stops early, FAIL CLOSED,
 * when a turn does not reach its heading within {@code Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC}: the robot is blocked or the controller is not working,
 * so pushing on would only grind the wheel against the ball. It first tries to turn back straight (also with a time limit) and then retreats, skipping
 * the rest of this pick-up; {@link #failed()} says so and the runner goes on with the rest of the route.
 *
 * <p>The time of the whole wiggle is estimated by {@code TimingModel#wiggleSeconds} (a half-turn is allowed {@code Auto.FLOWER_WIGGLE_TURN_SEC}); a real
 * turn ends as soon as the heading is within the tolerance, so a quick controller finishes sooner than the estimate.
 */
public final class FlowerWiggleSequence {
    /** Where in the cycle the robot is. */
    public enum Phase {
        /** Turning toward the ball. */
        TURN_IN,
        /** Turned in, the wheel pressed on the ball and spinning. */
        HOLD_PUSH,
        /** Turning back to straight. */
        TURN_BACK,
        /** Straight again, waiting for the next POLLEN to drop. */
        HOLD_RELEASE,
        /** The wiggle is over: back straight out. */
        RETREAT
    }

    /** What the runner is to do this loop. */
    public static final class Command {
        /** The pose for the follower to hold: the pose point stays put, the heading is the target heading of this phase (or the retreat pose). */
        public final Pose2 hold;
        /** True while the intake motor and the corner servos should run. */
        public final boolean intakeOn;
        /** The phase this command belongs to. */
        public final Phase phase;

        Command(Pose2 hold, boolean intakeOn, Phase phase) {
            this.hold = hold;
            this.intakeOn = intakeOn;
            this.phase = phase;
        }
    }

    private final FlowerPress press;
    private final int pollenHeldAtStart;
    private final double toleranceRad;
    private Phase phase = Phase.TURN_IN;
    private double phaseStartSec;
    private int cyclesDone;
    private boolean failed;

    /**
     * @param press            the pick-up being run (its start pose, pushed pose, cycles and retreat pose)
     * @param pollenHeldAtStart how many POLLEN the robot holds when the wiggle begins
     * @param startSec         the clock reading (seconds) at which the robot stands at the start pose and the sequence begins
     */
    public FlowerWiggleSequence(FlowerPress press, int pollenHeldAtStart, double startSec) {
        this.press = press;
        this.pollenHeldAtStart = pollenHeldAtStart;
        this.toleranceRad = Math.toRadians(BioBuzzConstants.Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG);
        this.phaseStartSec = startSec;
    }

    /** The phase the sequence is in. */
    public Phase phase() {
        return phase;
    }

    /** How many full push-and-release cycles are done. */
    public int cyclesDone() {
        return cyclesDone;
    }

    /** How many POLLEN the cycles done have taken out of the FLOWER (never more than were asked for, never more than fit in the robot). */
    public int pollenTaken() {
        int byCycles = cyclesDone / Math.max(1, press.spec.cyclesPerPollen);
        int room = Math.max(0, BioBuzzConstants.Auto.POLLEN_CAPACITY - pollenHeldAtStart);
        return Math.min(Math.min(byCycles, press.count), room);
    }

    /** True if the sequence gave up because a turn never got there (fail closed): the rest of this pick-up is skipped. */
    public boolean failed() {
        return failed;
    }

    /** True once the wiggle is over and the robot is to retreat (whether it finished, was full, or gave up). */
    public boolean finished() {
        return phase == Phase.RETREAT;
    }

    /** True if the robot cannot take another POLLEN: it holds as many as it may (G407). */
    private boolean full() {
        return pollenHeldAtStart + pollenTaken() >= BioBuzzConstants.Auto.POLLEN_CAPACITY;
    }

    /**
     * Advances the sequence and says what to do. Call it every loop.
     *
     * @param nowSec     the clock, in seconds (any origin, it only has to run forward)
     * @param headingRad the robot's heading now, in radians
     */
    public Command update(double nowSec, double headingRad) {
        double waited = nowSec - phaseStartSec;
        switch (phase) {
            case TURN_IN:
                if (arrived(headingRad, press.pushed.heading)) {
                    enter(Phase.HOLD_PUSH, nowSec);
                } else if (waited >= BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC) {
                    failed = true;                       // blocked, or the heading controller is not working: turn back and leave
                    enter(Phase.TURN_BACK, nowSec);
                }
                break;
            case HOLD_PUSH:
                if (waited >= BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC) {
                    enter(Phase.TURN_BACK, nowSec);
                }
                break;
            case TURN_BACK:
                if (arrived(headingRad, press.start.heading)) {
                    if (failed) {
                        enter(Phase.RETREAT, nowSec);
                    } else {
                        cyclesDone++;                    // the push and the release made one cycle: one POLLEN is out
                        enter(Phase.HOLD_RELEASE, nowSec);
                    }
                } else if (waited >= BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC) {
                    failed = true;
                    enter(Phase.RETREAT, nowSec);        // it cannot even straighten up: back out the way it came
                }
                break;
            case HOLD_RELEASE:
                if (waited >= BioBuzzConstants.Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC) {
                    if (cyclesDone >= press.cycles || full()) {
                        enter(Phase.RETREAT, nowSec);
                    } else {
                        enter(Phase.TURN_IN, nowSec);
                    }
                }
                break;
            default:
                break;
        }
        return command();
    }

    private void enter(Phase next, double nowSec) {
        phase = next;
        phaseStartSec = nowSec;
    }

    private boolean arrived(double headingRad, double targetRad) {
        return Math.abs(Angles.error(headingRad, targetRad)) <= toleranceRad;
    }

    /** The command for the phase the sequence is in now. */
    public Command command() {
        boolean intake = phase != Phase.RETREAT && !full() && !failed;
        switch (phase) {
            case TURN_IN:
            case HOLD_PUSH:
                return new Command(press.pushed, intake, phase);
            case RETREAT:
                return new Command(press.retreat, false, phase);
            default:
                return new Command(press.start, intake, phase);
        }
    }
}
