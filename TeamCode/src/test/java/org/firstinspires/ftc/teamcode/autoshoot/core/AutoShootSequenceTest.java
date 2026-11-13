package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Closed-loop tests of the whole hold-to-shoot sequence against a small physical simulation: a robot that follows
 * commanded paths with lag, a flywheel that dips when a ball is fed, a HIVE that tips after enough balls land in its UP
 * cell, and a camera that sees the real cluster geometry through the real camera model (and sees nothing from the wrong
 * side of the HIVE).
 *
 * <p>What 'closed loop' means: every 20 ms tick the simulated world produces measurements (pose, speed, flywheel speed,
 * a camera frame). The REAL {@link AutoShootController} turns them into commands (drive, flywheel speed, feeder). The
 * simulation applies those commands (the robot moves, the flywheel spins, a ball flies) and the result becomes the next
 * tick's measurements. Because the commands change what is measured next, the loop is 'closed', as on the real robot.
 * Only the world is fake: the decision-making code under test is the real code, wired the way the TeleOp wires it.
 *
 * <p>What the simulation deliberately simplifies: odometry is perfect (the controller is told the exact pose and
 * speed); there are no collisions, wheel slip or battery effects; the robot follows a smooth speed curve toward its
 * target; a ball leaves after every 0.45 s of feeding and dips the flywheel by 10%; the camera sees the four tags
 * exactly (unless a test adds noise or dropouts) while they are inside its field of view and the robot is on the side
 * of the HIVE whose cell is really up. So a pass here shows the logic is sound, NOT that the real robot will behave.
 * docs/START_HERE.md is the order to verify it on the robot.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*AutoShootSequenceTest"} to run
 * only this class). {@code run(25, DONE, FAULT)} advances the simulation up to 25 SIMULATED seconds (it takes only
 * milliseconds of real time) and stops early once the controller reaches one of the listed states.
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Tick / loop:</b> one 20 ms step of the simulation, like one pass of the robot's control loop.</li>
 *   <li><b>tps (ticks per second):</b> the flywheel speed unit. The motor encoder counts 'ticks' as it turns.</li>
 *   <li><b>Volley:</b> the shots fired during one button press. At most 4, because the robot may hold at most 4
 *       POLLEN (rule G407).</li>
 *   <li><b>DONE / FAULT:</b> the two end states. DONE means the volley finished; FAULT means the sequence stopped
 *       for a stated reason and waits for the driver to release the button.</li>
 *   <li><b>Belief vs truth:</b> the HiveTracker holds the robot's BELIEF about which cell is up. {@code ShootWorld.trueUp}
 *       is the simulated TRUTH. Some tests make them differ on purpose.</li>
 *   <li><b>Tip:</b> the HIVE flips over so the other cell becomes UP, after enough balls land in the UP cell.</li>
 *   <li><b>Shot gate:</b> the list of conditions that must all hold before a ball may be fed (pose trusted, cell
 *       confirmed, tags in view, in range, still, aimed, flywheel at speed). It is 'fail-closed': if anything is wrong
 *       or unknown the feeder stays off.</li>
 *   <li><b>Dropout:</b> a camera frame that arrives with no tags in it.</li>
 *   <li><b>Rule G417:</b> launch only into a cell that is UP. A ball fired at a DOWN cell hits its outside, which
 *       is a foul.</li>
 *   <li><b>Fixture:</b> the shared setup for the tests. Each test builds its own fresh {@code ShootWorld}.</li>
 * </ul>
 */
public class AutoShootSequenceTest {


    /**
     * The C1 autonomous start, written for the real robot: facing the audience wall (-90 degrees) with its FRONT edge on it,
     * so its position (center of rotation) is the front extent, 9.77 in, from the wall.
     */
    private static Pose2 c1Start() {
        return Pose2.ofDegrees(58.0, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90);
    }

    /** The B1 autonomous start, the same way: front edge on the audience wall. */
    private static Pose2 b1Start() {
        return Pose2.ofDegrees(35.0, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90);
    }

    /**
     * A RED robot at the C1 autonomous start (58, 9.77) facing the audience wall, with the HIVE in its match-start state
     * (red audience cell up), which is also what the robot believes.
     */
    private static ShootWorld redAtC1() {
        return new ShootWorld(Alliance.RED, c1Start());
    }

    /** Shorthand for the two end states the tests wait for. */
    private static final AutoShootController.State DONE = AutoShootController.State.DONE;
    private static final AutoShootController.State FAULT = AutoShootController.State.FAULT;

    // ---------------------------------------------------------------------------------------------

    /**
     * Situation: a RED robot at the C1 start, the camera healthy, 4 balls, nothing going wrong. The button is held for
     *     up to 25 simulated seconds.
     * Expected: the controller reaches DONE having counted 4 shots; the simulation agrees that 4 balls went into the UP
     *     cell, none at a DOWN cell, none badly aimed or too slow; the feeder ran below speed for at most 0.12 s per
     *     ball; the controller passed through ALIGN and FEED; the whole run took under 14 s; the hopper is empty.
     * Why: this is the 'happy path' of the whole feature. It also shows the shot gate works: a ball is fed only when
     *     the flywheel is at speed and the robot is aimed (rule G417).
     */
    @Test
    public void redFromC1ShootsAllFourWithoutEverFeedingBelowSpeed() {
        ShootWorld s = redAtC1();
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals("ended in " + s.controller.state() + " " + s.last.message, DONE, s.controller.state());
        assertEquals(4, s.controller.shotsFired());
        assertEquals(4, s.shotsIntoUpCell);
        assertEquals(0, s.shotsAtDownCell);
        assertEquals(0, s.badShots);
        // After each ball the speed dips and the gate closes within its short falling debounce: a couple of loops per ball at most.
        assertTrue("feeder ran below speed for " + s.feederBelowSpeedSec + " s", s.feederBelowSpeedSec <= 0.12 * 4);
        // The flywheel should already have been spinning when the first ball was fed: it spins up while the robot
        // drives and aligns.
        assertTrue("the flywheel was spun up during the drive, before feeding", s.flyAtFirstFeed > 0.9 * s.last.flywheelTargetTps || s.flyAtFirstFeed > 1000);
        assertTrue(s.visited.contains(AutoShootController.State.ALIGN));
        assertTrue(s.visited.contains(AutoShootController.State.FEED));
        // 14 s is a generous limit. A normal run finishes well inside it; it only catches a sequence that dawdles.
        assertTrue("took " + s.t + " s", s.t < 14.0);
        assertEquals(0, s.balls);
    }

    /**
     * Situation: a RED robot at the B1 autonomous start (35, 9.2), about 24 in from the HIVE axis, so it has to drive
     *     sideways into the shooting cone first.
     * Expected: DONE, 4 balls into the UP cell, none badly aimed.
     * Why: the sequence must handle a real drive (APPROACH), not just a robot that is already in place.
     */
    @Test
    public void redFromB1DrivesToTheCellAxisAndShootsAllFour() {
        ShootWorld s = new ShootWorld(Alliance.RED, b1Start());
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
        assertEquals(0, s.badShots);
    }

    /**
     * Situation: the red C1 run, and the same run for a BLUE robot at the half-turn image of the C1 start.
     * Expected: the blue run also ends in DONE with 4 balls into the UP cell, and the blue robot finishes within 0.5 in
     *     of the half-turn image of where the red robot finished.
     * Why: BLUE is the same code and numbers rotated half a turn. The 0.5 in allows for the robot settling a little
     *     differently; the two worlds are otherwise exact mirror images.
     */
    @Test
    public void blueGetsTheMirroredResult() {
        ShootWorld red = redAtC1();
        red.held = true;
        red.run(25, DONE, FAULT);
        ShootWorld blue = new ShootWorld(Alliance.BLUE, Alliance.BLUE.fromRed(c1Start(), BioBuzzConstants.Field.SIZE_IN));
        blue.held = true;
        blue.run(25, DONE, FAULT);
        assertEquals(DONE, blue.controller.state());
        assertEquals(4, blue.shotsIntoUpCell);
        // Where the blue robot should end up: the red robot's final pose, rotated half a turn about the field center.
        Pose2 expected = Alliance.BLUE.fromRed(red.pose(), BioBuzzConstants.Field.SIZE_IN);
        assertEquals(expected.x, blue.x, 0.5);
        assertEquals(expected.y, blue.y, 0.5);
    }

    /**
     * Situation: all four combinations of alliance (RED, BLUE) and which cell is really up (AUDIENCE, REAR), each
     *     starting from that alliance's C1 position. The robot's belief is set to match the truth but is not yet
     *     confirmed by the camera.
     * Expected: every one ends in DONE with 4 good shots, none at a DOWN cell and none badly aimed.
     * Why: this covers the far-side cells (a route around the HIVE) and the blue alliance in one sweep. The assertion
     *     messages name the alliance and side so a failure says which combination broke.
     */
    @Test
    public void everyAllianceSideAndStartingCornerWorks() {
        for (Alliance a : Alliance.values()) {
            for (Side up : Side.values()) {
                ShootWorld s = new ShootWorld(a, a.fromRed(c1Start(), BioBuzzConstants.Field.SIZE_IN), false);
                // Make this cell really up, and make the robot believe the same (not yet confirmed by the camera).
                s.trueUp = up;
                s.tracker.reset(up, false);
                s.step();
                s.held = true;
                s.run(40, DONE, FAULT);
                assertEquals(a + " " + up + ": " + s.last.message, DONE, s.controller.state());
                assertEquals(a + " " + up, 4, s.shotsIntoUpCell);
                assertEquals(a + " " + up, 0, s.shotsAtDownCell);
                assertEquals(a + " " + up, 0, s.badShots);
            }
        }
    }

    /**
     * Situation: the HIVE tips after every 2 balls that land in its UP cell. The robot starts at C1 and the button is
     *     held for up to 60 s.
     * Expected: DONE with 4 shots counted; all 4 balls landed in a cell that was UP; at most 1 ball went toward the
     *     cell that had just tipped; the HIVE ends with AUDIENCE up again (2 balls tipped it to REAR, 2 more tipped it
     *     back); the tracker noticed the first tip; the robot finishes on the far side of the HIVE.
     * Why: after a tip the tags vanish, the feeder pauses at once and the sequence plans a new pose on the other side,
     *     keeping the shots already fired (rules G417 and G409). At most one ball may slip through because one was
     *     already being fed.
     */
    @Test
    public void aTipMidVolleyPausesTheFeederRetargetsTheOtherSideAndFinishesTheRemainingShots() {
        ShootWorld s = redAtC1();
        // Tip after every 2 good balls (the real HIVE needs about 8; 2 keeps the test short).
        s.tipAfterHits = 2;
        s.held = true;
        s.run(60, DONE, FAULT);
        assertEquals("ended in " + s.controller.state() + " " + s.last.message, DONE, s.controller.state());
        assertEquals(4, s.controller.shotsFired());
        assertEquals("all four reached an UP cell", 4, s.shotsIntoUpCell);
        assertTrue("at most one ball may have gone toward the cell that just tipped, got " + s.shotsAtDownCell,
                s.shotsAtDownCell <= 1);
        assertEquals("two balls tipped it to REAR, two more tipped it back", Side.AUDIENCE, s.trueUp);
        assertTrue("the tracker noticed the first tip", s.tracker.changeCount() >= 1);
        // y above 91 in is past the HIVE frame (which ends at 90.1 in), so the robot ended on the rear side.
        assertTrue("finished on the far side of the HIVE: y=" + s.y, s.y > 91.0);
    }

    /**
     * Situation: the HIVE was really tipped earlier, so the REAR cell is up, but the robot still believes AUDIENCE. The
     *     button is held for up to 60 s.
     * Expected: DONE with 4 good shots into the UP cell, none at a DOWN cell, and the tracker now believes REAR.
     * Why: the robot must work out it was wrong: it stands at the audience pose, never sees any tags, flips its belief
     *     once and drives around to the rear side. It must never fire at the DOWN cell on the way (rule G417).
     */
    @Test
    public void aWrongBeliefIsCorrectedWithoutFiringAtTheDownCell() {
        ShootWorld s = new ShootWorld(Alliance.RED, c1Start(), false);
        s.trueUp = Side.REAR;                      // the HIVE was tipped earlier; the robot still believes AUDIENCE
        s.step();                                  // one loop with the button up
        s.held = true;
        s.run(60, DONE, FAULT);
        assertEquals("ended in " + s.controller.state() + " " + s.last.message, DONE, s.controller.state());
        assertEquals(0, s.shotsAtDownCell);
        assertEquals(4, s.shotsIntoUpCell);
        assertEquals(Side.REAR, s.tracker.upSide());
    }

    /**
     * Situation: the camera is connected and healthy but never sees a tag (dropoutRate 1.0), from either side of the
     *     HIVE.
     * Expected: the controller ends in FAULT with the reason HIVE_NOT_VISIBLE, and no ball was fired.
     * Why: if the robot cannot confirm which cell is up, it must stop rather than fire blind (rule G417). It first
     *     tries the other side once, then gives up.
     */
    @Test
    public void ifNeitherSideShowsTags_theSequenceStopsInsteadOfFiringBlind() {
        ShootWorld s = new ShootWorld(Alliance.RED, c1Start(), false);
        s.dropoutRate = 1.0;                       // the camera is healthy but never sees a tag
        s.step();                                  // one loop with the button up
        s.held = true;
        s.run(60, DONE, FAULT);
        assertEquals(FAULT, s.controller.state());
        assertEquals(AutoShootController.Fault.HIVE_NOT_VISIBLE, s.controller.fault());
        assertEquals(0, s.shotsFired);
    }

    /**
     * Situation: the camera is not connected at all; the robot's belief (the match-start state) happens to be right.
     * Expected: DONE with 4 good shots.
     * Why: the HIVE-confirmation rule only applies when a camera is working. With no camera the robot goes on its
     *     belief, which the driver can correct with the toggle button (the 'driver's word').
     */
    @Test
    public void withNoCameraAtAllItShootsFromItsBeliefAndTheDriversWord() {
        ShootWorld s = redAtC1();
        s.cameraAlive = false;
        s.held = true;
        s.run(30, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
    }

    /**
     * Situation: the camera works and sees the UP cell, the robot reaches FEED, and then the camera stops delivering
     *     images altogether (a stalled link, not a missing camera).
     * Expected: within a moment the feeder stops and stays off; no ball is fired once the pause has begun, and none
     *     ever goes to a DOWN cell.
     * Why: the robot was relying on seeing the tags to know the cell is still up. If it can no longer see, it cannot
     *     tell whether the HIVE has started to tip, and firing into a tipping cell is a foul (rule G417). A robot
     *     that never had a camera is different (see the test above): it never saw the tags, so it goes on its belief.
     */
    @Test
    public void ifTheCameraStopsAfterItSawTheUpCellTheFeederPausesInsteadOfFiringBlind() {
        ShootWorld s = redAtC1();
        s.held = true;
        s.run(30, AutoShootController.State.FEED, FAULT);
        assertEquals("reached FEED: " + s.last.message, AutoShootController.State.FEED, s.controller.state());

        double diedAt = s.t;
        s.cameraDeadFrom = diedAt;                 // from this tick on no image arrives
        s.run(0.6);                                // the pause threshold is 0.15 s; allow a ball already moving
        int firedWhenPaused = s.shotsFired;
        assertEquals(AutoShootController.Refusal.TAGS_LOST, s.last.refusal);
        while (s.t < diedAt + 6.0 && s.controller.state() != FAULT && s.controller.state() != DONE) {
            s.step();
            assertFalse("the feeder must stay off while the camera is dead (t=" + s.t + ")", s.last.feederOn);
        }
        assertEquals("no ball may be fired while the camera is dead", firedWhenPaused, s.shotsFired);
        assertEquals(0, s.shotsAtDownCell);
    }

    /**
     * Situation: the button is held for 1 s (the sequence is under way), then released.
     * Expected: on the very next loop: state IDLE, drive mode MANUAL, feeder off, flywheel target 0.
     * Why: letting go must hand control back to the driver at once. A robot that kept driving or feeding after release
     *     would be unsafe.
     */
    @Test
    public void releasingTheButtonStopsEverythingInTheSameLoop() {
        ShootWorld s = redAtC1();
        s.held = true;
        s.run(1.0);
        assertTrue(s.controller.state() != AutoShootController.State.IDLE);
        s.held = false;
        s.step();
        assertEquals(AutoShootController.State.IDLE, s.last.state);
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertFalse(s.last.feederOn);
        // Exactly 0 (tolerance 0.0): this assumes the idle flywheel speed FLYWHEEL_IDLE_TPS is 0, 'stop completely'.
        assertEquals(0.0, s.last.flywheelTargetTps, 0.0);
    }

    /**
     * Situation: the button is held until the controller reaches FEED, the sequence runs 1.2 s more (partway through
     *     the volley), then the button is released.
     * Expected: on the next loop the feeder is off and the state is IDLE.
     * Why: the feeder is the dangerous part: it must stop in the same loop as the release.
     */
    @Test
    public void releasingDuringTheFeedStopsTheFeederImmediately() {
        ShootWorld s = redAtC1();
        s.held = true;
        s.run(25, AutoShootController.State.FEED);
        // Let it feed for 1.2 s so we are partway through the volley.
        s.run(1.2);
        s.held = false;
        s.step();
        assertFalse(s.last.feederOn);
        assertEquals(AutoShootController.State.IDLE, s.last.state);
    }

    /**
     * Situation: the robot is driving itself; after 0.3 s the driver pushes the translation stick hard (0.9).
     * Expected: within 0.5 s the controller is in FAULT with the reason DRIVER_OVERRIDE; on the next loop the drive
     *     mode is MANUAL and the feeder is off.
     * Why: the driver must always be able to take over by moving the stick.
     */
    @Test
    public void aHardStickPushCancelsTheAutomaticDrive() {
        ShootWorld s = redAtC1();
        s.held = true;
        s.run(0.3);
        // 0.9 is above the override threshold (DRIVER_OVERRIDE_STICK). It must be held for DRIVER_OVERRIDE_SEC to
        // count, so 0.5 s of simulated time is plenty.
        s.stick = 0.9;
        s.run(0.5, FAULT);
        assertEquals(FAULT, s.controller.state());
        assertEquals(AutoShootController.Fault.DRIVER_OVERRIDE, s.controller.fault());
        s.step();
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertFalse(s.last.feederOn);
    }

    /**
     * Situation: the flywheel can only ever reach half of the commanded speed (flyMaxFrac 0.5), as with a weak battery
     *     or a slipping belt.
     * Expected: the controller ends in FAULT with the reason SPINUP_TIMEOUT; the feeder never turned on; no ball was
     *     fired.
     * Why: feeding balls into a flywheel that is too slow would waste them and miss. The robot must give up instead.
     */
    @Test
    public void aFlywheelThatNeverReachesSpeedFaultsWithoutFeeding() {
        ShootWorld s = redAtC1();
        s.flyMaxFrac = 0.5;
        s.held = true;
        s.run(30, DONE, FAULT);
        assertEquals(FAULT, s.controller.state());
        assertEquals(AutoShootController.Fault.SPINUP_TIMEOUT, s.controller.fault());
        assertTrue(Double.isNaN(s.firstFeedAt));
        assertEquals(0, s.shotsFired);
    }

    /**
     * Situation: only 2 balls are in the hopper, although a volley can hold up to 4.
     * Expected: the controller reaches DONE and has counted exactly 2 shots.
     * Why: the robot cannot know how many POLLEN it holds. When no new dip is seen for a while it must decide the
     *     hopper is empty and finish, rather than feed forever. (This assumes the shipped HYBRID shot-count mode.)
     */
    @Test
    public void anEmptyHopperEndsTheVolleyWithTheShotsThatWereFired() {
        ShootWorld s = redAtC1();
        // Only two POLLEN in the hopper.
        s.balls = 2;
        s.held = true;
        s.run(30, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(2, s.controller.shotsFired());
    }

    /**
     * Situation: the odometry has not been given a trustworthy pose (poseTrusted is false) and the button is held.
     * Expected: within 0.5 s the controller is in FAULT with the reason NO_POSE, and the robot has not moved at all.
     * Why: all the driving is based on the pose. Driving on an untrusted pose could send the robot into a wall or the
     *     HIVE.
     */
    @Test
    public void withoutATrustedPoseNothingMoves() {
        ShootWorld s = redAtC1();
        s.poseTrusted = false;
        s.held = true;
        s.run(0.5, FAULT);
        assertEquals(FAULT, s.controller.state());
        assertEquals(AutoShootController.Fault.NO_POSE, s.controller.fault());
        // The robot never moved: x is still the start x (1e-9 in means exactly).
        assertEquals(58.0, s.x, 1e-9);
    }

    /**
     * Situation: a volley finishes (DONE) while the button is still held; later it is released, the hopper is refilled
     *     and the button is pressed again.
     * Expected: while the button stays held nothing restarts (still DONE, no extra shots); after release the state is
     *     IDLE; the new press fires another 4, so 8 in total, all into the UP cell.
     * Why: one press is one volley of at most 4 POLLEN (rule G407). Holding the button must not make the robot keep
     *     feeding an empty hopper.
     */
    @Test
    public void aFinishedVolleyDoesNotRestartUntilTheButtonIsReleasedAndPressedAgain() {
        ShootWorld s = redAtC1();
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        int shots = s.shotsFired;
        // Keep holding for 2 more seconds: nothing may restart.
        s.run(2.0);
        assertEquals(DONE, s.controller.state());
        assertEquals(shots, s.shotsFired);
        s.held = false;
        s.step();
        assertEquals(AutoShootController.State.IDLE, s.controller.state());
        s.balls = 4;
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(8, s.shotsFired);
        assertEquals(8, s.shotsIntoUpCell);
    }

    /**
     * Situation: the driver was already holding the shoot button when the OpMode started (it is held from the first
     *     loop). Later they release and press it again.
     * Expected: for the first second the controller stays IDLE and the robot does not move (x stays 58); after a
     *     release and a new press the volley runs and ends in DONE with 4 good shots.
     * Why: a button held through INIT must not start the robot driving the moment the match begins.
     */
    @Test
    public void aButtonAlreadyHeldWhenTheOpModeStartsDoesNotAutoStartUntilReleasedAndPressedAgain() {
        ShootWorld s = new ShootWorld(Alliance.RED, c1Start(), false);
        s.held = true;                              // the driver was holding it through INIT
        s.run(1.0);
        assertEquals(AutoShootController.State.IDLE, s.controller.state());
        // The robot has not moved (1e-9 in means exactly).
        assertEquals(58.0, s.x, 1e-9);
        s.held = false;
        s.run(0.1);
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
    }

    /**
     * Situation: the camera is imperfect: 0.4 in of random noise on every tag position, and 1 frame in 5 arrives with
     *     no tags.
     * Expected: the volley still ends in DONE with 4 good shots and none badly aimed.
     * Why: the real camera is noisy and loses tags now and then. A short dropout must not be mistaken for a tipped
     *     HIVE, and noise must not push the shots out of range or off aim.
     */
    @Test
    public void visionNoiseAndFrameDropoutsDoNotBreakTheVolley() {
        ShootWorld s = redAtC1();
        // Noise of 0.4 in on every tag, and 1 camera frame in 5 loses all its tags.
        s.visionNoiseIn = 0.4;
        s.dropoutRate = 0.2;
        s.held = true;
        s.run(40, DONE, FAULT);
        assertEquals("ended in " + s.controller.state() + " " + s.last.message, DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
        assertEquals(0, s.badShots);
    }

    /**
     * Situation: the robot has driven to the scoring pose and is about to feed.
     * Expected: the flywheel target the controller commands is within 25 tps of the shot table's speed for the robot's
     *     actual range to the aim point.
     * Why: the whole point of the table: the flywheel speed must match the real distance. If the controller used the
     *     wrong range, balls would fall short or fly long.
     */
    @Test
    public void theFlywheelTargetComesFromTheShotTableForTheActualRange() {
        ShootWorld s = redAtC1();
        s.held = true;
        s.run(25, AutoShootController.State.FEED);
        CellSpec cell = s.field.cell(Alliance.RED, Side.AUDIENCE);
        double range = ShotGeometry.rangeTo(s.pose(), s.plannerCfg.launchForwardIn, s.plannerCfg.launchLeftIn,
                cell.originX, cell.originY);
        // 25 tps is a loose bound: the controller works from the camera's range and this test from the true pose. With
        // the current table that is roughly 3 to 4 in of range (about 6 to 8 tps per inch).
        assertEquals(s.table.velocityAt(range), s.last.flywheelTargetTps, 25.0);
    }

    /**
     * Runs the standard first volley (4 balls, the 3rd tips the HIVE) with a REAL ball flight and a REAL swing: a ball takes 0.5 s to reach the cell, the
     * tip starts when the 3rd ball LANDS, and the cluster stays readable by the camera for the first half of the swing, then its tags vanish.
     * The feed pace is 0.7 s of feeding per ball, which with the flywheel recovering after each ball is about 0.9 s between balls (the
     * route planner's {@code Auto.TIME_PER_SHOT_SEC}).
     *
     * <p>The run ends when the volley is over: DONE, or the moment the controller has seen the tip and leaves FEED to plan the drive to the OTHER cell (a
     * button that is still held would take the spare there, which is what TeleOp wants; the route plan counts the volley as over at the tip).
     */
    private static ShootWorld volleyWithAFlightAndASwing(double swingSec) {
        ShootWorld s = redAtC1();
        s.tipAfterHits = 3;
        s.flightSec = BioBuzzConstants.Auto.BALL_FLIGHT_SEC;
        s.swingSec = swingSec;
        s.feedSecPerBall = 0.7;
        s.held = true;
        boolean fed = false;
        while (s.t < 30.0) {
            s.step();
            AutoShootController.State state = s.controller.state();
            fed |= state == AutoShootController.State.FEED;
            if (state == DONE || state == FAULT || (fed && (state == AutoShootController.State.PLAN || state == AutoShootController.State.APPROACH))) {
                break;
            }
        }
        return s;
    }

    /**
     * Situation: the real controller fires the standard first volley (4 balls, the 3rd tips the HIVE) at a cell that really takes 0.5 s to be reached and
     *     then swings for 1.2 s (the only swing time the project has, the guess of PartnerTipSimulationTest), and, for contrast, at a cell that swings in 0.2 s.
     * Expected: with the slow swing the camera keeps reading the cell as UP for 0.6 s after the 3rd ball lands, the 4th ball is fed 0.4 s after that landing, so it
     *     is launched into the cell while it swings down: 1 ball in the swinging cell, all 4 balls fired, the hopper empty. With the fast swing the tags are gone
     *     0.1 s after the landing, the feeder stops before the 4th ball and it stays in the robot: 3 balls fired, none into a swinging cell, 1 ball left.
     * Why: this is the evidence behind the route model. The model used to say a camera-gated volley "stops exactly at the tip" and keeps its spare; with a
     *     flight time and a swing the real controller cannot, unless the cell swings in about half a second or less. The planner now counts the spare as
     *     spent (Auto.TIP_STOP_LATENCY_SEC, TipModel.launchedBeforeTheFeederStops) and the day-one checklist asks for the real number.
     */
    @Test
    public void theSpareBallIsLaunchedIntoTheSwingingCellUnlessTheHiveSwingsFast() {
        ShootWorld slow = volleyWithAFlightAndASwing(1.2);
        assertEquals("ended in " + slow.controller.state() + " " + (slow.last == null ? "" : slow.last.message), DONE, slow.controller.state());
        assertEquals("the 3 balls that tip it landed in the UP cell", 3, slow.shotsIntoUpCell);
        assertEquals("the 4th went into the cell while it was swinging down", 1, slow.shotsIntoSwingingCell);
        assertEquals(4, slow.shotsFired);
        assertEquals("the hopper is empty: the spare is spent", 0, slow.balls);
        assertEquals("no ball at a cell that was already down", 0, slow.shotsAtDownCell);

        ShootWorld fast = volleyWithAFlightAndASwing(0.2);
        assertEquals(3, fast.shotsIntoUpCell);
        assertEquals("the feeder stopped before the 4th ball", 0, fast.shotsIntoSwingingCell);
        assertEquals(3, fast.shotsFired);
        assertEquals("the spare is kept", 1, fast.balls);
        assertEquals(0, fast.shotsAtDownCell);
    }

    /**
     * Situation: the same volley with the old simplifications (a ball lands the instant it leaves, the HIVE flips at once, 0.45 s of feeding per ball).
     * Expected: nothing changes: 3 balls into the UP cell and at most one toward the cell that just tipped, as before.
     * Why: the new flight and swing are opt-in; every other test of this class keeps its meaning.
     */
    @Test
    public void withoutAFlightOrASwingTheTipIsInstantAsBefore() {
        ShootWorld s = redAtC1();
        s.tipAfterHits = 3;
        s.held = true;
        s.run(30, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(0, s.shotsIntoSwingingCell);
        assertTrue("at most one ball toward the cell that just tipped, got " + s.shotsAtDownCell, s.shotsAtDownCell <= 1);
    }
}
