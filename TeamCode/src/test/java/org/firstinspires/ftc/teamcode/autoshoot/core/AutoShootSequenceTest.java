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
 * docs/DAY_ONE_CHECKLIST.md is the order to verify it on the robot.
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
 *   <li><b>Belief vs truth:</b> the HiveTracker holds the robot's BELIEF about which cell is up. {@code Sim.trueUp}
 *       is the simulated TRUTH. Some tests make them differ on purpose.</li>
 *   <li><b>Tip:</b> the HIVE flips over so the other cell becomes UP, after enough balls land in the UP cell.</li>
 *   <li><b>Shot gate:</b> the list of conditions that must all hold before a ball may be fed (pose trusted, cell
 *       confirmed, tags in view, in range, still, aimed, flywheel at speed). It is 'fail-closed': if anything is wrong
 *       or unknown the feeder stays off.</li>
 *   <li><b>Dropout:</b> a camera frame that arrives with no tags in it.</li>
 *   <li><b>Rule G417:</b> launch only into a cell that is UP. A ball fired at a DOWN cell hits its outside, which
 *       is a foul.</li>
 *   <li><b>Fixture:</b> the shared setup for the tests. Each test builds its own fresh {@code Sim}.</li>
 * </ul>
 */
public class AutoShootSequenceTest {

    /**
     * The simulated world plus the real controller wired exactly as the TeleOp wires it.
     *
     * <p>Each call to {@link #step()} is one 20 ms tick: build the camera frame, update the HiveTracker, run the
     * controller, then apply what it commanded to the world. Tests change the fields (for example {@code balls},
     * {@code dropoutRate}, {@code cameraAlive}) to create a situation, set {@code held} to press the button, and call
     * {@link #run}.
     */
    private static final class Sim {
        /** The real field map. */
        final FieldModel field = new FieldModel();
        /** The real shot table: flywheel speed for each range. */
        final ShotTable table = ShotTable.fromConstants();
        /** The real camera model, used by the virtual camera. */
        final CameraModel camera = CameraModel.fromConstants();
        /** The real analyzer that turns the virtual camera's tags into ClusterViews. */
        final ClusterAnalyzer analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());
        /** The planner settings the controller's planner uses (also used to score shots). */
        final ScoringPlanner.Config plannerCfg = ScoringPlanner.Config.fromConstants(table);
        /** The controller's settings, straight from BioBuzzConstants. */
        final AutoShootController.Config ctrlCfg = AutoShootController.Config.fromConstants();

        /** Which alliance the simulated robot is on. */
        final Alliance alliance;
        /** The robot's belief about which cell is UP. Tests may reset or inspect it. */
        final HiveTracker tracker;
        /** The real state machine under test. */
        final AutoShootController controller;

        // world
        /**
         * The robot's true pose (inches, radians) and its velocity: vx and vy in inches per second, omega in radians
         * per second.
         */
        double x, y, heading, vx, vy, omega;
        /** The cell that is REALLY up in the simulated HIVE (the truth, not the robot's belief). */
        Side trueUp;
        /** Balls that have landed in the current UP cell since the HIVE last tipped. */
        int hitsIntoUp;
        /** Balls needed to tip the HIVE. 1000 means it never tips during a test. (The real HIVE tips after about 8.) */
        int tipAfterHits = 1000;
        /**
         * Seconds a ball is in the air between the shooter and the cell. 0 (the default) is the old simplification: it lands the instant it leaves.
         * The real flight is about 0.5 s ({@code Auto.BALL_FLIGHT_SEC}), and a tip happens when the ball LANDS.
         */
        double flightSec = 0.0;
        /**
         * Seconds the HIVE takes to swing over once the tipping ball has landed. 0 (the default) is the old simplification: the other cell is up at once.
         * With a swing, the cluster stays readable by the camera for the first half of it, then its tags vanish, and at the end the other cell is up.
         */
        double swingSec = 0.0;
        /** Seconds of feeding between two balls. The default 0.45 is the fast pace most tests use; a test about the tip uses the slower real one. */
        double feedSecPerBall = 0.45;
        /** Landing times of the balls in the air. */
        private final List<Double> landings = new ArrayList<Double>();
        /** When the swing started (the tipping ball landed); NaN when the HIVE is not swinging. */
        private double swingStartedAt = Double.NaN;
        /** POLLEN left in the hopper. The robot may hold at most 4 (rule G407). */
        int balls = 4;
        /** The flywheel's true speed in ticks per second. */
        double flyV;
        /**
         * The fraction of the commanded speed the flywheel can actually reach: 1.0 is healthy, 0.5 is a weak flywheel.
         */
        double flyMaxFrac = 1.0;
        /** Seconds the feeder has run since the last ball left. */
        double feedProgress;
        /** False simulates a camera that is not connected. */
        boolean cameraAlive = true;
        /** From this simulated time on, the camera stops delivering images (a stalled USB link). */
        double cameraDeadFrom = Double.POSITIVE_INFINITY;
        /** Fraction of camera frames that arrive with no tags (0 = never, 1 = always). */
        double dropoutRate = 0.0;
        /** Random error, in inches, added to the measured tag positions (0 = perfect). */
        double visionNoiseIn = 0.0;
        /** Whether the odometry has been given a trustworthy start pose. */
        boolean poseTrusted = true;
        /**
         * Random numbers for noise and dropouts. A fixed seed (11) makes every run identical, so a failure can be
         * repeated and the tests cannot fail at random.
         */
        final Random rng = new Random(11);

        // bookkeeping
        /** The simulated clock in seconds. */
        double t;
        /** Balls that went into a cell that was UP (good shots). */
        int shotsIntoUpCell;
        /**
         * Balls fired while the robot faced a cell that was DOWN (a foul under rule G417). Must stay 0, or 1 right
         * after a tip.
         */
        int shotsAtDownCell;
        /**
         * Balls launched while the HIVE was swinging down: after the tipping ball landed, before the swing was over. They only exist when a test sets
         * {@link #swingSec}; with the instant tip of the other tests it stays 0.
         */
        int shotsIntoSwingingCell;
        /** Balls fired at an UP cell but too slow, outside the shot table range or badly aimed. */
        int badShots;           // fired too slow, outside the table, or badly aimed
        /** Every ball that left the robot, good or bad. */
        int shotsFired;
        /** Time the feeder first turned on (NaN = it never has). */
        double firstFeedAt = Double.NaN;
        /** Flywheel speed at that moment, in tps. */
        double flyAtFirstFeed = Double.NaN;
        /** Total seconds the feeder ran while the flywheel was below 90% of its target (should be tiny). */
        double feederBelowSpeedSec;
        /** Every controller state seen so far. */
        final Set<AutoShootController.State> visited = EnumSet.noneOf(AutoShootController.State.class);
        /** What the controller commanded on the most recent tick. */
        AutoShootController.Outputs last;
        /** Clock time of the most recent new camera frame. */
        double lastFrameAt = -1;

        // commands from the test
        /** The test 'presses' the shoot button by setting this to true and releases it with false. */
        boolean held;
        /** How far the driver pushes the translation stick (0 to 1). */
        double stick;

        /** Length of one tick, in seconds (20 ms). */
        static final double DT = 0.02;
        /** Time between new camera frames: 0.04 s, i.e. 25 frames per second. */
        static final double FRAME_PERIOD = 0.04;
        /** The robot's top driving speed along a path, in inches per second. */
        static final double MAX_SPEED = 32.0;
        /** The robot's top turn rate, in radians per second. */
        static final double MAX_OMEGA = 3.0;

        /** A primed simulation: one loop with the button up has already run. */
        Sim(Alliance alliance, Pose2 startInAllianceFrame) {
            this(alliance, startInAllianceFrame, true);
        }

        /**
         * Builds a world with the robot standing at the given pose (in its own alliance's frame), the HIVE in its
         * match-start state, and the real controller attached. {@code primed} runs one loop with the button up, like
         * the TeleOp does before START: the controller must see the button released once, or it will ignore a button
         * that was already held.
         */
        Sim(Alliance alliance, Pose2 startInAllianceFrame, boolean primed) {
            this.alliance = alliance;
            this.x = startInAllianceFrame.x;
            this.y = startInAllianceFrame.y;
            this.heading = startInAllianceFrame.heading;
            this.trueUp = field.initialUpSide(alliance);
            this.tracker = new HiveTracker(field, alliance, HiveTracker.Config.fromConstants());
            ScoringPlanner planner = new ScoringPlanner(field, plannerCfg);
            this.controller = new AutoShootController(alliance, field, planner, table, tracker, ctrlCfg);
            if (primed) {
                step();                              // one loop with the button up, as the TeleOp does before START
            }
        }

        /** The robot's true pose as a Pose2. */
        Pose2 pose() {
            return new Pose2(x, y, heading);
        }

        /** Which side of the HIVE the robot is standing on (AUDIENCE or REAR). */
        Side robotSide() {
            return HiveTracker.sideOf(pose(), field);
        }

        /**
         * The virtual camera for one tick: builds the VisionFrame the controller would get from the Limelight.
         *
         * <p>It simplifies: a tag is either seen exactly (plus optional Gaussian noise) or not seen; there is no motion
         * blur and no partial detections. Tags are only seen from the side of the HIVE whose cell is really up, because
         * from the other side the stickers face away.
         */
        VisionFrame makeFrame() {
            if (!cameraAlive) {
                return VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, t);
            }
            if (t >= cameraDeadFrom) {
                return VisionFrame.unavailable(VisionFrame.Status.STALLED, t);
            }
            // A new camera image arrives only every FRAME_PERIOD. 1e-9 s of slack stops rounding from skipping a frame.
            boolean newFrame = t - lastFrameAt >= FRAME_PERIOD - 1e-9;
            if (newFrame) {
                lastFrameAt = t;
            }
            List<TagObservation> tags = new ArrayList<TagObservation>();
            // Tags are only produced on a new frame, when the frame is not randomly dropped, and when the robot stands
            // on the side of the HIVE whose cell is really UP. While the HIVE swings the tags stay readable for the first half of the swing
            // and are gone for the second half.
            boolean swingHidesTheTags = swinging() && t >= swingStartedAt + 0.5 * swingSec;
            if (newFrame && rng.nextDouble() >= dropoutRate && robotSide() == trueUp && !swingHidesTheTags) {
                CellSpec cell = field.cell(alliance, trueUp);
                Pose2 p = pose();
                for (int i = 0; i < cell.tagCount(); i++) {
                    double wx = cell.tagFieldX(i);
                    // Where the tag is relative to the robot, plus Gaussian noise (a bell curve of standard deviation
                    // visionNoiseIn inches).
                    double f = p.forwardOf(wx, cell.tagRowY) + visionNoiseIn * rng.nextGaussian();
                    double l = p.leftOf(wx, cell.tagRowY) + visionNoiseIn * rng.nextGaussian();
                    double u = cell.tagRowZ;
                    double[] tt = camera.txTyOf(f, l, u);
                    // The tag must be inside the camera's field of view (half-angles 27 and 21 degrees: about the
                    // Limelight's 54 x 42 degrees) and in front of the lens.
                    boolean inView = Math.abs(tt[0]) <= 27.0 && Math.abs(tt[1]) <= 21.0
                            && camera.robotToCamera(f, l, u)[2] > 0;
                    if (inView) {
                        tags.add(TagObservation.withPosition(cell.baseTagId + i, tt[0], tt[1], 0.4,
                                camera.azimuthOf(tt[0], tt[1]), f, l, u));
                    }
                }
            }
            return new VisionFrame(VisionFrame.Status.OK, newFrame, 0.05, t, tags);
        }

        /** Runs one 20 ms tick: measure the world, update the tracker, run the controller, then apply its commands. */
        void step() {
            // Time moves on one tick.
            t += DT;
            landBalls();
            VisionFrame frame = makeFrame();
            List<ClusterView> views = analyzer.analyze(alliance, heading, frame.tags);
            tracker.update(t, frame, views, robotSide());

            // Hand the controller everything the real robot would measure this loop.
            AutoShootController.Inputs in = new AutoShootController.Inputs();
            in.nowSec = t;
            in.shootHeld = held;
            in.pose = pose();
            in.speedInS = Math.hypot(vx, vy);
            in.omegaRadS = omega;
            in.poseTrusted = poseTrusted;
            in.flywheelTps = flyV;
            in.frame = frame;
            in.views = views;
            in.stickMagnitude = stick;
            AutoShootController.Outputs out = controller.update(in);
            last = out;
            visited.add(out.state);

            // Apply the commands to the simulated world.
            applyDrive(out);
            applyFlywheelAndFeeder(out);
        }

        /**
         * Moves the robot the way the controller asked. FOLLOW: drive through the waypoints (full speed, slowing down
         * for the last one). HOLD: pull toward the hold pose. MANUAL: the commanded speed is zero (the driver has
         * control but is not pushing). In FOLLOW and HOLD the robot also turns toward the commanded heading.
         *
         * <p>It simplifies: speed changes follow a smooth curve (a 0.12 s time constant) instead of real mecanum
         * dynamics, and there are no collisions or wheel slip.
         */
        void applyDrive(AutoShootController.Outputs out) {
            double tvx = 0, tvy = 0, tw = 0;
            if (out.driveMode == AutoShootController.DriveMode.FOLLOW && out.holdPose != null) {
                double[] wp = nextWaypoint(out);
                double dx = wp[0] - x;
                double dy = wp[1] - y;
                double dist = Math.hypot(dx, dy);
                boolean isLast = wp == out.waypoints.get(out.waypoints.size() - 1);
                double speed = isLast ? Math.min(MAX_SPEED, 4.0 * dist) : MAX_SPEED;
                if (dist > 1e-6) {
                    tvx = speed * dx / dist;
                    tvy = speed * dy / dist;
                }
                tw = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, 5.0 * Angles.error(heading, out.holdPose.heading)));
            } else if (out.driveMode == AutoShootController.DriveMode.HOLD && out.holdPose != null) {
                double dx = out.holdPose.x - x;
                double dy = out.holdPose.y - y;
                double dist = Math.hypot(dx, dy);
                double speed = Math.min(20.0, 5.0 * dist);
                if (dist > 1e-6) {
                    tvx = speed * dx / dist;
                    tvy = speed * dy / dist;
                }
                tw = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, 6.0 * Angles.error(heading, out.holdPose.heading)));
            }
            // First-order lag: each tick the real velocity closes DT / 0.12 of the gap to the target velocity. The
            // robot cannot change speed instantly.
            double a = Math.min(1.0, DT / 0.12);
            vx += (tvx - vx) * a;
            vy += (tvy - vy) * a;
            omega += (tw - omega) * a;
            x += vx * DT;
            y += vy * DT;
            heading = Angles.normalize(heading + omega * DT);
        }

        /** Index of the waypoint the robot is currently heading for. */
        private int waypointIndex = 0;
        /** The route the index above belongs to (a new route restarts it). */
        private List<double[]> activeRoute;

        /**
         * The waypoint to head for now: moves on to the next one once the robot is within 2 in of the current one, and
         * starts again at the first when the controller sends a new route.
         */
        double[] nextWaypoint(AutoShootController.Outputs out) {
            if (activeRoute != out.waypoints) {
                activeRoute = out.waypoints;
                waypointIndex = 0;
            }
            while (waypointIndex < out.waypoints.size() - 1
                    && Math.hypot(out.waypoints.get(waypointIndex)[0] - x, out.waypoints.get(waypointIndex)[1] - y) < 2.0) {
                waypointIndex++;
            }
            return out.waypoints.get(waypointIndex);
        }

        /** Spins the flywheel toward the commanded speed and, while the feeder runs, releases a ball every 0.45 s. */
        void applyFlywheelAndFeeder(AutoShootController.Outputs out) {
            double target = out.flywheelTargetTps * flyMaxFrac;
            // First-order lag again: the flywheel closes part of the gap to the target each tick (time constant
            // 0.35 s).
            flyV += (target - flyV) * (1.0 - Math.exp(-DT / 0.35));
            if (out.feederOn) {
                if (Double.isNaN(firstFeedAt)) {
                    firstFeedAt = t;
                    flyAtFirstFeed = flyV;
                }
                if (out.flywheelTargetTps > 0 && flyV < 0.9 * out.flywheelTargetTps) {
                    feederBelowSpeedSec += DT;
                }
                if (balls > 0) {
                    feedProgress += DT;
                    // One ball leaves for every feedSecPerBall (0.45 s unless a test says otherwise) of feeding.
                    if (feedProgress >= feedSecPerBall) {
                        feedProgress = 0;
                        fire(out);
                    }
                }
            }
        }

        /**
         * One ball leaves: the flywheel dips 10% and the ball is judged. Aimed at a DOWN cell: counted in
         * shotsAtDownCell (a foul). At an UP cell but too slow, out of range or badly aimed: counted in badShots.
         * Otherwise it is a good shot, and enough good shots tip the HIVE.
         */
        void fire(AutoShootController.Outputs out) {
            balls--;
            shotsFired++;
            flyV *= 0.90;                        // the dip a ball causes
            Side robotSide = robotSide();
            CellSpec aimed = field.cell(alliance, robotSide);
            double range = ShotGeometry.rangeTo(pose(), plannerCfg.launchForwardIn, plannerCfg.launchLeftIn, aimed.originX, aimed.originY);
            double aimErr = Math.abs(Math.toDegrees(ShotGeometry.aimError(pose(), plannerCfg.launchForwardIn,
                    plannerCfg.launchLeftIn, aimed.originX, aimed.originY)));
            double az = Math.abs(Math.toDegrees(ShotGeometry.azimuthOffAxis(pose(), plannerCfg.launchForwardIn,
                    plannerCfg.launchLeftIn, aimed)));
            // 'speedOk': the speed just BEFORE the dip (undo the 10% drop) was at least 90% of what the shot table asks
            // for this range.
            boolean speedOk = flyV / 0.90 >= 0.9 * table.velocityAt(range);
            // 'geometryOk': inside the measured range, pointed within 6 degrees, and inside the shooting cone (plus
            // 5 degrees of slack). The 6 degrees is a little looser than the controller's own SHOT_HEADING_TOL_DEG
            // gate, so a shot only counts as bad if the gate let something clearly wrong through.
            boolean geometryOk = table.inRange(range) && aimErr <= 6.0 && az <= plannerCfg.coneHalfAngleDeg + 5.0;
            // The robot is standing on the side of a cell that is DOWN: this ball is a foul (rule G417).
            if (robotSide != trueUp) {
                shotsAtDownCell++;
                return;
            }
            if (!speedOk || !geometryOk) {
                badShots++;
                return;
            }
            // A ball launched while the HIVE swings goes into a cell that is already tipping down (the case the planner's spare is about).
            if (swinging()) {
                shotsIntoSwingingCell++;
                return;
            }
            shotsIntoUpCell++;
            if (flightSec <= 0.0) {
                land();
            } else {
                landings.add(t + flightSec);
            }
        }

        /** True while the HIVE is swinging down after a tip. */
        boolean swinging() {
            return !Double.isNaN(swingStartedAt);
        }

        /** One ball has landed in the UP cell. Enough balls: the HIVE tips (at once, or after a swing) and the other cell becomes the UP one. */
        private void land() {
            hitsIntoUp++;
            if (hitsIntoUp >= tipAfterHits) {
                if (swingSec <= 0.0) {
                    hitsIntoUp = 0;
                    trueUp = trueUp.opposite();
                } else if (!swinging()) {
                    swingStartedAt = t;
                }
            }
        }

        /** Lands the balls whose flight is over, and finishes the swing of the HIVE when its time is up. */
        private void landBalls() {
            for (int i = landings.size() - 1; i >= 0; i--) {
                if (landings.get(i) <= t + 1e-9) {
                    landings.remove(i);
                    land();
                }
            }
            if (swinging() && t >= swingStartedAt + swingSec) {
                swingStartedAt = Double.NaN;
                hitsIntoUp = 0;
                trueUp = trueUp.opposite();
            }
        }

        /** Runs for up to {@code seconds}, stopping early once the controller reaches one of the given states. */
        void run(double seconds, AutoShootController.State... until) {
            double end = t + seconds;
            Set<AutoShootController.State> stop = EnumSet.noneOf(AutoShootController.State.class);
            for (AutoShootController.State s : until) {
                stop.add(s);
            }
            while (t < end) {
                step();
                if (!stop.isEmpty() && stop.contains(controller.state())) {
                    return;
                }
            }
        }
    }

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
    private static Sim redAtC1() {
        return new Sim(Alliance.RED, c1Start());
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
        Sim s = redAtC1();
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
        Sim s = new Sim(Alliance.RED, b1Start());
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
        Sim red = redAtC1();
        red.held = true;
        red.run(25, DONE, FAULT);
        Sim blue = new Sim(Alliance.BLUE, Alliance.BLUE.fromRed(c1Start(), BioBuzzConstants.Field.SIZE_IN));
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
                Sim s = new Sim(a, a.fromRed(c1Start(), BioBuzzConstants.Field.SIZE_IN), false);
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
        Sim s = redAtC1();
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
        Sim s = new Sim(Alliance.RED, c1Start(), false);
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
        Sim s = new Sim(Alliance.RED, c1Start(), false);
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
        Sim s = new Sim(Alliance.RED, c1Start(), false);
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
        Sim s = redAtC1();
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
        Sim s = redAtC1();
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
     * The feed pace is 0.7 s of feeding per ball, which with the flywheel recovering after each ball is about 0.9 s between balls (close to
     * the route planner's {@code Auto.TIME_PER_SHOT_SEC}, 1.0 s at the feeder power of 0.3).
     *
     * <p>The run ends when the volley is over: DONE, or the moment the controller has seen the tip and leaves FEED to plan the drive to the OTHER cell (a
     * button that is still held would take the spare there, which is what TeleOp wants; the route plan counts the volley as over at the tip).
     */
    private static Sim volleyWithAFlightAndASwing(double swingSec) {
        Sim s = redAtC1();
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
        Sim slow = volleyWithAFlightAndASwing(1.2);
        assertEquals("ended in " + slow.controller.state() + " " + (slow.last == null ? "" : slow.last.message), DONE, slow.controller.state());
        assertEquals("the 3 balls that tip it landed in the UP cell", 3, slow.shotsIntoUpCell);
        assertEquals("the 4th went into the cell while it was swinging down", 1, slow.shotsIntoSwingingCell);
        assertEquals(4, slow.shotsFired);
        assertEquals("the hopper is empty: the spare is spent", 0, slow.balls);
        assertEquals("no ball at a cell that was already down", 0, slow.shotsAtDownCell);

        Sim fast = volleyWithAFlightAndASwing(0.2);
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
        Sim s = redAtC1();
        s.tipAfterHits = 3;
        s.held = true;
        s.run(30, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(0, s.shotsIntoSwingingCell);
        assertTrue("at most one ball toward the cell that just tipped, got " + s.shotsAtDownCell, s.shotsAtDownCell <= 1);
    }
}
