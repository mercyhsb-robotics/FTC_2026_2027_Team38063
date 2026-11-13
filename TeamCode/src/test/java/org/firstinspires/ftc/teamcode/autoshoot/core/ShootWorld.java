package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 * The simulated world plus the real controller wired exactly as the TeleOp wires it: a robot that follows commanded paths with lag, a flywheel
 * that dips when a ball is fed, a HIVE that tips after enough balls land in its UP cell, and a camera that sees the real cluster geometry.
 * It was the private helper of {@code AutoShootSequenceTest} and is shared now: the route runner's closed-loop tests ({@code RouteWorld})
 * build on it instead of copying it.
 *
 * <p>Each call to {@link #step()} is one 20 ms tick: build the camera frame, update the HiveTracker, run the
 * controller, then apply what it commanded to the world. Tests change the fields (for example {@code balls},
 * {@code dropoutRate}, {@code cameraAlive}) to create a situation, set {@code held} to press the button, and call
 * {@link #run}. A test that drives the world with something other than the shooting controller (the route runner) uses the pieces
 * on their own: {@link #makeFrame}, {@link #landBalls}, {@link #applyFlywheelAndFeeder}, {@link #tipNow} and the public fields.
 */
public class ShootWorld {
    /** The real field map. */
    public final FieldModel field = new FieldModel();
    /** The real shot table: flywheel speed for each range. */
    public final ShotTable table = ShotTable.fromConstants();
    /** The real camera model, used by the virtual camera. */
    public final CameraModel camera = CameraModel.fromConstants();
    /** The real analyzer that turns the virtual camera's tags into ClusterViews. */
    public final ClusterAnalyzer analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());
    /** The planner settings the controller's planner uses (also used to score shots). */
    public final ScoringPlanner.Config plannerCfg = ScoringPlanner.Config.fromConstants(table);
    /** The controller's settings, straight from BioBuzzConstants. */
    public final AutoShootController.Config ctrlCfg = AutoShootController.Config.fromConstants();

    /** Which alliance the simulated robot is on. */
    public final Alliance alliance;
    /** The robot's belief about which cell is UP. Tests may reset or inspect it. */
    public final HiveTracker tracker;
    /** The real state machine under test. */
    public final AutoShootController controller;

    // world
    /**
     * The robot's true pose (inches, radians) and its velocity: vx and vy in inches per second, omega in radians
     * per second.
     */
    public double x, y, heading, vx, vy, omega;
    /** The cell that is REALLY up in the simulated HIVE (the truth, not the robot's belief). */
    public Side trueUp;
    /** Balls that have landed in the current UP cell since the HIVE last tipped. */
    public int hitsIntoUp;
    /** Balls needed to tip the HIVE. 1000 means it never tips during a test. (The real HIVE tips after about 8.) */
    public int tipAfterHits = 1000;
    /**
     * Balls needed for every tip AFTER the first one, when above 0 (0 = the same number every time). The cell that comes up after a tip holds
     * no NECTAR, so the real first tip needs 3 POLLEN and every later one 8.
     */
    public int tipAfterHitsLater = 0;
    /**
     * Seconds a ball is in the air between the shooter and the cell. 0 (the default) is the old simplification: it lands the instant it leaves.
     * The real flight is about 0.5 s ({@code Auto.BALL_FLIGHT_SEC}), and a tip happens when the ball LANDS.
     */
    public double flightSec = 0.0;
    /**
     * Seconds the HIVE takes to swing over once the tipping ball has landed. 0 (the default) is the old simplification: the other cell is up at once.
     * With a swing, the cluster stays readable by the camera for the first half of it, then its tags vanish, and at the end the other cell is up.
     */
    public double swingSec = 0.0;
    /** Seconds of feeding between two balls. The default 0.45 is the fast pace most tests use; a test about the tip uses the slower real one. */
    public double feedSecPerBall = 0.45;
    /** Landing times of the balls in the air. */
    private final List<Double> landings = new ArrayList<Double>();
    /** When the swing started (the tipping ball landed); NaN when the HIVE is not swinging. */
    private double swingStartedAt = Double.NaN;
    /** POLLEN left in the hopper. The robot may hold at most 4 (rule G407). */
    public int balls = 4;
    /** The flywheel's true speed in ticks per second. */
    public double flyV;
    /**
     * The fraction of the commanded speed the flywheel can actually reach: 1.0 is healthy, 0.5 is a weak flywheel.
     */
    public double flyMaxFrac = 1.0;
    /** Seconds the feeder has run since the last ball left. */
    public double feedProgress;
    /** False simulates a camera that is not connected. */
    public boolean cameraAlive = true;
    /** From this simulated time on, the camera stops delivering images (a stalled USB link). */
    public double cameraDeadFrom = Double.POSITIVE_INFINITY;
    /** Fraction of camera frames that arrive with no tags (0 = never, 1 = always). */
    public double dropoutRate = 0.0;
    /** Random error, in inches, added to the measured tag positions (0 = perfect). */
    public double visionNoiseIn = 0.0;
    /** Whether the odometry has been given a trustworthy start pose. */
    public boolean poseTrusted = true;
    /**
     * Random numbers for noise and dropouts. A fixed seed (11) makes every run identical, so a failure can be
     * repeated and the tests cannot fail at random.
     */
    public final Random rng = new Random(11);

    // bookkeeping
    /** The simulated clock in seconds. */
    public double t;
    /** Balls that went into a cell that was UP (good shots). */
    public int shotsIntoUpCell;
    /**
     * Balls fired while the robot faced a cell that was DOWN (a foul under rule G417). Must stay 0, or 1 right
     * after a tip.
     */
    public int shotsAtDownCell;
    /**
     * Balls launched while the HIVE was swinging down: after the tipping ball landed, before the swing was over. They only exist when a test sets
     * {@link #swingSec}; with the instant tip of the other tests it stays 0.
     */
    public int shotsIntoSwingingCell;
    /** Balls fired at an UP cell but too slow, outside the shot table range or badly aimed. */
    public int badShots;           // fired too slow, outside the table, or badly aimed
    /** Every ball that left the robot, good or bad. */
    public int shotsFired;
    /** Time the feeder first turned on (NaN = it never has). */
    public double firstFeedAt = Double.NaN;
    /** Flywheel speed at that moment, in tps. */
    public double flyAtFirstFeed = Double.NaN;
    /** Total seconds the feeder ran while the flywheel was below 90% of its target (should be tiny). */
    public double feederBelowSpeedSec;
    /** Every controller state seen so far. */
    public final Set<AutoShootController.State> visited = EnumSet.noneOf(AutoShootController.State.class);
    /** What the controller commanded on the most recent tick. */
    public AutoShootController.Outputs last;
    /** Clock time of the most recent new camera frame. */
    public double lastFrameAt = -1;

    // commands from the test
    /** The test 'presses' the shoot button by setting this to true and releases it with false. */
    public boolean held;
    /** How far the driver pushes the translation stick (0 to 1). */
    public double stick;

    /** Length of one tick, in seconds (20 ms). */
    public static final double DT = 0.02;
    /** Time between new camera frames: 0.04 s, i.e. 25 frames per second. */
    public static final double FRAME_PERIOD = 0.04;
    /** The robot's top driving speed along a path, in inches per second. */
    public static final double MAX_SPEED = 32.0;
    /** The robot's top turn rate, in radians per second. */
    public static final double MAX_OMEGA = 3.0;

    /** A primed simulation: one loop with the button up has already run. */
    public ShootWorld(Alliance alliance, Pose2 startInAllianceFrame) {
        this(alliance, startInAllianceFrame, true);
    }

    /**
     * Builds a world with the robot standing at the given pose (in its own alliance's frame), the HIVE in its
     * match-start state, and the real controller attached. {@code primed} runs one loop with the button up, like
     * the TeleOp does before START: the controller must see the button released once, or it will ignore a button
     * that was already held.
     */
    public ShootWorld(Alliance alliance, Pose2 startInAllianceFrame, boolean primed) {
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
    public Pose2 pose() {
        return new Pose2(x, y, heading);
    }

    /** Which side of the HIVE the robot is standing on (AUDIENCE or REAR). */
    public Side robotSide() {
        return HiveTracker.sideOf(pose(), field);
    }

    /**
     * The virtual camera for one tick: builds the VisionFrame the controller would get from the Limelight.
     *
     * <p>It simplifies: a tag is either seen exactly (plus optional Gaussian noise) or not seen; there is no motion
     * blur and no partial detections. Tags are only seen from the side of the HIVE whose cell is really up, because
     * from the other side the stickers face away.
     */
    public VisionFrame makeFrame() {
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
    public void step() {
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
    public void applyDrive(AutoShootController.Outputs out) {
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
    public double[] nextWaypoint(AutoShootController.Outputs out) {
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
    public void applyFlywheelAndFeeder(AutoShootController.Outputs out) {
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
    public void fire(AutoShootController.Outputs out) {
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
    public boolean swinging() {
        return !Double.isNaN(swingStartedAt);
    }

    /** One ball has landed in the UP cell. Enough balls: the HIVE tips (at once, or after a swing) and the other cell becomes the UP one. */
    private void land() {
        hitsIntoUp++;
        if (hitsIntoUp >= tipAfterHits) {
            if (swingSec <= 0.0) {
                flip();
            } else if (!swinging()) {
                swingStartedAt = t;
            }
        }
    }

    /** The times at which the HIVE finished tipping (the other cell became the UP one), in order: the camera can only confirm a tip after these. */
    public final List<Double> flipTimes = new ArrayList<Double>();

    /** The HIVE has tipped: the other cell is up, empty, and counts its own balls from 0. */
    private void flip() {
        flipTimes.add(t);
        hitsIntoUp = 0;
        trueUp = trueUp.opposite();
        if (tipAfterHitsLater > 0) {
            tipAfterHits = tipAfterHitsLater;
        }
    }

    /**
     * The HIVE tips right now, as if the alliance PARTNER's volley had tipped it: the other cell is up at once and holds nothing. (The
     * robot's camera sees the new cell's tags from the next frame on, if the robot stands on that side of the HIVE.)
     */
    public void tipNow() {
        flip();
    }

    /** Lands the balls whose flight is over, and finishes the swing of the HIVE when its time is up. */
    public void landBalls() {
        for (int i = landings.size() - 1; i >= 0; i--) {
            if (landings.get(i) <= t + 1e-9) {
                landings.remove(i);
                land();
            }
        }
        if (swinging() && t >= swingStartedAt + swingSec) {
            swingStartedAt = Double.NaN;
            flip();
        }
    }

    /** Runs for up to {@code seconds}, stopping early once the controller reaches one of the given states. */
    public void run(double seconds, AutoShootController.State... until) {
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
