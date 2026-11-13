package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The hold-a-button "park in my LOADING ZONE" helper for TeleOp, as a pure state machine (a sibling of
 * {@link AutoShootController}, which is the hold-to-shoot feature).
 *
 * <p><b>Why it exists.</b> At the end of TeleOp a robot that is at least partly inside its own LOADING ZONE scores PARK, 5 points. In the
 * last seconds a driver has other things to think about, so: hold one button, the robot drives itself into the zone by a road that is
 * safe for the whole robot, stops there and holds the spot; let go and you are driving again. Touching the wall in the zone is fine in
 * TeleOp (the "off the wall" rule is only for the end of AUTO).
 *
 * <p>Each loop the caller passes in everything that was measured ({@link Inputs}) and gets back everything to command
 * ({@link Outputs}). Nothing here touches hardware, a clock or the SDK, so the whole sequence runs in unit tests against a
 * simulated robot (see ParkAssistSimTest).
 *
 * <pre>
 *   IDLE --press--> PLAN --> DRIVING --> PARKED
 *                    ^          |          |
 *                    +----------+----------+   pushed far away: plan again from where the robot is
 *   any state --release--> IDLE        any state --problem--> FAULT(reason), manual driving until the button is released
 * </pre>
 *
 * <p><b>What it reuses (so it can never be looser than hold-to-shoot).</b>
 * <ul>
 *   <li>The road: {@link ScoringPlanner#routeTo} is the shooting planner's own road finder and wall check ({@link PathPlanner} around the
 *       HIVE frame and the FLOWERS, then {@link RouteSweep} slides the real turning robot rectangle along it, with the same repairs).
 *       There is no second collision checker here.</li>
 *   <li>The drive: the road is followed by {@code PedroDrive.follow} with the same turn window ({@link HeadingProfile}) and the same Pedro
 *       speed settings as hold-to-shoot, so it never moves faster than that feature.</li>
 *   <li>The cancel rules: the stick push, the stall and the time limit are the shooting ones ({@code AutoShoot.*} constants).</li>
 * </ul>
 *
 * <p><b>Where it stops (the park spot).</b> {@link #choose} lists spots on a grid near the zone, keeps those where the whole robot
 * outline is inside the field (with the wall clearance) and clear of the HIVE frame and the FLOWERS, and where the outline overlaps the
 * zone by at least {@code Park.MIN_OVERLAP_IN}; it tries them nearest first (driving distance plus the cost of turning) and takes the first
 * whose road is safe. The robot keeps its heading unless a different heading is closer in total, so "nearest" means the shortest trip.
 * Everything is worked out in the RED frame and turned half a turn for BLUE, so the two alliances behave as exact mirror images.
 *
 * <p><b>Fail-closed.</b> No trusted pose, or no safe road: it refuses with a plain message and the driver keeps the sticks. The shoot
 * button and the jam-clearing buttons take priority: while one of them is active ({@link Inputs#otherFeatureActive}) the park
 * assist stops, and it only starts again after the park button has been released and pressed once more.
 *
 * <p>Words used here. <b>Pose</b>: where the robot's center of rotation is (inches) and which way it faces (radians). <b>Footprint</b>: the
 * robot's outline on the floor, which is NOT centered on the pose (see {@link RobotShape}). <b>Overlap</b>: how deep the outline reaches
 * into the zone, measured exactly like the autonomous validator: the smaller side of the part of the outline that is inside the zone.
 * <b>Hold</b>: the drive actively stays at one pose and pushes back if bumped. Frame: the Pedro field frame, inches and radians.
 */
public final class ParkAssist {

    /**
     * Which step of the sequence the assist is in.
     * <ul>
     *   <li>IDLE: the park button is not being used; the driver has manual control.</li>
     *   <li>PLAN: choosing the park spot and the road (less than one loop; it moves on at once).</li>
     *   <li>DRIVING: following the road to the spot.</li>
     *   <li>PARKED: at the spot, holding it. The screen says "release the button".</li>
     *   <li>FAULT: refused or gave up ({@link Fault}); manual driving until the button is released.</li>
     * </ul>
     */
    public enum State { IDLE, PLAN, DRIVING, PARKED, FAULT }

    /**
     * Why the assist refused or stopped.
     * <ul>
     *   <li>NONE: no problem.</li>
     *   <li>NO_POSE: odometry was never told where the robot is standing.</li>
     *   <li>NO_SAFE_ROUTE: no park spot has a road the whole robot can drive without touching a wall, the HIVE or a FLOWER.</li>
     *   <li>TIMEOUT: driving took longer than the distance divided by the speed limit, plus a fixed allowance.</li>
     *   <li>STALLED: the robot barely moved for a while although it is still far from the spot (blocked?).</li>
     *   <li>DRIVER_OVERRIDE: the driver pushed the translation stick hard, which cancels the automatic drive.</li>
     * </ul>
     */
    public enum Fault { NONE, NO_POSE, NO_SAFE_ROUTE, TIMEOUT, STALLED, DRIVER_OVERRIDE }

    /**
     * Every number the assist uses; defaults come from {@link BioBuzzConstants}. The fields are public and mutable so a unit test can change
     * one value. Lengths are inches, angles degrees, times seconds, speeds inches per second.
     */
    public static final class Config {
        /** How deep the robot must overlap the zone at a park spot (the target; the choice only uses spots with at least this). */
        public double minOverlapIn;
        /** "Parked" is only ever shown if the robot's real measured outline overlaps the zone by at least this much (the validator's safe minimum). */
        public double safeOverlapIn;
        /** Spacing of the park spots tried, and the heading step (degrees) besides the current heading. */
        public double gridStepIn, headingStepDeg;
        /** The most park spots whose whole road is wall-checked before it gives up. */
        public int maxRouteChecks;
        /** Arrived = within arriveIn inches, arriveDeg degrees and arriveSpeedInS of the spot. */
        public double arriveIn, arriveDeg, arriveSpeedInS;
        /** Cost of turning, in inches of driving per degree, so "nearest" is about travel time (the shooting planner's number). */
        public double rotationCostPerDeg;
        /** The time allowed is distance / timeoutSpeedInS + timeoutExtraSec; a stall is slower than stallSpeedInS for stallSec after stallGraceSec, while farther than stallMinDistIn. */
        public double timeoutSpeedInS, timeoutExtraSec, stallSpeedInS, stallSec, stallGraceSec, stallMinDistIn;
        /** Pushed more than reapproachIn inches or reapproachHeadingDeg degrees off the spot: plan again. */
        public double reapproachIn, reapproachHeadingDeg;
        /** The stick (0 to 1) held beyond overrideStick for overrideSec cancels the assist. */
        public double overrideStick, overrideSec;
        /** A road leg shorter than this is not driven (the drive drops it, see {@link HeadingProfile}). */
        public double minLegIn;

        /** A Config holding the robot's real values, copied from {@link BioBuzzConstants}. */
        public static Config fromConstants() {
            Config c = new Config();
            c.minOverlapIn = BioBuzzConstants.Park.MIN_OVERLAP_IN;
            c.safeOverlapIn = BioBuzzConstants.Auto.PARK_MIN_OVERLAP_IN;
            c.gridStepIn = BioBuzzConstants.Park.GRID_STEP_IN;
            c.headingStepDeg = BioBuzzConstants.Park.HEADING_STEP_DEG;
            c.maxRouteChecks = BioBuzzConstants.Park.MAX_ROUTE_CHECKS;
            c.arriveIn = BioBuzzConstants.Park.ARRIVE_IN;
            c.arriveDeg = BioBuzzConstants.Park.ARRIVE_DEG;
            c.arriveSpeedInS = BioBuzzConstants.Park.ARRIVE_SPEED_IN_S;
            c.rotationCostPerDeg = BioBuzzConstants.AutoShoot.PLAN_ROTATION_COST_IN_PER_DEG;
            c.timeoutSpeedInS = BioBuzzConstants.AutoShoot.APPROACH_TIMEOUT_SPEED_IN_S;
            c.timeoutExtraSec = BioBuzzConstants.AutoShoot.APPROACH_TIMEOUT_EXTRA_SEC;
            c.stallSpeedInS = BioBuzzConstants.AutoShoot.APPROACH_STALL_SPEED_IN_S;
            c.stallSec = BioBuzzConstants.AutoShoot.APPROACH_STALL_SEC;
            c.stallGraceSec = BioBuzzConstants.AutoShoot.APPROACH_STALL_GRACE_SEC;
            c.stallMinDistIn = BioBuzzConstants.AutoShoot.APPROACH_STALL_MIN_DIST_IN;
            c.reapproachIn = BioBuzzConstants.AutoShoot.REAPPROACH_IN;
            c.reapproachHeadingDeg = BioBuzzConstants.AutoShoot.REAPPROACH_HEADING_DEG;
            c.overrideStick = BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_STICK;
            c.overrideSec = BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_SEC;
            c.minLegIn = BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN;
            return c;
        }
    }

    /** Everything measured this loop. Build a fresh one every loop. Units: seconds, inches, inches per second. */
    public static final class Inputs {
        /** The logic clock in seconds. Any steady clock works; only differences between loops matter. */
        public double nowSec;
        /** True while the park button is held down. */
        public boolean parkHeld;
        /**
         * True while something with priority is active: the shoot button or a jam-clearing button is held, or auto-shoot is running. The park
         * assist then stops, and needs a fresh press (release, then press) afterwards.
         */
        public boolean otherFeatureActive;
        /** The robot's pose from odometry (field frame). Null if there is no odometry. */
        public Pose2 pose;
        /** Translation speed in inches per second. */
        public double speedInS;
        /** True once the odometry has been given a trustworthy starting pose. */
        public boolean poseTrusted;
        /** Magnitude (0 to 1) of the driver's translation stick. */
        public double stickMagnitude;
    }

    /** Everything to command. Defaults are the safe ones: manual drive, no cue. */
    public static final class Outputs {
        /** Who drives this loop (the same {@link AutoShootController.DriveMode} the shooting feature uses). */
        public AutoShootController.DriveMode driveMode = AutoShootController.DriveMode.MANUAL;
        /** For FOLLOW: the points to drive through; the last is the park spot's position. The same list instance for the whole drive. */
        public List<double[]> waypoints = Collections.emptyList();
        /** For FOLLOW: keep the start heading for this many inches before turning (the road was checked with this exact turn). */
        public double turnHoldIn;
        /** For FOLLOW: be done turning this many inches before the road ends. */
        public double turnSettleIn;
        /** For FOLLOW (the destination) and HOLD (the pose to hold). */
        public Pose2 holdPose;
        /** A one-time driver notification this loop: DONE when the robot has parked, FAULT when it refused or gave up. */
        public AutoShootController.Cue cue = AutoShootController.Cue.NONE;
        /** The state after this loop's update. */
        public State state = State.IDLE;
        /** Why it stopped (NONE if it did not). */
        public Fault fault = Fault.NONE;
        /** How deep the robot's measured outline overlaps the zone right now, in inches (NaN while idle or without a pose). */
        public double overlapIn = Double.NaN;
        /** A plain-words line for the Driver Station. */
        public String message = "";

        /**
         * The drive command in the form {@code PedroDrive.apply} takes, so the park assist is driven by exactly the same code as
         * hold-to-shoot (the waypoint list is passed on as the same instance, which the drive uses to tell "same road" from "new road").
         */
        public AutoShootController.Outputs toDriveCommand() {
            AutoShootController.Outputs drive = new AutoShootController.Outputs();
            drive.driveMode = driveMode;
            drive.waypoints = waypoints;
            drive.turnHoldIn = turnHoldIn;
            drive.turnSettleIn = turnSettleIn;
            drive.holdPose = holdPose;
            return drive;
        }
    }

    /** The answer of {@link #choose}: where to stop, and the road there. */
    public static final class Plan {
        /** The park spot: the robot's center of rotation and heading (field frame). */
        public final Pose2 pose;
        /** The points to drive through (empty if the robot is already at the spot: it just holds). */
        public final List<double[]> waypoints;
        /** The turn window the road was checked with (see {@link HeadingProfile}). */
        public final double turnHoldIn, turnSettleIn;
        /** Length of the road in inches. */
        public final double travelIn;
        /** The number the choice minimized: the road length plus the cost of the turn, "inches of driving". */
        public final double cost;
        /** How deep the robot overlaps the zone at the spot, in inches. */
        public final double overlapIn;

        Plan(Pose2 pose, List<double[]> waypoints, double turnHoldIn, double turnSettleIn, double travelIn, double cost, double overlapIn) {
            this.pose = pose;
            this.waypoints = waypoints;
            this.turnHoldIn = turnHoldIn;
            this.turnSettleIn = turnSettleIn;
            this.travelIn = travelIn;
            this.cost = cost;
            this.overlapIn = overlapIn;
        }

        @Override
        public String toString() {
            return String.format(java.util.Locale.ROOT, "park at %s, travel %.1f in, overlap %.1f in", pose, travelIn, overlapIn);
        }
    }

    /** One park spot that fits, with the least the trip to it can cost (a straight line plus the turn). */
    private static final class Candidate {
        final Pose2 pose;
        final double lowerBound;

        Candidate(Pose2 pose, double lowerBound) {
            this.pose = pose;
            this.lowerBound = lowerBound;
        }
    }

    // ------------------------------------------------------------------------------------------

    private final Alliance alliance;
    private final FieldModel field;
    private final ScoringPlanner planner;
    private final Config cfg;
    private final RobotShape shape;
    // The own LOADING ZONE in the field frame, and the RED one it is the half turn of (all the searching is done in the RED frame).
    private final Rect zone;
    private final Rect redZone;

    private State state = State.IDLE;
    private Fault fault = Fault.NONE;
    private boolean prevHeld;
    /** False until the button has been seen released once, so holding it through INIT (or through a priority feature) cannot start a park. */
    private boolean releasedSeen = false;
    /** True while the button is held but something with priority has stopped the assist (shown on the screen). */
    private boolean blocked;
    private double stateSince;
    private Plan plan;
    private double driveTimeoutSec;
    private double stallSince = Double.NaN;
    private double overrideSince = Double.NaN;
    private AutoShootController.Cue pendingCue = AutoShootController.Cue.NONE;

    /**
     * @param alliance which alliance's LOADING ZONE to park in
     * @param field    the field map (its size, for the red / blue half turn)
     * @param planner  the shooting planner: it supplies the wall-checked roads and says whether a pose fits
     * @param cfg      all tunable numbers (normally {@link Config#fromConstants()})
     */
    public ParkAssist(Alliance alliance, FieldModel field, ScoringPlanner planner, Config cfg) {
        this.alliance = alliance;
        this.field = field;
        this.planner = planner;
        this.cfg = cfg;
        this.shape = planner.config().robot;
        this.redZone = new Rect(BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MAX_IN);
        this.zone = alliance == Alliance.RED ? redZone : redZone.rotated180(field.center(), field.center());
    }

    /** The current state. */
    public State state() {
        return state;
    }

    /** The reason for the last FAULT, or NONE. Cleared when the button is released. */
    public Fault fault() {
        return fault;
    }

    /** The plan of this press (null before the first PLAN, and after a release). */
    public Plan plan() {
        return plan;
    }

    /** The own LOADING ZONE as a box in the field frame, inches. */
    public Rect zone() {
        return zone;
    }

    /**
     * The priority rule in one place: the park assist yields to the shoot button, to either jam-clearing button and to auto-shoot that is
     * already running (in any of its states, DONE and FAULT included, until the shoot button is released).
     */
    public static boolean otherFeatureActive(boolean shootButtonHeld, boolean unjamHeld, boolean autoShootActive) {
        return shootButtonHeld || unjamHeld || autoShootActive;
    }

    // ------------------------------------------------------------------------------------------ the search

    /**
     * How deep the robot's outline at {@code pose} reaches into the own LOADING ZONE, in inches: the smaller side of the part of the
     * outline that lies inside the zone (0 if none of it does). This is the autonomous validator's measure, so a number here and a number
     * in a route report mean the same thing.
     */
    public double overlapIn(Pose2 pose) {
        Rect clipped = Footprint.of(pose, shape).clippedBounds(zone);
        return clipped == null ? 0.0 : Math.min(clipped.maxX - clipped.minX, clipped.maxY - clipped.minY);
    }

    /**
     * Picks the park spot and the road to it from where the robot is now. The spots are tried nearest first (a straight line is the
     * least a road can cost, so once the best road found is shorter than the next spot's straight line nothing better is left), and the
     * answer is the cheapest spot whose whole road passes the wall check. Null if there is none: then no safe park exists from here.
     *
     * @param robot where the robot is now (field frame)
     */
    public Plan choose(Pose2 robot) {
        List<Candidate> spots = candidates(robot);
        Plan best = null;
        int checks = 0;
        for (Candidate c : spots) {
            if (best != null && c.lowerBound >= best.cost - 1e-9) {
                break;                   // every spot left is at least this far: none can beat the road we have
            }
            if (checks >= cfg.maxRouteChecks) {
                break;
            }
            checks++;
            ScoringPlanner.Route road = planner.routeTo(robot, c.pose);
            if (road == null) {
                continue;
            }
            double turnDeg = Math.toDegrees(Angles.distance(robot.heading, c.pose.heading));
            double cost = road.travelIn + cfg.rotationCostPerDeg * turnDeg;
            if (best == null || cost < best.cost - 1e-9) {
                // A road shorter than the drive's shortest leg is no road: the robot is already there and just holds.
                boolean noRoad = road.travelIn < cfg.minLegIn;
                best = new Plan(c.pose, noRoad ? Collections.<double[]>emptyList() : road.waypoints,
                        noRoad ? 0.0 : road.turnHoldIn, noRoad ? 0.0 : road.turnSettleIn, road.travelIn, cost, overlapIn(c.pose));
            }
        }
        return best;
    }

    /**
     * Every park spot that fits, sorted nearest first. A spot fits if the robot outline there is inside the field (with the wall clearance),
     * clear of the HIVE frame and the FLOWERS ({@link ScoringPlanner#standingPoseFree}) and overlaps the zone by at least the minimum.
     * The spots are laid out on a grid in the RED frame (so BLUE is the exact half turn of RED), with the heading the robot has now and
     * every {@code headingStepDeg} degrees; the robot's own pose is a spot too if it fits, so a robot that is already parked stays put.
     */
    private List<Candidate> candidates(Pose2 robot) {
        Pose2 own = alliance.toRed(robot, field.size());
        double reach = shape.reachIn();
        double minX = Math.max(0.0, redZone.minX - reach);
        double maxX = Math.min(field.size(), redZone.maxX + reach);
        double minY = Math.max(0.0, redZone.minY - reach);
        double maxY = Math.min(field.size(), redZone.maxY + reach);

        // Headings to try (RED frame): the one the robot has now first, then the square ones.
        List<Double> headings = new ArrayList<Double>();
        headings.add(own.heading);
        double step = Math.toRadians(cfg.headingStepDeg);
        int count = (int) Math.round(Angles.TWO_PI / step);
        for (int k = 0; k < count; k++) {
            double h = Angles.normalize(k * step);
            if (Angles.distance(h, own.heading) > 1e-6) {
                headings.add(h);
            }
        }

        List<Candidate> out = new ArrayList<Candidate>();
        addIfFits(out, robot, own);
        // The grid lies on whole multiples of its step (not anchored at the region's corner), so the spots are round numbers.
        for (int i = (int) Math.ceil(minX / cfg.gridStepIn); i * cfg.gridStepIn <= maxX + 1e-9; i++) {
            for (int j = (int) Math.ceil(minY / cfg.gridStepIn); j * cfg.gridStepIn <= maxY + 1e-9; j++) {
                for (double h : headings) {
                    addIfFits(out, robot, new Pose2(i * cfg.gridStepIn, j * cfg.gridStepIn, h));
                }
            }
        }
        // Collections.sort is stable: spots with the same bound keep the order they were made in, so the answer is the same every time.
        Collections.sort(out, new Comparator<Candidate>() {
            @Override
            public int compare(Candidate a, Candidate b) {
                return Double.compare(a.lowerBound, b.lowerBound);
            }
        });
        return out;
    }

    /** Adds the spot (given in the RED frame) to the list if the robot fits and reaches deep enough into the zone there. */
    private void addIfFits(List<Candidate> out, Pose2 robot, Pose2 redSpot) {
        Pose2 spot = alliance.fromRed(redSpot, field.size());
        if (!planner.standingPoseFree(spot) || overlapIn(spot) < cfg.minOverlapIn - 1e-9) {
            return;
        }
        double turnDeg = Math.toDegrees(Angles.distance(robot.heading, spot.heading));
        out.add(new Candidate(spot, robot.distanceTo(spot) + cfg.rotationCostPerDeg * turnDeg));
    }

    // ------------------------------------------------------------------------------------------ the loop

    /**
     * Runs one loop and returns what to command. Call it exactly once per OpMode loop, every loop, even while the button is not held (it
     * needs to see the release). The returned {@link Outputs} is new each time.
     */
    public Outputs update(Inputs in) {
        boolean rising = in.parkHeld && !prevHeld;
        prevHeld = in.parkHeld;
        Outputs out = new Outputs();
        blocked = false;

        // Button released: hand control back this very loop and forget this press.
        if (!in.parkHeld) {
            releasedSeen = true;
            if (state != State.IDLE) {
                enter(State.IDLE, in.nowSec);
                pendingCue = AutoShootController.Cue.NONE;
                fault = Fault.NONE;
                resetPress();
            }
            return finish(out, in);
        }

        // Shooting or clearing a jam has priority: stop, and wait for a fresh press.
        if (in.otherFeatureActive) {
            releasedSeen = false;
            blocked = true;
            if (state != State.IDLE) {
                enter(State.IDLE, in.nowSec);
                pendingCue = AutoShootController.Cue.NONE;
                fault = Fault.NONE;
                resetPress();
            }
            return finish(out, in);
        }

        // Waiting with the button held: start only on a fresh press that came after a release.
        if (state == State.IDLE) {
            if (rising && releasedSeen) {
                releasedSeen = false;
                resetPress();
                enter(State.PLAN, in.nowSec);
            } else {
                return finish(out, in);
            }
        }

        // Safety checks for every active state.
        if (state != State.FAULT) {
            // Without a trusted pose the robot does not know where it is, so driving would be blind.
            if (!in.poseTrusted || in.pose == null) {
                fail(Fault.NO_POSE, in, out);
                return finish(out, in);
            }
            // Driver override (the shooting rule): the translation stick pushed hard for a moment means "I want to drive".
            if (in.stickMagnitude > cfg.overrideStick) {
                if (Double.isNaN(overrideSince)) {
                    overrideSince = in.nowSec;
                }
                if (in.nowSec - overrideSince >= cfg.overrideSec) {
                    fail(Fault.DRIVER_OVERRIDE, in, out);
                    return finish(out, in);
                }
            } else {
                overrideSince = Double.NaN;
            }
        }

        switch (state) {
            case PLAN:
                plan(in, out);
                break;
            case DRIVING:
                drive(in, out);
                break;
            case PARKED:
                parked(in, out);
                break;
            case FAULT:
            default:
                // Manual drive until the button is released.
                break;
        }
        return finish(out, in);
    }

    /** Clears everything that belongs to one press. */
    private void resetPress() {
        plan = null;
        stallSince = Double.NaN;
        overrideSince = Double.NaN;
    }

    private void enter(State next, double now) {
        state = next;
        stateSince = now;
        stallSince = Double.NaN;
        if (next == State.PARKED) {
            pendingCue = AutoShootController.Cue.DONE;
        }
    }

    /** PLAN: choose the spot and the road; drive there, or just hold if the robot is already parked. */
    private void plan(Inputs in, Outputs out) {
        Plan chosen = choose(in.pose);
        if (chosen == null) {
            fail(Fault.NO_SAFE_ROUTE, in, out);
            return;
        }
        plan = chosen;
        driveTimeoutSec = chosen.travelIn / cfg.timeoutSpeedInS + cfg.timeoutExtraSec;
        enter(State.DRIVING, in.nowSec);
        drive(in, out);
    }

    /** DRIVING: follow the road, and watch for arrival, a time-out or a stall. */
    private void drive(Inputs in, Outputs out) {
        out.holdPose = plan.pose;
        if (plan.waypoints.isEmpty()) {
            out.driveMode = AutoShootController.DriveMode.HOLD;
        } else {
            out.driveMode = AutoShootController.DriveMode.FOLLOW;
            out.waypoints = plan.waypoints;
            out.turnHoldIn = plan.turnHoldIn;
            out.turnSettleIn = plan.turnSettleIn;
        }
        // Arrived is judged on the MEASURED pose, never on a path-finished flag (it does not mean "arrived").
        if (arrived(in)) {
            enter(State.PARKED, in.nowSec);
            out.driveMode = AutoShootController.DriveMode.HOLD;
            out.waypoints = Collections.emptyList();
            return;
        }
        double elapsed = in.nowSec - stateSince;
        if (elapsed > driveTimeoutSec) {
            fail(Fault.TIMEOUT, in, out);
            return;
        }
        // Stall: after the start-up grace period, barely moving while still far from the spot for long enough means blocked.
        if (elapsed > cfg.stallGraceSec && in.speedInS < cfg.stallSpeedInS && in.pose.distanceTo(plan.pose) > cfg.stallMinDistIn) {
            if (Double.isNaN(stallSince)) {
                stallSince = in.nowSec;
            }
            if (in.nowSec - stallSince >= cfg.stallSec) {
                fail(Fault.STALLED, in, out);
            }
        } else {
            stallSince = Double.NaN;
        }
    }

    /**
     * True if the robot is at the spot (position, heading and speed within the arrive limits) AND its real outline reaches into the zone by
     * the validator's safe minimum. The second test is the one that matters for the points, so "Parked" is never shown for a robot that
     * would not count.
     */
    private boolean arrived(Inputs in) {
        return in.pose.distanceTo(plan.pose) <= cfg.arriveIn
                && Math.toDegrees(Angles.distance(in.pose.heading, plan.pose.heading)) <= cfg.arriveDeg
                && in.speedInS <= cfg.arriveSpeedInS
                && overlapIn(in.pose) >= cfg.safeOverlapIn;
    }

    /** PARKED: hold the spot; if the robot was shoved far away (or no longer reaches into the zone), plan again from where it is. */
    private void parked(Inputs in, Outputs out) {
        out.driveMode = AutoShootController.DriveMode.HOLD;
        out.holdPose = plan.pose;
        double posErr = in.pose.distanceTo(plan.pose);
        double headErr = Math.toDegrees(Angles.distance(in.pose.heading, plan.pose.heading));
        if (posErr > cfg.reapproachIn || headErr > cfg.reapproachHeadingDeg || overlapIn(in.pose) < cfg.safeOverlapIn) {
            enter(State.PLAN, in.nowSec);
            pendingCue = AutoShootController.Cue.NONE;
            plan(in, out);
        }
    }

    /** Gives up: records why, enters FAULT and hands control back (manual drive) until the button is released. */
    private void fail(Fault why, Inputs in, Outputs out) {
        fault = why;
        enter(State.FAULT, in.nowSec);
        pendingCue = AutoShootController.Cue.FAULT;
        out.driveMode = AutoShootController.DriveMode.MANUAL;
        out.waypoints = Collections.emptyList();
    }

    /** The last step of every update: copies the bookkeeping into the Outputs and writes the screen message. */
    private Outputs finish(Outputs out, Inputs in) {
        out.state = state;
        out.fault = fault;
        out.cue = pendingCue;
        pendingCue = AutoShootController.Cue.NONE;
        if (state != State.IDLE && in.pose != null) {
            out.overlapIn = overlapIn(in.pose);
        }
        out.message = describe();
        return out;
    }

    /** The plain-words line for the Driver Station. */
    private String describe() {
        switch (state) {
            case IDLE:
                return blocked
                        ? "Park off: the shoot or jam-clearing button has priority (release " + BioBuzzConstants.Controls.PARK_BUTTON + ", then press it again)"
                        : "Ready: hold " + BioBuzzConstants.Controls.PARK_BUTTON + " to park in the LOADING ZONE";
            case PLAN:
                return "Parking: planning the route";
            case DRIVING:
                return "Parking: driving to the LOADING ZONE";
            case PARKED:
                return "Parked: release the button";
            case FAULT:
            default:
                return faultText();
        }
    }

    /** Why the assist refused or stopped, in words a first-year student can act on. */
    private String faultText() {
        switch (fault) {
            case NO_POSE:
                return "Park refused: the robot does not know where it is. Stand it on the start pose and hold "
                        + BioBuzzConstants.Controls.RESET_POSE_BUTTON + " first";
            case NO_SAFE_ROUTE:
                return "Park refused: no safe route to the LOADING ZONE from here (a wall, the HIVE or a FLOWER is in the way)";
            case TIMEOUT:
                return "Park stopped: the robot took too long to get there (is something blocking it?)";
            case STALLED:
                return "Park stopped: the robot is stuck (is something blocking it?)";
            case DRIVER_OVERRIDE:
                return "Park cancelled: you pushed the stick, so you are driving";
            case NONE:
            default:
                return "Park stopped";
        }
    }
}
