package org.firstinspires.ftc.teamcode.autoshoot.run;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.AutoShootController;
import org.firstinspires.ftc.teamcode.autoshoot.core.BruteForceFootprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.HeadingProfile;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotShape;
import org.firstinspires.ftc.teamcode.autoshoot.core.ShootWorld;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AllianceField;
import org.firstinspires.ftc.teamcode.autoshoot.routes.DrivePath;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;

/**
 * The closed-loop world for the route runner's tests: {@link ShootWorld} (the HIVE that tips, the camera that sees the real tags, the flywheel
 * and the balls, shared with the hold-to-shoot tests) driven by the REAL {@link RouteExecutor} instead of the shooting controller, plus what a
 * route needs on top of it:
 * <ul>
 *   <li>a robot that follows the runner's three kinds of drive command the way Pedro does: it HOLDs a pose (a pull toward it, the heading
 *       chasing the target heading), follows a list of WAYPOINTS by the {@link HeadingProfile} rule, and follows a DRIVE LEG
 *       ({@link DrivePath}) along its curve with the leg's own heading rule; the heading lags what it is told, the speed lags what it wants;</li>
 *   <li>the alliance PARTNER: it tips the HIVE at {@link #partnerTipAt} (never, if infinity), and the cell that comes up holds nothing;</li>
 *   <li>POLLEN sources: a flat intake wheel that is pressed on a FLOWER's bottom POLLEN while the intake runs takes one POLLEN per press (and
 *       needs a release before the next), the GARDEN gives one POLLEN every {@link #gardenSecPerPollen} seconds while the robot stands at it
 *       with the intake on. The number the robot holds can go above 4 here, so a test can SEE a runner that breaks G407;</li>
 *   <li>a referee that checks the robot's true rectangle at every 20 ms tick: inside the field, outside the HIVE frame, the chassis outside
 *       every FLOWER plate (only a flat wheel may be over one). It shares no code with the route checker ({@link BruteForceFootprint}).</li>
 * </ul>
 * Odometry is perfect and there are no collisions or wheel slip: a pass shows the LOGIC is right, not that the robot will behave.
 */
public final class RouteWorld extends ShootWorld {
    /** The route as the runner is given it. */
    public final RouteAnalysis plan;
    /** The real runner under test. */
    public final RouteExecutor executor;
    private final AllianceField allianceField;
    private final RobotShape perimeter = RobotShape.fromConstants();
    private RobotShape chassis;
    /** The robot is stuck: it does not move at all, whatever it is told (a wall, another robot). */
    public boolean frozen;
    /** The heading controller is dead: the robot cannot turn, whatever it is told (the wiggle's turns never get there). */
    public boolean headingFrozen;

    /** The route time at which the alliance partner's tip is done (the HIVE flips); infinity = it never tips. */
    public double partnerTipAt = Double.POSITIVE_INFINITY;
    private boolean partnerTipped;
    /** Seconds the robot must stand at the GARDEN with the intake on for one POLLEN (0.8 s: the plan's dwell of 2.5 s then brings in 3). */
    public double gardenSecPerPollen = 0.8;
    /** How near the robot's center must be to the middle of the GARDEN POLLEN line to take from it, inches. */
    public double gardenReachIn = 14.0;

    // what the world counted
    /** Route times at which a POLLEN left the robot. */
    public final List<Double> shotTimes = new ArrayList<Double>();
    /** The most POLLEN the robot held at one moment (the hopper count of the simulation). */
    public int maxHeld = 4;
    /** POLLEN the robot took from FLOWERs and from the GARDEN. */
    public int takenFromFlowers;
    public int takenFromGarden;
    /** Seconds the intake motor was on while the robot held 4 or more POLLEN (it must be 0: G407). */
    public double intakeOnWhenFullSec;
    /** Ticks on which the robot's rectangle broke a rule of the field (walls, HIVE frame, a FLOWER plate hit by the chassis). */
    public int clipTicks;
    /** The first such moment in words, null if there was none. */
    public String firstClip;
    /** The smallest gap between any corner of the robot and a wall during the run, inches (negative = through the wall). */
    public double worstWallGapIn = Double.POSITIVE_INFINITY;
    /** Every heading the robot's pose had while the FLOWER wiggle ran, for tests that look at the alternating heading targets. */
    public final List<Double> wiggleTargets = new ArrayList<Double>();
    /** Every DRIVE leg the runner handed to the drive (each distinct leg once, in order). */
    public final List<DrivePath> legsFollowed = new ArrayList<DrivePath>();
    /** The last outputs the runner gave. */
    public RouteExecutor.Outputs lastOut;
    /** Every distinct phase the runner went through, in order. */
    public final List<RouteExecutor.Phase> phases = new ArrayList<RouteExecutor.Phase>();

    // the FLOWER and GARDEN sources
    private final int[] flowerBalls = new int[BioBuzzConstants.Field.FLOWER_CENTERS_IN.length];
    private final boolean[][] pressing = new boolean[BioBuzzConstants.Field.FLOWER_CENTERS_IN.length][2];
    private int gardenBalls = BioBuzzConstants.Auto.GARDEN_POLLEN;
    private double gardenProgress;

    // the follower
    private Object activeCommand;
    private HeadingProfile profile;
    private DrivePath leg;
    private double along;
    private double pathSpeed;
    private double legSpeedFraction = 1.0;

    /** Heading gain, speed lag and the like: the same numbers as the wall-safety simulation of the hold-to-shoot feature. */
    private static final double HEADING_GAIN = 10.0;
    private static final double TURN_LIMIT = 6.0;
    private static final double SLOW_DOWN_PER_SEC = 4.0;
    private static final double SPEED_LAG_SEC = 0.12;
    /** The follower's top speed along a path, inches per second (the sim's, below the 40 in/s of the plan, so a route runs a little slower than planned). */
    private static final double TOP_SPEED = 32.0;

    /** @param plan the route played through, in the frame of its alliance; the world puts the robot ON the route's start pose */
    public RouteWorld(RouteAnalysis plan) {
        super(plan.route().alliance(), plan.route().startPose(), false);
        this.plan = plan;
        this.allianceField = plan.field();
        this.executor = new RouteExecutor(plan);
        setIntakeReach(BioBuzzConstants.Robot.INTAKE_REACH_IN);
        for (int i = 0; i < flowerBalls.length; i++) {
            flowerBalls[i] = BioBuzzConstants.Auto.FLOWER_POLLEN;
        }
        // The real tips: the cell that is UP at the start holds the 3 staged NECTAR, so the 3rd POLLEN tips it; the cell that comes up after a
        // tip is empty and needs 8. A ball flies for half a second and the HIVE then takes over a second to swing, like the plan says. The feeder keeps the fast pace of the
        // hold-to-shoot tests (0.45 s a ball): a slower pace opens the gate for the flywheel to recover between balls and the shooting sequence then reads a
        // pause of more than a second as an empty hopper, which is a question for the shooter tuning, not for the runner.
        this.tipAfterHits = BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL;
        this.tipAfterHitsLater = BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD;
        this.flightSec = BioBuzzConstants.Auto.BALL_FLIGHT_SEC;
        this.swingSec = 1.2;
    }

    /**
     * How far the flat wheels stick out in front of the chassis in THIS world's robot (the referee's chassis is the perimeter without that stretch). A test
     * of the FLOWER wiggle with {@code WiggleTestRobot} (wheels that reach 4 in) says so here, so the referee judges the robot the plan was made for.
     */
    public void setIntakeReach(double reachIn) {
        this.chassis = new RobotShape(perimeter.frontIn() - reachIn, perimeter.rearIn(), perimeter.leftIn(), perimeter.rightIn());
    }

    // ---- one tick -----------------------------------------------------------------------------------------------------

    /** Runs one 20 ms tick: the world measures, the runner decides, the world moves. */
    public void tick() {
        t += DT;
        landBalls();
        if (!partnerTipped && t >= partnerTipAt) {
            partnerTipped = true;
            tipNow();
        }
        VisionFrame frame = makeFrame();
        RouteExecutor.Inputs in = new RouteExecutor.Inputs();
        in.nowSec = t;
        in.pose = pose();
        in.speedInS = Math.hypot(vx, vy);
        in.omegaRadS = omega;
        in.flywheelTps = flyV;
        in.frame = frame;
        RouteExecutor.Outputs out = executor.update(in);
        lastOut = out;
        if (phases.isEmpty() || phases.get(phases.size() - 1) != out.status.phase) {
            phases.add(out.status.phase);
        }
        if (out.setPose != null) {
            x = out.setPose.x;
            y = out.setPose.y;
            heading = out.setPose.heading;
            vx = 0.0;
            vy = 0.0;
            omega = 0.0;
        }
        if (frozen) {
            vx = 0.0;
            vy = 0.0;
            omega = 0.0;
        } else {
            drive(out.drive);
        }
        AutoShootController.Outputs shooter = new AutoShootController.Outputs();
        shooter.flywheelTargetTps = out.flywheelTps;
        shooter.feederOn = out.feederOn;
        applyFlywheelAndFeeder(shooter);
        intake(out.intakeOn);
        referee();
        if (out.status.phase == RouteExecutor.Phase.FLOWER_WIGGLE) {
            wiggleTargets.add(out.drive.pose.heading);
        }
    }

    /** Runs until the runner says it is over, or {@code maxSec} of simulated time has passed. */
    public void runUntilFinished(double maxSec) {
        while (t < maxSec && !executor.finished()) {
            tick();
        }
    }

    /** Runs for exactly {@code seconds} more of simulated time (the runner may finish earlier and then only holds). */
    public void runFor(double seconds) {
        double end = t + seconds;
        while (t < end) {
            tick();
        }
    }

    @Override
    public void fire(AutoShootController.Outputs out) {
        shotTimes.add(t);
        super.fire(out);
    }

    // ---- the robot follows the command ----------------------------------------------------------------------------------

    private void drive(DriveCommand cmd) {
        switch (cmd.kind) {
            case WAYPOINTS:
                if (activeCommand != cmd.waypoints) {
                    activeCommand = cmd.waypoints;
                    profile = HeadingProfile.build(pose(), cmd.waypoints, cmd.pose.heading, cmd.turnHoldIn, cmd.turnSettleIn);
                    leg = null;
                    along = 0.0;
                    pathSpeed = Math.hypot(vx, vy);
                    legSpeedFraction = 1.0;
                }
                if (profile.isEmpty()) {
                    hold(cmd.pose);
                } else {
                    followPath(profile.length());
                }
                break;
            case LEG:
                if (activeCommand != cmd.leg) {
                    legsFollowed.add(cmd.leg);
                    activeCommand = cmd.leg;
                    leg = cmd.leg;
                    profile = null;
                    along = 0.0;
                    pathSpeed = Math.hypot(vx, vy);
                    legSpeedFraction = cmd.speedFraction;
                }
                followPath(leg.length());
                break;
            case HOLD:
            default:
                activeCommand = null;
                hold(cmd.pose);
                break;
        }
    }

    /** Moves along the active path: a speed that rises to the top speed and slows for the last inches, and a heading that chases the path's. */
    private void followPath(double length) {
        double wanted = Math.min(TOP_SPEED * legSpeedFraction, SLOW_DOWN_PER_SEC * (length - along));
        pathSpeed += (wanted - pathSpeed) * Math.min(1.0, DT / SPEED_LAG_SEC);
        along = Math.min(length, along + pathSpeed * DT);
        Pose2 on = profile != null ? profile.poseAt(along) : leg.poseAt(along);
        vx = (on.x - x) / DT;
        vy = (on.y - y) / DT;
        x = on.x;
        y = on.y;
        omega = headingFrozen ? 0.0 : Math.max(-TURN_LIMIT, Math.min(TURN_LIMIT, HEADING_GAIN * Angles.error(heading, on.heading)));
        heading = Angles.normalize(heading + omega * DT);
    }

    private void hold(Pose2 target) {
        double dx = target.x - x;
        double dy = target.y - y;
        double dist = Math.hypot(dx, dy);
        double speed = Math.min(20.0, 5.0 * dist);
        double tvx = dist > 1e-6 ? speed * dx / dist : 0.0;
        double tvy = dist > 1e-6 ? speed * dy / dist : 0.0;
        double a = Math.min(1.0, DT / 0.12);
        vx += (tvx - vx) * a;
        vy += (tvy - vy) * a;
        omega = headingFrozen ? 0.0 : Math.max(-TURN_LIMIT, Math.min(TURN_LIMIT, HEADING_GAIN * Angles.error(heading, target.heading)));
        x += vx * DT;
        y += vy * DT;
        heading = Angles.normalize(heading + omega * DT);
    }

    // ---- taking POLLEN --------------------------------------------------------------------------------------------------

    private void intake(boolean on) {
        if (!on) {
            for (boolean[] row : pressing) {
                row[0] = false;
                row[1] = false;
            }
            return;
        }
        if (balls >= BioBuzzConstants.Auto.POLLEN_CAPACITY) {
            intakeOnWhenFullSec += DT;
        }
        double half = (BioBuzzConstants.Auto.FLOWER_WHEEL_PRESS_OFFSET_IN + BioBuzzConstants.Auto.FLOWER_WHEEL_START_OFFSET_IN) / 2.0;
        for (int f = 0; f < flowerBalls.length; f++) {
            Point2 ball = allianceField.flowerCenter(f);
            IntakeWheel[] wheels = IntakeWheel.physical();
            for (int w = 0; w < wheels.length; w++) {
                Point2 c = Footprint.wheelCenter(pose(), wheels[w]);
                boolean pressed = Math.hypot(c.x - ball.x, c.y - ball.y) < half;
                if (pressed && !pressing[f][w] && flowerBalls[f] > 0) {
                    flowerBalls[f]--;
                    balls++;
                    takenFromFlowers++;
                }
                pressing[f][w] = pressed;
            }
        }
        Point2 garden = allianceField.ownGardenPollenCenter();
        if (gardenBalls > 0 && Math.hypot(x - garden.x, y - garden.y) <= gardenReachIn) {
            gardenProgress += DT;
            if (gardenProgress >= gardenSecPerPollen) {
                gardenProgress = 0.0;
                gardenBalls--;
                balls++;
                takenFromGarden++;
            }
        }
        maxHeld = Math.max(maxHeld, balls);
    }

    // ---- the referee ----------------------------------------------------------------------------------------------------

    private void referee() {
        Pose2 p = pose();
        Rect fieldRect = field.fieldRect();
        double poke = BruteForceFootprint.wallPoke(p, perimeter, fieldRect);
        worstWallGapIn = Math.min(worstWallGapIn, -poke);
        String what = null;
        if (poke > BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN) {
            what = String.format("%.2f in through a wall", poke);
        } else if (BruteForceFootprint.overlaps(p, perimeter, field.hiveFrame(), BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN)) {
            what = "inside the HIVE frame";
        } else {
            for (int f = 0; f < flowerBalls.length && what == null; f++) {
                if (BruteForceFootprint.overlaps(p, chassis, allianceField.flowerPlate(f), BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN)) {
                    what = "the chassis is over the plate of FLOWER " + (f + 1);
                }
            }
        }
        if (what != null) {
            clipTicks++;
            if (firstClip == null) {
                firstClip = String.format("t=%.2f s at %s: %s", t, p, what);
            }
        }
    }
}
