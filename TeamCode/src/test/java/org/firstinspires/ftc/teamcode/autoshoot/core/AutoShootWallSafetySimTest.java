package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Closed-loop test of the wall safety of the hold-to-shoot drive: a simulated robot is left standing against a wall (or
 * in a corner), the REAL {@link AutoShootController} and {@link ScoringPlanner} drive it to a shooting pose and shoot,
 * and at EVERY 20 ms tick the robot's real rotated 17.8 in square is checked against the field walls, the HIVE frame
 * and the FLOWER plates.
 *
 * <p>Why a simulation on top of the planner tests: {@code ScoringPlannerWallSafetyTest} proves the planned route is
 * clear. This test proves the whole loop is: the controller hands the planner's route AND its turn window to the drive,
 * the drive follows it the way {@code PedroDrive} does (the heading comes from {@link HeadingProfile}, built from the
 * route, the end heading and the window), the heading lags behind what it is told, and a shove can throw the robot
 * somewhere else in the middle of it all. Only the world is fake (see AutoShootSequenceTest for the same idea).
 *
 * <p>What the simulated follower assumes (all of it to be checked on the real robot, see docs/DAY_ONE_CHECKLIST.md):
 * the robot's center stays exactly on the planned route; it moves at up to 32 in/s and slows down for the last
 * stretch; its heading chases the heading the route asks for at the robot's place on it, turning at 10 times the
 * heading error per second (so when it is asked to turn 100 degrees per second it runs about 10 degrees behind); and
 * odometry is perfect. A real robot also wanders sideways a little, and the planner keeps only a quarter inch of
 * clearance, so passing here shows the logic is right, not that a wall can never be touched. How much the heading
 * follower matters was tried by hand: the runs below still pass with a follower FIVE times slower (gain 2, so about 50
 * degrees behind at 100 degrees per second); with one ten times slower (gain 1) a corner goes 0.1 in through a wall in
 * one run. The planner finishes a squeezed turn 3 in before the robot reaches the wall zone (PATH_TURN_SETTLE_IN) for
 * exactly this reason.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*AutoShootWallSafetySimTest"} to
 * run only this class).
 *
 * <p>Words used in this file:
 * <ul>
 *   <li><b>Tick:</b> one 20 ms step of the simulation, like one pass of the robot's control loop.</li>
 *   <li><b>Clip:</b> some part of the robot's rectangle is through a wall, the HIVE frame or a FLOWER plate.</li>
 *   <li><b>Turn window:</b> the part of a route during which the robot turns (see {@link HeadingProfile}).</li>
 *   <li><b>Shove:</b> the test picks the robot up and puts it somewhere else, like another robot ramming it.</li>
 *   <li><b>Unchecked planner:</b> a planner with the wall-safety check switched off: the behaviour from before the fix,
 *       used to prove the test can see the bug.</li>
 * </ul>
 */
public class AutoShootWallSafetySimTest {
    private static final AutoShootController.State DONE = AutoShootController.State.DONE;
    private static final AutoShootController.State FAULT = AutoShootController.State.FAULT;
    private static final AutoShootController.State ALIGN = AutoShootController.State.ALIGN;

    /** The simulated world, with the real controller and planner wired the way the TeleOp wires them. */
    private static final class Sim {
        final FieldModel field = new FieldModel();
        final ShotTable table = ShotTable.fromConstants();
        final CameraModel camera = CameraModel.fromConstants();
        final ClusterAnalyzer analyzer = new ClusterAnalyzer(field, ClusterAnalyzer.Config.fromConstants());
        final ScoringPlanner.Config plannerCfg = ScoringPlanner.Config.fromConstants(table);
        final AutoShootController.Config ctrlCfg = AutoShootController.Config.fromConstants();
        final Alliance alliance;
        final HiveTracker tracker;
        final AutoShootController controller;
        /** Where the robot's edges lie around its center of rotation (the real robot: not centered). */
        final RobotShape robot = plannerCfg.robot;

        // world
        double x, y, heading, vx, vy, omega;
        Side trueUp;
        int balls = 4;
        double flyV;
        double feedProgress;
        double lastFrameAt = -1;
        double t;
        boolean held;

        // what the controller has done
        int shotsFired;
        int shotsAtDownCell;
        boolean usedTurnWindow;
        final Set<AutoShootController.State> visited = EnumSet.noneOf(AutoShootController.State.class);
        AutoShootController.Outputs last;

        // wall safety, checked at every tick
        /** Ticks on which some part of the robot was through a wall, the HIVE frame or a FLOWER plate. */
        int clipTicks;
        /** What was clipped the first time, with the time and pose; null if never. */
        String firstClip;
        /** The smallest gap between the robot and any wall during the run (inches; negative = through the wall). */
        double worstWallGapIn = Double.POSITIVE_INFINITY;

        // the follower (what PedroDrive does)
        private List<double[]> activeRoute;
        private HeadingProfile profile;
        private double along;
        private double pathSpeed;

        static final double DT = 0.02;
        static final double FRAME_PERIOD = 0.04;
        /** Top speed along a route, inches per second. */
        static final double MAX_SPEED = 32.0;
        /** The last stretch of a route is driven at this many inches per second per inch still to go. */
        static final double SLOW_DOWN_PER_SEC = 4.0;
        /** How quickly the speed along the route follows what is wanted (seconds). */
        static final double SPEED_LAG_SEC = 0.12;
        /** The heading turns at this many radians per second for every radian of error. */
        static final double HEADING_GAIN = 10.0;
        /** The robot cannot turn faster than this, radians per second. */
        static final double MAX_OMEGA = 6.0;

        Sim(Alliance alliance, Pose2 startInRedFrameOrOwn, boolean checkRoutes) {
            this.alliance = alliance;
            this.x = startInRedFrameOrOwn.x;
            this.y = startInRedFrameOrOwn.y;
            this.heading = startInRedFrameOrOwn.heading;
            this.trueUp = field.initialUpSide(alliance);
            this.tracker = new HiveTracker(field, alliance, HiveTracker.Config.fromConstants());
            ScoringPlanner.Config cfg = ScoringPlanner.Config.fromConstants(table);
            if (!checkRoutes) {
                cfg.sweepToleranceIn = 1.0e6;          // every route "passes": the behaviour from before the fix
            }
            this.controller = new AutoShootController(alliance, field, new ScoringPlanner(field, cfg), table, tracker, ctrlCfg);
            step();                                    // one loop with the button up, as the TeleOp does before START
        }

        Pose2 pose() {
            return new Pose2(x, y, heading);
        }

        Side robotSide() {
            return HiveTracker.sideOf(pose(), field);
        }

        /** The virtual camera: sees the real tags of the cell that is really up while the robot is on that side. */
        VisionFrame makeFrame() {
            boolean newFrame = t - lastFrameAt >= FRAME_PERIOD - 1e-9;
            if (newFrame) {
                lastFrameAt = t;
            }
            List<TagObservation> tags = new ArrayList<TagObservation>();
            if (newFrame && robotSide() == trueUp) {
                CellSpec cell = field.cell(alliance, trueUp);
                Pose2 p = pose();
                for (int i = 0; i < cell.tagCount(); i++) {
                    double wx = cell.tagFieldX(i);
                    double f = p.forwardOf(wx, cell.tagRowY);
                    double l = p.leftOf(wx, cell.tagRowY);
                    double u = cell.tagRowZ;
                    double[] tt = camera.txTyOf(f, l, u);
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

        /** One 20 ms tick: measure, decide, move, then check the robot against the walls. */
        void step() {
            t += DT;
            VisionFrame frame = makeFrame();
            List<ClusterView> views = analyzer.analyze(alliance, heading, frame.tags);
            tracker.update(t, frame, views, robotSide());

            AutoShootController.Inputs in = new AutoShootController.Inputs();
            in.nowSec = t;
            in.shootHeld = held;
            in.pose = pose();
            in.speedInS = Math.hypot(vx, vy);
            in.omegaRadS = omega;
            in.poseTrusted = true;
            in.flywheelTps = flyV;
            in.frame = frame;
            in.views = views;
            in.stickMagnitude = 0.0;
            AutoShootController.Outputs out = controller.update(in);
            last = out;
            visited.add(out.state);
            if (out.driveMode == AutoShootController.DriveMode.FOLLOW
                    && (out.turnHoldIn > 0.0 || out.turnSettleIn > 0.0)) {
                usedTurnWindow = true;
            }
            applyDrive(out);
            applyFlywheelAndFeeder(out);
            checkWalls();
        }

        /** Follows the route the way PedroDrive does, or holds a pose; any other mode stops the robot. */
        void applyDrive(AutoShootController.Outputs out) {
            if (out.driveMode == AutoShootController.DriveMode.FOLLOW && out.holdPose != null) {
                follow(out);
            } else if (out.driveMode == AutoShootController.DriveMode.HOLD && out.holdPose != null) {
                hold(out.holdPose);
            } else {
                activeRoute = null;
                vx *= 0.8;
                vy *= 0.8;
                omega *= 0.8;
                x += vx * DT;
                y += vy * DT;
                heading = Angles.normalize(heading + omega * DT);
            }
        }

        /**
         * FOLLOW: the same list object means the same route (nothing is rebuilt); a new list means a new route, built
         * from where the robot is NOW with the route's end heading and turn window, exactly as PedroDrive.follow does.
         */
        private void follow(AutoShootController.Outputs out) {
            if (out.waypoints != activeRoute) {
                activeRoute = out.waypoints;
                profile = HeadingProfile.build(pose(), out.waypoints, out.holdPose.heading, out.turnHoldIn, out.turnSettleIn);
                along = 0.0;
                pathSpeed = Math.hypot(vx, vy);
            }
            if (profile.isEmpty()) {
                hold(out.holdPose);
                return;
            }
            double remaining = profile.length() - along;
            double wanted = Math.min(MAX_SPEED, SLOW_DOWN_PER_SEC * remaining);
            pathSpeed += (wanted - pathSpeed) * Math.min(1.0, DT / SPEED_LAG_SEC);
            along = Math.min(profile.length(), along + pathSpeed * DT);
            Pose2 on = profile.poseAt(along);
            vx = (on.x - x) / DT;
            vy = (on.y - y) / DT;
            x = on.x;
            y = on.y;
            // The heading chases the one the route asks for at this place on it.
            omega = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, HEADING_GAIN * Angles.error(heading, on.heading)));
            heading = Angles.normalize(heading + omega * DT);
        }

        /** HOLD: pull toward the pose (the robot cannot change speed at once) and turn toward its heading. */
        private void hold(Pose2 target) {
            activeRoute = null;
            double dx = target.x - x;
            double dy = target.y - y;
            double dist = Math.hypot(dx, dy);
            double speed = Math.min(20.0, 5.0 * dist);
            double tvx = dist > 1e-6 ? speed * dx / dist : 0.0;
            double tvy = dist > 1e-6 ? speed * dy / dist : 0.0;
            double a = Math.min(1.0, DT / 0.12);
            vx += (tvx - vx) * a;
            vy += (tvy - vy) * a;
            omega = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, HEADING_GAIN * Angles.error(heading, target.heading)));
            x += vx * DT;
            y += vy * DT;
            heading = Angles.normalize(heading + omega * DT);
        }

        /** Spins the flywheel and, while the feeder runs, releases a ball every 0.45 s. */
        void applyFlywheelAndFeeder(AutoShootController.Outputs out) {
            flyV += (out.flywheelTargetTps - flyV) * (1.0 - Math.exp(-DT / 0.35));
            if (out.feederOn && balls > 0) {
                feedProgress += DT;
                if (feedProgress >= 0.45) {
                    feedProgress = 0;
                    balls--;
                    shotsFired++;
                    flyV *= 0.90;
                    if (robotSide() != trueUp) {
                        shotsAtDownCell++;
                    }
                }
            }
        }

        /** Checks the robot's real rectangle against the walls, the HIVE frame and the FLOWER plates (independent code). */
        void checkWalls() {
            Pose2 p = pose();
            Rect fieldRect = field.fieldRect();
            double gap = Double.POSITIVE_INFINITY;
            for (double[] c : BruteForceFootprint.corners(p, robot)) {
                gap = Math.min(gap, Math.min(Math.min(c[0] - fieldRect.minX, fieldRect.maxX - c[0]),
                        Math.min(c[1] - fieldRect.minY, fieldRect.maxY - c[1])));
            }
            worstWallGapIn = Math.min(worstWallGapIn, gap);
            String what = null;
            if (gap < -1e-6) {
                what = String.format("wall, %.2f in through it", -gap);
            } else {
                for (Rect o : field.hardObstacles()) {
                    if (BruteForceFootprint.overlaps(p, robot, o, BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN)) {
                        what = "obstacle " + o;
                        break;
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

        /** Runs up to {@code seconds} of simulated time, stopping at DONE or FAULT. */
        void run(double seconds) {
            double end = t + seconds;
            while (t < end) {
                step();
                if (controller.state() == DONE || controller.state() == FAULT) {
                    return;
                }
            }
        }
    }

    /** A RED start against a wall, as {wall, along, headingDeg} (see {@link WallStarts}); touching the wall. */
    private static final double[][] STARTS = {
            {0, 60, 90},      // audience wall below the HIVE axis, facing away from the wall: a 180 degree flip
            {0, 20, 180},     // audience wall, sideways
            {0, 30, 0},       // audience wall, sideways the other way
            {2, 25, 0},       // red wall, facing the field
            {2, 100, 180},    // red wall, back to the field
            {1, 60, 90},      // rear wall, front toward the wall: a long drive around the HIVE
            {1, 100, -45},    // rear wall at a diagonal
            {3, 80, 90},      // far wall (blue side), sideways
            {0, 57.92, 15},   // audience wall right at the shooting spot, 0.4 in from a candidate pose: the route has NO legs but must turn 105 degrees
            {0, 57.92, 90},   // the same, facing the field: 180 degrees to turn, with the back already at the wall
    };

    /** The robot's start for a row of {@link #STARTS}, built for the given alliance (BLUE gets the half-turn image). */
    private static Pose2 startFor(FieldModel field, Alliance alliance, double[] row) {
        Pose2 red = WallStarts.againstWall(field, RobotShape.fromConstants(), (int) row[0], row[1], row[2], 0.0);
        Pose2 own = alliance.fromRed(red, field.size());
        // A start inside a FLOWER plate or the HIVE would be the test's own mistake, not the planner's.
        for (Rect o : field.hardObstacles()) {
            assertFalse("the test start overlaps " + o + ": " + own,
                    BruteForceFootprint.overlaps(own, RobotShape.fromConstants(), o, 0.0));
        }
        return own;
    }

    /**
     * Situation: for each of 10 starts against a wall (see STARTS) and both alliances, 20 runs, the button is held for up
     *     to 40 simulated seconds.
     * Expected: in every run, at every tick, no part of the robot is through a wall, the HIVE frame or a FLOWER plate;
     *     and the volley still finishes: DONE with 4 shots, none into a DOWN cell. The runs from the first start (the
     *     180 degree flip against the audience wall) use a turn window.
     * Why: this is the user's rule in its closed-loop form. The drive must leave the wall before it turns, and the
     *     sequence built on top must still work.
     */
    @Test
    public void everyWallStartDrivesOutAndShootsWithoutEverClipping() {
        List<String> problems = new ArrayList<String>();
        boolean flipUsedAWindow = false;
        for (Alliance alliance : Alliance.values()) {
            for (int i = 0; i < STARTS.length; i++) {
                Sim s = new Sim(alliance, startFor(new FieldModel(), alliance, STARTS[i]), true);
                s.held = true;
                s.run(40.0);
                String name = alliance + " start " + i;
                if (s.clipTicks > 0) {
                    problems.add(name + " clipped on " + s.clipTicks + " ticks, first " + s.firstClip);
                }
                if (s.controller.state() != DONE || s.shotsFired != 4 || s.shotsAtDownCell != 0) {
                    problems.add(name + " ended " + s.controller.state() + " after " + s.shotsFired + " shots, "
                            + (s.last == null ? "" : s.last.message));
                }
                if (i == 0 && s.usedTurnWindow) {
                    flipUsedAWindow = true;
                }
            }
        }
        assertTrue("problems: " + problems, problems.isEmpty());
        assertTrue("the flip never used a turn window", flipUsedAWindow);
    }

    /**
     * Situation: the first start (the 180 degree flip against the audience wall), but the planner is the unchecked one:
     *     the behaviour from before the fix.
     * Expected: the same checker sees the robot clip a wall (more than one tick, and part of the robot goes well past
     *     it).
     * Why: this is the negative test. If it passed with no clip the checker could not see the bug, and the clean runs
     *     above would prove nothing.
     */
    @Test
    public void withoutTheCheckTheFlipAgainstTheWallClipsTheWall() {
        Sim s = new Sim(Alliance.RED, startFor(new FieldModel(), Alliance.RED, STARTS[0]), false);
        s.held = true;
        s.run(40.0);
        assertTrue("clip ticks " + s.clipTicks, s.clipTicks > 1);
        assertTrue("worst gap " + s.worstWallGapIn, s.worstWallGapIn < -1.0);
    }

    /**
     * Situation: a RED robot starts at the TeleOp default (front against the audience wall, facing it). When it reaches
     *     ALIGN it is shoved sideways along the wall and spun to face the other way, as if another robot rammed it.
     * Expected: the controller re-plans from where the robot now stands, drives out, turns and finishes with 4 shots;
     *     the robot never clips a wall.
     * Why: a robot shoved against a wall used to keep following the OLD route's first point along an unchecked line.
     *     Re-planning from the new place is what keeps the recovery wall-safe too.
     */
    @Test
    public void aShoveAgainstTheWallIsRecoveredWithoutClipping() {
        FieldModel field = new FieldModel();
        Pose2 start = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        Sim s = new Sim(Alliance.RED, start, true);
        s.held = true;
        // Run until the robot has arrived and is aligning.
        while (s.t < 20.0 && s.controller.state() != ALIGN) {
            s.step();
        }
        assertEquals(ALIGN, s.controller.state());
        // The shove: 30 in along the wall, pressed against it, facing the field.
        Pose2 shoved = WallStarts.againstWall(field, s.robot, 0, start.x - 30.0, 90.0, 0.0);
        s.x = shoved.x;
        s.y = shoved.y;
        s.heading = shoved.heading;
        s.vx = 0.0;
        s.vy = 0.0;
        s.omega = 0.0;
        s.run(40.0);
        assertNull("clipped: " + s.firstClip, s.firstClip);
        assertEquals(DONE, s.controller.state());
        assertEquals(4, s.shotsFired);
        assertTrue("the recovery used a turn window", s.usedTurnWindow);
        assertFalse(s.visited.contains(FAULT));
    }

    /**
     * Situation: a RED robot starts at the TeleOp default, drives to the shooting pose and is aligning there. It is then TURNED 90 degrees on
     *     the spot (its position is not changed at all), as if another robot had hit one of its corners.
     * Expected: the controller plans a new route (the heading is off by more than REAPPROACH_HEADING_DEG), the planner checks the turn, the
     *     robot never clips a wall, and the volley still ends DONE with 4 shots.
     * Why: ALIGN used to re-plan only when the robot was pushed more than 8 in away. A robot that was only turned was spun back by holding the
     *     pose, which swings a corner through the audience wall (up to 3 in) with no check at all.
     * Negative twin: with the new heading trigger switched off the same shove clips the wall, so the clean run means something.
     */
    @Test
    public void aRobotThatIsOnlyTurnedAtTheShootingSpotIsReplannedNotSpunAgainstTheWall() {
        Pose2 start = Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG);
        Sim fixed = turnedAtTheShootingSpot(start, BioBuzzConstants.AutoShoot.REAPPROACH_HEADING_DEG);
        assertNull("clipped: " + fixed.firstClip, fixed.firstClip);
        assertEquals(DONE, fixed.controller.state());
        assertEquals(4, fixed.shotsFired);
        assertFalse(fixed.visited.contains(FAULT));

        Sim old = turnedAtTheShootingSpot(start, 1.0e9);
        assertTrue("without the trigger the spin clips the wall: " + old.clipTicks, old.clipTicks > 0);
        assertTrue("worst gap " + old.worstWallGapIn, old.worstWallGapIn < -1.0);
    }

    /** Runs the default start to ALIGN, turns the robot 90 degrees without moving it, and runs on. */
    private static Sim turnedAtTheShootingSpot(Pose2 start, double reapproachHeadingDeg) {
        Sim s = new Sim(Alliance.RED, start, true);
        s.ctrlCfg.reapproachHeadingDeg = reapproachHeadingDeg;
        s.held = true;
        while (s.t < 20.0 && s.controller.state() != ALIGN) {
            s.step();
        }
        assertEquals(ALIGN, s.controller.state());
        s.heading = Angles.normalize(s.heading + Math.toRadians(90.0));
        s.vx = 0.0;
        s.vy = 0.0;
        s.omega = 0.0;
        s.run(40.0);
        return s;
    }
}
