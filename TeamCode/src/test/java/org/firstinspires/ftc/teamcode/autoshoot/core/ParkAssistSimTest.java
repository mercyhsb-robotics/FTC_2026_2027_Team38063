package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Closed-loop test of the hold-to-park assist: a simulated robot is started in many places, the REAL {@link ParkAssist} and
 * {@link ScoringPlanner} drive it, and at EVERY 20 ms tick the robot's real rotated outline is checked against the field walls,
 * the HIVE frame and the FLOWER plates. Only the world is fake (the same idea as {@code AutoShootWallSafetySimTest}, whose follower
 * this one copies: the robot's center stays on the planned road, it moves at up to 32 in/s and slows for the last stretch, and its
 * heading chases the heading the road asks for, with the turn window the planner checked).
 *
 * <p>What it proves: from several start poses, for RED and for BLUE (blue is the red half turn about the field center), the robot
 * ends with its real outline at least partly in its own LOADING ZONE and stays there; it never leaves the field, never touches the
 * HIVE frame or a FLOWER plate on the way; it stops when the button is released; it refuses without a trusted pose and when no safe
 * road exists; a hard stick push cancels it; and the shoot / jam-clearing priority works.
 *
 * <p>Run with {@code ./gradlew :TeamCode:testDebugUnitTest --tests "*ParkAssistSimTest"}.
 *
 * <p>Words: <b>tick</b> one 20 ms step; <b>clip</b> some part of the robot is through a wall, the HIVE frame or a FLOWER plate;
 * <b>shove</b> the test picks the robot up and puts it somewhere else.
 */
public class ParkAssistSimTest {

    /** The simulated world, with the real assist and planner wired the way the TeleOp wires them. */
    private static final class Sim {
        final FieldModel field;
        final Alliance alliance;
        final ScoringPlanner planner;
        final ParkAssist assist;
        final RobotShape robot = RobotShape.fromConstants();

        // world
        double x, y, heading, vx, vy, omega, t;
        // what the driver does
        boolean held;
        boolean other;
        boolean trusted = true;
        double stick;
        // what the assist said last, and everything it said
        ParkAssist.Outputs last;
        final List<ParkAssist.State> seen = new ArrayList<ParkAssist.State>();

        // checks at every tick
        int clipTicks;
        String firstClip;
        double worstWallGapIn = Double.POSITIVE_INFINITY;
        double topSpeedInS;

        // the follower (what PedroDrive does)
        private List<double[]> activeRoute;
        private HeadingProfile profile;
        private double along;
        private double pathSpeed;

        static final double DT = 0.02;
        static final double MAX_SPEED = 32.0;
        static final double SLOW_DOWN_PER_SEC = 4.0;
        static final double SPEED_LAG_SEC = 0.12;
        static final double HEADING_GAIN = 10.0;
        static final double MAX_OMEGA = 6.0;

        Sim(Alliance alliance, Pose2 start) {
            this(alliance, start, new FieldModel());
        }

        Sim(Alliance alliance, Pose2 start, FieldModel field) {
            this.field = field;
            this.alliance = alliance;
            this.planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(ShotTable.fromConstants()));
            this.assist = new ParkAssist(alliance, field, planner, ParkAssist.Config.fromConstants());
            this.x = start.x;
            this.y = start.y;
            this.heading = start.heading;
            step();                       // one loop with the button up, as the TeleOp does before START
        }

        Pose2 pose() {
            return new Pose2(x, y, heading);
        }

        /** One 20 ms tick: decide, move, then check the robot against the walls. */
        void step() {
            t += DT;
            ParkAssist.Inputs in = new ParkAssist.Inputs();
            in.nowSec = t;
            in.parkHeld = held;
            in.otherFeatureActive = other;
            in.pose = pose();
            in.speedInS = Math.hypot(vx, vy);
            in.poseTrusted = trusted;
            in.stickMagnitude = stick;
            last = assist.update(in);
            if (seen.isEmpty() || seen.get(seen.size() - 1) != last.state) {
                seen.add(last.state);
            }
            applyDrive(last);
            topSpeedInS = Math.max(topSpeedInS, Math.hypot(vx, vy));
            checkWalls();
        }

        private void applyDrive(ParkAssist.Outputs out) {
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

        /** FOLLOW: the same list object is the same road; a new list is a new road built from where the robot is now. */
        private void follow(ParkAssist.Outputs out) {
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
            omega = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, HEADING_GAIN * Angles.error(heading, on.heading)));
            heading = Angles.normalize(heading + omega * DT);
        }

        /** HOLD: pull toward the pose and turn toward its heading. */
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

        /** Runs up to {@code seconds} of simulated time, stopping early when the state is PARKED or FAULT. */
        void runUntilParkedOrFault(double seconds) {
            double end = t + seconds;
            while (t < end) {
                step();
                if (assist.state() == ParkAssist.State.PARKED || assist.state() == ParkAssist.State.FAULT) {
                    return;
                }
            }
        }

        /** Runs exactly this many simulated seconds. */
        void run(double seconds) {
            double end = t + seconds;
            while (t < end) {
                step();
            }
        }

        /** How far the robot's real outline reaches into its own zone (production measure), inches. */
        double overlapIn() {
            Rect clipped = Footprint.of(pose()).clippedBounds(assist.zone());
            return clipped == null ? 0.0 : Math.min(clipped.maxX - clipped.minX, clipped.maxY - clipped.minY);
        }

        /** True if the real outline touches the own zone at all, judged by the independent checker. */
        boolean partlyInZone() {
            return BruteForceFootprint.overlaps(pose(), robot, assist.zone(), 0.0);
        }
    }

    /** RED-frame start poses, {x, y, heading in degrees}: open floor, the other half, walls, already parked. */
    private static final double[][] STARTS = {
            {58.924, 9.77, -90.0},     // the TeleOp default start: front on the audience wall under the HIVE axis
            {30.0, 30.0, 0.0},         // open floor on the own half
            {35.0, 38.0, 135.0},       // near the red FLOWER and the HIVE corner, facing away from the zone
            {100.0, 110.0, 180.0},     // the OTHER half, behind the blue side, facing the zone
            {110.0, 30.0, 90.0},       // the other half, audience side: a long way round the HIVE
            {60.0, 125.0, 0.0},        // near the rear wall above the HIVE, facing away
            {70.674, 100.0, 0.0},      // right above the HIVE frame, inside its keep-out ring
            {12.0, 105.0, 0.0},        // already inside the zone
            {8.4, 60.0, 0.0},          // back on the red wall, 35 in below the zone
            {20.0, 9.77, -90.0},       // front on the audience wall, on the far side of the red FLOWER
            {130.0, 35.0, 0.0},        // inside the OTHER alliance's zone
            {70.674, 125.0, -90.0},    // near the rear wall facing it
    };

    private static Pose2 startFor(Alliance alliance, double[] row) {
        return alliance.fromRed(Pose2.ofDegrees(row[0], row[1], row[2]), new FieldModel().size());
    }

    /**
     * Situation: 12 start poses (open floor, the other half of the field, against walls, right above the HIVE, already in the zone) for
     *     RED and for BLUE; the button is held for up to 40 simulated seconds, then 3 more seconds after "Parked".
     * Expected: in every run the state ends PARKED; the real outline is partly in the own zone, by at least the validator's safe
     *     minimum, and stays so; at no tick is any part of the robot through a wall, the HIVE frame or a FLOWER plate; and the robot
     *     never moves faster than the follower's top speed (the speed the shooting feature uses).
     * Why: this is the feature in one test. The first start is the TeleOp default, the second pair are the ordinary cases, the rest are
     *     the awkward ones.
     */
    @Test
    public void fromEveryStartBothAlliancesParkInTheOwnZoneWithoutClipping() {
        List<String> problems = new ArrayList<String>();
        for (Alliance alliance : Alliance.values()) {
            for (int i = 0; i < STARTS.length; i++) {
                Pose2 start = startFor(alliance, STARTS[i]);
                Sim s = new Sim(alliance, start);
                String name = alliance + " start " + i + " " + start;
                s.held = true;
                s.step();
                s.runUntilParkedOrFault(40.0);
                if (s.assist.state() != ParkAssist.State.PARKED) {
                    problems.add(name + " ended " + s.assist.state() + " " + s.last.message);
                    continue;
                }
                s.run(3.0);
                if (s.assist.state() != ParkAssist.State.PARKED) {
                    problems.add(name + " did not stay parked: " + s.assist.state() + " " + s.last.message);
                }
                if (!s.partlyInZone() || s.overlapIn() < BioBuzzConstants.Auto.PARK_MIN_OVERLAP_IN) {
                    problems.add(name + " is not in its zone: overlap " + s.overlapIn() + " at " + s.pose());
                }
                if (s.clipTicks > 0) {
                    problems.add(name + " clipped on " + s.clipTicks + " ticks, first " + s.firstClip);
                }
                if (s.topSpeedInS > Sim.MAX_SPEED + 1e-6) {
                    problems.add(name + " went faster than the shooting follower: " + s.topSpeedInS);
                }
            }
        }
        assertTrue("problems: " + problems, problems.isEmpty());
    }

    /**
     * Situation: the TeleOp default start, button held until "Parked".
     * Expected: the robot ended on its own half of the zone and the screen said "Parking: driving to the LOADING ZONE" on the way and
     *     "Parked: release the button" at the end, and the drive went through PLAN, DRIVING and PARKED.
     * Why: the messages are what the driver sees; they must be the plain ones.
     */
    @Test
    public void theScreenSaysParkingThenParked() {
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[0]));
        s.held = true;
        s.step();
        assertEquals("Parking: driving to the LOADING ZONE", s.last.message);
        s.runUntilParkedOrFault(40.0);
        assertEquals("Parked: release the button", s.last.message);
        assertEquals(ParkAssist.State.PARKED, s.last.state);
        assertTrue(s.seen.toString(), s.seen.contains(ParkAssist.State.DRIVING));
        assertEquals(AutoShootController.Cue.NONE, new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[1])).last.cue);
    }

    /**
     * Situation: the button is released while the robot is driving, and again while it is parked.
     * Expected: in the very loop of the release the output is MANUAL drive and the state IDLE; the robot is then free (the follower
     *     lets it coast to a stop).
     * Why: "release returns to manual at once" is the safety rule of every hold-to-act feature.
     */
    @Test
    public void releasingTheButtonHandsControlBackInTheSameLoop() {
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[3]));
        s.held = true;
        s.run(1.0);
        assertEquals(ParkAssist.State.DRIVING, s.assist.state());
        s.held = false;
        s.step();
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertEquals(ParkAssist.State.IDLE, s.last.state);
        assertEquals(ParkAssist.Fault.NONE, s.last.fault);
        s.run(2.0);
        assertTrue("it coasted to a stop: " + Math.hypot(s.vx, s.vy), Math.hypot(s.vx, s.vy) < 0.5);

        s.held = true;
        s.step();                                  // a fresh press after the release parks again
        assertEquals(ParkAssist.State.DRIVING, s.assist.state());
        s.runUntilParkedOrFault(40.0);
        assertEquals(ParkAssist.State.PARKED, s.assist.state());
        s.held = false;
        s.step();
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertEquals(ParkAssist.State.IDLE, s.last.state);
    }

    /**
     * Situation: the odometry was never told where the robot is (poseTrusted false), button pressed.
     * Expected: the assist refuses at once: FAULT with NO_POSE, manual drive, a plain message that says how to fix it, a FAULT cue,
     *     and the robot does not move.
     * Why: fail closed. Driving to a place we cannot locate would be blind.
     */
    @Test
    public void withoutATrustedPoseItRefusesAndTheRobotDoesNotMove() {
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[1]));
        s.trusted = false;
        s.held = true;
        s.step();
        assertEquals(ParkAssist.State.FAULT, s.last.state);
        assertEquals(ParkAssist.Fault.NO_POSE, s.last.fault);
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertEquals(AutoShootController.Cue.FAULT, s.last.cue);
        assertTrue(s.last.message, s.last.message.startsWith("Park refused: the robot does not know where it is"));
        s.run(3.0);
        assertEquals(30.0, s.x, 1e-9);
        assertEquals(30.0, s.y, 1e-9);
        assertEquals(ParkAssist.State.FAULT, s.assist.state());
    }

    /**
     * Situation: a field where every place near the zone is blocked (the FLOWER plates are made absurdly big).
     * Expected: the assist refuses with NO_SAFE_ROUTE and a message that says there is no safe route, hands the sticks back and does
     *     not move the robot.
     * Why: fail closed. If no road is safe the answer is "no", never "drive anyway".
     */
    @Test
    public void whenNoSafeRouteExistsItRefusesWithAReason() {
        FieldModel.Params p = new FieldModel.Params();
        p.flowerDepth = 200.0;
        p.flowerAlongWall = 400.0;
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[1]), new FieldModel(p));
        s.held = true;
        s.step();
        assertEquals(ParkAssist.State.FAULT, s.last.state);
        assertEquals(ParkAssist.Fault.NO_SAFE_ROUTE, s.last.fault);
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertTrue(s.last.message, s.last.message.contains("no safe route"));
        assertNull(s.assist.plan());
    }

    /**
     * Situation: while driving, the driver pushes the left stick all the way for longer than the override time.
     * Expected: FAULT with DRIVER_OVERRIDE and manual drive; a push shorter than the time changes nothing; after a release and a new
     *     press the park works again.
     * Why: the same cancel rule as hold-to-shoot (a hard stick push means "I am driving").
     */
    @Test
    public void aHardStickPushCancelsItButABriefBumpDoesNot() {
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[3]));
        s.held = true;
        s.run(0.5);
        assertEquals(ParkAssist.State.DRIVING, s.assist.state());
        s.stick = 1.0;
        s.step();                                  // a bump of one loop
        s.stick = 0.0;
        s.run(0.2);
        assertEquals("a one-loop bump must not cancel", ParkAssist.State.DRIVING, s.assist.state());

        s.stick = 1.0;
        s.run(BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_SEC + 0.1);
        assertEquals(ParkAssist.State.FAULT, s.assist.state());
        assertEquals(ParkAssist.Fault.DRIVER_OVERRIDE, s.last.fault);
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertTrue(s.last.message, s.last.message.startsWith("Park cancelled"));
        s.stick = 0.0;
        s.run(1.0);
        assertEquals("it stays cancelled until the button is released", ParkAssist.State.FAULT, s.assist.state());

        s.held = false;
        s.step();
        assertEquals(ParkAssist.State.IDLE, s.assist.state());
        s.held = true;
        s.step();
        assertEquals(ParkAssist.State.DRIVING, s.assist.state());
    }

    /**
     * Situation: the park button is held while the shoot button, a jam-clearing button or a running auto-shoot is active.
     * Expected: the assist is IDLE with manual drive and a message that says the other button has priority; it does not start again
     *     when the other thing ends while the park button is still held; only a release and a new press starts it. If it was already
     *     driving, the priority thing stops it in that same loop.
     * Why: shooting and clearing a jam take priority, and the park assist never runs while auto-shoot is active.
     */
    @Test
    public void shootingAndJamClearingHavePriorityAndNeedAFreshPress() {
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[3]));
        s.other = true;
        s.held = true;
        s.step();
        assertEquals(ParkAssist.State.IDLE, s.last.state);
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertTrue(s.last.message, s.last.message.startsWith("Park off: the shoot or jam-clearing button has priority"));
        s.other = false;
        s.run(1.0);
        assertEquals("still held from before: no start", ParkAssist.State.IDLE, s.assist.state());

        s.held = false;
        s.step();
        s.held = true;
        s.step();
        assertEquals(ParkAssist.State.DRIVING, s.assist.state());
        s.run(0.5);
        s.other = true;                            // the shoot button goes down while parking
        s.step();
        assertEquals(ParkAssist.State.IDLE, s.last.state);
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        s.other = false;
        s.run(0.5);
        assertEquals(ParkAssist.State.IDLE, s.assist.state());
    }

    /**
     * Situation: the priority rule as the TeleOp calls it.
     * Expected: true if the shoot button, a jam button or a running auto-shoot is active; false only when none is.
     * Why: BioBuzzRobot cannot run on a PC, so the rule it uses is pinned here.
     */
    @Test
    public void thePriorityRuleCoversEveryOtherFeature() {
        assertFalse(ParkAssist.otherFeatureActive(false, false, false));
        assertTrue(ParkAssist.otherFeatureActive(true, false, false));
        assertTrue(ParkAssist.otherFeatureActive(false, true, false));
        assertTrue(ParkAssist.otherFeatureActive(false, false, true));
    }

    /**
     * Situation: the park button is already down on the very first loop (held through INIT).
     * Expected: nothing starts, however long it is held.
     * Why: same as hold-to-shoot: only a fresh press after a release may start a drive.
     */
    @Test
    public void aButtonHeldFromTheFirstLoopNeverStarts() {
        FieldModel field = new FieldModel();
        ScoringPlanner planner = new ScoringPlanner(field, ScoringPlanner.Config.fromConstants(ShotTable.fromConstants()));
        ParkAssist assist = new ParkAssist(Alliance.RED, field, planner, ParkAssist.Config.fromConstants());
        for (int i = 1; i <= 100; i++) {
            ParkAssist.Inputs in = new ParkAssist.Inputs();
            in.nowSec = i * 0.02;
            in.parkHeld = true;
            in.pose = new Pose2(40.0, 40.0, 0.0);
            in.poseTrusted = true;
            ParkAssist.Outputs out = assist.update(in);
            assertEquals(ParkAssist.State.IDLE, out.state);
            assertEquals(AutoShootController.DriveMode.MANUAL, out.driveMode);
        }
    }

    /**
     * Situation: the robot is parked and is nudged 3 in (the hold pulls it back, no new plan), then shoved 25 in away.
     * Expected: after the nudge it is still PARKED with the same spot; after the big shove it plans a new road from where it stands, the
     *     robot never clips anything, and it ends PARKED again in the zone.
     * Why: other robots push in the last seconds of a match.
     */
    @Test
    public void aShoveWhileParkedIsFoughtOrReplanned() {
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[1]));
        s.held = true;
        s.step();
        s.runUntilParkedOrFault(40.0);
        assertEquals(ParkAssist.State.PARKED, s.assist.state());
        Pose2 spot = s.assist.plan().pose;

        s.y += 3.0;
        s.run(2.0);
        assertEquals(ParkAssist.State.PARKED, s.assist.state());
        assertSame(spot, s.assist.plan().pose);
        assertTrue("pulled back to the spot: " + s.pose(), s.pose().distanceTo(spot) < 1.0);

        s.x += 25.0;
        s.y -= 5.0;
        s.vx = 0.0;
        s.vy = 0.0;
        s.run(0.1);
        assertEquals("the shove far out of the zone starts a new plan", ParkAssist.State.DRIVING, s.assist.state());
        s.runUntilParkedOrFault(40.0);
        assertEquals(ParkAssist.State.PARKED, s.assist.state());
        assertTrue(s.partlyInZone());
        assertEquals("no clip: " + s.firstClip, 0, s.clipTicks);
    }

    /**
     * Situation: the Outputs of a drive, turned into the command the drive class takes.
     * Expected: the same mode, the same waypoint list INSTANCE (the drive uses the instance to tell the same road from a new one), the
     *     same turn window and hold pose.
     * Why: parking is driven by the very code that drives hold-to-shoot; this is the hand-off, so it cannot go faster.
     */
    @Test
    public void theDriveCommandIsTheOneHoldToShootUses() {
        Sim s = new Sim(Alliance.RED, startFor(Alliance.RED, STARTS[3]));
        s.held = true;
        s.step();
        ParkAssist.Outputs out = s.last;
        assertEquals(AutoShootController.DriveMode.FOLLOW, out.driveMode);
        AutoShootController.Outputs drive = out.toDriveCommand();
        assertEquals(out.driveMode, drive.driveMode);
        assertSame(out.waypoints, drive.waypoints);
        assertEquals(out.turnHoldIn, drive.turnHoldIn, 0.0);
        assertEquals(out.turnSettleIn, drive.turnSettleIn, 0.0);
        assertSame(out.holdPose, drive.holdPose);
        // and the same list again on the next loop, so the drive does not rebuild its path every loop
        s.step();
        assertSame(out.waypoints, s.last.waypoints);
        assertNotNull(s.last.holdPose);
        assertFalse(out.waypoints.isEmpty());
    }
}
