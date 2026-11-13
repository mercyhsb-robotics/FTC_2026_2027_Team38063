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

/**
 * Closed-loop tests of the whole hold-to-shoot sequence against a small physical simulation: a robot that
 * follows commanded paths with lag, a flywheel that dips when a ball is fed, a HIVE that tips after
 * enough balls land in its UP cell, and a camera that sees the real cluster geometry through the real
 * camera model (and sees nothing from the wrong side of the HIVE).
 */
public class AutoShootSequenceTest {

    /** The simulated world plus the real controller wired exactly as the TeleOp wires it. */
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

        // world
        double x, y, heading, vx, vy, omega;
        Side trueUp;
        int hitsIntoUp;
        int tipAfterHits = 1000;
        int balls = 4;
        double flyV;
        double flyMaxFrac = 1.0;
        double feedProgress;
        boolean cameraAlive = true;
        double dropoutRate = 0.0;
        double visionNoiseIn = 0.0;
        boolean poseTrusted = true;
        final Random rng = new Random(11);

        // bookkeeping
        double t;
        int shotsIntoUpCell;
        int shotsAtDownCell;
        int badShots;           // fired too slow, outside the table, or badly aimed
        int shotsFired;
        double firstFeedAt = Double.NaN;
        double flyAtFirstFeed = Double.NaN;
        double feederBelowSpeedSec;
        final Set<AutoShootController.State> visited = EnumSet.noneOf(AutoShootController.State.class);
        AutoShootController.Outputs last;
        double lastFrameAt = -1;

        // commands from the test
        boolean held;
        double stick;

        static final double DT = 0.02;
        static final double FRAME_PERIOD = 0.04;
        static final double MAX_SPEED = 32.0;
        static final double MAX_OMEGA = 3.0;

        Sim(Alliance alliance, Pose2 startInAllianceFrame) {
            this(alliance, startInAllianceFrame, true);
        }

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

        Pose2 pose() {
            return new Pose2(x, y, heading);
        }

        Side robotSide() {
            return HiveTracker.sideOf(pose(), field);
        }

        VisionFrame makeFrame() {
            if (!cameraAlive) {
                return VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, t);
            }
            boolean newFrame = t - lastFrameAt >= FRAME_PERIOD - 1e-9;
            if (newFrame) {
                lastFrameAt = t;
            }
            List<TagObservation> tags = new ArrayList<TagObservation>();
            if (newFrame && rng.nextDouble() >= dropoutRate && robotSide() == trueUp) {
                CellSpec cell = field.cell(alliance, trueUp);
                Pose2 p = pose();
                for (int i = 0; i < cell.tagCount(); i++) {
                    double wx = cell.tagFieldX(i);
                    double f = p.forwardOf(wx, cell.tagRowY) + visionNoiseIn * rng.nextGaussian();
                    double l = p.leftOf(wx, cell.tagRowY) + visionNoiseIn * rng.nextGaussian();
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
            in.poseTrusted = poseTrusted;
            in.flywheelTps = flyV;
            in.frame = frame;
            in.views = views;
            in.stickMagnitude = stick;
            AutoShootController.Outputs out = controller.update(in);
            last = out;
            visited.add(out.state);

            applyDrive(out);
            applyFlywheelAndFeeder(out);
        }

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
            double a = Math.min(1.0, DT / 0.12);
            vx += (tvx - vx) * a;
            vy += (tvy - vy) * a;
            omega += (tw - omega) * a;
            x += vx * DT;
            y += vy * DT;
            heading = Angles.normalize(heading + omega * DT);
        }

        private int waypointIndex = 0;
        private List<double[]> activeRoute;

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

        void applyFlywheelAndFeeder(AutoShootController.Outputs out) {
            double target = out.flywheelTargetTps * flyMaxFrac;
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
                    if (feedProgress >= 0.45) {
                        feedProgress = 0;
                        fire(out);
                    }
                }
            }
        }

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
            boolean speedOk = flyV / 0.90 >= 0.9 * table.velocityAt(range);
            boolean geometryOk = table.inRange(range) && aimErr <= 6.0 && az <= plannerCfg.coneHalfAngleDeg + 5.0;
            if (robotSide != trueUp) {
                shotsAtDownCell++;
                return;
            }
            if (!speedOk || !geometryOk) {
                badShots++;
                return;
            }
            shotsIntoUpCell++;
            hitsIntoUp++;
            if (hitsIntoUp >= tipAfterHits) {
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

    private static Sim redAtC1() {
        return new Sim(Alliance.RED, Pose2.ofDegrees(58.0, 9.2, -90));
    }

    private static final AutoShootController.State DONE = AutoShootController.State.DONE;
    private static final AutoShootController.State FAULT = AutoShootController.State.FAULT;

    // ---------------------------------------------------------------------------------------------

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
        assertTrue("the flywheel was spun up during the drive, before feeding", s.flyAtFirstFeed > 0.9 * s.last.flywheelTargetTps || s.flyAtFirstFeed > 1000);
        assertTrue(s.visited.contains(AutoShootController.State.ALIGN));
        assertTrue(s.visited.contains(AutoShootController.State.FEED));
        assertTrue("took " + s.t + " s", s.t < 14.0);
        assertEquals(0, s.balls);
    }

    @Test
    public void redFromB1DrivesToTheCellAxisAndShootsAllFour() {
        Sim s = new Sim(Alliance.RED, Pose2.ofDegrees(35.0, 9.2, -90));
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
        assertEquals(0, s.badShots);
    }

    @Test
    public void blueGetsTheMirroredResult() {
        Sim red = redAtC1();
        red.held = true;
        red.run(25, DONE, FAULT);
        Sim blue = new Sim(Alliance.BLUE, Alliance.BLUE.fromRed(Pose2.ofDegrees(58.0, 9.2, -90), 144.0));
        blue.held = true;
        blue.run(25, DONE, FAULT);
        assertEquals(DONE, blue.controller.state());
        assertEquals(4, blue.shotsIntoUpCell);
        Pose2 expected = Alliance.BLUE.fromRed(red.pose(), 144.0);
        assertEquals(expected.x, blue.x, 0.5);
        assertEquals(expected.y, blue.y, 0.5);
    }

    @Test
    public void everyAllianceSideAndStartingCornerWorks() {
        for (Alliance a : Alliance.values()) {
            for (Side up : Side.values()) {
                Sim s = new Sim(a, a.fromRed(Pose2.ofDegrees(58.0, 9.2, -90), 144.0), false);
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

    @Test
    public void aTipMidVolleyPausesTheFeederRetargetsTheOtherSideAndFinishesTheRemainingShots() {
        Sim s = redAtC1();
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
        assertTrue("finished on the far side of the HIVE: y=" + s.y, s.y > 91.0);
    }

    @Test
    public void aWrongBeliefIsCorrectedWithoutFiringAtTheDownCell() {
        Sim s = new Sim(Alliance.RED, Pose2.ofDegrees(58.0, 9.2, -90), false);
        s.trueUp = Side.REAR;                      // the HIVE was tipped earlier; the robot still believes AUDIENCE
        s.step();                                  // one loop with the button up
        s.held = true;
        s.run(60, DONE, FAULT);
        assertEquals("ended in " + s.controller.state() + " " + s.last.message, DONE, s.controller.state());
        assertEquals(0, s.shotsAtDownCell);
        assertEquals(4, s.shotsIntoUpCell);
        assertEquals(Side.REAR, s.tracker.upSide());
    }

    @Test
    public void ifNeitherSideShowsTags_theSequenceStopsInsteadOfFiringBlind() {
        Sim s = new Sim(Alliance.RED, Pose2.ofDegrees(58.0, 9.2, -90), false);
        s.dropoutRate = 1.0;                       // the camera is healthy but never sees a tag
        s.step();                                  // one loop with the button up
        s.held = true;
        s.run(60, DONE, FAULT);
        assertEquals(FAULT, s.controller.state());
        assertEquals(AutoShootController.Fault.HIVE_NOT_VISIBLE, s.controller.fault());
        assertEquals(0, s.shotsFired);
    }

    @Test
    public void withNoCameraAtAllItShootsFromItsBeliefAndTheDriversWord() {
        Sim s = redAtC1();
        s.cameraAlive = false;
        s.held = true;
        s.run(30, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
    }

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
        assertEquals(0.0, s.last.flywheelTargetTps, 0.0);
    }

    @Test
    public void releasingDuringTheFeedStopsTheFeederImmediately() {
        Sim s = redAtC1();
        s.held = true;
        s.run(25, AutoShootController.State.FEED);
        s.run(1.2);
        s.held = false;
        s.step();
        assertFalse(s.last.feederOn);
        assertEquals(AutoShootController.State.IDLE, s.last.state);
    }

    @Test
    public void aHardStickPushCancelsTheAutomaticDrive() {
        Sim s = redAtC1();
        s.held = true;
        s.run(0.3);
        s.stick = 0.9;
        s.run(0.5, FAULT);
        assertEquals(FAULT, s.controller.state());
        assertEquals(AutoShootController.Fault.DRIVER_OVERRIDE, s.controller.fault());
        s.step();
        assertEquals(AutoShootController.DriveMode.MANUAL, s.last.driveMode);
        assertFalse(s.last.feederOn);
    }

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

    @Test
    public void anEmptyHopperEndsTheVolleyWithTheShotsThatWereFired() {
        Sim s = redAtC1();
        s.balls = 2;
        s.held = true;
        s.run(30, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(2, s.controller.shotsFired());
    }

    @Test
    public void withoutATrustedPoseNothingMoves() {
        Sim s = redAtC1();
        s.poseTrusted = false;
        s.held = true;
        s.run(0.5, FAULT);
        assertEquals(FAULT, s.controller.state());
        assertEquals(AutoShootController.Fault.NO_POSE, s.controller.fault());
        assertEquals(58.0, s.x, 1e-9);
    }

    @Test
    public void aFinishedVolleyDoesNotRestartUntilTheButtonIsReleasedAndPressedAgain() {
        Sim s = redAtC1();
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        int shots = s.shotsFired;
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

    @Test
    public void aButtonAlreadyHeldWhenTheOpModeStartsDoesNotAutoStartUntilReleasedAndPressedAgain() {
        Sim s = new Sim(Alliance.RED, Pose2.ofDegrees(58.0, 9.2, -90), false);
        s.held = true;                              // the driver was holding it through INIT
        s.run(1.0);
        assertEquals(AutoShootController.State.IDLE, s.controller.state());
        assertEquals(58.0, s.x, 1e-9);
        s.held = false;
        s.run(0.1);
        s.held = true;
        s.run(25, DONE, FAULT);
        assertEquals(DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
    }

    @Test
    public void visionNoiseAndFrameDropoutsDoNotBreakTheVolley() {
        Sim s = redAtC1();
        s.visionNoiseIn = 0.4;
        s.dropoutRate = 0.2;
        s.held = true;
        s.run(40, DONE, FAULT);
        assertEquals("ended in " + s.controller.state() + " " + s.last.message, DONE, s.controller.state());
        assertEquals(4, s.shotsIntoUpCell);
        assertEquals(0, s.badShots);
    }

    @Test
    public void theFlywheelTargetComesFromTheShotTableForTheActualRange() {
        Sim s = redAtC1();
        s.held = true;
        s.run(25, AutoShootController.State.FEED);
        CellSpec cell = s.field.cell(Alliance.RED, Side.AUDIENCE);
        double range = ShotGeometry.rangeTo(s.pose(), s.plannerCfg.launchForwardIn, s.plannerCfg.launchLeftIn,
                cell.originX, cell.originY);
        assertEquals(s.table.velocityAt(range), s.last.flywheelTargetTps, 25.0);
    }
}
