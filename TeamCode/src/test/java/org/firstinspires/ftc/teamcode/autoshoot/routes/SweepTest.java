package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The sweep guarantee: the validator tests a leg at samples (every 1 in and every 5 degrees) but nothing, not even a
 * 0.02 in thick post, can slip between two samples. The proof is in the class comment of {@link Sweep}; these tests
 * check each step of it and then try to break it with thin obstacles, a robot that turns fast, and a brute-force
 * comparison against sampling a thousand times more densely.
 *
 * <p>Numbers: the farthest corner of the robot is R = 13.22 in from its center of rotation (a front corner: 9.77 in ahead and
 * 8.90 in to the side; the robot is not centered on that point).
 */
public class SweepTest {
    private static final double R = Footprint.circumscribedRadiusIn();
    private static final double EPS = 1e-9;
    private static final double RESOLUTION = BioBuzzConstants.Auto.SWEEP_RESOLUTION_IN;

    // ---- helpers -------------------------------------------------------------------------------------

    private static DrivePath leg(Pose2 from, AutoRoute.Builder b) {
        AutoRoute r = b.build();
        return DrivePath.of(from, (Step.Drive) r.step(r.size() - 1));
    }

    /** Depth of the robot into a rectangle obstacle: above 0 = overlapping by that much. */
    private static Sweep.Depth into(final Rect obstacle) {
        return new Sweep.Depth() {
            @Override
            public double at(Pose2 pose) {
                return -Footprint.of(pose).signedGap(obstacle) - EPS;
            }
        };
    }

    /** A thin post (0.02 in square) centered on a point. */
    private static Rect post(double x, double y) {
        return Rect.centered(x, y, 0.02, 0.02);
    }

    private static boolean anyOverlap(List<Pose2> poses, Rect obstacle) {
        for (Pose2 p : poses) {
            if (Footprint.of(p).overlaps(obstacle)) {
                return true;
            }
        }
        return false;
    }

    // ---- 1. the numbers of the proof ---------------------------------------------------------------------

    @Test
    public void theTravelBoundIsDerivedFromTheStepsAndTheHalfDiagonal() {
        // the farthest corner from the center of rotation: a FRONT corner, 9.77 in ahead and 8.90 in to the side
        double expected = BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN
                + Math.hypot(BioBuzzConstants.Robot.FRONT_EXTENT_IN, BioBuzzConstants.Robot.LEFT_EXTENT_IN)
                * Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG);
        assertEquals(expected, Sweep.maxTravelBetweenSamples(), 1e-12);
        assertEquals("1 in + 13.22 in * 5 degrees", 2.153, Sweep.maxTravelBetweenSamples(), 0.001);
        assertEquals(Sweep.maxTravelBetweenSamples() / 2.0, Sweep.obstacleGrowthIn(), 1e-12);
        // R is the distance to the farthest corner; the two rear corners are nearer (the position is not the robot's middle)
        Footprint fp = Footprint.of(Pose2.ofDegrees(10, 20, 33));
        double farthest = 0.0;
        for (int i = 0; i < 4; i++) {
            double d = fp.corner(i).distanceTo(new Point2(10, 20));
            assertTrue(d <= R + 1e-9);
            farthest = Math.max(farthest, d);
        }
        assertEquals(R, farthest, 1e-9);
        assertEquals("front corners", R, fp.corner(0).distanceTo(new Point2(10, 20)), 1e-9);
        assertEquals("front corners", R, fp.corner(3).distanceTo(new Point2(10, 20)), 1e-9);
        assertEquals("rear corners", Math.hypot(8.03, 8.9), fp.corner(1).distanceTo(new Point2(10, 20)), 1e-9);
    }

    @Test
    public void everyCornerMovesByAtMostTravelWhenThePoseChanges() {
        // step 1 of the proof: a pose change of d inches and t radians moves no point of the robot more than d + R t
        Random rnd = new Random(7);
        for (int k = 0; k < 2000; k++) {
            Pose2 a = new Pose2(rnd.nextDouble() * 100, rnd.nextDouble() * 100, rnd.nextDouble() * 6.0 - 3.0);
            Pose2 b = new Pose2(a.x + rnd.nextGaussian() * 2, a.y + rnd.nextGaussian() * 2, a.heading + rnd.nextGaussian() * 0.3);
            Footprint fa = Footprint.of(a);
            Footprint fb = Footprint.of(b);
            double travel = a.distanceTo(b) + R * Angles.distance(a.heading, b.heading);
            for (int i = 0; i < 4; i++) {
                assertTrue("corner " + i + " of " + a + " -> " + b, fa.corner(i).distanceTo(fb.corner(i)) <= travel + 1e-9);
            }
        }
    }

    @Test
    public void theGapToAnObstacleChangesByNoMoreThanTheTravel() {
        // step 3 of the proof: the signed gap is a distance, so it is 1-Lipschitz in the travel of the robot's points
        Random rnd = new Random(11);
        for (int k = 0; k < 4000; k++) {
            Rect box = Rect.centered(50 + rnd.nextGaussian() * 6, 50 + rnd.nextGaussian() * 6, 0.02 + rnd.nextDouble() * 10,
                    0.02 + rnd.nextDouble() * 10);
            Pose2 a = new Pose2(50 + rnd.nextGaussian() * 12, 50 + rnd.nextGaussian() * 12, rnd.nextDouble() * 6.0 - 3.0);
            Pose2 b = new Pose2(a.x + rnd.nextGaussian(), a.y + rnd.nextGaussian(), a.heading + rnd.nextGaussian() * 0.15);
            double travel = a.distanceTo(b) + R * Angles.distance(a.heading, b.heading);
            double ga = Footprint.of(a).signedGap(box);
            double gb = Footprint.of(b).signedGap(box);
            assertTrue("gap " + ga + " -> " + gb + " for travel " + travel, Math.abs(ga - gb) <= travel + 1e-9);
        }
    }

    @Test
    public void theSignedGapIsADistanceWhenApartAndADepthWhenOverlapping() {
        Rect wall = new Rect(0, 100, -10, 0);
        // robot facing -y, center 20 in above the wall: the FRONT edge is 20 - 9.77 = 10.23 in from it
        assertEquals(10.23, Footprint.of(Pose2.ofDegrees(35, 20, -90)).signedGap(wall), 1e-9);
        // touching: the front edge is 9.77 in from the center of rotation
        assertEquals(0.0, Footprint.of(Pose2.ofDegrees(35, 9.77, -90)).signedGap(wall), 1e-9);
        // 0.4 in into it
        assertEquals(-0.4, Footprint.of(Pose2.ofDegrees(35, 9.37, -90)).signedGap(wall), 1e-9);
        // facing away from the wall the BACK edge is the near one, 8.03 in from the center
        assertEquals(0.0, Footprint.of(Pose2.ofDegrees(35, 8.03, 90)).signedGap(wall), 1e-9);
        assertEquals(-0.4, Footprint.of(Pose2.ofDegrees(35, 7.63, 90)).signedGap(wall), 1e-9);
        // a flat edge facing the corner of a box: a diamond (45 degrees) at (40, 40) has its FRONT edge on x + y = 80 + 9.77 * sqrt 2
        Rect box = new Rect(60, 70, 60, 70);
        double edgeToCorner = (60 + 60 - 80 - 9.77 * Math.sqrt(2.0)) / Math.sqrt(2.0);
        assertEquals(edgeToCorner, Footprint.of(Pose2.ofDegrees(40, 40, 45)).signedGap(box), 1e-9);
        // corner to corner: the axis-aligned robot's front-left corner (49.77, 48.9) and the box corner (60, 60); every flat side is
        // farther apart in only ONE direction (10.23 in in x, 11.1 in in y), so a gap measured only along the four edge directions
        // would be wrong
        assertEquals(Math.hypot(10.23, 11.1), Footprint.of(Pose2.ofDegrees(40, 40, 0)).signedGap(box), 1e-9);
    }

    @Test
    public void theChassisAndTheWheelDiscsNeverReachFartherFromTheCenterThanTheWholeFootprint() {
        // the proof of the sweep uses ONE radius R for every shape of the robot: the chassis and the two wheel discs are
        // parts of the footprint, so a corner of either never travels more than a corner of the footprint does
        Random rnd = new Random(5);
        for (int k = 0; k < 200; k++) {
            Pose2 p = new Pose2(rnd.nextDouble() * 100, rnd.nextDouble() * 100, rnd.nextDouble() * 6.0 - 3.0);
            Footprint[] parts = {Footprint.chassis(p), Footprint.wheelDisc(p, IntakeWheel.LEFT),
                    Footprint.wheelDisc(p, IntakeWheel.RIGHT)};
            for (Footprint part : parts) {
                for (int i = 0; i < 4; i++) {
                    assertTrue("a corner of a part is within R of the center", part.corner(i).distanceTo(new Point2(p.x, p.y)) <= R + EPS);
                }
            }
        }
    }

    // ---- 2. the sampler keeps its promises ------------------------------------------------------------------

    @Test
    public void neighbouringCoarseSamplesAreWithinTheStepAndTheTurnLimit() {
        double stepIn = BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN;
        double stepRad = Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG);
        Pose2 from = Pose2.ofDegrees(30, 30, -90);
        DrivePath[] legs = {
                leg(from, AutoRoute.builder("a").start(30, 30, -90).driveLinear(30, 31, 0)),                       // 90 degrees in 1 in
                leg(from, AutoRoute.builder("b").start(30, 30, -90).driveTangent(60, 70).via(30, 70).via(60, 30)), // S curve
                leg(from, AutoRoute.builder("c").start(30, 30, -90).driveReverseTangent(30, 90)),
                leg(from, AutoRoute.builder("d").start(30, 30, -90).driveFacing(40, 30.5, 36, 31)),               // passes right by the face point
                leg(from, AutoRoute.builder("e").start(30, 30, -90).driveLinear(37.3, 30, 170)),
        };
        for (DrivePath p : legs) {
            List<Double> arcs = p.sampleArcLengths(stepIn, stepRad);
            assertEquals("the start is always a sample", 0.0, arcs.get(0), 0.0);
            assertEquals("the end is always a sample", p.length(), arcs.get(arcs.size() - 1), 1e-12);
            for (int i = 1; i < arcs.size(); i++) {
                assertTrue("ascending", arcs.get(i) > arcs.get(i - 1));
                assertTrue("at most one step apart along the path", arcs.get(i) - arcs.get(i - 1) <= stepIn + 1e-9);
                double turn = Angles.distance(p.headingAt(arcs.get(i - 1)), p.headingAt(arcs.get(i)));
                assertTrue("neighbours turn at most the limit: " + Math.toDegrees(turn), turn <= stepRad + 1e-9);
                assertTrue("and the travel between them stays within the derived bound",
                        (arcs.get(i) - arcs.get(i - 1)) + R * turn <= Sweep.maxTravelBetweenSamples() + 1e-9);
            }
        }
    }

    @Test
    public void theFirstAndTheLastPoseOfEveryLegAreAlwaysTested() {
        // a 4.5 in leg: samples at 0, 1, 2, 3, 4 and the end at 4.5 (not a multiple of the step)
        DrivePath p = leg(Pose2.ofDegrees(0, 50, 0), AutoRoute.builder("a").start(0, 50, 0).driveConstant(4.5, 50, 0));
        // a rule that is only broken beyond x = 4.4: only the last pose can see it
        Sweep.Result r = Sweep.run(p, new Sweep.Depth() {
            @Override
            public double at(Pose2 pose) {
                return pose.x - 4.4;
            }
        });
        assertTrue(r.broken());
        assertEquals("the end of the leg", 4.5, r.lastArc, 1e-9);
        // and a rule that is only broken at the very first pose (x = 0)
        Sweep.Result first = Sweep.run(p, new Sweep.Depth() {
            @Override
            public double at(Pose2 pose) {
                return 0.1 - pose.x;
            }
        });
        assertTrue(first.broken());
        assertEquals("the start of the leg", 0.0, first.firstArc, 1e-9);
    }

    @Test
    public void theLastPoseOfATurnInPlaceIsTestedToo() {
        // 47 degrees is not a multiple of 5: the final 2 degrees must still be looked at
        DrivePath turn = leg(Pose2.ofDegrees(50, 50, 0), AutoRoute.builder("a").start(50, 50, 0).driveConstant(50, 50, 47));
        assertEquals(0.0, turn.length(), 0.0);
        Sweep.Result r = Sweep.run(turn, new Sweep.Depth() {
            @Override
            public double at(Pose2 pose) {
                return R * (pose.heading - Math.toRadians(46.9));      // only the last two degrees break this rule
            }
        });
        assertTrue(r.broken());
        assertEquals(47.0, Math.toDegrees(r.firstPose.heading), 0.01 + 1e-6);
    }

    // ---- 3. a thin obstacle and a fast-turning robot --------------------------------------------------------

    @Test
    public void aThinPostBetweenTwoTurnSamplesIsNotMissed() {
        // The robot turns in place at (50, 50) from 0 to 90 degrees. Its front-left corner is 13.22 in from the center of
        // rotation (9.77 in ahead, 8.90 in left) and a sample every 5 degrees moves it 1.15 in, so a post 0.2 in inside that
        // radius, placed where the corner is at heading 82.5 degrees (halfway between the samples at 80 and 85), sits in the
        // strip the corner sweeps yet in none of the sampled rectangles: at 80 degrees it is 0.3 in beyond the left edge, at 85
        // degrees 0.2 in beyond the front edge.
        DrivePath turn = leg(Pose2.ofDegrees(50, 50, 0), AutoRoute.builder("a").start(50, 50, 0).driveConstant(50, 50, 90));
        double radius = Footprint.circumscribedRadiusIn() - 0.2;
        double psi = Math.toRadians(82.5) + Math.atan2(BioBuzzConstants.Robot.LEFT_EXTENT_IN, BioBuzzConstants.Robot.FRONT_EXTENT_IN);
        Rect thin = post(50 + radius * Math.cos(psi), 50 + radius * Math.sin(psi));

        // the premise: what the old sampler looked at (every 5 degrees plus the leg's own samples) finds nothing
        List<Pose2> oldSamples = turn.turnInPlaceSamples(Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG));
        oldSamples.addAll(turn.sample(BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN));
        assertFalse("the samples alone miss the post", anyOverlap(oldSamples, thin));

        // the sweep still finds it, and at the right heading
        Sweep.Result r = Sweep.run(turn, into(thin));
        assertTrue("the sweep finds the post", r.broken());
        assertEquals("the corner meets the post near 82.5 degrees", 82.5, Math.toDegrees(r.firstPose.heading), 1.5);
        assertTrue("a shallow touch: " + r.worst, r.worst > 0.0 && r.worst < 0.2);
    }

    private static int errors(AutoRoute.Builder b, Rule rule) {
        int n = 0;
        AutoRoute red = b.build();
        int blue = 0;
        for (Issue i : RouteValidator.check(red)) {
            if (i.rule == rule && i.severity == Severity.ERROR) {
                n++;
            }
        }
        for (Issue i : RouteValidator.check(red.forAlliance(Alliance.BLUE))) {
            if (i.rule == rule && i.severity == Severity.ERROR) {
                blue++;
            }
        }
        assertEquals("red and blue agree", n, blue);
        return n;
    }

    @Test
    public void aLegThatTurnsAQuarterTurnInOneInchIsCheckedWhileItTurns() {
        // 1 in long, from -90 to 0 degrees, center about 11.5 in from the audience wall: both END poses are clear
        // (front edge 2.2 in off, side 2.1 in off) but half way the front corner swings out to 13.2 in: 1.7 in through the wall
        AutoRoute.Builder b = AutoRoute.builder("x").start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN + 0.5, -90)
                .driveConstant(58, 12.0, -90).driveLinear(58, 11.0, 0);
        Pose2 start = Pose2.ofDegrees(58, 12.0, -90);
        Pose2 end = Pose2.ofDegrees(58, 11.0, 0);
        Rect field = new Rect(0, 144, 0, 144);
        assertTrue("the start pose is clear", Footprint.of(start).signedOvershoot(field) < 0);
        assertTrue("the end pose is clear", Footprint.of(end).signedOvershoot(field) < 0);
        assertEquals(1, errors(b, Rule.FIELD_BOUNDS));
        // the report names the depth reached mid-turn: R - 11.5 = 1.7 in (the center is about 11.5 in out at the 45 degree heading)
        List<Issue> l = RouteValidator.check(b.build());
        boolean found = false;
        for (Issue i : l) {
            found |= i.rule == Rule.FIELD_BOUNDS && i.message.contains("1.7 in");
        }
        assertTrue(l.toString(), found);
    }

    @Test
    public void aFastTurningCurveBesideAThinWallPost() {
        // a curve that swings the heading 90 degrees in about 6 in of path; the post sits where a corner passes between samples
        DrivePath curve = leg(Pose2.ofDegrees(40, 60, 0), AutoRoute.builder("a").start(40, 60, 0).driveTangent(46, 66).via(46, 60));
        List<Double> arcs = curve.sampleArcLengths(BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN,
                Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG));
        // find a post location that the coarse poses miss but a dense walk along the curve hits
        Random rnd = new Random(3);
        int found = 0;
        for (int k = 0; k < 1500 && found < 3; k++) {
            Rect thin = post(34 + rnd.nextDouble() * 20, 50 + rnd.nextDouble() * 24);
            boolean coarseHit = false;
            for (double s : arcs) {
                coarseHit |= Footprint.of(curve.poseAt(s)).overlaps(thin);
            }
            boolean denseHit = false;
            for (double s = 0; s <= curve.length() + 1e-9 && !denseHit; s += 0.005) {
                denseHit = -Footprint.of(curve.poseAt(Math.min(s, curve.length()))).signedGap(thin) > 0.01;
            }
            if (!coarseHit && denseHit) {
                found++;
                assertTrue("a post only a dense walk finds must be found by the sweep: " + thin, Sweep.run(curve, into(thin)).broken());
            }
        }
        assertTrue("the experiment found posts that the coarse samples miss (" + found + ")", found >= 1);
    }

    // ---- 4. brute force: nothing the dense walk finds can be missed -----------------------------------------

    /**
     * The deepest the robot gets into the obstacle over a dense walk along the leg: first the turn in place in steps of
     * 0.01 degrees, then the drive cut into pieces until no point of the robot moves more than 0.004 in within one
     * (1/250 of the coarse samples). This reference knows nothing about obstacles: it just looks at every piece.
     */
    private static double denseDeepest(DrivePath leg, Rect obstacle) {
        double deepest = Double.NEGATIVE_INFINITY;
        double turn = Math.abs(leg.preTurnSignedRadians());
        double angleStep = Math.toRadians(0.01);
        for (double u = 0.0; u <= turn + 1e-12; u += angleStep) {
            deepest = Math.max(deepest, -Footprint.of(leg.turnPoseAt(Math.min(u, turn))).signedGap(obstacle));
        }
        deepest = Math.max(deepest, -Footprint.of(leg.poseAt(0.0)).signedGap(obstacle));
        return Math.max(deepest, walk(leg, obstacle, 0.0, leg.length(), 0));
    }

    /** Cuts [s0, s1] until the robot moves at most 0.004 in inside a piece, and returns the deepest end pose found. */
    private static double walk(DrivePath leg, Rect obstacle, double s0, double s1, int level) {
        double sm = (s0 + s1) / 2.0;
        double turned = Angles.distance(leg.headingAt(s0), leg.headingAt(sm)) + Angles.distance(leg.headingAt(sm), leg.headingAt(s1));
        double depthAtEnd = -Footprint.of(leg.poseAt(s1)).signedGap(obstacle);
        if ((s1 - s0) + R * turned <= 0.004 || level >= 40) {
            return depthAtEnd;
        }
        return Math.max(walk(leg, obstacle, s0, sm, level + 1), walk(leg, obstacle, sm, s1, level + 1));
    }

    @Test
    public void everyCollisionADenseWalkFindsIsFoundBySweepAndNothingIsInvented() {
        Random rnd = new Random(2026);
        int collisions = 0;
        int clear = 0;
        for (int k = 0; k < 120; k++) {
            Pose2 from = Pose2.ofDegrees(50, 50, rnd.nextInt(360));
            AutoRoute.Builder b = AutoRoute.builder("p").start(50, 50, 0);
            double tx = 50 + rnd.nextGaussian() * 12;
            double ty = 50 + rnd.nextGaussian() * 12;
            double heading = rnd.nextInt(360) - 180;
            switch (k % 4) {
                case 0:
                    b.driveConstant(50, 50, heading);                                   // pure turn in place
                    break;
                case 1:
                    b.driveLinear(tx, ty, heading);                                     // slide while turning evenly
                    break;
                case 2:
                    b.driveTangent(tx, ty).via(50 + rnd.nextGaussian() * 15, 50 + rnd.nextGaussian() * 15);
                    break;
                default:
                    b.driveConstant(tx, ty, heading);                                   // turn first, then slide
                    break;
            }
            DrivePath leg = leg(from, b);
            // a thin wall segment or a post, somewhere the robot could reach
            double width = 0.02 + rnd.nextDouble() * 0.05;
            double length = rnd.nextBoolean() ? width : 0.5 + rnd.nextDouble() * 5;
            Rect obstacle = rnd.nextBoolean()
                    ? Rect.centered(50 + rnd.nextGaussian() * 14, 50 + rnd.nextGaussian() * 14, width, length)
                    : Rect.centered(50 + rnd.nextGaussian() * 14, 50 + rnd.nextGaussian() * 14, length, width);
            double dense = denseDeepest(leg, obstacle);
            Sweep.Result sweep = Sweep.run(leg, new Sweep.Depth() {
                private final Rect o = obstacle;

                @Override
                public double at(Pose2 pose) {
                    return -Footprint.of(pose).signedGap(o);
                }
            });
            if (dense > 2 * RESOLUTION + 0.01) {
                collisions++;
                assertTrue("a collision of depth " + dense + " was missed: leg " + leg.controlPoints() + " obstacle " + obstacle, sweep.broken());
            }
            if (sweep.broken()) {
                assertTrue("an invented collision (case " + k + "): swept depth " + sweep.worst + " but the dense walk reaches only " + dense
                        + " leg " + leg.controlPoints() + " mode " + leg.mode() + " length " + leg.length() + " preturn " + Math.toDegrees(leg.preTurnSignedRadians()) + " obstacle " + obstacle,
                        sweep.worst <= dense + 0.01);
            } else {
                clear++;
                assertTrue("nothing reported, so the dense walk may not be deeper than the resolution: " + dense,
                        dense <= RESOLUTION + 0.01);
            }
        }
        assertTrue("the experiment must contain both outcomes", collisions > 5 && clear > 5);
    }
}
