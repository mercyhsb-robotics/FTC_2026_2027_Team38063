package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.junit.Test;

/** Path geometry: length, sampling and the heading each heading mode produces. */
public class DrivePathTest {
    private static final double EPS = 1e-6;

    private static Step.Drive drive(AutoRoute.Builder b) {
        AutoRoute r = b.build();
        return (Step.Drive) r.step(r.size() - 1);
    }

    private static DrivePath path(Pose2 from, AutoRoute.Builder b) {
        return DrivePath.of(from, drive(b));
    }

    private static void assertHeadingDeg(double expectedDeg, double actualRad) {
        double err = Math.toDegrees(Angles.error(Math.toRadians(expectedDeg), actualRad));
        assertEquals("heading " + Math.toDegrees(actualRad) + " vs " + expectedDeg, 0.0, err, 1e-6);
    }

    @Test
    public void aStraightLegHasTheLengthOfItsChord() {
        DrivePath p = path(Pose2.ofDegrees(0, 0, 0),
                AutoRoute.builder("a").start(0, 0, 0).driveTangent(3, 4));
        assertEquals(5.0, p.length(), 1e-9);
        assertEquals(1, p.degree());
        assertEquals(3.0, p.end().x, EPS);
        assertEquals(4.0, p.end().y, EPS);
    }

    @Test
    public void samplingIncludesBothEndsAndIsSpacedOneInchApart() {
        DrivePath ten = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveTangent(10, 0));
        List<Pose2> s = ten.sample(1.0);
        assertEquals(11, s.size());
        assertEquals(0.0, s.get(0).x, EPS);
        assertEquals(10.0, s.get(10).x, EPS);
        for (int i = 1; i < s.size(); i++) {
            assertEquals(1.0, s.get(i).distanceTo(s.get(i - 1)), 1e-6);
        }
        DrivePath odd = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveTangent(4.5, 0));
        List<Pose2> o = odd.sample(1.0);
        assertEquals("0,1,2,3,4 and the end at 4.5", 6, o.size());
        assertEquals(4.5, o.get(5).x, EPS);
        assertEquals(o.size(), odd.sampleArcLengths(1.0).size());
    }

    @Test
    public void aViaPointBendsTheLegIntoABezierCurve() {
        // quadratic Bezier (0,0) -> control (10,0) -> (10,10)
        DrivePath p = path(Pose2.ofDegrees(0, 0, 0),
                AutoRoute.builder("a").start(0, 0, 0).driveTangent(10, 10).via(10, 0));
        assertEquals(2, p.degree());
        // numeric length of B(t) = (1-t)^2 P0 + 2(1-t)t P1 + t^2 P2
        double len = 0;
        double px = 0;
        double py = 0;
        for (int i = 1; i <= 200000; i++) {
            double t = i / 200000.0;
            double x = 2 * (1 - t) * t * 10 + t * t * 10;
            double y = t * t * 10;
            len += Math.hypot(x - px, y - py);
            px = x;
            py = y;
        }
        assertEquals(len, p.length(), 1e-3);
        // the curve passes through B(0.5) = (7.5, 2.5) at its arc-length middle (symmetric curve)
        Pose2 mid = p.poseAt(p.length() / 2.0);
        assertEquals(7.5, mid.x, 1e-2);
        assertEquals(2.5, mid.y, 1e-2);
        // and does not pass through the control point itself
        assertTrue(mid.distanceTo(10, 0) > 2.0);
        // the tangent heading swings from along +x to along +y
        assertHeadingDeg(0.0, p.poseAt(0).heading);
        assertHeadingDeg(90.0, p.end().heading);
    }

    @Test
    public void tangentAndReverseTangentFollowTheTravelDirection() {
        Pose2 from = Pose2.ofDegrees(0, 0, 0);
        assertHeadingDeg(90.0, path(from, AutoRoute.builder("a").start(0, 0, 0).driveTangent(0, 10)).end().heading);
        assertHeadingDeg(-90.0,
                path(from, AutoRoute.builder("a").start(0, 0, 0).driveReverseTangent(0, 10)).end().heading);
        // driving toward -x backwards leaves the robot facing +x: the B1 and C1 park legs end at heading 0
        assertHeadingDeg(0.0,
                path(Pose2.ofDegrees(35.8, 90, -90), AutoRoute.builder("a").start(0, 0, 0).driveReverseTangent(12, 90, 0))
                        .end().heading);
    }

    @Test
    public void tangentModesIgnoreTheTargetHeadingTheOldCodeWrote() {
        DrivePath a = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveReverseTangent(10, 0, 0));
        DrivePath b = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveReverseTangent(10, 0, 77));
        assertEquals(a.end().heading, b.end().heading, EPS);
    }

    @Test
    public void linearTurnsEvenlyTheShortWayRoundIncludingAcrossTheSeam() {
        DrivePath quarter = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveLinear(10, 0, 90));
        assertHeadingDeg(0.0, quarter.poseAt(0).heading);
        assertHeadingDeg(45.0, quarter.poseAt(5).heading);
        assertHeadingDeg(90.0, quarter.end().heading);
        assertEquals(Math.PI / 2, quarter.totalTurnRadians(), 1e-6);

        // 170 -> -170 is a 20 degree turn through 180, not a 340 degree one through 0
        DrivePath seam = path(Pose2.ofDegrees(0, 0, 170), AutoRoute.builder("a").start(0, 0, 0).driveLinear(10, 0, -170));
        assertHeadingDeg(180.0, seam.poseAt(5).heading);
        assertEquals(Math.toRadians(20), seam.totalTurnRadians(), 1e-6);
    }

    @Test
    public void constantTurnsFirstThenHoldsTheHeading() {
        DrivePath p = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveConstant(10, 10, -90));
        for (Pose2 s : p.sample(1.0)) {
            assertHeadingDeg(-90.0, s.heading);
        }
        assertEquals("all the turning happens in place at the start", Math.PI / 2, p.preTurnRadians(), 1e-6);
        assertEquals(Math.PI / 2, p.totalTurnRadians(), 1e-6);
        List<Pose2> turn = p.turnInPlaceSamples(Math.toRadians(5));
        assertEquals(18, turn.size());
        assertEquals(0.0, turn.get(0).x, EPS);
        assertHeadingDeg(-90.0, turn.get(turn.size() - 1).heading);
    }

    @Test
    public void facePointKeepsTheFrontOrTheBackOnThePoint() {
        // robot drives along y = 0 from x = 0 to x = 10 looking at (5, 10)
        DrivePath front = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveFacing(10, 0, 5, 10));
        assertHeadingDeg(Math.toDegrees(Math.atan2(10, 5)), front.poseAt(0).heading);
        assertHeadingDeg(90.0, front.poseAt(5).heading);
        assertHeadingDeg(Math.toDegrees(Math.atan2(10, -5)), front.end().heading);
        DrivePath back = path(Pose2.ofDegrees(0, 0, 0), AutoRoute.builder("a").start(0, 0, 0).driveBackFacing(10, 0, 5, 10));
        assertHeadingDeg(-90.0, back.poseAt(5).heading);
    }

    @Test
    public void theTurnInPlaceBeforeALegIsThePreviousHeadingToTheLegsFirstHeading() {
        // B1: hold at -115 deg, then reverse tangent along +y wants -90 deg
        DrivePath b1 = path(Pose2.ofDegrees(35, 9.2, -115),
                AutoRoute.builder("a").start(0, 0, 0).driveReverseTangent(35.8, 90, -90));
        assertEquals(25.0, Math.toDegrees(b1.preTurnRadians()), 0.6);
        // C1 path2: after path1 the robot faces about -91 and path2 wants 0: a 91 degree turn inside a 4 in leg
        DrivePath c1 = path(Pose2.ofDegrees(62, 118.1, -91.1),
                AutoRoute.builder("a").start(0, 0, 0).driveReverseTangent(58, 118.1, -90));
        assertEquals(4.0, c1.length(), 1e-9);
        assertEquals(91.1, Math.toDegrees(c1.preTurnRadians()), 0.05);
    }

    @Test
    public void aLegOfZeroLengthIsASinglePoseAndStillTurnsToItsTargetHeading() {
        DrivePath p = path(Pose2.ofDegrees(10, 10, 0), AutoRoute.builder("a").start(0, 0, 0).driveLinear(10, 10, 90));
        assertEquals(0.0, p.length(), EPS);
        assertEquals(1, p.sample(1.0).size());
        assertHeadingDeg(90.0, p.end().heading);
        // tangent of nothing keeps the old heading instead of inventing one
        DrivePath t = path(Pose2.ofDegrees(10, 10, 33), AutoRoute.builder("a").start(0, 0, 0).driveTangent(10, 10));
        assertHeadingDeg(33.0, t.end().heading);
    }

    @Test
    public void anImplicitMoveIsAStraightLinearMove() {
        DrivePath p = DrivePath.straight(Pose2.ofDegrees(0, 0, 0), Pose2.ofDegrees(30, 40, 90));
        assertEquals(50.0, p.length(), 1e-9);
        assertEquals(HeadingMode.LINEAR, p.mode());
        assertHeadingDeg(90.0, p.end().heading);
    }
}
