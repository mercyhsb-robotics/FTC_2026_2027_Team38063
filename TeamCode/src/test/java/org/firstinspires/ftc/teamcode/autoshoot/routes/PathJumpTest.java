package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.junit.Test;

/**
 * A leg whose heading JUMPS: the robot would have to turn by a large angle in no distance at all. It happens when a student treats {@code via()} as a point to
 * pass through (a point beyond the target makes the path turn back on itself, a cusp, in the two tangent modes), or when a leg that keeps its front or back
 * toward a point drives exactly through that point. The validator used to cut a gap in half again and again around such a jump (a jump of about 180 degrees
 * keeps the robot's travel bound at about 41 in however small the gap) and ended in a StackOverflowError, although its own comment says that it "never throws
 * for a bad route; it reports".
 */
public class PathJumpTest {
    private static List<Issue> check(AutoRoute r) {
        return RouteValidator.check(r);
    }

    private static boolean hasPathShapeError(List<Issue> issues, int stepIndex) {
        for (Issue i : issues) {
            if (i.rule == Rule.PATH_SHAPE && i.isError() && i.stepIndex == stepIndex) {
                return true;
            }
        }
        return false;
    }

    /**
     * Situation: a forward-tangent leg whose via point lies past its target, so the path runs out and back along the same line.
     * Expected: the check returns (it does not overflow the stack) and says, as an ERROR of the path shape on that step, that the robot cannot follow it.
     */
    @Test
    public void aCuspInAForwardTangentLegIsReportedNotACrash() {
        AutoRoute r = AutoRoute.builder("cusp").start(9.77, 100, 180).driveTangent(30, 100).via(60, 100).build();
        List<Issue> issues = check(r);
        assertTrue(issues.toString(), hasPathShapeError(issues, 1));
    }

    /** As above for a backwards (reverse-tangent) leg from the middle of the own half. */
    @Test
    public void aCuspInAReverseTangentLegIsReportedNotACrash() {
        AutoRoute r = AutoRoute.builder("cusp_reverse").start(30, 30, 0).driveReverseTangent(20, 30).via(60, 30).build();
        List<Issue> issues = check(r);
        assertTrue(issues.toString(), hasPathShapeError(issues, 1));
    }

    /**
     * Situation: a leg that keeps its front toward a point and drives exactly through that point.
     * Expected: the same, an ERROR instead of a crash: the bearing to the point flips by 180 degrees there.
     */
    @Test
    public void aLegThroughItsOwnFacingPointIsReportedNotACrash() {
        AutoRoute r = AutoRoute.builder("through_point").start(20, 100, 0).driveFacing(40, 100, 30, 100).build();
        List<Issue> issues = check(r);
        assertTrue(issues.toString(), hasPathShapeError(issues, 1));
    }

    /** The full report, not only the quick first-error check, also returns. */
    @Test
    public void theFullReportAlsoReturns() {
        AutoRoute r = AutoRoute.builder("cusp").start(9.77, 100, 180).driveTangent(30, 100).via(60, 100).build();
        assertTrue(new RouteValidator().validate(r).toString(), hasPathShapeError(new RouteValidator().validate(r), 1));
        assertTrue(StartRobustness.check(r).nominalErrors.size() > 0);
    }

    /**
     * Negative twin: legs that turn a lot but smoothly, and a plain straight leg, are NOT reported as a jump. A hairpin that turns the robot 180 degrees over
     * a few inches is a sharp curve, not a jump.
     */
    @Test
    public void sharpButSmoothCurvesAndStraightLegsAreNotJumps() {
        AutoRoute straight = AutoRoute.builder("straight").start(30, 30, 90).driveConstant(30, 60, 90).build();
        assertFalse(check(straight).toString(), hasPathShapeError(check(straight), 1));
        AutoRoute hairpin = AutoRoute.builder("hairpin").start(20, 40, 0).driveTangent(20, 50).via(32, 40).via(32, 50).build();
        assertFalse(check(hairpin).toString(), hasPathShapeError(check(hairpin), 1));
        AutoRoute turning = AutoRoute.builder("turning").start(30, 30, 0).driveLinear(20, 40, 180).build();
        assertFalse(check(turning).toString(), hasPathShapeError(check(turning), 1));
    }

    /**
     * Situation: the sweep itself, given a motion whose travel bound can never be proven (a jump anywhere in it).
     * Expected: it stops after the limit of cuts instead of recursing without end, and says that it could not prove the gap.
     */
    @Test
    public void theSweepStopsAtTheLimitOfCutsAndSaysItCouldNotProveTheGap() {
        Sweep.Motion jumpy = new Sweep.Motion() {
            @Override
            public Pose2 poseAt(double u) {
                return new Pose2(70.0, 70.0, 0.0);
            }

            @Override
            public double travel(double u0, double u1) {
                return 41.0;                       // a jump: the bound does not shrink with the gap
            }

            @Override
            public double arc(double u) {
                return u;
            }
        };
        Sweep.Depth nearAnObstacle = new Sweep.Depth() {
            @Override
            public double at(Pose2 pose) {
                return -1.0;                       // clear by 1 in, but the bound says it may be 20 in deep
            }
        };
        Sweep.Result result = new Sweep.Result();
        Sweep.run(jumpy, java.util.Arrays.asList(0.0, 10.0), nearAnObstacle, true, 1, result);
        assertTrue("could not be proven", result.unprovable);
        assertEquals("not reported as a collision: nothing was found there", 0, result.count);
        assertTrue(BioBuzzConstants.Auto.SWEEP_MAX_CUTS >= 20);
    }
}
