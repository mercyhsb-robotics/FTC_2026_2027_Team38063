package org.firstinspires.ftc.teamcode.autoshoot.hardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;

import org.junit.Test;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.DrivePath;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Step;

/**
 * The claim behind "what the route checker swept is what the robot drives": the Pedro path that {@code PedroDrive.followLeg} builds from a
 * {@link DrivePath} has, at every point of the leg, the same place and the same heading as the DrivePath. Pedro's path classes are plain Java, so
 * this runs on a PC with no robot. It catches a heading mode that is mapped to the wrong interpolator (a reverse tangent that faces forward), a
 * curve whose control points are used in the wrong order, and a back-facing leg that faces front.
 */
public class PedroLegPathTest {
    private static final double POSITION_TOL_IN = 0.05;
    private static final double HEADING_TOL_DEG = 1.0;

    private static DrivePath leg(Pose2 from, AutoRoute.Builder b) {
        AutoRoute r = b.build();
        return DrivePath.of(from, (Step.Drive) r.step(1));
    }

    private static AutoRoute.Builder route() {
        return AutoRoute.builder("t").start(30, 30, 0);
    }

    /** Checks the Pedro path of {@code leg} against the leg at {@code samples} points along it (not the very ends, where a tangent is not defined). */
    private static void assertSame(String what, DrivePath leg, int samples) {
        Path path = PedroDrive.legPath(leg);
        for (int i = 1; i < samples; i++) {
            double s = leg.length() * i / samples;
            Pose2 expected = leg.poseAt(s);
            double t = path.curve.closestParameter(Vector2D.cartesian(expected.x, expected.y));
            Pose got = path.get(t);
            assertEquals(what + " x at " + i + "/" + samples, expected.x, got.x(), POSITION_TOL_IN);
            assertEquals(what + " y at " + i + "/" + samples, expected.y, got.y(), POSITION_TOL_IN);
            double off = Math.toDegrees(Angles.distance(expected.heading, got.heading()));
            assertTrue(what + ": heading " + Math.toDegrees(got.heading()) + " but the leg says " + Math.toDegrees(expected.heading) + " at " + i + "/" + samples,
                    off <= HEADING_TOL_DEG);
        }
    }

    @Test
    public void aTangentLegFacesTheWayItDrivesOnALineAndOnACurve() {
        assertSame("tangent line", leg(new Pose2(30, 30, 0), route().driveTangent(80, 60)), 12);
        assertSame("tangent curve", leg(new Pose2(30, 30, 0), route().driveTangent(80, 60).via(60, 20)), 12);
    }

    @Test
    public void aReverseTangentLegDrivesBackwardsShooterFirstOnALineAndOnACurve() {
        assertSame("reverse tangent line", leg(new Pose2(30, 30, 0), route().driveReverseTangent(80, 60)), 12);
        assertSame("reverse tangent curve", leg(new Pose2(30, 30, 0), route().driveReverseTangent(80, 60).via(60, 20)), 12);
        assertSame("reverse tangent cubic", leg(new Pose2(30, 30, 0), route().driveReverseTangent(80, 60).via(60, 20).via(50, 70)), 12);
    }

    @Test
    public void aConstantLegHoldsTheHeadingTheStepNames() {
        assertSame("constant", leg(new Pose2(30, 30, 0), route().driveConstant(80, 60, 135)), 12);
        assertSame("constant curve", leg(new Pose2(30, 30, 0), route().driveConstant(80, 60, -90).via(60, 20)), 12);
    }

    @Test
    public void aLinearLegTurnsEvenlyTheShortWayRound() {
        assertSame("linear", leg(new Pose2(30, 30, Math.toRadians(170)), route().driveLinear(80, 60, -170)), 12);
        assertSame("linear the other way", leg(new Pose2(30, 30, Math.toRadians(-20)), route().driveLinear(80, 60, 100)), 12);
    }

    @Test
    public void aLegThatFacesAPointKeepsItsFrontOrItsBackToIt() {
        assertSame("facing a point", leg(new Pose2(30, 30, 0), route().driveFacing(80, 40, 60, 90)), 12);
        assertSame("back facing a point", leg(new Pose2(30, 30, 0), route().driveBackFacing(80, 40, 60, 90)), 12);
        assertSame("back facing a point, curved", leg(new Pose2(30, 30, 0), route().driveBackFacing(80, 40, 60, 90).via(55, 30)), 12);
    }

    @Test
    public void everyHeadingModeIsCovered() {
        // a new heading mode must get a case in legPath and a line in this test
        assertEquals(Arrays.asList("TANGENT", "REVERSE_TANGENT", "LINEAR", "CONSTANT", "FACE_POINT"),
                Arrays.asList(names(org.firstinspires.ftc.teamcode.autoshoot.routes.HeadingMode.values())));
    }

    private static String[] names(Enum<?>[] values) {
        String[] out = new String[values.length];
        for (int i = 0; i < values.length; i++) {
            out[i] = values[i].name();
        }
        return out;
    }
}
