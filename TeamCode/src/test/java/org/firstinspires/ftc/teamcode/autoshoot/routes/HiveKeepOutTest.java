package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotShape;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/**
 * The HIVE keep-out: a superset of the frame's floor footprint plus a margin, checked edge by edge and corner by
 * corner. Why a footprint and not the 3D shape: the robot is 12.13 in tall, the lowest point of any cell is 30.6 in
 * up, so only the frame (rails and legs, all inside its footprint) can be hit. Anything under or inside the frame is
 * treated as solid.
 *
 * <p>Numbers (official CAD, wall-corner frame): the frame footprint is x 45.94 to 95.41 and y 51.20 to 90.15 (the two foot
 * bars, 49.466 x 38.944 in around the field center). With the 3.0 in margin the keep-out is x 42.94 to 98.41 and y 48.20 to
 * 93.15. The robot is 17.80 x 17.80 in but its
 * position (its center of rotation) is not in the middle of it: the front edge is 9.77 in from the position, the back edge
 * 8.03 in and each side 8.90 in, so how close the position may come to the keep-out depends on which way the robot faces.
 */
public class HiveKeepOutTest {
    private static final double MARGIN = BioBuzzConstants.Auto.HIVE_KEEPOUT_MARGIN_IN;
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double REAR = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double SIDE = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
    private static final RobotShape ROBOT = RobotShape.fromConstants();
    private static final double EPS = 1e-9;

    private final AllianceField red = new AllianceField(new FieldModel(), Alliance.RED);
    private final Rect frame = red.hiveFrame();
    private final Rect keepOut = red.hiveKeepOut();

    private static boolean touchesOrOverlaps(Pose2 pose, Rect r) {
        return Footprint.of(pose).overlaps(r);
    }

    // ---- 1. the keep-out is a superset of the floor footprint plus a margin -----------------------------------

    @Test
    public void theFrameFootprintIsTheManualsSizeAroundTheFieldCenter() {
        double mid = BioBuzzConstants.Field.CENTER_IN;
        assertEquals(mid - 49.466 / 2.0, frame.minX, 1e-9);
        assertEquals(mid + 49.466 / 2.0, frame.maxX, 1e-9);
        assertEquals(mid - 38.944 / 2.0, frame.minY, 1e-9);
        assertEquals(mid + 38.944 / 2.0, frame.maxY, 1e-9);
    }

    @Test
    public void theKeepOutIsTheFrameGrownByTheMarginOnEverySide() {
        assertEquals(frame.minX - MARGIN, keepOut.minX, EPS);
        assertEquals(frame.maxX + MARGIN, keepOut.maxX, EPS);
        assertEquals(frame.minY - MARGIN, keepOut.minY, EPS);
        assertEquals(frame.maxY + MARGIN, keepOut.maxY, EPS);
        assertTrue("a real margin", MARGIN >= 1.0);
        // a superset: every corner and every edge point of the frame is inside the keep-out, and not on its boundary
        double[][] corners = {{frame.minX, frame.minY}, {frame.maxX, frame.minY}, {frame.maxX, frame.maxY}, {frame.minX, frame.maxY}};
        for (double[] c : corners) {
            assertTrue(keepOut.containsStrictly(c[0], c[1]));
        }
        for (double t = 0.0; t <= 1.0; t += 0.05) {
            assertTrue(keepOut.containsStrictly(frame.minX + t * (frame.maxX - frame.minX), frame.minY));
            assertTrue(keepOut.containsStrictly(frame.minX + t * (frame.maxX - frame.minX), frame.maxY));
            assertTrue(keepOut.containsStrictly(frame.minX, frame.minY + t * (frame.maxY - frame.minY)));
            assertTrue(keepOut.containsStrictly(frame.maxX, frame.minY + t * (frame.maxY - frame.minY)));
        }
    }

    @Test
    public void theKeepOutCoversTheFramesMeasuredEnvelopeWithRoomToSpare() {
        // official CAD: everything of the frame that is lower than the robot (a plan "shadow" of the foot bars, the leg stubs and
        // the bolts at floor level) spans x 47.26 to 96.76 and y 52.52 to 91.48 in the centre-72 numbering, which is x 45.934 to
        // 95.434 and y 51.194 to 90.154 in the wall-corner frame: at most 0.027 in beyond the foot bars. The field itself is good
        // to about +/- 1 in.
        double[] measured = {45.934, 95.434, 51.194, 90.154};
        double fieldTolerance = 1.0;
        assertTrue(keepOut.minX <= measured[0] - fieldTolerance);
        assertTrue(keepOut.maxX >= measured[1] + fieldTolerance);
        assertTrue(keepOut.minY <= measured[2] - fieldTolerance);
        assertTrue(keepOut.maxY >= measured[3] + fieldTolerance);
        double worstOverhang = Math.max(Math.max(frame.minX - measured[0], measured[1] - frame.maxX),
                Math.max(frame.minY - measured[2], measured[3] - frame.maxY));
        assertEquals("the CAD's worst overhang of the low parts beyond the foot bars", 0.027, worstOverhang, 0.002);
        assertTrue(MARGIN >= worstOverhang + fieldTolerance);
    }

    @Test
    public void bothAlliancesSeeTheSameKeepOutBecauseTheFrameIsSymmetricAboutTheFieldCenter() {
        Rect blue = new AllianceField(new FieldModel(), Alliance.BLUE).hiveKeepOut();
        assertEquals(keepOut.minX, blue.minX, EPS);
        assertEquals(keepOut.maxX, blue.maxX, EPS);
        assertEquals(keepOut.minY, blue.minY, EPS);
        assertEquals(keepOut.maxY, blue.maxY, EPS);
    }

    // ---- 2. every edge: touching is legal, 0.05 in closer is not -------------------------------------------------

    @Test
    public void eachEdgeOfTheKeepOutStopsTheRobot() {
        double mid = BioBuzzConstants.Field.CENTER_IN;
        // heading 0 faces +x, 90 faces +y. The robot is NOT the same distance from its center to every side: how close the center
        // may come to the keep-out edge is the reach of the side that faces it, 9.77 in for the front, 8.03 in for the back and
        // 8.90 in for a side
        Object[][] approaches = {
                // name, direction from the robot toward the frame (radians), and the way to move toward it (dx, dy)
                {"left edge", 0.0, 1.0, 0.0},
                {"right edge", Math.PI, -1.0, 0.0},
                {"audience edge", Math.PI / 2.0, 0.0, 1.0},
                {"rear edge", -Math.PI / 2.0, 0.0, -1.0},
        };
        for (Object[] a : approaches) {
            double dir = (Double) a[1];
            double dx = (Double) a[2];
            double dy = (Double) a[3];
            for (double heading : new double[] {0, 90, 180, -90}) {
                double reach = ROBOT.reachTowardIn(Math.toRadians(heading), dir);
                double x = dx > 0 ? keepOut.minX - reach : dx < 0 ? keepOut.maxX + reach : mid;
                double y = dy > 0 ? keepOut.minY - reach : dy < 0 ? keepOut.maxY + reach : mid;
                assertFalse(a[0] + " touching at heading " + heading, touchesOrOverlaps(Pose2.ofDegrees(x, y, heading), keepOut));
                assertTrue(a[0] + " 0.05 in closer at heading " + heading,
                        touchesOrOverlaps(Pose2.ofDegrees(x + 0.05 * dx, y + 0.05 * dy, heading), keepOut));
                assertEquals(a[0] + " gap", 0.0, Footprint.of(Pose2.ofDegrees(x, y, heading)).signedGap(keepOut), 1e-9);
            }
        }
    }

    @Test
    public void eachEdgeIsAlsoCaughtByTheValidatorOnALeg() {
        // left edge (blue: the right edge): drive up the corridor 0.1 in past the limit, and exactly to it
        // (facing -90 the robot's +x side is a side edge, 8.90 in from the center)
        double leftLimit = keepOut.minX - SIDE;     // 34.04
        double startX = leftLimit - 1.0;
        assertEquals(0, hiveErrors(AutoRoute.builder("x").start(startX, FRONT, -90).driveConstant(leftLimit, 90, -90)));
        assertEquals(1, hiveErrors(AutoRoute.builder("x").start(startX, FRONT, -90).driveConstant(leftLimit + 0.1, 90, -90)));
        // audience edge (blue: the rear edge): drive along the front of the frame toward it from the audience side; facing -90
        // the BACK of the robot leads, 8.03 in from the center
        double audienceLimit = keepOut.minY - REAR;     // 40.17
        assertEquals(0, hiveErrors(AutoRoute.builder("x").start(58, FRONT, -90).driveConstant(58, audienceLimit, -90)));
        assertEquals(1, hiveErrors(AutoRoute.builder("x").start(58, FRONT, -90).driveConstant(58, audienceLimit + 0.1, -90)));
    }

    // ---- 3. every corner -----------------------------------------------------------------------------------------

    @Test
    public void eachCornerOfTheKeepOutStopsTheRobot() {
        double[][] corners = {
                {keepOut.minX, keepOut.minY, -1, -1}, {keepOut.maxX, keepOut.minY, 1, -1},
                {keepOut.maxX, keepOut.maxY, 1, 1}, {keepOut.minX, keepOut.maxY, -1, 1}};
        for (double[] c : corners) {
            // an axis-aligned robot (facing +x) whose own corner just reaches the keep-out corner from outside, diagonally: on the
            // side of the corner where it is toward +x it reaches 9.77 in (its front), toward -x 8.03 in (its back), and 8.90 in in y
            double x = c[0] + c[2] * (c[2] < 0 ? FRONT : REAR);
            double y = c[1] + c[3] * SIDE;
            Pose2 touching = Pose2.ofDegrees(x, y, 0);
            assertFalse("corner touching", touchesOrOverlaps(touching, keepOut));
            assertEquals("corner to corner", 0.0, Footprint.of(touching).signedGap(keepOut), 1e-9);
            assertTrue("0.05 in further in", touchesOrOverlaps(Pose2.ofDegrees(x - 0.05 * c[2], y - 0.05 * c[3], 0), keepOut));
            // pulled back along the diagonal the gap is the corner-to-corner distance, not a smaller edge distance
            double back = 2.0;
            assertEquals(back * Math.sqrt(2.0), Footprint.of(Pose2.ofDegrees(x + back * c[2], y + back * c[3], 0)).signedGap(keepOut), 1e-9);
            // a robot turned 45 degrees (a diamond) meets the corner with the middle of a flat side instead: whichever side faces
            // the corner, 9.77 in (front), 8.03 in (back) or 8.90 in (a side) from the center
            double reach = ROBOT.reachTowardIn(Math.toRadians(45.0), Math.atan2(-c[3], -c[2]));
            double diagX = c[0] + c[2] * reach / Math.sqrt(2.0);
            double diagY = c[1] + c[3] * reach / Math.sqrt(2.0);
            Pose2 diamond = Pose2.ofDegrees(diagX, diagY, 45.0);
            assertEquals("diamond side touching the corner", 0.0, Footprint.of(diamond).signedGap(keepOut), 1e-9);
            assertFalse(touchesOrOverlaps(diamond, keepOut));
            assertTrue("0.05 in further in", touchesOrOverlaps(
                    Pose2.ofDegrees(diagX - 0.05 * c[2], diagY - 0.05 * c[3], 45.0), keepOut));
        }
    }

    @Test
    public void theCornersAreCaughtByTheValidatorToo() {
        // facing -90: the robot's +x side is a side edge (8.90 in), its +y side the BACK (8.03 in), its -y side the FRONT (9.77 in)
        double lx = keepOut.minX - SIDE;
        double startX = lx - 1.0;                       // starts clear of the keep-out, one inch to the red-wall side of the limit
        double nearY = keepOut.minY - REAR;            // audience-side corner of the left edge
        assertEquals("corner to corner is touching", 0, hiveErrors(AutoRoute.builder("x").start(startX, FRONT, -90).driveConstant(lx, nearY, -90)));
        assertEquals("0.15 in into the corner", 1, hiveErrors(AutoRoute.builder("x").start(startX, FRONT, -90).driveConstant(lx + 0.15, nearY + 0.15, -90)));
        double farY = keepOut.maxY + FRONT;            // rear-side corner of the left edge
        assertEquals("corner to corner is touching", 0, hiveErrors(AutoRoute.builder("x").start(startX, FRONT, -90).driveConstant(lx, farY, -90)));
        assertEquals("0.15 in into the corner", 1, hiveErrors(AutoRoute.builder("x").start(startX, FRONT, -90).driveConstant(lx + 0.15, farY - 0.15, -90)));
    }

    // ---- 4. a gap under or inside the frame is solid -------------------------------------------------------------

    @Test
    public void everyPoseWithItsCenterInsideTheFrameIsInsideTheKeepOut() {
        for (double x = frame.minX; x <= frame.maxX + EPS; x += 1.0) {
            for (double y = frame.minY; y <= frame.maxY + EPS; y += 1.0) {
                for (double heading = 0; heading < 360; heading += 45) {
                    assertTrue("(" + x + ", " + y + ") at " + heading, touchesOrOverlaps(Pose2.ofDegrees(x, y, heading), keepOut));
                }
            }
        }
    }

    @Test
    public void theOpenWindowUnderTheCrossbarIsNotAPath() {
        // x = 57.924 is the red HIVE axis: between the legs and under the crossbar a 12.13 in robot DOES physically fit (the CAD
        // lane is 40 in wide); by default the validator still calls a leg through it an ERROR, and so a leg through the gap between
        // the two HIVEs (x = 70.674). Only a step that says allowUnderHive may use the lane (see HiveLaneTest).
        for (double x : new double[] {BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, 61.0, 66.0}) {
            List<Issue> l = RouteValidator.check(AutoRoute.builder("x").start(58, FRONT, -90).driveConstant(x, 100, -90).build());
            assertTrue("x = " + x + ": " + l, count(l, Rule.HIVE_KEEPOUT) >= 1);
        }
    }

    @Test
    public void routesMayNeitherEndNorStartInsideTheKeepOutRegionOfTheFrame() {
        List<Issue> l = RouteValidator.check(AutoRoute.builder("x").start(58, FRONT, -90).driveConstant(60, 70, 0).build());
        assertTrue(l.toString(), count(l, Rule.END_POSE) == 1);
    }

    // ---- 5. what is deliberately NOT in the keep-out -------------------------------------------------------------

    @Test
    public void aDownCellHangsOverTheFrameButOnlyHighAboveTheRobot() {
        double lowestCellPointIn = 30.652;         // official CAD: the rim of a DOWN cell, the lowest point of any cell
        assertTrue("the tallest part of the robot is far below the lowest cell",
                BioBuzzConstants.Robot.HEIGHT_IN + 10.0 < lowestCellPointIn);
        assertTrue("and so is the camera", BioBuzzConstants.Vision.CAMERA_HEIGHT_IN + 10.0 < lowestCellPointIn);
        // the red rear DOWN cell reaches y 95.80, past the frame (90.15) and past the keep-out (93.15): the robot may stand
        // under that overhang (it is at 35.5 in or more above the floor), which is why it is not solid
        Rect down = red.cellPlan(Alliance.RED, Side.REAR, false);
        assertEquals(95.804, down.maxY, 1e-9);
        assertTrue(down.maxY > keepOut.maxY);
        assertEquals("the overhang past the frame is the 5.66 in of the CAD", 5.658, down.maxY - frame.maxY, 0.01);
        // the UP cell (lowest point about 44 in) also pokes out of the frame by 0.59 in: not solid either
        Rect up = red.cellPlan(Alliance.RED, Side.AUDIENCE, true);
        assertTrue(up.minY < frame.minY);
        assertTrue(up.minY > keepOut.minY);
    }

    // ---- helpers ---------------------------------------------------------------------------------------------------

    private static int count(List<Issue> issues, Rule rule) {
        int n = 0;
        for (Issue i : issues) {
            if (i.rule == rule && i.severity == Severity.ERROR) {
                n++;
            }
        }
        return n;
    }

    /** HIVE keep-out ERRORs for the route, for RED and for its BLUE half turn (they must agree). */
    private static int hiveErrors(AutoRoute.Builder b) {
        AutoRoute route = b.build();
        int redErrors = count(RouteValidator.check(route), Rule.HIVE_KEEPOUT);
        int blueErrors = count(RouteValidator.check(route.forAlliance(Alliance.BLUE)), Rule.HIVE_KEEPOUT);
        assertEquals("red and blue agree", redErrors, blueErrors);
        return redErrors;
    }
}
