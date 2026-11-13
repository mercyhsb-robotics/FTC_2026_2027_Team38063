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
 * <p>Numbers: the frame footprint is x 47.27 to 96.73 and y 52.53 to 91.47 (49.46 x 38.95 in around the field center).
 * With the 3.0 in margin the keep-out is x 44.27 to 99.73 and y 49.53 to 94.47. The robot is a 17.80 in square: half
 * of it, 8.9 in, is the distance from its center to each side.
 */
public class HiveKeepOutTest {
    private static final double MARGIN = BioBuzzConstants.Auto.HIVE_KEEPOUT_MARGIN_IN;
    private static final double HALF = BioBuzzConstants.Robot.WIDTH_IN / 2.0;
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
        assertEquals(72.0 - 49.46 / 2.0, frame.minX, 1e-9);
        assertEquals(72.0 + 49.46 / 2.0, frame.maxX, 1e-9);
        assertEquals(72.0 - 38.95 / 2.0, frame.minY, 1e-9);
        assertEquals(72.0 + 38.95 / 2.0, frame.maxY, 1e-9);
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
        // field report 2.1: the plan view of the real frame measured x 47.0 to 97.0 and y 51.9 to 91.8 (rail thickness and
        // figure error): up to 0.63 in beyond the nominal footprint. The field itself is good to about +/- 1 in.
        double[] measured = {47.0, 97.0, 51.9, 91.8};
        double fieldTolerance = 1.0;
        assertTrue(keepOut.minX <= measured[0] - fieldTolerance);
        assertTrue(keepOut.maxX >= measured[1] + fieldTolerance);
        assertTrue(keepOut.minY <= measured[2] - fieldTolerance);
        assertTrue(keepOut.maxY >= measured[3] + fieldTolerance);
        double worstOverhang = Math.max(Math.max(frame.minX - measured[0], measured[1] - frame.maxX),
                Math.max(frame.minY - measured[2], measured[3] - frame.maxY));
        assertEquals("the audited worst overhang of the measured plan", 0.63, worstOverhang, 0.01);
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
        double mid = 72.0;
        // heading 0 faces +x, 90 faces +y: a square robot is 8.9 in from its center to every side whatever it faces
        Object[][] approaches = {
                // name, x, y, how far to move toward the frame (dx, dy)
                {"left edge", keepOut.minX - HALF, mid, 1.0, 0.0},
                {"right edge", keepOut.maxX + HALF, mid, -1.0, 0.0},
                {"audience edge", mid, keepOut.minY - HALF, 0.0, 1.0},
                {"rear edge", mid, keepOut.maxY + HALF, 0.0, -1.0},
        };
        for (Object[] a : approaches) {
            double x = (Double) a[1];
            double y = (Double) a[2];
            double dx = (Double) a[3];
            double dy = (Double) a[4];
            for (double heading : new double[] {0, 90, 180, -90}) {
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
        double leftLimit = keepOut.minX - HALF;     // 35.37
        assertEquals(0, hiveErrors(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(leftLimit, 90, -90)));
        assertEquals(1, hiveErrors(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(leftLimit + 0.1, 90, -90)));
        // audience edge (blue: the rear edge): drive along the front of the frame toward it from the audience side
        double audienceLimit = keepOut.minY - HALF;     // 40.63
        assertEquals(0, hiveErrors(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(58, audienceLimit, -90)));
        assertEquals(1, hiveErrors(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(58, audienceLimit + 0.1, -90)));
    }

    // ---- 3. every corner -----------------------------------------------------------------------------------------

    @Test
    public void eachCornerOfTheKeepOutStopsTheRobot() {
        double[][] corners = {
                {keepOut.minX, keepOut.minY, -1, -1}, {keepOut.maxX, keepOut.minY, 1, -1},
                {keepOut.maxX, keepOut.maxY, 1, 1}, {keepOut.minX, keepOut.maxY, -1, 1}};
        for (double[] c : corners) {
            // an axis-aligned robot whose own corner just reaches the keep-out corner from outside, diagonally
            double x = c[0] + c[2] * HALF;
            double y = c[1] + c[3] * HALF;
            Pose2 touching = Pose2.ofDegrees(x, y, 0);
            assertFalse("corner touching", touchesOrOverlaps(touching, keepOut));
            assertEquals("corner to corner", 0.0, Footprint.of(touching).signedGap(keepOut), 1e-9);
            assertTrue("0.05 in further in", touchesOrOverlaps(Pose2.ofDegrees(x - 0.05 * c[2], y - 0.05 * c[3], 0), keepOut));
            // pulled back along the diagonal the gap is the corner-to-corner distance, not a smaller edge distance
            double back = 2.0;
            assertEquals(back * Math.sqrt(2.0), Footprint.of(Pose2.ofDegrees(x + back * c[2], y + back * c[3], 0)).signedGap(keepOut), 1e-9);
            // a robot turned 45 degrees (a diamond) meets the corner with the middle of a flat side instead, 8.9 in from its center
            double diagX = c[0] + c[2] * HALF / Math.sqrt(2.0);
            double diagY = c[1] + c[3] * HALF / Math.sqrt(2.0);
            Pose2 diamond = Pose2.ofDegrees(diagX, diagY, 45.0);
            assertEquals("diamond side touching the corner", 0.0, Footprint.of(diamond).signedGap(keepOut), 1e-9);
            assertFalse(touchesOrOverlaps(diamond, keepOut));
            assertTrue("0.05 in further in", touchesOrOverlaps(
                    Pose2.ofDegrees(diagX - 0.05 * c[2], diagY - 0.05 * c[3], 45.0), keepOut));
        }
    }

    @Test
    public void theCornersAreCaughtByTheValidatorToo() {
        double lx = keepOut.minX - HALF;
        double nearY = keepOut.minY - HALF;            // audience-side corner of the left edge
        assertEquals("corner to corner is touching", 0, hiveErrors(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(lx, nearY, -90)));
        assertEquals("0.15 in into the corner", 1, hiveErrors(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(lx + 0.15, nearY + 0.15, -90)));
        double farY = keepOut.maxY + HALF;             // rear-side corner of the left edge
        assertEquals("corner to corner is touching", 0, hiveErrors(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(lx, farY, -90)));
        assertEquals("0.15 in into the corner", 1, hiveErrors(AutoRoute.builder("x").start(35, 9.2, -90).driveConstant(lx + 0.15, farY - 0.15, -90)));
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
        // x = 59.25 is the red HIVE axis: between the legs and under the crossbar a 12.13 in robot might physically fit;
        // the validator still calls a leg through it an ERROR, and so a leg through the gap between the two HIVEs (x = 72)
        for (double x : new double[] {59.25, 62.0, 66.0}) {
            List<Issue> l = RouteValidator.check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(x, 100, -90).build());
            assertTrue("x = " + x + ": " + l, count(l, Rule.HIVE_KEEPOUT) >= 1);
        }
    }

    @Test
    public void routesMayNeitherEndNorStartInsideTheKeepOutRegionOfTheFrame() {
        List<Issue> l = RouteValidator.check(AutoRoute.builder("x").start(58, 9.2, -90).driveConstant(60, 70, 0).build());
        assertTrue(l.toString(), count(l, Rule.END_POSE) == 1);
    }

    // ---- 5. what is deliberately NOT in the keep-out -------------------------------------------------------------

    @Test
    public void aDownCellHangsOverTheFrameButOnlyHighAboveTheRobot() {
        double lowestCellPointIn = 30.6;           // field report 2.1: the rim of a DOWN cell, the lowest point of any cell
        assertTrue("the tallest part of the robot is far below the lowest cell",
                BioBuzzConstants.Robot.HEIGHT_IN + 10.0 < lowestCellPointIn);
        assertTrue("and so is the camera", BioBuzzConstants.Vision.CAMERA_HEIGHT_IN + 10.0 < lowestCellPointIn);
        // the red rear DOWN cell reaches y 96.9, past the frame (91.47) and past the keep-out (94.47): the robot may stand
        // under that overhang, which is why it is not solid
        Rect down = red.cellPlan(Alliance.RED, Side.REAR, false);
        assertEquals(96.9, down.maxY, 1e-9);
        assertTrue(down.maxY > keepOut.maxY);
        assertEquals("the overhang past the frame is the 5.4 in of the audit", 5.43, down.maxY - frame.maxY, 0.01);
        // the UP cell (lowest point about 46.6 in) also pokes out of the frame by a fraction of an inch: not solid either
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
