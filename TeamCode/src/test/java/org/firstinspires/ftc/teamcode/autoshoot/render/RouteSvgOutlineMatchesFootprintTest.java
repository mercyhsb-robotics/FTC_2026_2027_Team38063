package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;

import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;

/**
 * The robot outline in a route diagram is the SAME rectangle the validator checks: every corner of the drawn outline and of
 * the drawn body, the two drawn wheels, the green front edge, the orange back edge and the little center-of-rotation dot sit
 * where {@link Footprint} puts them for that pose. Why a test: the robot is not centered on its pose (its center of rotation is
 * 9.77 in from the front edge and 8.03 in from the back edge), and a picture that drew a centered rectangle would show a
 * robot that fits where the real one does not.
 *
 * <p>How it works: a one-step route whose start is the pose under test is rendered (the start footprint is always drawn), the
 * SVG is read back with a DOM parser, and the corner pixels are compared with {@code Footprint} corners converted to pixels.
 * Twelve headings (every 30 degrees) at three places are tried, for RED and for BLUE.
 */
public class RouteSvgOutlineMatchesFootprintTest {
    private static final double S = BioBuzzConstants.Auto.RENDER_FIELD_SCALE_PX_PER_IN;
    private static final double FIELD_PX = BioBuzzConstants.Field.SIZE_IN * S;
    private static final double X0 = (BioBuzzConstants.Auto.RENDER_CANVAS_PX - FIELD_PX) / 2.0;
    private static final double Y_BOTTOM = BioBuzzConstants.Auto.RENDER_FIELD_TOP_PX + FIELD_PX;
    private static final double PIXEL = 0.06;

    private static double px(double fieldX) {
        return X0 + fieldX * S;
    }

    private static double py(double fieldY) {
        return Y_BOTTOM - fieldY * S;
    }

    private static Document parse(String svg) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        return f.newDocumentBuilder().parse(new ByteArrayInputStream(svg.getBytes(Charset.forName("UTF-8"))));
    }

    private static List<Element> all(Node root, String tag) {
        List<Element> out = new ArrayList<Element>();
        NodeList nl = root instanceof Document ? ((Document) root).getElementsByTagName(tag) : ((Element) root).getElementsByTagName(tag);
        for (int i = 0; i < nl.getLength(); i++) {
            out.add((Element) nl.item(i));
        }
        return out;
    }

    private static Element withClass(Node root, String tag, String cls) {
        for (Element e : all(root, tag)) {
            for (String c : e.getAttribute("class").split("\\s+")) {
                if (c.equals(cls)) {
                    return e;
                }
            }
        }
        throw new AssertionError("no <" + tag + " class=" + cls + ">");
    }

    private static Element byId(Document d, String id) {
        for (Element e : all(d, "*")) {
            if (id.equals(e.getAttribute("id"))) {
                return e;
            }
        }
        throw new AssertionError("no element with id " + id);
    }

    private static double num(Element e, String attr) {
        return Double.parseDouble(e.getAttribute(attr));
    }

    /** The corners of a drawn polygon, in the order the renderer wrote them (front-left first, counter-clockwise on the field). */
    private static double[][] corners(Element polygon) {
        String[] parts = polygon.getAttribute("points").trim().split("\\s+");
        double[][] out = new double[parts.length][2];
        for (int i = 0; i < parts.length; i++) {
            String[] xy = parts[i].split(",");
            out[i][0] = Double.parseDouble(xy[0]);
            out[i][1] = Double.parseDouble(xy[1]);
        }
        return out;
    }

    private static void assertCornersMatch(String what, Footprint expected, Element drawn) {
        double[][] c = corners(drawn);
        assertEquals(what, 4, c.length);
        for (int i = 0; i < 4; i++) {
            Point2 p = expected.corner(i);
            assertEquals(what + " corner " + i + " x", px(p.x), c[i][0], PIXEL);
            assertEquals(what + " corner " + i + " y", py(p.y), c[i][1], PIXEL);
        }
    }

    /**
     * Situation: the start footprint of a route that starts at (30, 40), (60, 15) and (20, 100) turned to each of 12 headings, RED,
     *     and the half-turn images for BLUE.
     * Expected: the drawn outline, body, both wheel discs, the front edge, the back edge and the center-of-rotation dot are at the
     *     places {@code Footprint} gives for that pose; the dot is at the pose and is NOT the middle of the outline (it is 0.87 in
     *     off it along the robot's length).
     * Why: the picture and the validator must agree about where the robot is.
     */
    @Test
    public void theDrawnOutlineBodyWheelsAndCenterDotAreExactlyTheFootprint() throws Exception {
        double[][] places = {{30, 40}, {60, 15}, {20, 100}};
        int checked = 0;
        for (double[] place : places) {
            for (int deg = -180; deg < 180; deg += 30) {
                AutoRoute red = AutoRoute.builder("p").startTile("B1").start(place[0], place[1], deg).waitSeconds(1).build();
                for (Alliance alliance : Alliance.values()) {
                    Pose2 pose = alliance == Alliance.RED ? red.startPose() : Alliance.BLUE.fromRed(red.startPose(), 144.0);
                    Document d = parse(RouteSvgRenderer.render(red, alliance));
                    Element start = byId(d, "start-footprint");
                    String where = alliance + " " + pose;
                    assertCornersMatch(where + " outline", Footprint.of(pose), withClass(start, "polygon", "outline"));
                    assertCornersMatch(where + " body", Footprint.chassis(pose), withClass(start, "polygon", "chassis"));
                    for (IntakeWheel wheel : IntakeWheel.values()) {
                        Element disc = null;
                        for (Element c : all(start, "circle")) {
                            if (wheel.name().equals(c.getAttribute("data-wheel"))) {
                                disc = c;
                            }
                        }
                        Point2 want = Footprint.wheelCenter(pose, wheel);
                        assertTrue(where + " wheel " + wheel, disc != null);
                        assertEquals(where + " wheel x", px(want.x), num(disc, "cx"), PIXEL);
                        assertEquals(where + " wheel y", py(want.y), num(disc, "cy"), PIXEL);
                    }
                    // the green front edge joins corners 0 and 3 (the front of the perimeter); the orange back edge corners 1 and 2
                    Footprint fp = Footprint.of(pose);
                    Element front = withClass(start, "line", "front-edge");
                    Element back = withClass(start, "line", "back-edge");
                    assertEquals(where + " front x1", px(fp.corner(0).x), num(front, "x1"), PIXEL);
                    assertEquals(where + " front y2", py(fp.corner(3).y), num(front, "y2"), PIXEL);
                    assertEquals(where + " back x1", px(fp.corner(1).x), num(back, "x1"), PIXEL);
                    assertEquals(where + " back y2", py(fp.corner(2).y), num(back, "y2"), PIXEL);
                    // the center of rotation: at the pose, and not the middle of the outline
                    Element dot = all(withClass(start, "g", "pose-point"), "circle").get(0);
                    assertEquals(where + " dot x", px(pose.x), num(dot, "cx"), PIXEL);
                    assertEquals(where + " dot y", py(pose.y), num(dot, "cy"), PIXEL);
                    double[][] c = corners(withClass(start, "polygon", "outline"));
                    double midX = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0;
                    double midY = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0;
                    double offIn = Math.hypot(midX - num(dot, "cx"), midY - num(dot, "cy")) / S;
                    assertEquals(where + ": the dot is half the front-rear difference off the middle",
                            (BioBuzzConstants.Robot.FRONT_EXTENT_IN - BioBuzzConstants.Robot.REAR_EXTENT_IN) / 2.0, offIn, 0.02);
                    checked++;
                }
            }
        }
        assertEquals(3 * 12 * 2, checked);
    }

    /**
     * Situation: the same start footprint, but the real robot's extents are compared with what a CENTERED 17.8 in square would give.
     * Expected: facing -90 (front toward the bottom of the picture) the drawn front edge is 9.77 in below the dot and the back edge
     *     8.03 in above it, and each side edge 8.90 in left and right of it.
     * Why: a quick sanity check in plain numbers, so a mistake in the corner order of the test above cannot hide a centered drawing.
     */
    @Test
    public void facingTheAudienceWallTheFrontEdgeIsNineAndAHalfInchesBelowTheDot() throws Exception {
        AutoRoute r = AutoRoute.builder("p").startTile("C1").start(58, 20, -90).waitSeconds(1).build();
        Element start = byId(parse(RouteSvgRenderer.render(r, Alliance.RED)), "start-footprint");
        double[][] c = corners(withClass(start, "polygon", "outline"));
        double minY = Math.min(Math.min(c[0][1], c[1][1]), Math.min(c[2][1], c[3][1]));
        double maxY = Math.max(Math.max(c[0][1], c[1][1]), Math.max(c[2][1], c[3][1]));
        double minX = Math.min(Math.min(c[0][0], c[1][0]), Math.min(c[2][0], c[3][0]));
        double maxX = Math.max(Math.max(c[0][0], c[1][0]), Math.max(c[2][0], c[3][0]));
        Element dot = all(withClass(start, "g", "pose-point"), "circle").get(0);
        assertEquals("front edge below the dot", 9.77, (maxY - num(dot, "cy")) / S, 0.01);
        assertEquals("back edge above the dot", 8.03, (num(dot, "cy") - minY) / S, 0.01);
        assertEquals("left side edge", 8.90, (num(dot, "cx") - minX) / S, 0.01);
        assertEquals("right side edge", 8.90, (maxX - num(dot, "cx")) / S, 0.01);
    }
}
