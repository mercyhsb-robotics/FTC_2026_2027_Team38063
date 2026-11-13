package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteValidator;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StartPoseSearch;
import org.firstinspires.ftc.teamcode.autoshoot.routes.TimingModel;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Wall;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

/**
 * The legal start stretches of rule G304 in the diagram: tinted strips along the own walls (where a robot facing the start's heading may be set up), under the route, so
 * the start footprint of the route lies on top. They are drawn only when asked for, so the committed gallery does not change until it asks.
 */
public class RouteSvgStartBandsTest {
    private static final double S = BioBuzzConstants.Auto.RENDER_FIELD_SCALE_PX_PER_IN;
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final double FIELD_PX = SIZE * S;
    private static final double X0 = (BioBuzzConstants.Auto.RENDER_CANVAS_PX - FIELD_PX) / 2.0;
    private static final double Y_BOTTOM = BioBuzzConstants.Auto.RENDER_FIELD_TOP_PX + FIELD_PX;
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;

    private static Document parse(String svg) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        return f.newDocumentBuilder().parse(new ByteArrayInputStream(svg.getBytes(Charset.forName("UTF-8"))));
    }

    private static List<Element> bands(Document d) {
        List<Element> out = new ArrayList<Element>();
        NodeList nl = d.getElementsByTagName("rect");
        for (int i = 0; i < nl.getLength(); i++) {
            Element e = (Element) nl.item(i);
            if ("start-band".equals(e.getAttribute("class"))) {
                out.add(e);
            }
        }
        return out;
    }

    private static Element byId(Document d, String id) {
        NodeList nl = d.getElementsByTagName("*");
        for (int i = 0; i < nl.getLength(); i++) {
            Element e = (Element) nl.item(i);
            if (id.equals(e.getAttribute("id"))) {
                return e;
            }
        }
        return null;
    }

    private static String render(AutoRoute route, Alliance alliance, boolean bands) {
        AutoRoute own = route.forAlliance(alliance);
        List<Issue> issues = new RouteValidator().validate(own);
        return RouteSvgRenderer.render(route, alliance, issues, new TimingModel(), bands);
    }

    private static AutoRoute onTheAudienceWall() {
        return RouteCatalog.byId(RouteCatalog.WALL_SHOOT_PARK);
    }

    /** A route that starts on the red wall facing it (heading 180), so the red wall has three stretches: the FLOWER and the LOADING ZONE are the gaps. */
    private static AutoRoute onTheRedWall() {
        return AutoRoute.builder("red_wall").start(FRONT, 70.27, 180).waitSeconds(1).build();
    }

    // ---- off by default, on when asked ------------------------------------------------------------------------------------

    /**
     * Situation: a route drawn without the stretches (the default and an explicit false), and the same route with them.
     * Expected: the default drawings are exactly the old ones (so docs/autos stays the committed copy), they hold no {@code start-bands}; the drawing with the stretches
     *     is well-formed XML and holds them.
     */
    @Test
    public void theStretchesAreDrawnOnlyWhenAskedFor() throws Exception {
        AutoRoute route = onTheAudienceWall();
        String plain = RouteSvgRenderer.render(route, Alliance.RED);
        assertFalse(plain.contains("start-band"));
        assertEquals("false is the default drawing", plain, RouteSvgRenderer.render(route, Alliance.RED, new RouteValidator().validate(route), new TimingModel(), false));
        String with = render(route, Alliance.RED, true);
        assertTrue(with.contains("id=\"start-bands\""));
        Document d = parse(with);
        assertNotNull(byId(d, "start-bands"));
        assertTrue(bands(d).size() >= 3);
        assertFalse("the gallery does not draw them yet (see RENDER_GALLERY_SHOWS_START_BANDS)", BioBuzzConstants.Auto.RENDER_GALLERY_SHOWS_START_BANDS);
    }

    // ---- where the strips are ---------------------------------------------------------------------------------------------

    private static double d(Element e, String attr) {
        return Double.parseDouble(e.getAttribute(attr));
    }

    /**
     * Situation: the audience-wall route for RED.
     * Expected: strips on the red wall, the audience wall and the rear wall and nowhere else (never on the blue wall); each strip is where the search says (the data
     *     attributes are the stretch in inches along the wall) and is drawn there: along the wall it spans those inches at the diagram's scale, and it is as thick as
     *     the constant says; the audience wall strip is at the bottom of the picture.
     */
    @Test
    public void theStripsAreOnTheOwnWallsWhereTheSearchSaysAndOnlyThere() throws Exception {
        AutoRoute route = onTheAudienceWall();
        Document doc = parse(render(route, Alliance.RED, true));
        List<StartPoseSearch.Interval> expected = StartPoseSearch.legalIntervals(Alliance.RED, route.startPose().headingDegrees());
        List<Element> got = bands(doc);
        assertEquals(expected.size(), got.size());
        double thick = BioBuzzConstants.Auto.RENDER_START_BAND_THICKNESS_IN * S;
        for (int i = 0; i < got.size(); i++) {
            Element e = got.get(i);
            StartPoseSearch.Interval want = expected.get(i);
            assertEquals(want.wall.name(), e.getAttribute("data-wall"));
            assertEquals(want.fromIn, d(e, "data-from"), 0.01);
            assertEquals(want.toIn, d(e, "data-to"), 0.01);
            assertFalse("never on the blue wall", e.getAttribute("data-wall").equals("BLUE"));
            switch (want.wall) {
                case RED:
                    assertEquals(X0, d(e, "x"), 0.05);                                         // against the red wall, the left edge of the field
                    assertEquals(thick, d(e, "width"), 0.05);
                    assertEquals((want.toIn - want.fromIn) * S, d(e, "height"), 0.1);
                    assertEquals(Y_BOTTOM - want.toIn * S, d(e, "y"), 0.1);
                    break;
                case AUDIENCE:
                    assertEquals(Y_BOTTOM - thick, d(e, "y"), 0.05);                           // the bottom of the picture
                    assertEquals(thick, d(e, "height"), 0.05);
                    assertEquals((want.toIn - want.fromIn) * S, d(e, "width"), 0.1);
                    assertEquals(X0 + want.fromIn * S, d(e, "x"), 0.1);
                    break;
                default:                                                                       // the rear wall: the top of the field
                    assertEquals(BioBuzzConstants.Auto.RENDER_FIELD_TOP_PX, d(e, "y"), 0.05);
                    assertEquals(X0 + want.fromIn * S, d(e, "x"), 0.1);
                    assertEquals((want.toIn - want.fromIn) * S, d(e, "width"), 0.1);
                    break;
            }
            assertEquals(RouteSvgRenderer.inkFor(Alliance.RED), e.getAttribute("fill"));
        }
        boolean red = false;
        boolean audience = false;
        boolean rear = false;
        for (Element e : got) {
            red |= e.getAttribute("data-wall").equals("RED");
            audience |= e.getAttribute("data-wall").equals("AUDIENCE");
            rear |= e.getAttribute("data-wall").equals("REAR");
        }
        assertTrue(red && audience && rear);
    }

    /**
     * Situation: a start on the red wall facing it, the red wall's three stretches.
     * Expected: three strips on the red wall, and the gaps between them are the FLOWER (with its bracket and the robot's width) and the LOADING ZONE (with the robot's
     *     width): no strip overlaps the LOADING ZONE rows (y 94.58 to 117.27) widened by the robot's 8.9 in either way, none the FLOWER's.
     */
    @Test
    public void theGapsAreTheFlowerAndTheLoadingZone() throws Exception {
        Document doc = parse(render(onTheRedWall(), Alliance.RED, true));
        List<Element> onRed = new ArrayList<Element>();
        for (Element e : bands(doc)) {
            if (e.getAttribute("data-wall").equals("RED")) {
                onRed.add(e);
            }
        }
        assertEquals(3, onRed.size());
        double half = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
        for (Element e : onRed) {
            double from = d(e, "data-from");
            double to = d(e, "data-to");
            // the robot, 2 x 8.9 in wide along the wall, centered on the stretch's ends, does not reach into the zone or onto the FLOWER bracket
            // (the numbers in the drawing have two decimals, hence the 0.01 in)
            assertTrue(from + "-" + to, to + half <= BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN + 0.01 || from - half >= BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MAX_IN - 0.01);
            double flowerY = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1];
            double bracket = BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN / 2.0;
            assertTrue(from + "-" + to, to + half <= flowerY - bracket + 0.01 || from - half >= flowerY + bracket - 0.01);
        }
    }

    /**
     * Situation: the BLUE alliance's drawing of the same route.
     * Expected: the strips are on the blue wall, the audience wall and the rear wall (never the red wall), and each is the half turn of a red one.
     */
    @Test
    public void theBlueDrawingHasTheHalfTurnedStripsOnTheBlueWalls() throws Exception {
        AutoRoute route = onTheAudienceWall();
        Document blue = parse(render(route, Alliance.BLUE, true));
        for (Element e : bands(blue)) {
            assertFalse("never on the red wall", e.getAttribute("data-wall").equals("RED"));
        }
        Document red = parse(render(route, Alliance.RED, true));
        List<Element> redBands = bands(red);
        List<Element> blueBands = bands(blue);
        assertEquals(redBands.size(), blueBands.size());
        for (Element r : redBands) {
            Wall w = Wall.valueOf(r.getAttribute("data-wall"));
            String turned = w == Wall.RED ? "BLUE" : w == Wall.AUDIENCE ? "REAR" : "AUDIENCE";
            boolean found = false;
            for (Element b : blueBands) {
                found |= b.getAttribute("data-wall").equals(turned) && Math.abs(d(b, "data-from") - (SIZE - d(r, "data-to"))) < 0.02
                        && Math.abs(d(b, "data-to") - (SIZE - d(r, "data-from"))) < 0.02;
            }
            assertTrue("no blue strip is the half turn of " + w + " " + r.getAttribute("data-from") + " to " + r.getAttribute("data-to"), found);
        }
    }

    // ---- the footprint is on top ------------------------------------------------------------------------------------------------

    /**
     * Situation: the drawing with the strips.
     * Expected: the strips come before the route in the document, so the route's start footprint (and every other footprint) is drawn on top of them; and the start
     *     footprint of a legal start stands on a strip: its pose point is inside the stretch of the wall it touches.
     */
    @Test
    public void theStartFootprintIsDrawnOnTopOfTheStrips() throws Exception {
        AutoRoute route = onTheAudienceWall();
        String svg = render(route, Alliance.RED, true);
        assertTrue(svg.indexOf("id=\"start-bands\"") < svg.indexOf("id=\"route\""));
        assertTrue(svg.indexOf("id=\"start-bands\"") < svg.indexOf("id=\"start-footprint\""));
        Pose2 start = route.startPose();
        boolean standsOnAStrip = false;
        for (Element e : bands(parse(svg))) {
            standsOnAStrip |= e.getAttribute("data-wall").equals("AUDIENCE") && d(e, "data-from") <= start.x && start.x <= d(e, "data-to");
        }
        assertTrue("the start (" + start + ") is inside a legal stretch of the audience wall", standsOnAStrip);
    }

    /**
     * Situation: the old OpMode start 0.57 in through the wall (red_b1, y = 9.2), and a start on the HIVE side of a FLOWER.
     * Expected: the stretches do not change with the route's quality: they come from the start's heading alone, so a route with an illegal start is drawn with the
     *     stretches around it (its start is outside them or on the edge), and the picture still renders.
     */
    @Test
    public void everyCatalogRouteRendersWithTheStripsForBothAlliances() throws Exception {
        for (AutoRoute route : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                Document d = parse(render(route, a, true));
                assertTrue(route.id() + " " + a, bands(d).size() >= 2);
                assertNotNull(route.id() + " " + a, byId(d, "start-footprint"));
            }
        }
    }
}
