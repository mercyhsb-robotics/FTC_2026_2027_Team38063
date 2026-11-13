package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StepKind;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/** The SVG renderer: well-formed XML, the right elements in the right places, the right ink. */
public class RouteSvgRendererTest {
    private static final double S = BioBuzzConstants.Auto.RENDER_FIELD_SCALE_PX_PER_IN;
    private static final double CANVAS = BioBuzzConstants.Auto.RENDER_CANVAS_PX;
    private static final double FIELD_PX = BioBuzzConstants.Field.SIZE_IN * S;
    private static final double X0 = (CANVAS - FIELD_PX) / 2.0;
    private static final double Y_BOTTOM = BioBuzzConstants.Auto.RENDER_FIELD_TOP_PX + FIELD_PX;

    // ---- DOM helpers -------------------------------------------------------------------------------

    private static Document parse(String svg) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        DocumentBuilder b = f.newDocumentBuilder();
        return b.parse(new ByteArrayInputStream(svg.getBytes(Charset.forName("UTF-8"))));
    }

    private static List<Element> all(Node root, String tag) {
        List<Element> out = new ArrayList<Element>();
        NodeList nl = root instanceof Document ? ((Document) root).getElementsByTagName(tag)
                : ((Element) root).getElementsByTagName(tag);
        for (int i = 0; i < nl.getLength(); i++) {
            out.add((Element) nl.item(i));
        }
        return out;
    }

    private static boolean hasClass(Element e, String cls) {
        for (String c : e.getAttribute("class").split("\\s+")) {
            if (c.equals(cls)) {
                return true;
            }
        }
        return false;
    }

    private static List<Element> withClass(Node root, String tag, String cls) {
        List<Element> out = new ArrayList<Element>();
        for (Element e : all(root, tag)) {
            if (hasClass(e, cls)) {
                out.add(e);
            }
        }
        return out;
    }

    private static Element byId(Document d, String id) {
        for (Element e : all(d, "*")) {
            if (id.equals(e.getAttribute("id"))) {
                return e;
            }
        }
        return null;
    }

    private static int countSteps(AutoRoute r, StepKind kind) {
        int n = 0;
        for (int i = 0; i < r.size(); i++) {
            if (r.step(i).kind() == kind) {
                n++;
            }
        }
        return n;
    }

    private static double[] polygonBounds(Element polygon) {
        String[] pts = polygon.getAttribute("points").trim().split("\\s+");
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (String p : pts) {
            String[] xy = p.split(",");
            double x = Double.parseDouble(xy[0]);
            double y = Double.parseDouble(xy[1]);
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
        }
        return new double[] {minX, maxX, minY, maxY};
    }

    // ---- well-formedness ---------------------------------------------------------------------------

    @Test
    public void everyCatalogRouteRendersToWellFormedXmlForBothAlliances() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                String svg = RouteSvgRenderer.render(r, a);
                Document d = parse(svg);
                Element root = d.getDocumentElement();
                assertEquals("svg", root.getLocalName());
                assertEquals("http://www.w3.org/2000/svg", root.getNamespaceURI());
                assertEquals("0 0 900 900", root.getAttribute("viewBox"));
                assertEquals("900", root.getAttribute("width"));
                assertEquals("900", root.getAttribute("height"));
                assertFalse(r.id() + " " + a, svg.contains("NaN") || svg.contains("Infinity"));
                for (int i = 0; i < svg.length(); i++) {
                    assertTrue("plain ASCII output, got char " + (int) svg.charAt(i), svg.charAt(i) < 128);
                }
            }
        }
    }

    @Test
    public void theSvgHasAccessibleTitleAndDescription() throws Exception {
        AutoRoute r = RouteCatalog.byId("garden_cycle");
        Document d = parse(RouteSvgRenderer.render(r, Alliance.BLUE));
        Element title = byId(d, "svg-title");
        Element desc = byId(d, "svg-desc");
        assertNotNull(title);
        assertNotNull(desc);
        assertTrue(title.getTextContent().contains(r.title()));
        assertTrue(title.getTextContent().contains("BLUE"));
        assertTrue(desc.getTextContent().contains("audience at the bottom"));
        assertTrue(desc.getTextContent().contains("SHOOT 4"));
        assertEquals("svg-title svg-desc", d.getDocumentElement().getAttribute("aria-labelledby"));
        assertEquals("img", d.getDocumentElement().getAttribute("role"));
    }

    @Test
    public void textWithMarkupCharactersCannotBreakTheXml() throws Exception {
        AutoRoute r = AutoRoute.builder("odd").title("A & B <fast> \"auto\" it's").startTile("B1")
                .start(35, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).waitSeconds(1).note("x < y & z").build();
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        assertTrue(byId(d, "svg-title").getTextContent().contains("A & B <fast> \"auto\" it's"));
    }

    @Test
    public void renderingIsDeterministic() {
        AutoRoute r = RouteCatalog.byId("wall_shoot_park");
        assertEquals(RouteSvgRenderer.render(r, Alliance.RED), RouteSvgRenderer.render(r, Alliance.RED));
    }

    // ---- the field ---------------------------------------------------------------------------------

    /**
     * Situation: any route diagram, and a route with a leg that may go under the HIVE.
     * Expected: the picture draws the exact HIVE solid (the two foot bars and the four leg stubs, 6 shapes, the bars where the CAD puts them) next
     *     to the footprint box, and the leg that is allowed under the HIVE says so in its label.
     */
    @Test
    public void theExactHiveSolidIsDrawnAndAnUnderHiveLegSaysSo() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_b1"), Alliance.RED));
        List<Element> feet = withClass(byId(d, "hive-solid"), "polygon", "hive-foot");
        assertEquals(6, feet.size());
        double[] left = polygonBounds(feet.get(0));
        // the left foot bar: x 45.941 to 47.924 in the wall-corner frame, y 51.202 to 90.146
        assertEquals(X0 + 45.941 * S, left[0], 0.02);
        assertEquals(X0 + 47.924 * S, left[1], 0.02);
        assertEquals(Y_BOTTOM - 90.146 * S, left[2], 0.02);
        assertEquals(Y_BOTTOM - 51.202 * S, left[3], 0.02);
        double bar = left[1] - left[0];
        for (int i = 2; i < feet.size(); i++) {
            double[] stub = polygonBounds(feet.get(i));
            assertTrue("a stub is inside the frame footprint", stub[0] >= left[0] - 0.02 && stub[3] <= left[3] + 0.5 * S);
            assertTrue("and is smaller than the frame", stub[1] - stub[0] < 50.0 * S);
        }
        assertTrue(bar > 0.0);
        AutoRoute lane = AutoRoute.builder("lane").title("Lane").startTile("C1").start(58.0, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .driveConstant(64.0, 38.0, -90).driveConstant(64.0, 105.0, -90).allowUnderHive().label("the lane").build();
        Document under = parse(RouteSvgRenderer.render(lane, Alliance.RED));
        boolean labelled = false;
        for (Element t : withClass(byId(under, "route"), "text", "step-label")) {
            labelled |= t.getTextContent().equals("the lane (under HIVE)");
        }
        assertTrue(labelled);
    }

    @Test
    public void theFieldHas36TilesFourFlowersTwoGardensTwoLoadingZonesAndTwoHives() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_b1"), Alliance.RED));
        assertEquals(36, withClass(d, "rect", "tile").size());
        // tiles alternate light/dark like a checkerboard
        String lightA1 = null;
        String darkA2 = null;
        for (Element t : withClass(d, "rect", "tile")) {
            if ("A1".equals(t.getAttribute("data-tile"))) {
                lightA1 = t.getAttribute("fill");
            }
            if ("A2".equals(t.getAttribute("data-tile"))) {
                darkA2 = t.getAttribute("fill");
            }
        }
        assertEquals(BioBuzzConstants.Auto.RENDER_TILE_LIGHT, lightA1);
        assertEquals(BioBuzzConstants.Auto.RENDER_TILE_DARK, darkA2);
        assertEquals(4, withClass(d, "g", "flower").size());
        assertEquals(2, withClass(d, "g", "garden").size());
        assertEquals(8, withClass(d, "circle", "pollen").size());
        assertEquals(2, withClass(d, "g", "loading-zone").size());
        assertEquals(2, withClass(d, "g", "hive").size());
        assertEquals(2, withClass(d, "g", "cell-up").size());
        assertEquals(2, withClass(d, "g", "cell-down").size());
        assertEquals(2, withClass(d, "rect", "alliance-area").size());
        assertNotNull(byId(d, "hive-frame"));
        assertNotNull(byId(d, "field-border"));
        // column letters and row numbers
        List<String> labels = new ArrayList<String>();
        for (Element t : withClass(d, "text", "tile-label")) {
            labels.add(t.getTextContent());
        }
        assertEquals(12, labels.size());
        for (String s : new String[] {"A", "B", "C", "D", "E", "F", "1", "2", "3", "4", "5", "6"}) {
            assertTrue(s, labels.contains(s));
        }
    }

    @Test
    public void theUpCellAtMatchStartIsFilledAndLabelledWithItsTagIds() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_b1"), Alliance.RED));
        for (Element cell : withClass(d, "g", "cell")) {
            boolean up = hasClass(cell, "cell-up");
            String alliance = cell.getAttribute("data-alliance");
            String side = cell.getAttribute("data-side");
            String text = all(cell, "text").get(0).getTextContent();
            if (alliance.equals("RED")) {
                assertEquals(up ? "AUDIENCE" : "REAR", side);
                assertEquals(up ? "UP 34-37" : "DOWN 30-33", text);
            } else {
                assertEquals(up ? "REAR" : "AUDIENCE", side);
                assertEquals(up ? "UP 42-45" : "DOWN 38-41", text);
            }
            Element rect = all(cell, "rect").get(0);
            assertEquals("the UP cell is filled and solid, the DOWN cell dashed", up, rect.getAttribute("stroke-dasharray").isEmpty());
        }
    }

    @Test
    public void audienceIsAtTheBottomAndRedIsOnTheLeft() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_c1"), Alliance.RED));
        Element audience = byId(d, "audience-label");
        assertEquals("AUDIENCE", audience.getTextContent());
        double y = Double.parseDouble(audience.getAttribute("y"));
        assertTrue("near the bottom of the image: " + y, y > 0.75 * CANVAS);
        assertTrue("below the field border", y > Y_BOTTOM);
        assertEquals(CANVAS / 2.0, Double.parseDouble(audience.getAttribute("x")), 1e-6);

        // red LOADING ZONE is left of the blue one, red GARDEN POLLEN left of blue; red garden is at the bottom, blue at the top
        double redLz = Double.NaN;
        double blueLz = Double.NaN;
        for (Element lz : withClass(d, "g", "loading-zone")) {
            double x = Double.parseDouble(all(lz, "rect").get(0).getAttribute("x"));
            if (lz.getAttribute("data-alliance").equals("RED")) {
                redLz = x;
            } else {
                blueLz = x;
            }
        }
        assertTrue(redLz < blueLz);
        assertEquals("red LOADING ZONE starts at the red wall", X0, redLz, 0.006);
        double redGardenY = Double.NaN;
        double blueGardenY = Double.NaN;
        for (Element g : withClass(d, "g", "garden")) {
            double cy = Double.parseDouble(all(g, "circle").get(0).getAttribute("cy"));
            if (g.getAttribute("data-alliance").equals("RED")) {
                redGardenY = cy;
            } else {
                blueGardenY = cy;
            }
        }
        assertTrue("red GARDEN is on the audience wall (larger pixel y)", redGardenY > blueGardenY);
        assertEquals("red POLLEN row is one radius off the audience wall", Y_BOTTOM - 1.4 * S, redGardenY, 0.006);
    }

    // ---- the route ---------------------------------------------------------------------------------

    @Test
    public void thereIsOnePathGroupPerDriveStepAndOneBadgePerStep() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                Document d = parse(RouteSvgRenderer.render(r, a));
                Element route = byId(d, "route");
                assertEquals(r.id(), countSteps(r, StepKind.DRIVE), withClass(route, "g", "drive-leg").size());
                assertEquals(r.id(), countSteps(r, StepKind.DRIVE), withClass(route, "path", "leg").size());
                assertEquals(r.id(), r.size(), withClass(route, "g", "badge").size());
                assertEquals(r.id(), countSteps(r, StepKind.INTAKE), withClass(route, "circle", "intake-ring").size());
                assertEquals(r.id(), countSteps(r, StepKind.SHOOT), withClass(route, "g", "shoot-pose").size());
                assertEquals(r.id(), countSteps(r, StepKind.SHOOT), withClass(route, "line", "shot").size());
                assertEquals(r.id(), countSteps(r, StepKind.PARK), withClass(route, "g", "park-outline").size());
                assertEquals(r.id(), countSteps(r, StepKind.WAIT), withClass(route, "g", "clock").size());
                // badges are numbered 1..N in order
                List<Element> badges = withClass(route, "g", "badge");
                for (int i = 0; i < badges.size(); i++) {
                    assertEquals(Integer.toString(i + 1), badges.get(i).getAttribute("data-step"));
                    assertEquals(Integer.toString(i + 1), all(badges.get(i), "text").get(0).getTextContent());
                }
            }
        }
    }

    @Test
    public void redRoutesUseRedInkAndBlueRoutesUseBlueInk() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            Document red = parse(RouteSvgRenderer.render(r, Alliance.RED));
            Document blue = parse(RouteSvgRenderer.render(r, Alliance.BLUE));
            Element redRoute = byId(red, "route");
            Element blueRoute = byId(blue, "route");
            assertEquals(BioBuzzConstants.Auto.RENDER_INK_RED, redRoute.getAttribute("data-ink"));
            assertEquals(BioBuzzConstants.Auto.RENDER_INK_BLUE, blueRoute.getAttribute("data-ink"));
            assertEquals(RouteSvgRenderer.inkFor(Alliance.RED), BioBuzzConstants.Auto.RENDER_INK_RED);
            for (Element leg : withClass(redRoute, "path", "leg")) {
                assertEquals(BioBuzzConstants.Auto.RENDER_INK_RED, leg.getAttribute("stroke"));
            }
            for (Element leg : withClass(blueRoute, "path", "leg")) {
                assertEquals(BioBuzzConstants.Auto.RENDER_INK_BLUE, leg.getAttribute("stroke"));
            }
            // nothing inside the blue route is drawn in red ink and vice versa
            for (Element e : all(blueRoute, "*")) {
                assertFalse(BioBuzzConstants.Auto.RENDER_INK_RED.equals(e.getAttribute("stroke")));
                assertFalse(BioBuzzConstants.Auto.RENDER_INK_RED.equals(e.getAttribute("fill")));
            }
            for (Element e : all(redRoute, "*")) {
                assertFalse(BioBuzzConstants.Auto.RENDER_INK_BLUE.equals(e.getAttribute("stroke")));
                assertFalse(BioBuzzConstants.Auto.RENDER_INK_BLUE.equals(e.getAttribute("fill")));
            }
        }
    }

    @Test
    public void theStartFootprintIsToScaleAtTheStartPoseWithFrontAndBackMarked() throws Exception {
        AutoRoute r = RouteCatalog.byId("red_c1");
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        Element start = byId(d, "start-footprint");
        assertNotNull(start);
        Element polygon = withClass(start, "polygon", "outline").get(0);
        double[] b = polygonBounds(polygon);
        // C1 start (58, 9.2, -90): the 17.80 x 17.80 in robot, its length along y and its width along x
        assertEquals(BioBuzzConstants.Robot.WIDTH_IN * S, b[1] - b[0], 0.05);
        assertEquals(BioBuzzConstants.Robot.LENGTH_IN * S, b[3] - b[2], 0.05);
        assertEquals("center x", X0 + 58.0 * S, (b[0] + b[1]) / 2.0, 0.05);
        // The pose (58, 9.2) is the robot's center of rotation, NOT the middle of its outline: facing -90 the FRONT edge is 9.77 in
        // below it (toward the audience wall, the bottom of the picture) and the BACK edge 8.03 in above it.
        assertEquals("front edge", Y_BOTTOM - (9.2 - BioBuzzConstants.Robot.FRONT_EXTENT_IN) * S, b[3], 0.05);
        assertEquals("back edge", Y_BOTTOM - (9.2 + BioBuzzConstants.Robot.REAR_EXTENT_IN) * S, b[2], 0.05);
        assertTrue("the outline is not centered on the pose", Math.abs((b[2] + b[3]) / 2.0 - (Y_BOTTOM - 9.2 * S)) > 3.0);
        // the center of rotation is marked, at the pose itself
        Element mark = withClass(start, "g", "pose-point").get(0);
        Element dot = all(mark, "circle").get(0);
        assertEquals("pose point x", X0 + 58.0 * S, Double.parseDouble(dot.getAttribute("cx")), 0.05);
        assertEquals("pose point y", Y_BOTTOM - 9.2 * S, Double.parseDouble(dot.getAttribute("cy")), 0.05);
        assertEquals(58.0, Double.parseDouble(mark.getAttribute("data-x")), 1e-6);
        // FRONT at the audience wall (bottom of the image), BACK at the top, in different colours
        Element front = withClass(start, "line", "front-edge").get(0);
        Element back = withClass(start, "line", "back-edge").get(0);
        assertEquals(BioBuzzConstants.Auto.RENDER_FRONT_COLOR, front.getAttribute("stroke"));
        assertEquals(BioBuzzConstants.Auto.RENDER_BACK_COLOR, back.getAttribute("stroke"));
        assertTrue(!front.getAttribute("stroke").equals(back.getAttribute("stroke")));
        assertTrue("the front edge is lower on the page than the back edge",
                Double.parseDouble(front.getAttribute("y1")) > Double.parseDouble(back.getAttribute("y1")));
        assertEquals("front edge lies on the footprint's near-wall side", b[3], Double.parseDouble(front.getAttribute("y1")), 0.05);
        // a heading arrow exists
        assertEquals(1, withClass(start, "line", "heading-arrow").size());
    }

    @Test
    public void theBlueStartFootprintIsTheHalfTurnOfTheRedOne() throws Exception {
        AutoRoute r = RouteCatalog.byId("red_b1");
        Document d = parse(RouteSvgRenderer.render(r, Alliance.BLUE));
        Element polygon = withClass(byId(d, "start-footprint"), "polygon", "outline").get(0);
        double[] b = polygonBounds(polygon);
        Pose2 blueStart = Alliance.BLUE.fromRed(r.startPose(), BioBuzzConstants.Field.SIZE_IN);
        Footprint fp = Footprint.of(blueStart);
        // the outline is the real Footprint of the blue pose (not centered on it), and the pose point is marked at the pose
        assertEquals(X0 + (fp.bounds().minX + fp.bounds().maxX) / 2.0 * S, (b[0] + b[1]) / 2.0, 0.05);
        assertEquals(Y_BOTTOM - (fp.bounds().minY + fp.bounds().maxY) / 2.0 * S, (b[2] + b[3]) / 2.0, 0.05);
        assertEquals((fp.bounds().maxX - fp.bounds().minX) * S, b[1] - b[0], 0.05);
        Element dot = all(withClass(byId(d, "start-footprint"), "g", "pose-point").get(0), "circle").get(0);
        assertEquals(X0 + blueStart.x * S, Double.parseDouble(dot.getAttribute("cx")), 0.05);
        assertEquals(Y_BOTTOM - blueStart.y * S, Double.parseDouble(dot.getAttribute("cy")), 0.05);
        // the blue robot starts at the top (rear wall) of the image: smaller pixel y than the field middle
        assertTrue((b[2] + b[3]) / 2.0 < Y_BOTTOM - BioBuzzConstants.Field.CENTER_IN * S);
    }

    @Test
    public void straightLegsAreLinesAndCurvedLegsAreSmoothPaths() throws Exception {
        // a curved leg (a via point) followed by two straight ones
        AutoRoute curve = AutoRoute.builder("curve").title("Curve").startTile("B1").start(35, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .driveConstant(30.0, 45.0, -90.0).via(30.0, 24.0)
                .driveReverseTangent(30.0, 100.0)
                .driveLinear(12.0, 108.0, 0.0).build();
        Document curved = parse(RouteSvgRenderer.render(curve, Alliance.RED));
        List<Element> legs = withClass(byId(curved, "route"), "path", "leg");
        assertEquals(3, legs.size());
        assertTrue(legs.get(0).getAttribute("d"), legs.get(0).getAttribute("d").contains(" Q "));
        assertTrue(legs.get(1).getAttribute("d").contains(" L "));
        assertFalse(legs.get(1).getAttribute("d").contains(" Q "));
        assertTrue(legs.get(0).getAttribute("d").startsWith("M "));
        // every leg carries a direction arrowhead
        for (Element g : withClass(byId(curved, "route"), "g", "drive-leg")) {
            assertTrue(withClass(g, "polygon", "arrowhead").size() >= 1);
        }
    }

    @Test
    public void shootStepsDrawTheShotArrowFromTheLaunchPointToTheAimPoint() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_c1"), Alliance.RED));
        Element shoot = withClass(byId(d, "route"), "g", "shoot-pose").get(0);
        Element shot = withClass(shoot, "line", "shot").get(0);
        assertFalse("dashed", shot.getAttribute("stroke-dasharray").isEmpty());
        // C1 (58, 9.2, -90): launch point (58, 9.2 + 8.03 = 17.23), the rear edge; the aim point of the red audience cell is on the HIVE axis, 54.156 in out
        assertEquals(X0 + 58.0 * S, Double.parseDouble(shot.getAttribute("x1")), 0.05);
        assertEquals(Y_BOTTOM - (9.2 - BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN) * S, Double.parseDouble(shot.getAttribute("y1")), 0.05);
        // it points up the page, toward the HIVE
        assertTrue(Double.parseDouble(shot.getAttribute("y2")) < Double.parseDouble(shot.getAttribute("y1")));
        assertFalse("an explicit pose is drawn solid", "true".equals(shoot.getAttribute("data-nominal")));
        Document auto = parse(RouteSvgRenderer.render(RouteCatalog.byId("wall_shoot_park"), Alliance.RED));
        assertEquals("true", withClass(byId(auto, "route"), "g", "shoot-pose").get(0).getAttribute("data-nominal"));
    }

    @Test
    public void intakeRingSurroundsTheTargetAndIsLabelled() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("garden_cycle"), Alliance.RED));
        Element ring = withClass(byId(d, "route"), "circle", "intake-ring").get(0);
        assertEquals(X0 + 5.7 * S, Double.parseDouble(ring.getAttribute("cx")), 0.05);
        assertEquals(Y_BOTTOM - 1.4 * S, Double.parseDouble(ring.getAttribute("cy")), 0.05);
        assertFalse(ring.getAttribute("stroke-dasharray").isEmpty());
        boolean label = false;
        for (Element t : withClass(byId(d, "route"), "text", "step-label")) {
            label |= t.getTextContent().equals("INTAKE GARDEN +3");
        }
        assertTrue(label);
    }

    @Test
    public void aTipIsLabelledOnTheShotThatCausesIt() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("garden_cycle"), Alliance.RED));
        boolean tip = false;
        for (Element t : withClass(byId(d, "route"), "text", "step-label")) {
            tip |= t.getTextContent().equals("HIVE TIP +20");
        }
        assertTrue(tip);
    }

    @Test
    public void parkIsAnOutlineOfTheFinalFootprintInsideTheLoadingZone() throws Exception {
        AutoRoute r = RouteCatalog.byId("wall_shoot_park");
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        Element park = withClass(byId(d, "route"), "g", "park-outline").get(0);
        double[] b = polygonBounds(all(park, "polygon").get(0));
        // end pose (12, 108, 0) facing +x: the BACK edge is 8.03 in behind the center of rotation and the FRONT 9.77 in ahead, so the
        // outline runs x 3.97..21.77, y 99.1..116.9, overlapping the zone x 0..11 y 96..120
        assertEquals(X0 + (12.0 - BioBuzzConstants.Robot.REAR_EXTENT_IN) * S, b[0], 0.1);
        assertEquals(X0 + 3.97 * S, b[0], 0.1);
        assertEquals(X0 + (12.0 + BioBuzzConstants.Robot.FRONT_EXTENT_IN) * S, b[1], 0.1);
        assertEquals(Y_BOTTOM - (108.0 + BioBuzzConstants.Robot.LEFT_EXTENT_IN) * S, b[2], 0.1);
        assertEquals(Y_BOTTOM - 116.9 * S, b[2], 0.1);
        // the center of rotation is marked inside the park outline too
        Element dot = all(withClass(park, "g", "pose-point").get(0), "circle").get(0);
        assertEquals(X0 + 12.0 * S, Double.parseDouble(dot.getAttribute("cx")), 0.05);
        assertEquals(Y_BOTTOM - 108.0 * S, Double.parseDouble(dot.getAttribute("cy")), 0.05);
        assertFalse(all(park, "polygon").get(0).getAttribute("stroke-dasharray").isEmpty());
        double zoneRight = X0 + 11.0 * S;
        assertTrue("part of the outline lies inside the zone", b[0] < zoneRight);
    }

    @Test
    public void waitIsMarkedWithAClockAndItsSeconds() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_b1"), Alliance.RED));
        boolean found = false;
        for (Element t : withClass(byId(d, "route"), "text", "step-label")) {
            found |= t.getTextContent().equals("WAIT 4.5 s");
        }
        assertTrue(found);
        assertEquals(1, withClass(byId(d, "route"), "g", "clock").size());
    }

    // ---- footer, legend, findings ----------------------------------------------------------------------

    @Test
    public void theFooterStatesTimePollenAndPoints() throws Exception {
        AutoRoute r = RouteCatalog.byId("garden_cycle");
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        String metrics = byId(d, "footer-metrics").getTextContent();
        assertTrue(metrics, metrics.contains(String.format(java.util.Locale.ROOT, "%.1f s", r.estimatedSeconds())));
        assertTrue(metrics, metrics.contains("Our AUTO points: 23"));
        assertTrue(metrics, metrics.contains("LEAVE 3"));
        assertTrue(metrics, metrics.contains("HIVE TIP 20"));
        String pollen = byId(d, "footer-pollen").getTextContent();
        assertTrue(pollen, pollen.contains("+3 taken"));
        assertTrue(pollen, pollen.contains("-3 fired"));
        assertTrue(pollen, pollen.contains("balance 0"));
        assertTrue(pollen, pollen.contains("no findings"));
        assertNotNull(byId(d, "legend"));
        assertTrue(withClass(byId(d, "legend"), "g", "legend-item").size() >= 10);
    }

    // ---- who shoots the first tip ----------------------------------------------------------------------

    private static AutoRoute partnerRoute() {
        return AutoRoute.builder("partner_rear").title("Partner tips first").startTile("C6").start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90)
                .partnerTipsFirst().waitForTip(20).shootUp(61.5, 130.0, 90, 4).build();
    }

    @Test
    public void theSubtitleSaysWhoShootsTheFirstTipForEveryRoute() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                String sub = byId(parse(RouteSvgRenderer.render(r, a)), "subtitle").getTextContent();
                assertTrue(r.id() + ": " + sub, sub.contains("first tip: " + r.firstTipSummary()));
            }
        }
        // a partner route also says how late the partner may tip: it gives up waiting at 20 s, the rest needs about 6 s
        String partnerSub = byId(parse(RouteSvgRenderer.render(partnerRoute(), Alliance.RED)), "subtitle").getTextContent();
        assertTrue(partnerSub, partnerSub.contains("first tip: partner tips first, assumed at 12.0 s; latest that works 19.5 s"));
        // the description for screen readers carries it too
        assertTrue(byId(parse(RouteSvgRenderer.render(partnerRoute(), Alliance.BLUE)), "svg-desc").getTextContent()
                .contains("First tip: partner tips first, assumed at 12.0 s; latest that works 19.5 s."));
        // a partner route that never shoots at the cell that comes up does not depend on the time at all
        AutoRoute independent = AutoRoute.builder("partner_walk").title("t").startTile("C6").start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90).partnerTipsFirst()
                .driveConstant(61.5, 120, 90).build();
        String walkSub = byId(parse(RouteSvgRenderer.render(independent, Alliance.RED)), "subtitle").getTextContent();
        assertTrue(walkSub, walkSub.contains("assumed at 12.0 s; any tip time works"));
        // a route that shoots the first tip itself has no such words
        String selfSub = byId(parse(RouteSvgRenderer.render(RouteCatalog.byId("garden_cycle"), Alliance.RED)), "subtitle").getTextContent();
        assertFalse(selfSub, selfSub.contains("latest that works"));
    }

    @Test
    public void thePartnersAssumedTipIsShownInTheFooterButNeverInOurPoints() throws Exception {
        Document partner = parse(RouteSvgRenderer.render(partnerRoute(), Alliance.RED));
        assertTrue(byId(partner, "footer-pollen").getTextContent(), byId(partner, "footer-pollen").getTextContent().contains("Partner tip +20 (not ours)"));
        // our points: the robot ends off the rear wall (LEAVE 3) and tips nothing itself
        assertTrue(byId(partner, "footer-metrics").getTextContent(), byId(partner, "footer-metrics").getTextContent().contains("Our AUTO points: 3 "));
        assertTrue(byId(partner, "footer-metrics").getTextContent().contains("HIVE TIP 0"));
        // a route that shoots the first tip itself has no partner text
        Document self = parse(RouteSvgRenderer.render(RouteCatalog.byId("garden_cycle"), Alliance.RED));
        assertFalse(byId(self, "footer-pollen").getTextContent().contains("Partner tip"));
    }

    @Test
    public void waitForTipIsDrawnAsAClockWithItsGiveUpTime() throws Exception {
        Document d = parse(RouteSvgRenderer.render(partnerRoute(), Alliance.RED));
        boolean found = false;
        for (Element t : withClass(byId(d, "route"), "text", "step-label")) {
            found |= t.getTextContent().equals("WAIT FOR TIP (give up at 20 s)");
        }
        assertTrue(found);
        assertEquals(1, withClass(byId(d, "route"), "g", "clock").size());
        assertEquals(1, countSteps(partnerRoute(), StepKind.WAIT_FOR_TIP));
        assertEquals("one numbered badge for every step", partnerRoute().size(), withClass(byId(d, "route"), "g", "badge").size());
    }

    @Test
    public void validatorFindingsAreListedAndMarkedOnTheirSteps() throws Exception {
        // a route with exactly one finding: its only leg runs straight through the HIVE frame (a legal start, front edge on the wall)
        AutoRoute throughTheHive = AutoRoute.builder("through_the_hive").title("Through the HIVE").startTile("C1")
                .start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).driveConstant(58, 118, -90).build();
        Document d = parse(RouteSvgRenderer.render(throughTheHive, Alliance.RED));
        String pollen = byId(d, "footer-pollen").getTextContent();
        assertTrue(pollen, pollen.contains("1 ERROR"));
        List<Element> lines = withClass(byId(d, "footer"), "g", "issue-line");
        assertEquals(1, lines.size());
        assertEquals("ERROR", lines.get(0).getAttribute("data-severity"));
        assertTrue(lines.get(0).getTextContent().contains("HIVE frame keep-out"));
        List<Element> marks = withClass(byId(d, "route"), "g", "issue-mark");
        assertEquals(1, marks.size());
        assertEquals("2", marks.get(0).getAttribute("data-step"));
        assertEquals("ERROR", marks.get(0).getAttribute("data-severity"));
    }

    @Test
    public void aCleanRouteHasNoIssueMarks() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("wall_shoot_park"), Alliance.BLUE));
        assertEquals(0, withClass(byId(d, "route"), "g", "issue-mark").size());
        assertEquals(0, withClass(byId(d, "footer"), "g", "issue-line").size());
    }

    @Test
    public void manyFindingsAreCappedWithAPointerToTheGallery() throws Exception {
        // a deliberately terrible route: crosses the wall, the HIVE and the center line, then overruns the time
        AutoRoute bad = AutoRoute.builder("bad").title("Bad").startTile("C1").start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .shootUp(5).shootUp(5)
                .driveConstant(75, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .driveConstant(30, -3, -90)
                .driveConstant(58, 118, -90)
                .waitSeconds(40).build();
        Document d = parse(RouteSvgRenderer.render(bad, Alliance.RED));
        List<Element> lines = withClass(byId(d, "footer"), "g", "issue-line");
        assertEquals(4, lines.size());
        assertTrue(byId(d, "footer").getTextContent().contains("more findings"));
    }
}
