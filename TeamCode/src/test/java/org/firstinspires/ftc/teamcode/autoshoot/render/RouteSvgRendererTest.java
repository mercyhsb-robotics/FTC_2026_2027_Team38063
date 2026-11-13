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
import org.firstinspires.ftc.teamcode.autoshoot.routes.Footprint;
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
                .start(35, 9.2, -90).waitSeconds(1).note("x < y & z").build();
        Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
        assertTrue(byId(d, "svg-title").getTextContent().contains("A & B <fast> \"auto\" it's"));
    }

    @Test
    public void renderingIsDeterministic() {
        AutoRoute r = RouteCatalog.byId("wall_shoot_park");
        assertEquals(RouteSvgRenderer.render(r, Alliance.RED), RouteSvgRenderer.render(r, Alliance.RED));
    }

    // ---- the field ---------------------------------------------------------------------------------

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
        assertEquals("red LOADING ZONE starts at the red wall", X0, redLz, 1e-6);
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
        assertEquals("red POLLEN row is one radius off the audience wall", Y_BOTTOM - 1.4 * S, redGardenY, 1e-6);
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
        // C1 start (58, 9.2, -90): the square 17.80 in robot, its length along y and its width along x
        assertEquals(BioBuzzConstants.Robot.WIDTH_IN * S, b[1] - b[0], 0.05);
        assertEquals(BioBuzzConstants.Robot.LENGTH_IN * S, b[3] - b[2], 0.05);
        assertEquals("center x", X0 + 58.0 * S, (b[0] + b[1]) / 2.0, 0.05);
        assertEquals("center y", Y_BOTTOM - 9.2 * S, (b[2] + b[3]) / 2.0, 0.05);
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
        Pose2 blueStart = Alliance.BLUE.fromRed(r.startPose(), 144.0);
        Footprint fp = Footprint.of(blueStart);
        assertEquals(X0 + blueStart.x * S, (b[0] + b[1]) / 2.0, 0.05);
        assertEquals(Y_BOTTOM - blueStart.y * S, (b[2] + b[3]) / 2.0, 0.05);
        assertEquals((fp.bounds().maxX - fp.bounds().minX) * S, b[1] - b[0], 0.05);
        // the blue robot starts at the top (rear wall) of the image: smaller pixel y than the field middle
        assertTrue((b[2] + b[3]) / 2.0 < Y_BOTTOM - 72 * S);
    }

    @Test
    public void straightLegsAreLinesAndCurvedLegsAreSmoothPaths() throws Exception {
        Document curved = parse(RouteSvgRenderer.render(RouteCatalog.byId("wall_shoot_park"), Alliance.RED));
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
        // C1 (58, 9.2, -90): launch point (58, 18.1); the aim point of the red audience cell is (59.25, 55.52)
        assertEquals(X0 + 58.0 * S, Double.parseDouble(shot.getAttribute("x1")), 0.05);
        assertEquals(Y_BOTTOM - 18.1 * S, Double.parseDouble(shot.getAttribute("y1")), 0.05);
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
        assertEquals(X0 + 5.9 * S, Double.parseDouble(ring.getAttribute("cx")), 0.05);
        assertEquals(Y_BOTTOM - 1.4 * S, Double.parseDouble(ring.getAttribute("cy")), 0.05);
        assertFalse(ring.getAttribute("stroke-dasharray").isEmpty());
        boolean label = false;
        for (Element t : withClass(byId(d, "route"), "text", "step-label")) {
            label |= t.getTextContent().equals("INTAKE GARDEN +4");
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
        // end pose (12, 108, 0): x 3.1..20.9, y 99.1..116.9, overlapping the zone x 0..11 y 96..120
        assertEquals(X0 + (12.0 - BioBuzzConstants.Robot.LENGTH_IN / 2.0) * S, b[0], 0.1);
        assertEquals(X0 + 3.1 * S, b[0], 0.1);
        assertEquals(Y_BOTTOM - (108.0 + BioBuzzConstants.Robot.WIDTH_IN / 2.0) * S, b[2], 0.1);
        assertEquals(Y_BOTTOM - 116.9 * S, b[2], 0.1);
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
        assertTrue(metrics, metrics.contains("Claimable AUTO points: 23"));
        assertTrue(metrics, metrics.contains("LEAVE 3"));
        assertTrue(metrics, metrics.contains("HIVE TIP 20"));
        String pollen = byId(d, "footer-pollen").getTextContent();
        assertTrue(pollen, pollen.contains("+4 taken"));
        assertTrue(pollen, pollen.contains("-8 fired"));
        assertTrue(pollen, pollen.contains("balance -4"));
        assertTrue(pollen, pollen.contains("no findings"));
        assertNotNull(byId(d, "legend"));
        assertTrue(withClass(byId(d, "legend"), "g", "legend-item").size() >= 10);
    }

    @Test
    public void validatorFindingsAreListedAndMarkedOnTheirSteps() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("red_c1"), Alliance.RED));
        String pollen = byId(d, "footer-pollen").getTextContent();
        assertTrue(pollen, pollen.contains("1 ERROR"));
        List<Element> lines = withClass(byId(d, "footer"), "g", "issue-line");
        assertEquals(1, lines.size());
        assertEquals("ERROR", lines.get(0).getAttribute("data-severity"));
        assertTrue(lines.get(0).getTextContent().contains("HIVE frame keep-out"));
        List<Element> marks = withClass(byId(d, "route"), "g", "issue-mark");
        assertEquals(1, marks.size());
        assertEquals("4", marks.get(0).getAttribute("data-step"));
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
        AutoRoute bad = AutoRoute.builder("bad").title("Bad").startTile("C1").start(58, 9.2, -90)
                .shootUp(5).shootUp(5)
                .driveConstant(75, 9.2, -90)
                .driveConstant(30, -3, -90)
                .driveConstant(58, 118, -90)
                .waitSeconds(40).build();
        Document d = parse(RouteSvgRenderer.render(bad, Alliance.RED));
        List<Element> lines = withClass(byId(d, "footer"), "g", "issue-line");
        assertEquals(4, lines.size());
        assertTrue(byId(d, "footer").getTextContent().contains("more findings"));
    }
}
