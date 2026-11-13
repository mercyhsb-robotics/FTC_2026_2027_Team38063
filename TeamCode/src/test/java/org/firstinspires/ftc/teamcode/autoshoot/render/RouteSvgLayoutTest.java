package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StepKind;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/**
 * Where things are drawn: the field layout, the placement of badges and labels, and what the footer and
 * legend leave of the 900 x 900 px image. {@link RouteSvgRendererTest} covers well-formedness, the element
 * counts and the inks.
 */
public class RouteSvgLayoutTest {
    private static final double S = BioBuzzConstants.Auto.RENDER_FIELD_SCALE_PX_PER_IN;
    private static final double CANVAS = BioBuzzConstants.Auto.RENDER_CANVAS_PX;
    private static final double FIELD_PX = BioBuzzConstants.Field.SIZE_IN * S;
    private static final double X0 = (CANVAS - FIELD_PX) / 2.0;
    /** The SVG writes every number rounded to 0.01, so a coordinate computed here may differ from the file by up to half of that. */
    private static final double PX = 0.006;
    private static final double Y_TOP = BioBuzzConstants.Auto.RENDER_FIELD_TOP_PX;
    private static final double Y_BOTTOM = Y_TOP + FIELD_PX;

    // ---- DOM helpers -------------------------------------------------------------------------------

    private static Document parse(String svg) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        return f.newDocumentBuilder().parse(new ByteArrayInputStream(svg.getBytes(Charset.forName("UTF-8"))));
    }

    private static Document render(String routeId, Alliance alliance) throws Exception {
        return parse(RouteSvgRenderer.render(RouteCatalog.byId(routeId), alliance));
    }

    private static List<Element> all(Node root, String tag) {
        List<Element> out = new ArrayList<Element>();
        NodeList nl = root instanceof Document ? ((Document) root).getElementsByTagName(tag) : ((Element) root).getElementsByTagName(tag);
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

    private static double num(Element e, String attribute) {
        return Double.parseDouble(e.getAttribute(attribute));
    }

    private static double[] polygonBounds(Element polygon) {
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (String p : polygon.getAttribute("points").trim().split("\\s+")) {
            String[] xy = p.split(",");
            minX = Math.min(minX, Double.parseDouble(xy[0]));
            maxX = Math.max(maxX, Double.parseDouble(xy[0]));
            minY = Math.min(minY, Double.parseDouble(xy[1]));
            maxY = Math.max(maxY, Double.parseDouble(xy[1]));
        }
        return new double[] {minX, maxX, minY, maxY};
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

    private static List<String> stepLabels(Document d) {
        List<String> out = new ArrayList<String>();
        for (Element t : withClass(byId(d, "route"), "text", "step-label")) {
            out.add(t.getTextContent());
        }
        return out;
    }

    // ---- the field ---------------------------------------------------------------------------------

    @Test
    public void tilesAreLaidOutWithA1AtTheAudienceCornerOfTheRedWall() throws Exception {
        Document d = render("red_b1", Alliance.RED);
        List<Element> tiles = withClass(d, "rect", "tile");
        assertEquals(36, tiles.size());
        for (Element t : tiles) {
            String name = t.getAttribute("data-tile");
            int col = name.charAt(0) - 'A';
            int row = name.charAt(1) - '1';
            // each tile is drawn between its own two seams (23.5 in pitch, the outer tiles run to the walls)
            Rect tile = TileGrid.tile(name);
            assertEquals(name, X0 + tile.minX * S, num(t, "x"), PX);
            assertEquals(name, Y_BOTTOM - tile.maxY * S, num(t, "y"), PX);
            assertEquals(name, (tile.maxX - tile.minX) * S, num(t, "width"), PX);
            assertEquals(name, (tile.maxY - tile.minY) * S, num(t, "height"), PX);
            boolean light = (col + row) % 2 == 0;
            assertEquals(name, light ? BioBuzzConstants.Auto.RENDER_TILE_LIGHT : BioBuzzConstants.Auto.RENDER_TILE_DARK, t.getAttribute("fill"));
        }
    }

    @Test
    public void theBorderIsTheWholeFieldAndTheAreasOutsideItAreFaint() throws Exception {
        Document d = render("red_b1", Alliance.RED);
        Element border = byId(d, "field-border");
        assertEquals(X0, num(border, "x"), PX);
        assertEquals(Y_TOP, num(border, "y"), PX);
        assertEquals(FIELD_PX, num(border, "width"), PX);
        assertEquals(FIELD_PX, num(border, "height"), PX);
        for (Element area : withClass(d, "rect", "alliance-area")) {
            boolean red = area.getAttribute("data-alliance").equals("RED");
            double shown = BioBuzzConstants.Auto.RENDER_ALLIANCE_AREA_SHOWN_IN * S;
            assertEquals(shown, num(area, "width"), PX);
            assertEquals(red ? X0 - shown : X0 + FIELD_PX, num(area, "x"), PX);
            assertTrue("outside the border", red ? num(area, "x") + num(area, "width") <= X0 + PX : num(area, "x") >= X0 + FIELD_PX - PX);
        }
    }

    @Test
    public void theTwoHivesHangOnTheirAxesAndTheirUpCellsAreOnTheRightSides() throws Exception {
        Document d = render("red_b1", Alliance.RED);
        for (Element cell : withClass(d, "g", "cell")) {
            Element rect = all(cell, "rect").get(0);
            double centerX = num(rect, "x") + num(rect, "width") / 2.0;
            double axis = cell.getAttribute("data-alliance").equals("RED") ? BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN
                    : BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN;
            assertEquals(cell.getAttribute("data-alliance") + " HIVE axis", X0 + axis * S, centerX, PX);
            assertEquals("a CELL is drawn as wide as its plan outline", BioBuzzConstants.Field.CELL_PLAN_WIDTH_X_IN * S, num(rect, "width"), PX);
        }
        // the red UP cell is the audience one: below the HIVE center in the picture; the blue UP cell is above it
        double centerY = Y_BOTTOM - BioBuzzConstants.Field.CENTER_IN * S;
        List<Element> up = withClass(d, "g", "cell-up");
        assertEquals(2, up.size());
        for (Element cell : up) {
            boolean below = num(all(cell, "rect").get(0), "y") > centerY;
            assertEquals(cell.getAttribute("data-alliance"), cell.getAttribute("data-alliance").equals("RED"), below);
        }
        for (Element cell : withClass(d, "g", "cell-down")) {
            boolean below = num(all(cell, "rect").get(0), "y") > centerY;
            assertEquals(cell.getAttribute("data-alliance"), cell.getAttribute("data-alliance").equals("BLUE"), below);
        }
    }

    @Test
    public void flowersAndGardenPollenSitWhereTheFieldModelPutsThem() throws Exception {
        Document d = render("red_leave_park", Alliance.RED);
        double[][] centers = BioBuzzConstants.Field.FLOWER_CENTERS_IN;
        List<Element> flowers = withClass(d, "g", "flower");
        assertEquals(4, flowers.size());
        for (int i = 0; i < 4; i++) {
            Element dot = all(flowers.get(i), "circle").get(0);
            assertEquals("FLOWER " + (i + 1), X0 + centers[i][0] * S, num(dot, "cx"), PX);
            assertEquals("FLOWER " + (i + 1), Y_BOTTOM - centers[i][1] * S, num(dot, "cy"), PX);
        }
        // the blue GARDEN POLLEN is the half turn of the red one: (size - x, size - 1.4)
        for (Element g : withClass(d, "g", "garden")) {
            List<Element> pollen = withClass(g, "circle", "pollen");
            assertEquals(4, pollen.size());
            boolean red = g.getAttribute("data-alliance").equals("RED");
            for (int i = 0; i < 4; i++) {
                double x = BioBuzzConstants.Auto.RED_GARDEN_POLLEN_X_IN[i];
                double y = BioBuzzConstants.Auto.RED_GARDEN_POLLEN_Y_IN;
                assertEquals(X0 + (red ? x : BioBuzzConstants.Field.SIZE_IN - x) * S, num(pollen.get(i), "cx"), PX);
                assertEquals(Y_BOTTOM - (red ? y : BioBuzzConstants.Field.SIZE_IN - y) * S, num(pollen.get(i), "cy"), PX);
                assertEquals(BioBuzzConstants.Auto.POLLEN_DIAMETER_IN / 2.0 * S, num(pollen.get(i), "r"), PX);
            }
        }
    }

    @Test
    public void theOpponentSideLabelIsOnTheOpponentSideOfTheCenterLine() throws Exception {
        double center = X0 + BioBuzzConstants.Field.CENTER_IN * S;
        Element red = byId(render("red_c1", Alliance.RED), "opponent-label");
        Element blue = byId(render("red_c1", Alliance.BLUE), "opponent-label");
        assertTrue("red's opponent is on the right", num(red, "x") > center);
        assertTrue("blue's opponent is on the left", num(blue, "x") < center);
        assertTrue(red.getAttribute("transform").startsWith("rotate(-90"));
        assertEquals("opponent side in AUTO (G402)", red.getTextContent());
    }

    // ---- the robot ---------------------------------------------------------------------------------

    @Test
    public void theHeadingArrowOfTheStartPointsTheWayTheRobotFaces() throws Exception {
        // B1 start (35, 9.2, -90): facing the audience wall, which is down the page
        Element down = withClass(byId(render("red_b1", Alliance.RED), "start-footprint"), "line", "heading-arrow").get(0);
        assertEquals(num(down, "x1"), num(down, "x2"), PX);
        assertTrue(num(down, "y2") > num(down, "y1"));
        // A3 start (9.2, 70.3, 180): facing the red wall, which is on the left
        Element left = withClass(byId(render("red_leave_park", Alliance.RED), "start-footprint"), "line", "heading-arrow").get(0);
        assertTrue(num(left, "x2") < num(left, "x1"));
        assertEquals(num(left, "y1"), num(left, "y2"), PX);
        // the blue half turn of B1 faces up the page
        Element up = withClass(byId(render("red_b1", Alliance.BLUE), "start-footprint"), "line", "heading-arrow").get(0);
        assertTrue(num(up, "y2") < num(up, "y1"));
    }

    @Test
    public void everyFootprintMarksItsFrontAndBackInTwoColours() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
            int fronts = 0;
            int backs = 0;
            for (Element e : all(byId(d, "route"), "line")) {
                if (hasClass(e, "front-edge")) {
                    fronts++;
                    assertEquals(BioBuzzConstants.Auto.RENDER_FRONT_COLOR, e.getAttribute("stroke"));
                }
                if (hasClass(e, "back-edge")) {
                    backs++;
                    assertEquals(BioBuzzConstants.Auto.RENDER_BACK_COLOR, e.getAttribute("stroke"));
                }
            }
            int expected = countSteps(r, StepKind.START) + countSteps(r, StepKind.SHOOT) + countSteps(r, StepKind.INTAKE)
                    + countSteps(r, StepKind.PARK) + (endsOnADrive(r) ? 1 : 0);      // a route that ends on a DRIVE draws an END outline
            assertEquals(r.id(), expected, fronts);
            assertEquals(r.id(), expected, backs);
        }
    }

    @Test
    public void aRouteThatEndsOnADriveShowsWhereTheRobotStopsAndOneThatParksDoesNot() throws Exception {
        AutoRoute open = AutoRoute.builder("open").title("Open end").startTile("C1").start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .driveConstant(30, 30, -90).build();
        Document d = parse(RouteSvgRenderer.render(open, Alliance.RED));
        List<Element> end = withClass(byId(d, "route"), "g", "end-outline");
        assertEquals(1, end.size());
        assertEquals("2", end.get(0).getAttribute("data-step"));
        double[] b = polygonBounds(all(end.get(0), "polygon").get(0));
        // the outline of the last pose (30, 30, -90): the FRONT edge 9.77 in below the center of rotation, the BACK edge 8.03 in above it
        assertEquals(X0 + 30.0 * S, (b[0] + b[1]) / 2.0, 0.05);
        assertEquals(Y_BOTTOM - (30.0 - BioBuzzConstants.Robot.FRONT_EXTENT_IN) * S, b[3], 0.05);
        assertEquals(Y_BOTTOM - (30.0 + BioBuzzConstants.Robot.REAR_EXTENT_IN) * S, b[2], 0.05);
        Element dot = all(withClass(end.get(0), "g", "pose-point").get(0), "circle").get(0);
        assertEquals("the center of rotation is marked on the last pose", Y_BOTTOM - 30.0 * S, num(dot, "cy"), 0.05);
        assertTrue(stepLabels(d).contains("END"));
        for (AutoRoute r : RouteCatalog.all()) {
            assertEquals(r.id(), endsOnADrive(r) ? 1 : 0,
                    withClass(byId(parse(RouteSvgRenderer.render(r, Alliance.RED)), "route"), "g", "end-outline").size());
        }
    }

    private static boolean endsOnADrive(AutoRoute r) {
        return r.step(r.size() - 1).kind() == StepKind.DRIVE;
    }

    // ---- the 900 x 900 px image --------------------------------------------------------------------

    @Test
    public void everythingIsDrawnInsideTheCanvas() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                Document d = parse(RouteSvgRenderer.render(r, a));
                String who = r.id() + " " + a;
                for (Element e : all(d, "text")) {
                    assertTrue(who + " text '" + e.getTextContent() + "' x", num(e, "x") >= 0 && num(e, "x") <= CANVAS);
                    assertTrue(who + " text '" + e.getTextContent() + "' y", num(e, "y") >= 0 && num(e, "y") <= CANVAS);
                }
                for (Element e : all(d, "circle")) {
                    assertTrue(who + " circle x", num(e, "cx") - num(e, "r") >= 0 && num(e, "cx") + num(e, "r") <= CANVAS);
                    assertTrue(who + " circle y", num(e, "cy") - num(e, "r") >= 0 && num(e, "cy") + num(e, "r") <= CANVAS);
                }
                for (Element e : all(d, "polygon")) {
                    double[] b = polygonBounds(e);
                    assertTrue(who + " polygon " + e.getAttribute("points"), b[0] >= 0 && b[1] <= CANVAS && b[2] >= 0 && b[3] <= CANVAS);
                }
            }
        }
    }

    @Test
    public void theSubtitleAndFooterLinesWithTheFirstTipTextStillFitTheWidthOfTheImage() throws Exception {
        AutoRoute partner = AutoRoute.builder("partner_rear_wait_then_two_volleys").title("Partner tips first").startTile("C6")
                .start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90).partnerTipsFirst().waitForTip(20).shootUp(61.5, 130.0, 90, 4).build();
        List<AutoRoute> routes = new ArrayList<AutoRoute>(RouteCatalog.all());
        routes.add(partner);
        for (AutoRoute r : routes) {
            for (Alliance a : Alliance.values()) {
                Document d = parse(RouteSvgRenderer.render(r, a));
                for (String id : new String[] {"subtitle", "footer-metrics", "footer-pollen"}) {
                    Element line = byId(d, id);
                    // these lines are 12 px; a line that would be too long shrinks (an SVG never wraps text)
                    double perChar = BioBuzzConstants.Auto.RENDER_LABEL_CHAR_WIDTH_EM * num(line, "font-size");
                    double right = num(line, "x") + line.getTextContent().length() * perChar;
                    assertTrue(r.id() + " " + a + " " + id + " is about " + (int) right + " px wide: " + line.getTextContent(),
                            right <= CANVAS - 24.0);
                }
            }
        }
        // the long id really did need the shrinking (without it the line would run 942 px), and a short line keeps its 12 px
        assertTrue(num(byId(parse(RouteSvgRenderer.render(partner, Alliance.RED)), "subtitle"), "font-size") < 12.0);
        assertEquals(12.0, num(byId(parse(RouteSvgRenderer.render(RouteCatalog.byId("red_leave_park"), Alliance.RED)), "subtitle"),
                "font-size"), 1e-9);
    }

    @Test
    public void theLegendAndFooterSitBelowTheFieldInsideTheImage() throws Exception {
        for (AutoRoute r : RouteCatalog.all()) {
            Document d = parse(RouteSvgRenderer.render(r, Alliance.RED));
            List<Element> items = withClass(byId(d, "legend"), "g", "legend-item");
            assertEquals(13, items.size());
            for (Element item : items) {
                for (Element text : all(item, "text")) {
                    assertTrue(r.id() + " legend below the AUDIENCE label", num(text, "y") > Y_BOTTOM + 40);
                }
            }
            for (Element text : all(byId(d, "footer"), "text")) {
                assertTrue(r.id() + " footer text inside the image: " + num(text, "y"), num(text, "y") < CANVAS);
                assertTrue(r.id() + " footer below the legend", num(text, "y") > Y_BOTTOM + 70);
            }
        }
    }

    @Test
    public void stepNumberCirclesNeverOverlapEachOther() throws Exception {
        double radius = BioBuzzConstants.Auto.RENDER_BADGE_RADIUS_PX;
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                List<Element> badges = withClass(byId(parse(RouteSvgRenderer.render(r, a)), "badges"), "g", "badge");
                for (int i = 0; i < badges.size(); i++) {
                    Element ci = all(badges.get(i), "circle").get(0);
                    assertEquals(radius, num(ci, "r"), PX);
                    for (int j = i + 1; j < badges.size(); j++) {
                        Element cj = all(badges.get(j), "circle").get(0);
                        double gap = Math.hypot(num(ci, "cx") - num(cj, "cx"), num(ci, "cy") - num(cj, "cy"));
                        assertTrue(r.id() + " " + a + ": badges " + (i + 1) + " and " + (j + 1) + " are " + gap + " px apart", gap >= 2 * radius);
                    }
                }
            }
        }
    }

    // ---- labels ------------------------------------------------------------------------------------

    @Test
    public void aSingleShotIsLabelledPlainlyAndShotsFromTheSamePoseCarryTheirStepNumber() throws Exception {
        List<String> single = stepLabels(render("wall_shoot_park", Alliance.RED));
        assertTrue(single.toString(), single.contains("SHOOT 4"));
        // two volleys of 2 into the same UP cell from the same solver pose (together they are the first tip)
        AutoRoute split = AutoRoute.builder("split").title("Two volleys").startTile("B1").start(35, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .shootUp(2).shootUp(2).build();
        List<String> twice = stepLabels(parse(RouteSvgRenderer.render(split, Alliance.RED)));
        assertTrue(twice.toString(), twice.contains("SHOOT 2 (step 2)"));
        assertTrue(twice.toString(), twice.contains("SHOOT 2 (step 3)"));
        assertFalse(twice.toString(), twice.contains("SHOOT 2"));
    }

    @Test
    public void startParkWaitAndNamedLegsAreLabelled() throws Exception {
        List<String> labels = stepLabels(render("red_c1", Alliance.RED));
        for (String expected : new String[] {"START C1", "SHOOT 4", "WAIT 4.5 s", "path1", "path2", "path3", "PARK"}) {
            assertTrue(expected + " in " + labels, labels.contains(expected));
        }
    }

    @Test
    public void intakeLabelsNameTheSourceAndTheCount() throws Exception {
        List<String> labels = stepLabels(render("garden_cycle", Alliance.BLUE));
        assertTrue(labels.toString(), labels.contains("INTAKE GARDEN +3"));
        AutoRoute flower = AutoRoute.builder("f").title("Flower").startTile("A3").start(BioBuzzConstants.Robot.FRONT_EXTENT_IN, 61.5, 180)
                .shootUp(4)
                .driveConstant(13.5, 62, 180).driveConstant(13.5, 48, 180)
                .intakeFlower(2.4, 48, 13.5, 48, 180, 4).build();
        List<String> flowerLabels = stepLabels(parse(RouteSvgRenderer.render(flower, Alliance.RED)));
        assertTrue(flowerLabels.toString(), flowerLabels.contains("INTAKE FLOWER +4"));
    }

    // ---- findings ----------------------------------------------------------------------------------

    @Test
    public void thePrecomputedFindingsOverloadDrawsExactlyTheFindingsItIsGiven() throws Exception {
        List<Issue> given = new ArrayList<Issue>();
        given.add(new Issue(Issue.Severity.WARN, 2, Issue.Rule.SHOOT_POSE, "step 3 made up for the test"));
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("wall_shoot_park"), Alliance.RED, given));
        assertEquals(1, withClass(byId(d, "footer"), "g", "issue-line").size());
        List<Element> marks = withClass(byId(d, "route"), "g", "issue-mark");
        assertEquals(1, marks.size());
        assertEquals("3", marks.get(0).getAttribute("data-step"));
        assertEquals("WARN", marks.get(0).getAttribute("data-severity"));
        assertTrue(byId(d, "footer-pollen").getTextContent().contains("0 ERROR, 1 WARN"));
    }

    @Test
    public void aBlueAuthoredRouteDrawnForRedIsTheSameDrawingAsTheRedOne() {
        AutoRoute blueAuthored = RouteCatalog.byId("red_c1").forAlliance(Alliance.BLUE);
        assertEquals(RouteSvgRenderer.render(RouteCatalog.byId("red_c1"), Alliance.RED), RouteSvgRenderer.render(blueAuthored, Alliance.RED));
    }
}
