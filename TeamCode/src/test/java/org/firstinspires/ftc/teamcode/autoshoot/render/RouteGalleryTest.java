package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import javax.xml.parsers.DocumentBuilderFactory;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.WiggleTestRobot;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/** The gallery page, its files and the command line tool. */
public class RouteGalleryTest {
    @Rule
    public TemporaryFolder temp = new TemporaryFolder();

    // ---- helpers -----------------------------------------------------------------------------------

    private static Document parse(String xml) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        return f.newDocumentBuilder().parse(new ByteArrayInputStream(xml.getBytes(Charset.forName("UTF-8"))));
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

    private static Element section(Document page, String routeId) {
        for (Element s : withClass(page, "section", "route")) {
            if (routeId.equals(s.getAttribute("id"))) {
                return s;
            }
        }
        fail("no section for " + routeId);
        return null;
    }

    private static String rowValue(Element section, String name) {
        for (Element tr : all(section, "tr")) {
            List<Element> th = all(tr, "th");
            if (!th.isEmpty() && th.get(0).getTextContent().equals(name)) {
                return all(tr, "td").get(0).getTextContent();
            }
        }
        fail("no row '" + name + "'");
        return null;
    }

    private static AutoRoute simple(String id, String title) {
        return AutoRoute.builder(id).title(title).description("d").startTile("C1").start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).waitSeconds(1).build();
    }

    // ---- the set of files --------------------------------------------------------------------------

    @Test
    public void theGalleryIsAnIndexPlusARedAndABlueSvgPerRoute() {
        Map<String, String> files = RouteGallery.build(RouteCatalog.all());
        assertEquals(1 + 2 * RouteCatalog.all().size(), files.size());
        assertEquals("index.html first", "index.html", files.keySet().iterator().next());
        for (AutoRoute r : RouteCatalog.all()) {
            assertTrue(files.containsKey(r.id() + "_red.svg"));
            assertTrue(files.containsKey(r.id() + "_blue.svg"));
            assertEquals(r.id() + "_blue.svg", RouteGallery.svgFileName(r, Alliance.BLUE));
        }
    }

    @Test
    public void theSvgFilesAreTheRenderersOutputWithTheValidatorsFindings() {
        Map<String, String> files = RouteGallery.build(RouteCatalog.all());
        for (AutoRoute r : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                assertEquals(r.id() + " " + a, RouteSvgRenderer.render(r, a), files.get(RouteGallery.svgFileName(r, a)));
            }
        }
    }

    @Test
    public void routeIdsThatCannotBeFileNamesOrRepeatAreRejected() {
        for (String bad : new String[] {"a/b", "a b", "..", "a.svg", "a:b", "a*b"}) {
            try {
                RouteGallery.build(Collections.singletonList(simple(bad, "x")));
                fail("id '" + bad + "' must be rejected");
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains("file name"));
            }
        }
        try {
            RouteGallery.build(Arrays.asList(simple("same", "one"), simple("same", "two")));
            fail("duplicate ids must be rejected");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("same"));
        }
    }

    @Test
    public void buildingTwiceGivesIdenticalFiles() {
        assertEquals(RouteGallery.build(RouteCatalog.all()), RouteGallery.build(RouteCatalog.all()));
    }

    // ---- the page ----------------------------------------------------------------------------------

    @Test
    public void theIndexIsWellFormedAndHasOneSectionAndOneSummaryRowPerRoute() throws Exception {
        String html = RouteGallery.indexHtml(RouteCatalog.all());
        Document page = parse(html);
        assertEquals("html", page.getDocumentElement().getLocalName());
        assertEquals("en", page.getDocumentElement().getAttribute("lang"));
        assertEquals("BIOBUZZ autonomous routes", all(page, "title").get(0).getTextContent());
        assertEquals(RouteCatalog.all().size(), withClass(page, "section", "route").size());
        List<Element> rows = withClass(page, "tr", "summary-row");
        assertEquals(RouteCatalog.all().size(), rows.size());
        for (int i = 0; i < rows.size(); i++) {
            AutoRoute r = RouteCatalog.all().get(i);
            assertEquals("same order as the catalog", r.id(), rows.get(i).getAttribute("data-route"));
            assertEquals("the summary links to the section", "#" + r.id(), all(rows.get(i), "a").get(0).getAttribute("href"));
            assertNotNull(section(page, r.id()));
        }
        for (int i = 0; i < html.length(); i++) {
            assertTrue("plain ASCII, got " + (int) html.charAt(i), html.charAt(i) < 128);
        }
    }

    @Test
    public void everySectionShowsTheRedAndTheBlueDiagramSideBySide() throws Exception {
        Document page = parse(RouteGallery.indexHtml(RouteCatalog.all()));
        for (AutoRoute r : RouteCatalog.all()) {
            Element s = section(page, r.id());
            List<Element> pair = withClass(s, "div", "pair");
            assertEquals(1, pair.size());
            List<Element> imgs = all(pair.get(0), "img");
            assertEquals(2, imgs.size());
            assertEquals(r.id() + "_red.svg", imgs.get(0).getAttribute("src"));
            assertEquals(r.id() + "_blue.svg", imgs.get(1).getAttribute("src"));
            assertEquals(SvgWriter.n(BioBuzzConstants.Auto.RENDER_CANVAS_PX), imgs.get(0).getAttribute("width"));
            assertEquals(SvgWriter.n(BioBuzzConstants.Auto.RENDER_CANVAS_PX), imgs.get(0).getAttribute("height"));
            for (Element img : imgs) {
                assertTrue("alt text names the route and the alliance: " + img.getAttribute("alt"),
                        img.getAttribute("alt").contains(r.title()) && img.getAttribute("alt").contains("alliance"));
            }
            // each image is also a link to the full-size SVG
            assertEquals(r.id() + "_red.svg", all(pair.get(0), "a").get(0).getAttribute("href"));
        }
    }

    @Test
    public void theMetadataTableStatesTheNumbersOfTheRoute() throws Exception {
        Document page = parse(RouteGallery.indexHtml(RouteCatalog.all()));
        Element garden = section(page, "garden_cycle");
        assertEquals("B1", rowValue(garden, "Start tile"));
        assertEquals(String.format(java.util.Locale.ROOT, "(35.0, %.1f) in, heading -90 deg", BioBuzzConstants.Robot.FRONT_EXTENT_IN), rowValue(garden, "Start pose, RED"));
        assertEquals(String.format(java.util.Locale.ROOT, "(%.1f, %.1f) in, heading 90 deg", BioBuzzConstants.Field.SIZE_IN - 35.0,
                BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN), rowValue(garden, "Start pose, BLUE"));
        assertEquals("6", rowValue(garden, "Steps"));
        assertTrue(rowValue(garden, "Estimated time"), rowValue(garden, "Estimated time").contains("placeholder"));
        assertEquals("23 = LEAVE 3 + PARK 0 + our HIVE TIP 20", rowValue(garden, "Our AUTO points"));
        assertEquals("the camera stops the feeder only after the 4th is launched: 4 fired, 0 kept + 3 taken = 3 at the end",
                "hold 4 at the start, +3 taken, -4 fired, 3 at the end (balance -1)", rowValue(garden, "POLLEN"));
        assertEquals("1 (the first needs 3 POLLEN into the UP cell, each later one 8; 1 POLLEN to spare, so a field that needs 1 more still tips)",
                rowValue(garden, "Our HIVE TIPs"));
        assertEquals("the one WARN: the spare is probably spent", "1 WARN", rowValue(garden, "Validator"));
        Element c1 = section(page, "red_c1");
        assertEquals("3 ERROR, 1 WARN", rowValue(c1, "Validator"));
        assertTrue(rowValue(c1, "Ends"), rowValue(c1, "Ends").startsWith("LEAVE yes, PARK yes"));
    }

    // ---- who shoots the first tip, and the tip arithmetic ------------------------------------------------

    /** A partner route that waits at the rear wall for the partner's tip and then fires 4 POLLEN at the rear cell. */
    private static AutoRoute partnerRoute() {
        return AutoRoute.builder("partner_rear").title("Partner tips first").description("d").startTile("C6")
                .start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90).partnerTipsFirst().waitForTip(20).shootUp(61.5, 130.0, 90, 4).build();
    }

    private static List<AutoRoute> catalogAndPartner() {
        List<AutoRoute> routes = new ArrayList<AutoRoute>(RouteCatalog.all());
        routes.add(partnerRoute());
        return routes;
    }

    @Test
    public void everyRouteShowsItsFirstTipAssumptionInTheSummaryAndInItsOwnSection() throws Exception {
        List<AutoRoute> routes = catalogAndPartner();
        Document page = parse(RouteGallery.indexHtml(routes));
        List<Element> rows = withClass(page, "tr", "summary-row");
        assertEquals(routes.size(), rows.size());
        for (int i = 0; i < routes.size(); i++) {
            AutoRoute r = routes.get(i);
            assertEquals(r.id(), r.firstTipSummary(), rowValue(section(page, r.id()), "First tip"));
            assertTrue(r.id() + " summary row: " + rows.get(i).getTextContent(), rows.get(i).getTextContent().contains(r.firstTipSummary()));
        }
        assertEquals("we shoot the first tip", rowValue(section(page, "garden_cycle"), "First tip"));
        assertEquals("partner tips first, assumed at 12.0 s", rowValue(section(page, "partner_rear"), "First tip"));
    }

    @Test
    public void aPartnerRoutesPointsAreOursAndThePartnersTipIsShownSeparately() throws Exception {
        Document page = parse(RouteGallery.indexHtml(catalogAndPartner()));
        Element partner = section(page, "partner_rear");
        // the robot ends off the rear wall (LEAVE 3) and tips nothing itself: 4 POLLEN into the empty cell are half a tip
        assertEquals("3 = LEAVE 3 + PARK 0 + our HIVE TIP 0", rowValue(partner, "Our AUTO points"));
        assertEquals("+20 for the alliance at 12.0 s: an assumption, shown here and NOT counted in our points",
                rowValue(partner, "Assumed partner tip"));
        assertEquals("0", rowValue(partner, "Our HIVE TIPs"));
        // a route that shoots the first tip itself has no partner row at all
        assertFalse(section(page, "garden_cycle").getTextContent().contains("Assumed partner tip"));
        // and when our own volley tips first the partner row says the partner is not needed
        AutoRoute early = AutoRoute.builder("early").title("Early").description("d").startTile("C1").start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .partnerTipsFirst().shootUp(4).build();
        String voided = rowValue(section(parse(RouteGallery.indexHtml(Collections.singletonList(early))), "early"), "Assumed partner tip");
        assertTrue(voided, voided.startsWith("none:"));
    }

    @Test
    public void everyPartnerRouteShowsTheLatestPartnerTipThatStillWorksAndOtherRoutesDoNot() throws Exception {
        Document page = parse(RouteGallery.indexHtml(catalogAndPartner()));
        Element partner = section(page, "partner_rear");
        // gives up at 20 s, the tip is assumed at 12 s: 8 s of slack
        String slack = rowValue(partner, "Latest partner tip that works");
        // the wait may give up at 20 s, and the camera needs the settle time after the tip: 19.5 s, 7.5 s of slack
        assertTrue(slack, slack.startsWith("19.5 s is the latest partner tip the route can still use, 7.5 s of slack after the assumed 12.0 s."));
        assertTrue(slack, slack.contains("skips the volleys that need the tip"));
        // the summary table repeats the number next to the assumption
        boolean inSummary = false;
        for (Element row : withClass(page, "tr", "summary-row")) {
            if (row.getAttribute("data-route").equals("partner_rear")) {
                inSummary = row.getTextContent().contains("latest tip that works: 19.5 s");
            }
        }
        assertTrue("the summary row of the partner route carries the latest tip", inSummary);
        // routes that shoot the first tip themselves have no such row
        assertFalse(section(page, "garden_cycle").getTextContent().contains("Latest partner tip"));
        for (Element row : withClass(page, "tr", "summary-row")) {
            // (the two partner routes: this test's own and the one of the catalog)
            if (!row.getAttribute("data-route").equals("partner_rear") && !row.getAttribute("data-route").equals("partner_rear_shoot_park")) {
                assertFalse(row.getTextContent(), row.getTextContent().contains("latest tip that works"));
            }
        }
    }

    @Test
    public void everyPartnerRouteOfTheCatalogShowsItsSlackInTheGallery() throws Exception {
        Document page = parse(RouteGallery.indexHtml(RouteCatalog.all()));
        for (AutoRoute r : RouteCatalog.all()) {
            boolean hasRow = section(page, r.id()).getTextContent().contains("Latest partner tip that works");
            assertEquals(r.id(), r.firstTip() == org.firstinspires.ftc.teamcode.autoshoot.routes.FirstTip.PARTNER, hasRow);
        }
    }

    @Test
    public void theSlackTextSaysWhenARouteHasLittleOrNoSlackOrDoesNotDependOnThePartner() throws Exception {
        AutoRoute tight = AutoRoute.builder("tight").title("t").description("d").startTile("C6").start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90)
                .partnerTipsFirst().waitForTip(13).shootUp(61.5, 130.0, 90, 4).build();
        AutoRoute broken = AutoRoute.builder("broken").title("t").description("d").startTile("C6").start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90)
                .partnerTipsFirst().waitForTip(10).shootUp(61.5, 130.0, 90, 4).build();
        AutoRoute free = AutoRoute.builder("free").title("t").description("d").startTile("C6").start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90)
                .partnerTipsFirst().driveConstant(61.5, 120, 90).build();
        Document page = parse(RouteGallery.indexHtml(java.util.Arrays.asList(tight, broken, free)));
        String tightText = rowValue(section(page, "tight"), "Latest partner tip that works");
        assertTrue(tightText, tightText.contains("0.5 s of slack"));
        assertTrue(tightText, tightText.contains("less than 3.0 s"));
        String brokenText = rowValue(section(page, "broken"), "Latest partner tip that works");
        assertTrue(brokenText, brokenText.contains("2.5 s BEFORE the assumed 12.0 s"));
        assertTrue(brokenText, brokenText.contains("does not work with the assumed tip time"));
        String freeText = rowValue(section(page, "free"), "Latest partner tip that works");
        assertTrue(freeText, freeText.startsWith("any time"));
    }

    @Test
    public void aWaitForTipStepSaysThatTheCameraDecidesAndHowLateItMayGiveUp() throws Exception {
        Document page = parse(RouteGallery.indexHtml(catalogAndPartner()));
        List<Element> notes = withClass(section(page, "partner_rear"), "div", "waitnote");
        assertEquals(1, notes.size());
        String text = notes.get(0).getTextContent();
        assertTrue(text, text.contains("after the camera has SEEN the HIVE tipped, never at a time"));
        assertTrue(text, text.contains("stops waiting at 20.0 s on the route clock"));
        assertTrue(text, text.contains("may wait until"));
        // no other route has a WAIT_FOR_TIP, so none has such a note
        assertEquals(0, withClass(section(page, "garden_cycle"), "div", "waitnote").size());
    }

    @Test
    public void theTipArithmeticIsWorkedOutInPlainWordsPerVolley() throws Exception {
        Document page = parse(RouteGallery.indexHtml(catalogAndPartner()));
        // the rule once, at the top of the page: FIRST's calibration, not the team's old guess
        String rule = "";
        for (Element p : all(page, "p")) {
            if (p.getTextContent().contains("A HIVE cell tips when")) {
                rule = p.getTextContent();
            }
        }
        assertTrue("the rule is explained on the page: " + rule, rule.contains("A HIVE cell tips when its POLLEN plus 5/3 POLLEN for every NECTAR add up to 8 POLLEN-equivalents"));
        assertTrue(rule, rule.contains("the FIRST tip needs only 3 POLLEN"));
        assertTrue(rule, rule.contains("each LATER tip needs 8 POLLEN"));
        assertTrue(rule, rule.contains("our pre-loads are the first tip: 3 of the 4 tip it"));
        assertTrue(rule, rule.contains("the partner's launches count for it"));
        assertTrue(rule, rule.contains("SEEN the tipped cell steady for 0.5 s, never because a time has passed"));
        assertFalse("the old team figure is gone", rule.contains("5 NECTAR, or 8 POLLEN"));
        // our pre-loads are the first tip: 3 NECTAR + the 3rd POLLEN, the 4th is the spare
        List<Element> garden = withClass(section(page, "garden_cycle"), "li", "tipline");
        assertEquals(1, garden.size());
        assertEquals("Step 2: the UP cell holds 3 NECTAR and 0 POLLEN of ours; this volley fires up to 4: 3 NECTAR x 5/3 + 3 POLLEN = 8 POLLEN-equivalents,"
                + " the 8 that tip a cell, so the HIVE tips on POLLEN 3 of the 4 and the camera stops the volley, but only about 0.85 s after the tip, so the next 1 POLLEN"
                + " is probably launched into the cell while it swings down and is spent; none stays in the robot as a spare (the spare is for a field that needs"
                + " one more POLLEN, not a POLLEN to carry on).",
                garden.get(0).getTextContent());
        // the partner family: the partner's tip, then our volley into the empty cell that came up
        List<Element> partner = withClass(section(page, "partner_rear"), "li", "tipline");
        assertEquals(2, partner.size());
        String partnerTip = partner.get(0).getTextContent();
        assertTrue(partnerTip, partnerTip.contains("partner's assumed tip at 12.0 s"));
        assertTrue(partnerTip, partnerTip.contains("the REAR cell comes up with no NECTAR"));
        assertTrue(partnerTip, partnerTip.contains("our robot needs 8 POLLEN there for the next tip (plan 9: a real field drifts)"));
        assertTrue(partnerTip, partnerTip.contains("The robot sees it confirmed 0.5 s after the tip, at 12.5 s."));
        assertTrue(partner.get(1).getTextContent(), partner.get(1).getTextContent().contains("0 NECTAR and 0 POLLEN"));
        assertTrue(partner.get(1).getTextContent(), partner.get(1).getTextContent().contains("0 NECTAR x 5/3 + 4 POLLEN = 4 POLLEN-equivalents, short of the 8 that tip a cell, so no tip yet"));
        // a route without a volley says so
        assertTrue(withClass(section(page, "red_leave_park"), "li", "tipline").get(0).getTextContent().contains("launches no POLLEN"));
        // two POLLEN do not tip the first HIVE (3 NECTAR + 2 POLLEN = 7 POLLEN-equivalents, one short)
        AutoRoute two = AutoRoute.builder("two").title("Two").description("d").startTile("C1").start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).shootUp(2).build();
        String text = withClass(section(parse(RouteGallery.indexHtml(Collections.singletonList(two))), "two"), "li", "tipline").get(0)
                .getTextContent();
        assertTrue(text, text.contains("3 NECTAR x 5/3 + 2 POLLEN = 7 POLLEN-equivalents, short of the 8 that tip a cell, so no tip yet (1 more POLLEN"));
    }

    @Test
    public void theTimedVolleysOfTheOldOpModesAreWorkedOutAsOnesThatCannotStop() throws Exception {
        Document page = parse(RouteGallery.indexHtml(RouteCatalog.all()));
        String b1 = withClass(section(page, "red_b1"), "li", "tipline").get(0).getTextContent();
        assertTrue(b1, b1.contains("so the HIVE tips on POLLEN 3 of the 4; this volley is TIMED and cannot stop, so the last 1 goes into a cell that is already tipping down."));
        assertFalse(b1, b1.contains("stays in the robot"));
        // a camera-gated volley is better, but not instant: it stops only after the camera has seen the tip, so the spare is probably spent
        String wall = withClass(section(page, "wall_shoot_park"), "li", "tipline").get(0).getTextContent();
        assertTrue(wall, wall.contains("the camera stops the volley, but only about 0.85 s after the tip, so the next 1 POLLEN is probably launched into the cell while it swings down"));
        assertFalse(wall, wall.contains("stays in the robot as the spare"));
    }

    @Test
    public void theSpareOfEveryRouteIsSaidInWordsAndNoSpareIsSaidLoudly() throws Exception {
        AutoRoute exact = AutoRoute.builder("exact").title("Exact").description("d").startTile("C1")
                .start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).shootUp(3).build();
        AutoRoute none = AutoRoute.builder("none").title("None").description("d").startTile("C1")
                .start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).shootUp(2).build();
        Document page = parse(RouteGallery.indexHtml(java.util.Arrays.asList(exact, none)));
        String tight = rowValue(section(page, "exact"), "Our HIVE TIPs");
        assertTrue(tight, tight.contains("NO spare: exactly the minimum, a field that needs one more POLLEN would not tip"));
        // ...and the validator says it too, as a WARN of the planning-margin rule
        String findings = section(page, "exact").getTextContent();
        assertTrue(findings, findings.contains("plan a spare POLLEN for a HIVE that drifts"));
        assertTrue(findings, findings.contains("Plan 4 POLLEN"));
        // a route that does not tip has no spare to talk about
        assertEquals("0", rowValue(section(page, "none"), "Our HIVE TIPs"));
        // the real catalog routes that tip carry their spare
        Document catalog = parse(RouteGallery.indexHtml(RouteCatalog.all()));
        assertTrue(rowValue(section(catalog, "wall_shoot_park"), "Our HIVE TIPs").contains("1 POLLEN to spare"));
    }

    @Test
    public void theEndsRowSaysWhyLeaveIsLostWhenTheRobotParksTouchingTheWall() throws Exception {
        // QA21: a robot that parks in the LOADING ZONE touching the wall keeps its PARK but loses its LEAVE
        double rear = BioBuzzConstants.Robot.REAR_EXTENT_IN;
        AutoRoute touching = AutoRoute.builder("touching").title("T").description("d").startTile("A3").start(BioBuzzConstants.Robot.FRONT_EXTENT_IN, 70.2702, 180)
                .driveLinear(rear, 106.0, 0).park(rear, 106.0, 0).build();
        AutoRoute off = AutoRoute.builder("off").title("O").description("d").startTile("A3").start(BioBuzzConstants.Robot.FRONT_EXTENT_IN, 70.2702, 180)
                .driveLinear(rear + 2.0, 106.0, 0).park(rear + 2.0, 106.0, 0).build();
        Document page = parse(RouteGallery.indexHtml(java.util.Arrays.asList(touching, off)));
        String lost = rowValue(section(page, "touching"), "Ends");
        assertTrue(lost, lost.startsWith("LEAVE no (the robot ends 0.00 in from the red wall (x = 0)"));
        assertTrue(lost, lost.contains("touching any wall loses it, the one in the LOADING ZONE too: QA21"));
        assertTrue(lost, lost.contains("PARK yes"));
        assertEquals("5 = LEAVE 0 + PARK 5 + our HIVE TIP 0", rowValue(section(page, "touching"), "Our AUTO points"));
        // the same park 2 in off the wall keeps both, and says nothing about QA21
        String kept = rowValue(section(page, "off"), "Ends");
        assertTrue(kept, kept.startsWith("LEAVE yes, PARK yes"));
        assertFalse(kept, kept.contains("QA21"));
        assertEquals("8 = LEAVE 3 + PARK 5 + our HIVE TIP 0", rowValue(section(page, "off"), "Our AUTO points"));
        // the validator's finding is listed too
        assertTrue(section(page, "touching").getTextContent().contains("LEAVE needs the robot off every wall at the end"));
    }

    @Test
    public void aRouteThatDependsOnATipSaysWhatItDoesWhenTheTipIsNeverConfirmed() throws Exception {
        Document page = parse(RouteGallery.indexHtml(catalogAndPartner()));
        String text = rowValue(section(page, "partner_rear"), "If the tip is never confirmed");
        // the wait gives up at its 20 s deadline, the one volley is left out, the robot stays where it waited: no LEAVE, no PARK
        assertTrue(text, text.startsWith("the robot waits until 20.0 s, leaves 1 volley out (no camera, no shot) and carries on: it ends at "));
        assertTrue(text, text.contains("after 20.0 s with LEAVE no, PARK no: 0 points"));
        // routes that depend on no tip have no such row
        assertFalse(section(page, "garden_cycle").getTextContent().contains("If the tip is never confirmed"));
        assertFalse(section(page, "red_leave_park").getTextContent().contains("If the tip is never confirmed"));
        // a route that follows its volley with the safe drive off the wall keeps its LEAVE and says so
        AutoRoute safe = AutoRoute.builder("safe").title("Safe").description("d").startTile("C6")
                .start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90).partnerTipsFirst().waitForTip(20)
                .shootUp(61.5, 130.0, 90, 4).driveConstant(61.5, 112.0, 90).build();
        String kept = rowValue(section(parse(RouteGallery.indexHtml(Collections.singletonList(safe))), "safe"), "If the tip is never confirmed");
        assertTrue(kept, kept.contains("with LEAVE yes, PARK no: 3 points"));
    }

    @Test
    public void aVolleyThatWaitsForTheCameraToConfirmOurOwnTipSaysSo() throws Exception {
        // two volleys from the same pose: the second goes into the cell that came up with OUR tip and starts only after the camera has seen it steady
        // (the first fires exactly the 3 POLLEN that tip, so nothing of it is spent after the tip)
        AutoRoute two = AutoRoute.builder("two").title("Two").description("d").startTile("C1").start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .shootUp(58, 30, -90, 3).shootSide(org.firstinspires.ftc.teamcode.autoshoot.core.Side.REAR, 58, 30, -90, 2).build();
        Document page = parse(RouteGallery.indexHtml(Collections.singletonList(two)));
        List<Element> notes = withClass(section(page, "two"), "div", "confirmnote");
        assertEquals(1, notes.size());
        // the volley ends when the 3rd POLLEN has left; the tip lands 0.5 s after that and is confirmed 0.5 s after that: the robot waits 1.0 s
        assertTrue(notes.get(0).getTextContent(), notes.get(0).getTextContent().contains("stands here for 1.0 s until its camera has seen the cell that came up with its own tip steady"));
        assertTrue(notes.get(0).getTextContent(), notes.get(0).getTextContent().contains("never because a time has passed"));
        // a route with a single volley has no such wait
        assertEquals(0, withClass(section(parse(RouteGallery.indexHtml(RouteCatalog.all())), "garden_cycle"), "div", "confirmnote").size());
    }

    @Test
    public void everyStepIsListedWithItsNote() throws Exception {
        Document page = parse(RouteGallery.indexHtml(RouteCatalog.all()));
        for (AutoRoute r : RouteCatalog.all()) {
            List<Element> steps = withClass(section(page, r.id()), "li", "step");
            assertEquals(r.id(), r.size(), steps.size());
            for (int i = 0; i < steps.size(); i++) {
                assertEquals(r.step(i).kind().name(), steps.get(i).getAttribute("data-kind"));
                if (!r.step(i).note().isEmpty()) {
                    assertTrue(r.id() + " step " + (i + 1), steps.get(i).getTextContent().contains(r.step(i).note()));
                }
            }
        }
    }

    @Test
    public void findingsAreListedWithTheirSeverityAndCleanRoutesSayOk() throws Exception {
        Document page = parse(RouteGallery.indexHtml(RouteCatalog.all()));
        Element b1 = section(page, "red_b1");
        List<Element> issues = withClass(b1, "li", "issue");
        assertEquals("5 ERRORs and 2 WARNs (the shallow-PARK WARN went away when the CAD moved the LOADING ZONE; the timed volley's 4th POLLEN is a WARN)",
                7, issues.size());
        int errors = 0;
        for (Element i : issues) {
            errors += i.getAttribute("data-severity").equals("ERROR") ? 1 : 0;
            assertTrue(i.getTextContent(), i.getTextContent().contains("step "));
        }
        assertEquals(5, errors);
        assertTrue("one shared list when red and blue agree", b1.getTextContent().contains("Same for RED and BLUE."));
        // wall_shoot_park has no ERROR; its one finding is the WARN that the spare is probably spent
        Element demo = section(page, "wall_shoot_park");
        assertEquals(0, withClass(demo, "li", "ok").size());
        assertEquals(1, withClass(demo, "li", "issue").size());
        assertEquals("WARN", withClass(demo, "li", "issue").get(0).getAttribute("data-severity"));
        // a route with nothing to say says "ok"
        AutoRoute clean = AutoRoute.builder("clean").title("Clean").description("d").startTile("B1").start(30, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .driveConstant(30, 60, -90).build();
        Element ok = section(parse(RouteGallery.indexHtml(Collections.singletonList(clean))), "clean");
        assertEquals(1, withClass(ok, "li", "ok").size());
        assertEquals(0, withClass(ok, "li", "error").size());
    }

    @Test
    public void thePageIsSelfContainedAndFollowsTheReadersColourScheme() throws Exception {
        String html = RouteGallery.indexHtml(RouteCatalog.all());
        assertTrue(html.contains("@media (prefers-color-scheme: dark)"));
        assertTrue(html.contains("color-scheme: light dark"));
        assertTrue(html.contains("var(--bg)"));
        assertTrue("the page uses the same inks as the diagrams", html.contains("--red: " + BioBuzzConstants.Auto.RENDER_INK_RED)
                && html.contains("--blue: " + BioBuzzConstants.Auto.RENDER_INK_BLUE));
        assertFalse("no scripts", html.contains("<script"));
        assertFalse("nothing is fetched from the internet", html.contains("http://") || html.contains("https://") || html.contains("//cdn"));
        assertFalse("no @import", html.contains("@import"));
        Document page = parse(html);
        assertEquals(1, all(page, "style").size());
        assertEquals(0, all(page, "link").size());
        assertEquals("charset and viewport meta", 2, all(page, "meta").size());
        // the only images are the route diagrams, and they are siblings of the page, never absolute
        for (Element img : all(page, "img")) {
            assertTrue(img.getAttribute("src"), img.getAttribute("src").matches("[A-Za-z0-9_-]+_(red|blue)\\.svg"));
        }
    }

    @Test
    public void markupInATitleOrNoteCannotBreakThePage() throws Exception {
        AutoRoute odd = AutoRoute.builder("odd").title("A & B <fast> \"auto\"").description("<b>x</b> & y").startTile("C1")
                .start(58, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90).waitSeconds(1).note("1 < 2 & 3 > 2").build();
        Document page = parse(RouteGallery.indexHtml(Collections.singletonList(odd)));
        Element s = section(page, "odd");
        assertEquals("A & B <fast> \"auto\"", all(s, "h2").get(0).getTextContent());
        assertTrue(s.getTextContent().contains("<b>x</b> & y"));
        assertTrue(s.getTextContent().contains("1 < 2 & 3 > 2"));
        assertEquals(0, all(s, "b").size());
    }

    @Test
    public void aGalleryOfOneRouteAndOfNoneStillWorks() throws Exception {
        assertEquals(1, withClass(parse(RouteGallery.indexHtml(Collections.singletonList(simple("one", "One")))), "section", "route").size());
        Map<String, String> none = RouteGallery.build(Collections.<AutoRoute>emptyList());
        assertEquals(Collections.singleton("index.html"), none.keySet());
        assertEquals(0, withClass(parse(none.get("index.html")), "section", "route").size());
    }

    // ---- writing the files -------------------------------------------------------------------------

    private static String read(File f) throws IOException {
        InputStream in = new FileInputStream(f);
        try {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[8192];
            int n;
            while ((n = in.read(buffer)) > 0) {
                out.write(buffer, 0, n);
            }
            return new String(out.toByteArray(), "UTF-8");
        } finally {
            in.close();
        }
    }

    @Test
    public void writeCreatesTheDirectoryAndTheFilesMatchBuild() throws Exception {
        File dir = new File(temp.getRoot(), "nested/out");
        List<String> written = RouteGallery.write(dir, RouteCatalog.all());
        Map<String, String> expected = RouteGallery.build(RouteCatalog.all());
        assertEquals(new ArrayList<String>(expected.keySet()), written);
        for (Map.Entry<String, String> e : expected.entrySet()) {
            assertEquals(e.getKey(), e.getValue(), read(new File(dir, e.getKey())));
        }
        // writing again over the same folder is fine
        assertEquals(written, RouteGallery.write(dir, RouteCatalog.all()));
    }

    @Test
    public void theCommandLineToolWritesIntoTheGivenDirectoryAndSummarisesTheRoutes() throws Exception {
        File dir = temp.newFolder("cli");
        PrintStream old = System.out;
        ByteArrayOutputStream captured = new ByteArrayOutputStream();
        System.setOut(new PrintStream(captured, true, "UTF-8"));
        try {
            RenderRoutesMain.main(new String[] {dir.getPath()});
        } finally {
            System.setOut(old);
        }
        String out = captured.toString("UTF-8");
        assertTrue(out, out.contains("Wrote " + (1 + 2 * RouteCatalog.all().size()) + " files"));
        for (AutoRoute r : RouteCatalog.all()) {
            assertTrue(out, out.contains(r.id()));
            assertTrue(new File(dir, r.id() + "_red.svg").isFile());
            assertTrue(new File(dir, r.id() + "_blue.svg").isFile());
        }
        assertTrue(out, out.contains("5 ERROR, 2 WARN"));
        assertTrue(out, out.contains("1 WARN"));
        // every route line says who is assumed to shoot the first tip
        assertTrue(out, out.contains("1st tip"));
        for (String line : out.split("\\r?\\n")) {
            for (AutoRoute r : RouteCatalog.all()) {
                if (line.startsWith(r.id() + " ")) {
                    assertTrue(line, line.contains(r.firstTip().name()));
                }
            }
        }
        assertTrue(new File(dir, "index.html").isFile());
        assertFalse("nothing stale to report in a fresh folder", out.contains("Note:"));
    }

    @Test
    public void theCommandLineToolReportsOldDiagramsOfRemovedRoutesInsteadOfDeletingThem() throws Exception {
        File dir = temp.newFolder("stale");
        File old = new File(dir, "removed_route_red.svg");
        assertTrue(old.createNewFile());
        PrintStream before = System.out;
        ByteArrayOutputStream captured = new ByteArrayOutputStream();
        System.setOut(new PrintStream(captured, true, "UTF-8"));
        try {
            RenderRoutesMain.main(new String[] {dir.getPath()});
        } finally {
            System.setOut(before);
        }
        assertTrue(captured.toString("UTF-8"), captured.toString("UTF-8").contains("removed_route_red.svg belongs to no route"));
        assertTrue("left alone", old.isFile());
    }

    // ---- FLOWER pick-ups ---------------------------------------------------------------------------------

    private static AutoRoute flowerRoute() {
        return AutoRoute.builder("flower_pickup").title("FLOWER pick-up").startTile("B1").start(35, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90)
                .shootUp(4).intakeFlower(2.634, 47.281, 3).build();
    }

    @Test
    public void aFlowerPickUpIsDescribedWithItsTimeItsWheelAndWhyThatWheel() throws Exception {
        // a robot whose wheels reach far enough for the wiggle to work
        List<AutoRoute> routes = Collections.singletonList(flowerRoute());
        String html = RouteGallery.indexHtml(routes, WiggleTestRobot.timing());
        java.util.regex.Matcher m = java.util.regex.Pattern.compile("FLOWER pick-up: 3 POLLEN in about (\\d+\\.\\d) s with the (LEFT|RIGHT) wheel").matcher(html);
        assertTrue(html, m.find());
        assertTrue("and why that wheel", html.contains("chosen automatically: "));
        assertTrue("and what the wiggle does", html.contains("deg toward the POLLEN and back, 3 times"));
        assertFalse("a pick-up that works has no ERROR", html.contains("THE WHEEL CANNOT REACH"));
        assertTrue(html.contains("status-ok"));
        // the diagrams of the same robot show the wiggle with its label
        Map<String, String> files = RouteGallery.build(routes, WiggleTestRobot.timing());
        assertTrue(files.get("flower_pickup_red.svg").contains("wiggle x3"));
        assertTrue(files.get("flower_pickup_blue.svg").contains("wiggle x3"));
        assertTrue(files.get("flower_pickup_red.svg").contains("class=\"wiggle\""));
    }

    @Test
    public void withTheShippedWheelsThePageSaysTheyFallShortAndByHowMuch() throws Exception {
        // the numbers of the drawing: the wheel falls 0.77 in short of the ball once the chassis keeps clear of the plate through the swing
        String html = RouteGallery.indexHtml(Collections.singletonList(flowerRoute()));
        assertTrue(html, html.contains("THE WHEEL CANNOT REACH"));
        assertTrue(html, html.contains("0.8 in more than they do"));
        assertTrue(html, html.contains("status-error"));
        assertTrue("the text of the step is still there, so the page is not empty", html.contains("FLOWER pick-up: 3 POLLEN in about "));
        // and the default build is the same as asking for the default timing
        assertEquals(html, RouteGallery.indexHtml(Collections.singletonList(flowerRoute()), new org.firstinspires.ftc.teamcode.autoshoot.routes.TimingModel()));
    }

    // ---- the committed copy ------------------------------------------------------------------------

    /** The folder with the committed gallery, whichever folder the tests were started from (Gradle starts in TeamCode, the second path covers the repository root). */
    private static File committedGalleryFolder() {
        for (String candidate : new String[] {"../docs/autos", "docs/autos"}) {
            if (new File(candidate, "index.html").isFile()) {
                return new File(candidate);
            }
        }
        throw new AssertionError("docs/autos/index.html was not found from " + new File(".").getAbsolutePath()
                + ": the committed gallery is part of the repository, run tools/render-autos.sh (or .bat) and commit the result");
    }

    /** A text file with the line ends turned into plain line feeds: git may check the committed files out with CRLF on Windows, and that is not a difference. */
    private static String lineFeeds(String text) {
        return text.replace("\r\n", "\n");
    }

    @Test
    public void theCommittedGalleryCoversEveryRouteOfTheCatalog() throws Exception {
        File dir = committedGalleryFolder();
        String index = read(new File(dir, "index.html"));
        for (AutoRoute r : RouteCatalog.all()) {
            for (String name : new String[] {r.id() + "_red.svg", r.id() + "_blue.svg"}) {
                assertTrue("docs/autos/" + name + " is missing: run tools/render-autos.sh (or .bat) and commit the result",
                        new File(dir, name).isFile());
                assertTrue("docs/autos/index.html does not show " + name + ": run tools/render-autos.sh (or .bat) and commit the result",
                        index.contains("src=\"" + name + "\""));
            }
        }
    }

    /**
     * Situation: the page and the diagrams that the CURRENT code draws for the whole catalog, compared one file at a time with the committed copy in
     *     docs/autos.
     * Expected: every committed file is exactly what is drawn now (line ends aside), and the committed folder holds no route file the catalog does not have.
     * Why: the team reads docs/autos/index.html, not the code. The committed gallery was once three commits behind the code and said the opposite about what
     *     scores 20 points (it said 8 POLLEN are the first tip while the code needs 3), and the old check only looked that the files existed, so nothing failed.
     *     This test fails, with the first line that differs, until the page is regenerated: run tools/render-autos.sh (or .bat) and commit docs/autos in the
     *     SAME change as the rule, constant, route or text that moved it.
     * Negative twin: the same comparison against a copy with one word changed reports it, so a green result means the files really are equal.
     */
    @Test
    public void theCommittedGalleryIsExactlyWhatTheCurrentCodeDraws() throws Exception {
        File dir = committedGalleryFolder();
        Map<String, String> fresh = RouteGallery.build(RouteCatalog.all());
        List<String> problems = new ArrayList<String>();
        for (Map.Entry<String, String> e : fresh.entrySet()) {
            File committed = new File(dir, e.getKey());
            if (!committed.isFile()) {
                problems.add(e.getKey() + " is missing");
                continue;
            }
            String why = firstDifference(lineFeeds(read(committed)), lineFeeds(e.getValue()));
            if (why != null) {
                problems.add(e.getKey() + " is out of date: " + why);
            }
        }
        File[] files = dir.listFiles();
        for (File f : files == null ? new File[0] : files) {
            if ((f.getName().endsWith("_red.svg") || f.getName().endsWith("_blue.svg")) && !fresh.containsKey(f.getName())) {
                problems.add(f.getName() + " belongs to no route of the catalog any more (delete it)");
            }
        }
        assertTrue("docs/autos is not what the code draws now; run tools/render-autos.sh (or .bat) and commit docs/autos:\n  " + join(problems),
                problems.isEmpty());
    }

    @Test
    public void theGalleryComparisonSeesAChangedWordAndIgnoresLineEnds() {
        String page = RouteGallery.indexHtml(RouteCatalog.all());
        assertNull("the same text is not different", firstDifference(page, page));
        assertNull("CRLF is not a difference", firstDifference(lineFeeds(page.replace("\n", "\r\n")), page));
        String old = page.replace("HIVE TIP", "HIVE tip");
        String why = firstDifference(old, page);
        assertNotNull("a changed word is reported", why);
        assertTrue(why, why.contains("line "));
    }

    /** Null if the two texts are equal, else a sentence naming the first line that differs, with both versions of it (cut to 160 characters). */
    private static String firstDifference(String committed, String fresh) {
        if (committed.equals(fresh)) {
            return null;
        }
        String[] a = committed.split("\n", -1);
        String[] b = fresh.split("\n", -1);
        for (int i = 0; i < Math.max(a.length, b.length); i++) {
            String x = i < a.length ? a[i] : "(no such line)";
            String y = i < b.length ? b[i] : "(no such line)";
            if (!x.equals(y)) {
                return "first difference at line " + (i + 1) + ": committed [" + cut(x) + "] but now [" + cut(y) + "]";
            }
        }
        return "the texts differ";
    }

    private static String cut(String line) {
        return line.length() <= 160 ? line : line.substring(0, 160) + "...";
    }

    private static String join(List<String> lines) {
        StringBuilder sb = new StringBuilder();
        for (String l : lines) {
            sb.append(l).append("\n  ");
        }
        return sb.toString();
    }
}
