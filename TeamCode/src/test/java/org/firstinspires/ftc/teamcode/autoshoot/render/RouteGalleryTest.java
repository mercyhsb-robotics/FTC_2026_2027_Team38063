package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
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
import org.junit.Assume;
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
        return AutoRoute.builder(id).title(title).description("d").startTile("C1").start(58, 9.2, -90).waitSeconds(1).build();
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
        assertEquals("(35.0, 9.2) in, heading -90 deg", rowValue(garden, "Start pose, RED"));
        assertEquals("(109.0, 134.8) in, heading 90 deg", rowValue(garden, "Start pose, BLUE"));
        assertEquals("6", rowValue(garden, "Steps"));
        assertTrue(rowValue(garden, "Estimated time"), rowValue(garden, "Estimated time").contains("placeholder"));
        assertEquals("23 = LEAVE 3 + PARK 0 + HIVE TIP 20", rowValue(garden, "Claimable AUTO points"));
        assertEquals("hold 4 at the start, +4 taken, -8 fired, 0 at the end (balance -4)", rowValue(garden, "POLLEN"));
        assertEquals("1 (8 POLLEN into the UP cell each)", rowValue(garden, "HIVE TIPs"));
        assertEquals("no findings", rowValue(garden, "Validator"));
        Element c1 = section(page, "red_c1");
        assertEquals("1 ERROR", rowValue(c1, "Validator"));
        assertTrue(rowValue(c1, "Ends"), rowValue(c1, "Ends").startsWith("LEAVE yes, PARK yes"));
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
        assertEquals(6, issues.size());
        int errors = 0;
        for (Element i : issues) {
            errors += i.getAttribute("data-severity").equals("ERROR") ? 1 : 0;
            assertTrue(i.getTextContent(), i.getTextContent().contains("step "));
        }
        assertEquals(4, errors);
        assertTrue("one shared list when red and blue agree", b1.getTextContent().contains("Same for RED and BLUE."));
        Element clean = section(page, "wall_shoot_park");
        List<Element> ok = withClass(clean, "li", "ok");
        assertEquals(1, ok.size());
        assertEquals(0, withClass(clean, "li", "error").size());
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
                .start(58, 9.2, -90).waitSeconds(1).note("1 < 2 & 3 > 2").build();
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
        assertTrue(out, out.contains("Wrote 11 files"));
        for (AutoRoute r : RouteCatalog.all()) {
            assertTrue(out, out.contains(r.id()));
            assertTrue(new File(dir, r.id() + "_red.svg").isFile());
            assertTrue(new File(dir, r.id() + "_blue.svg").isFile());
        }
        assertTrue(out, out.contains("4 ERROR, 2 WARN"));
        assertTrue(out, out.contains("no findings"));
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

    // ---- the committed copy ------------------------------------------------------------------------

    @Test
    public void theCommittedGalleryCoversEveryRouteOfTheCatalog() throws Exception {
        File dir = null;
        for (String candidate : new String[] {"../docs/autos", "docs/autos"}) {
            if (new File(candidate, "index.html").isFile()) {
                dir = new File(candidate);
            }
        }
        Assume.assumeNotNull(dir);
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
}
