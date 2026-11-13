package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

import javax.xml.parsers.DocumentBuilderFactory;

import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

/**
 * The words on a BLUE diagram and on the gallery page must describe the BLUE route. The BLUE route is the RED one turned half a turn about the field center, so
 * the tile it starts in is another tile (B1 for RED is E6 for BLUE) and every word that names a wall or a side turns with it. The first gallery copied the RED
 * tile name and the RED words onto the BLUE pictures: a student who read "start tile B1" for the BLUE route would have put the robot on the wrong half of the field.
 */
public class BlueAllianceWordsTest {
    private static Document parse(String xml) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        return f.newDocumentBuilder().parse(new ByteArrayInputStream(xml.getBytes(Charset.forName("UTF-8"))));
    }

    private static List<Element> all(Document d, String tag) {
        List<Element> out = new ArrayList<Element>();
        NodeList nl = d.getElementsByTagName(tag);
        for (int i = 0; i < nl.getLength(); i++) {
            out.add((Element) nl.item(i));
        }
        return out;
    }

    private static Element byId(Document d, String id) {
        for (Element e : all(d, "*")) {
            if (id.equals(e.getAttribute("id"))) {
                return e;
            }
        }
        throw new AssertionError("no element with id " + id);
    }

    private static String tileOf(AutoRoute r) {
        return TileGrid.nameAt(r.startPose().x, r.startPose().y);
    }

    /**
     * Situation: every route of the catalog, in both frames.
     * Expected: the tile name of the route is the tile its own start pose is in (RED B1 is BLUE E6, C1 is D6, A3 is F4, B6 is E1).
     */
    @Test
    public void everyRoutesStartTileIsTheTileOfItsStartPoseInBothFrames() {
        for (AutoRoute red : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                AutoRoute r = red.forAlliance(a);
                assertEquals(red.id() + " " + a, tileOf(r), r.startTile());
            }
        }
        assertEquals("E6", RouteCatalog.byId("red_b1").forAlliance(Alliance.BLUE).startTile());
        assertEquals("D6", RouteCatalog.byId("red_c1").forAlliance(Alliance.BLUE).startTile());
        assertEquals("F4", RouteCatalog.byId("red_leave_park").forAlliance(Alliance.BLUE).startTile());
        assertEquals("E1", RouteCatalog.byId("partner_rear_shoot_park").forAlliance(Alliance.BLUE).startTile());
    }

    /**
     * Situation: the BLUE diagram of a route that starts in tile B1 for RED.
     * Expected: its subtitle, its START label and its screen-reader description name tile E6, and nowhere say B1.
     */
    @Test
    public void theBlueDiagramNamesTheBlueTile() throws Exception {
        Document d = parse(RouteSvgRenderer.render(RouteCatalog.byId("garden_cycle"), Alliance.BLUE));
        String subtitle = byId(d, "subtitle").getTextContent();
        assertTrue(subtitle, subtitle.contains("start tile E6"));
        assertFalse(subtitle, subtitle.contains("B1"));
        String desc = byId(d, "svg-desc").getTextContent();
        assertTrue(desc, desc.contains("starting in tile E6"));
        assertFalse(desc, desc.contains("B1"));
        boolean start = false;
        for (Element t : all(d, "text")) {
            if ("step-label".equals(t.getAttribute("class")) && t.getTextContent().startsWith("START ")) {
                assertEquals("START E6", t.getTextContent());
                start = true;
            }
        }
        assertTrue("a START label", start);
        // and the RED diagram still says B1
        assertTrue(byId(parse(RouteSvgRenderer.render(RouteCatalog.byId("garden_cycle"), Alliance.RED)), "subtitle").getTextContent().contains("start tile B1"));
    }

    /**
     * Situation: the titles of the catalog, which both diagrams of a route print.
     * Expected: none names a tile, "Red", a left or right of the field, or a wall that is another wall for BLUE: a title that is true for RED only would be wrong
     *     on the BLUE picture.
     */
    @Test
    public void noTitleUsesWordsThatAreOnlyTrueForRed() {
        Pattern tile = Pattern.compile("\\b[A-F][1-6]\\b");
        for (AutoRoute r : RouteCatalog.all()) {
            // RedAlliance_B1 is the name of an OpMode, not a color word and not a tile
            String t = r.title().replace("RedAlliance_B1", "OpMode").replace("RedAlliance_C1", "OpMode").replace("RedAlliance_", "OpMode_");
            assertFalse(r.id() + ": " + t, tile.matcher(t).find());
            for (String word : new String[] {"Red ", "red ", "left", "right", "rear", "audience", "Audience"}) {
                assertFalse(r.id() + " says '" + word + "': " + t, t.contains(word));
            }
        }
    }

    /**
     * Situation: the notes of a step, which are written in RED words.
     * Expected: the RED diagram's tooltip carries the note, the BLUE diagram's does not (it would describe the other half of the field).
     */
    @Test
    public void theBlueTooltipsLeaveOutTheRedWordsOfTheNotes() {
        AutoRoute route = RouteCatalog.byId("wall_shoot_park");
        String note = route.step(0).note();
        assertFalse(note.isEmpty());
        assertTrue(RouteSvgRenderer.render(route, Alliance.RED).contains(note));
        assertFalse(RouteSvgRenderer.render(route, Alliance.BLUE).contains(note));
    }

    /**
     * Situation: the gallery page.
     * Expected: every route says its tile for both alliances, and carries the key that says how to read its words for BLUE (with its own tile names); the header
     *     no longer says that a blue robot starts at the top.
     */
    @Test
    public void theGalleryNamesBothTilesAndSaysHowToReadTheRedWordsForBlue() {
        String html = RouteGallery.indexHtml(RouteCatalog.all());
        assertFalse("a blue robot does not always start at the top", html.contains("so a blue robot starts at the top"));
        assertTrue(html.contains("a RED start on the audience wall (the bottom of the picture) becomes a BLUE start on the rear wall"));
        for (AutoRoute red : RouteCatalog.all()) {
            AutoRoute blue = red.forAlliance(Alliance.BLUE);
            assertTrue(red.id(), html.contains("<td>RED " + tileOf(red) + ", BLUE " + tileOf(blue) + "</td>"));
            assertTrue(red.id(), html.contains("start tile " + tileOf(red) + " is " + tileOf(blue) + " for BLUE"));
        }
        assertEquals("one key per route", RouteCatalog.all().size(), html.split("class=\"readingkey", -1).length - 1);
    }
}
