package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;

import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.PartnerTipRoutes;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

/**
 * What a student reads off a diagram and off the page: the cell that a partner route shoots into must not look like a DOWN cell with nothing said (the picture is the
 * state at the START of the match), and the page must show a diagram at a size where its small labels can be read.
 */
public class DiagramReadabilityTest {
    private static Document parse(String xml) throws Exception {
        DocumentBuilderFactory f = DocumentBuilderFactory.newInstance();
        f.setNamespaceAware(true);
        return f.newDocumentBuilder().parse(new ByteArrayInputStream(xml.getBytes(Charset.forName("UTF-8"))));
    }

    /** Every {@code text.cell-cue} with the cell (alliance, side) it sits in. */
    private static List<String> cues(String svg) throws Exception {
        List<String> out = new ArrayList<String>();
        NodeList groups = parse(svg).getElementsByTagName("g");
        for (int i = 0; i < groups.getLength(); i++) {
            Element g = (Element) groups.item(i);
            if (!"cell cell-down".equals(g.getAttribute("class"))) {
                continue;
            }
            NodeList texts = g.getElementsByTagName("text");
            for (int k = 0; k < texts.getLength(); k++) {
                Element t = (Element) texts.item(k);
                if ("cell-cue".equals(t.getAttribute("class"))) {
                    out.add(g.getAttribute("data-alliance") + " " + g.getAttribute("data-side") + ": " + t.getTextContent());
                }
            }
        }
        return out;
    }

    /**
     * Situation: the partner route, whose volley goes into the cell that is DOWN when the match starts and comes up with the partner's tip.
     * Expected: that cell (the REAR cell of the red HIVE, the AUDIENCE cell of the blue one) says "UP after the tip" under its DOWN label, and no other cell does.
     */
    @Test
    public void theCellThatComesUpWithThePartnersTipSaysSo() throws Exception {
        AutoRoute partner = RouteCatalog.byId(PartnerTipRoutes.PARTNER_REAR_SHOOT_PARK);
        assertEquals("[RED REAR: UP after the tip]", cues(RouteSvgRenderer.render(partner, Alliance.RED)).toString());
        assertEquals("[BLUE AUDIENCE: UP after the tip]", cues(RouteSvgRenderer.render(partner, Alliance.BLUE)).toString());
    }

    /** Negative twin: a route that only shoots at the cell that is UP from the start has nothing to explain. */
    @Test
    public void aRouteThatShootsAtTheCellThatIsUpFromTheStartHasNoCue() throws Exception {
        for (Alliance a : Alliance.values()) {
            assertTrue(cues(RouteSvgRenderer.render(RouteCatalog.byId("wall_shoot_park"), a)).isEmpty());
            assertTrue(cues(RouteSvgRenderer.render(RouteCatalog.byId("garden_cycle"), a)).isEmpty());
        }
    }

    /**
     * Situation: the gallery page.
     * Expected: the two diagrams of a route are one under the other, not two to a row: on a laptop two to a row shrank the 9 px labels of the 900 px picture to
     *     about 6 px. The click-through to the full size file stays.
     */
    @Test
    public void thePageShowsOneDiagramPerRow() {
        String html = RouteGallery.indexHtml(RouteCatalog.all());
        assertTrue(html.contains(".pair { display: grid; grid-template-columns: minmax(0, 1fr);"));
        assertFalse(html.contains("repeat(2, minmax(0, 1fr))"));
        assertTrue(html.contains("<a href=\"red_b1_red.svg\"><img class=\"diagram\""));
    }
}
