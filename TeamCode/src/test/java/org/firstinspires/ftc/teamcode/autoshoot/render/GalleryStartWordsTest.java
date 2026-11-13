package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Map;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StartPlacement;
import org.junit.Test;

/**
 * The gallery must tell the drive team WHERE TO PUT THE ROBOT, in words a student can follow with a tape measure to the robot's edges, and must draw where a
 * start is legal (rule G304). The first gallery printed only "Start pose (58.0, 9.8) in", the robot's center of rotation, a point inside the robot that nobody can
 * measure to, and drew no tinted stretches of wall.
 */
public class GalleryStartWordsTest {
    /**
     * Situation: every route of the catalog on the gallery page.
     * Expected: for both alliances the meta table has a "Place the robot" row with exactly the words of {@code StartPlacement.placementText} for that alliance's
     *     frame (for RED "from the red wall", for BLUE "from the blue wall").
     */
    @Test
    public void everyRouteSaysWhereToPutTheRobotForBothAlliances() {
        String html = RouteGallery.indexHtml(RouteCatalog.all());
        for (AutoRoute red : RouteCatalog.all()) {
            AutoRoute blue = red.forAlliance(Alliance.BLUE);
            String redWords = StartPlacement.placementText(red);
            String blueWords = StartPlacement.placementText(blue);
            assertTrue(red.id() + " RED", html.contains("<tr><th>Place the robot, RED</th><td>" + SvgWriter.esc(redWords) + "</td></tr>"));
            assertTrue(red.id() + " BLUE", html.contains("<tr><th>Place the robot, BLUE</th><td>" + SvgWriter.esc(blueWords) + "</td></tr>"));
            assertTrue(blueWords, blueWords.contains("blue wall") || blueWords.contains("audience wall") || blueWords.contains("rear wall"));
        }
    }

    /**
     * Situation: the words for a robot that stands on the same wall for both alliances (the start of {@code garden_cycle}: front on the audience wall for RED,
     *     front on the rear wall for BLUE).
     * Expected: the BLUE words name the blue frame: the distance is measured from the blue wall, and the wall the robot touches is the rear wall.
     */
    @Test
    public void theBlueWordsAreWorkedOutInTheBlueFrame() {
        AutoRoute garden = RouteCatalog.byId("garden_cycle");
        assertTrue(StartPlacement.placementText(garden), StartPlacement.placementText(garden).contains("from the red wall"));
        String blue = StartPlacement.placementText(garden.forAlliance(Alliance.BLUE));
        assertTrue(blue, blue.contains("from the blue wall"));
        assertTrue(blue, blue.contains("rear wall"));
    }

    /**
     * Situation: the SVG files of the gallery.
     * Expected: every one draws the legal start stretches ({@code g#start-bands}); the constant that asks for it is on.
     */
    @Test
    public void everyGalleryDiagramDrawsTheLegalStartStretches() {
        assertTrue(BioBuzzConstants.Auto.RENDER_GALLERY_SHOWS_START_BANDS);
        Map<String, String> files = RouteGallery.build(RouteCatalog.all());
        int svgs = 0;
        for (Map.Entry<String, String> f : files.entrySet()) {
            if (f.getKey().endsWith(".svg")) {
                svgs++;
                assertTrue(f.getKey(), f.getValue().contains("id=\"start-bands\""));
                assertTrue(f.getKey(), f.getValue().contains("class=\"start-band\""));
            }
        }
        assertEquals(2 * RouteCatalog.all().size(), svgs);
    }
}
