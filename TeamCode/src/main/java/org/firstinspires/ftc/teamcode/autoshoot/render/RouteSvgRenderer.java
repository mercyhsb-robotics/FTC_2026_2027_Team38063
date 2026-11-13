package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.firstinspires.ftc.teamcode.autoshoot.render.SvgWriter.n;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AllianceField;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.DrivePath;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.routes.FlowerPress;
import org.firstinspires.ftc.teamcode.autoshoot.routes.IntakeSource;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteValidator;
import org.firstinspires.ftc.teamcode.autoshoot.routes.ShotGeometry;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Step;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StepKind;

/**
 * Draws an {@link AutoRoute} as a standalone SVG: a top view of the whole 144 x 144 in field with the
 * AUDIENCE at the BOTTOM and the red alliance on the LEFT (the same picture as the manual's figures
 * and the Pedro frame), the route in the alliance's ink colour, a legend, the estimated time, POLLEN
 * balance and points, and the validator's findings.
 *
 * <p>Only things that exist are drawn: the field from {@link BioBuzzConstants}, the route from its
 * steps and the simulation in {@link RouteAnalysis}. Scale, size, layout numbers and the main colours come from
 * {@link BioBuzzConstants.Auto}; the small style details (stroke widths, muted greys) are private constants and
 * literals in this file. Pure Java (no libraries), so a PC can run it; the output is plain ASCII.
 *
 * <p>SVG structure (ids and classes are used by the tests and the gallery): {@code #field} (tiles, border, alliance
 * areas, {@code #opponent-label}), {@code #structures} (HIVE, FLOWERS, GARDENS, LOADING ZONES, {@code #audience-label}),
 * {@code #route[data-ink]} containing {@code #start-footprint}, one {@code g.drive-leg[data-step]} per DRIVE,
 * {@code g.approach} for the implicit moves, {@code g.intake}, {@code g.shoot-pose}, {@code g.park-outline} (or
 * {@code g.end-outline} when the route ends on a DRIVE), {@code #badges} with one {@code g.badge} per step,
 * {@code #step-labels} and {@code #issue-marks}; then {@code #legend} and {@code #footer}. The text of every step is
 * also in a {@code <title>} child, which browsers show as a tooltip.
 *
 * <p>Every robot footprint (start, INTAKE, SHOOT, PARK, end) is drawn in two tones: {@code polygon.outline} is the
 * whole perimeter, {@code polygon.chassis} the body and {@code circle.wheel} the two flat intake wheels (the one that
 * presses a FLOWER also has the class {@code pressing}); a footprint at a step with an ERROR gets the extra class
 * {@code error} on its outline and is drawn in amber. A FLOWER INTAKE also contains {@code g.press-in}: a short arrow
 * ({@code line.press-arrow}) along the pressing wheel's path into the FLOWER and a dotted one ({@code line.retreat-arrow})
 * for the retreat.
 */
public final class RouteSvgRenderer {
    private RouteSvgRenderer() {
    }

    /** The ink colour of routes of this alliance (red ink for RED, blue ink for BLUE). */
    public static String inkFor(Alliance alliance) {
        return alliance == Alliance.RED ? BioBuzzConstants.Auto.RENDER_INK_RED : BioBuzzConstants.Auto.RENDER_INK_BLUE;
    }

    /** Renders the route for the alliance (a RED-authored route is half-turned for BLUE). */
    public static String render(AutoRoute route, Alliance alliance) {
        AutoRoute own = route.forAlliance(alliance);
        return new Drawing(own, new RouteValidator().validate(own)).draw();
    }

    /** As {@link #render(AutoRoute, Alliance)}, reusing validator findings the caller already has. */
    public static String render(AutoRoute route, Alliance alliance, List<Issue> issues) {
        return new Drawing(route.forAlliance(alliance), issues).draw();
    }

    // =============================================================================================
    // STYLE: small details that are not worth a constant in BioBuzzConstants
    // =============================================================================================
    private static final String FONT = "Helvetica, Arial, sans-serif";
    private static final String BORDER_COLOR = "#333333";
    private static final String MUTED_TEXT = "#555555";
    private static final String FRAME_FILL = "#9a9a9a";
    private static final String FRAME_STROKE = "#555555";
    private static final String FLOWER_FILL = "#f3b562";
    private static final String FLOWER_STROKE = "#b26a00";
    private static final String POLLEN_FILL = "#f2c230";
    private static final String POLLEN_STROKE = "#a67c00";
    private static final String ERROR_COLOR = "#b71c1c";
    private static final String WARN_COLOR = "#f9a825";
    private static final String HIGHLIGHT = "#ffffff";
    /** The flat intake wheels: dark discs, so that they read as the second tone of the robot next to the body in ink. */
    private static final String WHEEL_FILL = "#37474f";
    private static final String CHASSIS_OPACITY = "0.2";
    private static final double BROKEN_FILL_OPACITY = 0.3;
    private static final double LEGEND_ROW_SPACING_PX = 17.0;
    private static final double FOOTER_GAP_PX = 22.0;
    private static final double ISSUE_LINE_SPACING_PX = 11.2;
    private static final int ISSUE_LINE_MAX_CHARS = 158;
    private static final double SAME_POSE_IN = 0.5;

    // =============================================================================================

    private static final class Drawing {
        private final AutoRoute route;
        private final RouteAnalysis a;
        private final List<Issue> issues;
        private final AllianceField f;
        private final Alliance alliance;
        private final String ink;
        private final double s = BioBuzzConstants.Auto.RENDER_FIELD_SCALE_PX_PER_IN;
        private final double canvas = BioBuzzConstants.Auto.RENDER_CANVAS_PX;
        private final double fieldPx;
        private final double x0;
        private final double yTop;
        private final double yBottom;
        private final double legendTop;
        private final double fontPx = BioBuzzConstants.Auto.RENDER_FONT_PX;
        private final double badgeR = BioBuzzConstants.Auto.RENDER_BADGE_RADIUS_PX;
        private final SvgWriter w = new SvgWriter();
        private final List<double[]> badges = new ArrayList<double[]>();     // {x, y}
        private final List<double[]> labelBoxes = new ArrayList<double[]>(); // {x0, y0, x1, y1}
        private final Set<String> labelledNominal = new HashSet<String>();

        Drawing(AutoRoute route, List<Issue> issues) {
            this.route = route;
            this.alliance = route.alliance();
            this.a = route.analyze();
            this.f = a.field();
            this.issues = issues;
            this.ink = inkFor(alliance);
            this.fieldPx = f.model().size() * s;
            this.x0 = (canvas - fieldPx) / 2.0;
            this.yTop = BioBuzzConstants.Auto.RENDER_FIELD_TOP_PX;
            this.yBottom = yTop + fieldPx;
            this.legendTop = yBottom + BioBuzzConstants.Auto.RENDER_LEGEND_GAP_PX;
        }

        // ---- coordinates ------------------------------------------------------------------------

        private double sx(double x) {
            return x0 + x * s;
        }

        private double sy(double y) {
            return yBottom - y * s;
        }

        private double len(double inches) {
            return inches * s;
        }

        private String pt(double x, double y) {
            return n(sx(x)) + "," + n(sy(y));
        }

        private String points(Footprint fp) {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < 4; i++) {
                if (i > 0) {
                    sb.append(' ');
                }
                sb.append(pt(fp.corner(i).x, fp.corner(i).y));
            }
            return sb.toString();
        }

        private String inkOf(Alliance x) {
            return inkFor(x);
        }

        private double textWidth(String text, double size) {
            return text.length() * size * BioBuzzConstants.Auto.RENDER_LABEL_CHAR_WIDTH_EM;
        }

        // ---- the whole drawing ------------------------------------------------------------------

        String draw() {
            header();
            w.empty("rect", "id", "background", "x", "0", "y", "0", "width", n(canvas), "height", n(canvas), "fill",
                    BioBuzzConstants.Auto.RENDER_BACKGROUND);
            title();
            floor();
            structures();
            route();
            legend();
            footer();
            w.close("svg");
            return w.toString();
        }

        private void header() {
            w.open("svg", "xmlns", "http://www.w3.org/2000/svg", "viewBox", "0 0 " + n(canvas) + " " + n(canvas),
                    "width", n(canvas), "height", n(canvas), "role", "img", "aria-labelledby", "svg-title svg-desc",
                    "font-family", FONT);
            w.element("title", route.title() + " - " + alliance + " alliance", "id", "svg-title");
            StringBuilder d = new StringBuilder();
            d.append("Top view of the 144 by 144 inch BIOBUZZ field, audience at the bottom, red alliance on the left. ");
            d.append("Route ").append(route.id()).append(" for the ").append(alliance).append(" alliance, starting in tile ")
                    .append(route.startTile()).append(", with ").append(route.size()).append(" steps: ");
            for (int i = 0; i < route.size(); i++) {
                d.append(i == 0 ? "" : "; ").append(i + 1).append(' ').append(route.step(i).describe());
            }
            d.append(". Estimated ").append(sec(a.totalSeconds())).append(", POLLEN balance ").append(signed(a.pollenBalance()))
                    .append(", up to ").append(a.points()).append(" points; ").append(countOf(Issue.Severity.ERROR))
                    .append(" errors and ").append(countOf(Issue.Severity.WARN)).append(" warnings.");
            w.element("desc", d.toString(), "id", "svg-desc");
            w.open("defs");
            gradient("area-red", BioBuzzConstants.Auto.RENDER_INK_RED, true);
            gradient("area-blue", BioBuzzConstants.Auto.RENDER_INK_BLUE, false);
            w.open("pattern", "id", "opponent-hatch", "width", "9", "height", "9", "patternUnits", "userSpaceOnUse",
                    "patternTransform", "rotate(45)");
            w.empty("line", "x1", "0", "y1", "0", "x2", "0", "y2", "9", "stroke", "#000000", "stroke-opacity", "0.07",
                    "stroke-width", "2.5");
            w.close("pattern");
            w.close("defs");
        }

        /** Fades from the wall (opaque) outward (transparent). */
        private void gradient(String id, String color, boolean towardLeft) {
            w.open("linearGradient", "id", id, "x1", towardLeft ? "1" : "0", "y1", "0", "x2", towardLeft ? "0" : "1", "y2", "0");
            w.empty("stop", "offset", "0", "stop-color", color, "stop-opacity", "0.30");
            w.empty("stop", "offset", "1", "stop-color", color, "stop-opacity", "0");
            w.close("linearGradient");
        }

        private int countOf(Issue.Severity severity) {
            int c = 0;
            for (Issue i : issues) {
                if (i.severity == severity) {
                    c++;
                }
            }
            return c;
        }

        private void title() {
            String text = route.title() + " - " + alliance + " alliance";
            double size = Math.min(20.0, 852.0 / (text.length() * 0.56));
            w.element("text", text, "id", "title", "x", "24", "y", "27", "font-size", n(size), "font-weight", "bold",
                    "fill", BioBuzzConstants.Auto.RENDER_TEXT_COLOR);
            String sub = "id " + route.id() + "  |  start tile " + route.startTile() + "  |  " + route.size() + " steps  |  "
                    + (alliance == Alliance.RED ? "red" : "blue") + " ink";
            w.element("text", sub, "id", "subtitle", "x", "24", "y", "45", "font-size", "12", "fill", MUTED_TEXT);
            // ink swatch so the colour is explained once
            w.empty("rect", "x", "24", "y", "52", "width", "40", "height", "4", "fill", ink);
        }

        // ---- floor, tiles, border ---------------------------------------------------------------

        private void floor() {
            w.open("g", "id", "field");
            double shown = BioBuzzConstants.Auto.RENDER_ALLIANCE_AREA_SHOWN_IN;
            double yMin = BioBuzzConstants.Auto.RENDER_ALLIANCE_AREA_Y_MIN_IN;
            double yMax = BioBuzzConstants.Auto.RENDER_ALLIANCE_AREA_Y_MAX_IN;
            double size = f.model().size();
            // faint ALLIANCE AREAs outside the walls (only the first part of their 54 in depth is shown)
            w.empty("rect", "class", "alliance-area", "data-alliance", "RED", "x", n(sx(-shown)), "y", n(sy(yMax)), "width",
                    n(len(shown)), "height", n(len(yMax - yMin)), "fill", "url(#area-red)");
            w.empty("rect", "class", "alliance-area", "data-alliance", "BLUE", "x", n(sx(size)), "y", n(sy(yMax)), "width",
                    n(len(shown)), "height", n(len(yMax - yMin)), "fill", "url(#area-blue)");
            areaLabel("RED ALLIANCE AREA", sx(-shown / 2.0), sy((yMin + yMax) / 2.0), BioBuzzConstants.Auto.RENDER_INK_RED);
            areaLabel("BLUE ALLIANCE AREA", sx(size + shown / 2.0), sy((yMin + yMax) / 2.0), BioBuzzConstants.Auto.RENDER_INK_BLUE);

            for (int col = 0; col < 6; col++) {
                for (int row = 0; row < 6; row++) {
                    boolean light = (col + row) % 2 == 0;
                    w.empty("rect", "class", "tile", "data-tile", "" + (char) ('A' + col) + (row + 1), "x", n(sx(24.0 * col)),
                            "y", n(sy(24.0 * (row + 1))), "width", n(len(24)), "height", n(len(24)), "fill",
                            light ? BioBuzzConstants.Auto.RENDER_TILE_LIGHT : BioBuzzConstants.Auto.RENDER_TILE_DARK);
                }
            }
            // the opponent half of the field, off limits to the robot center in AUTO (G402)
            Rect own = f.ownHalf();
            double ox = own.minX < 1.0 ? f.model().center() : 0.0;
            w.empty("rect", "id", "opponent-side", "x", n(sx(ox)), "y", n(sy(size)), "width", n(len(size / 2.0)), "height",
                    n(len(size)), "fill", "url(#opponent-hatch)");
            w.empty("line", "id", "side-line", "x1", n(sx(f.model().center())), "y1", n(sy(0)), "x2", n(sx(f.model().center())),
                    "y2", n(sy(size)), "stroke", MUTED_TEXT, "stroke-width", "1", "stroke-dasharray", "6 4");
            // The label reads upward beside the center line, on the opponent side, in the strip between the
            // HIVE keep-out and the rear wall, where no FLOWER, cell or legal route is.
            double labelX = sx(f.model().center()) + (alliance == Alliance.RED ? 12.0 : -3.0);
            double labelY = sy((f.hiveKeepOut().maxY + size) / 2.0);
            String opponent = "opponent side in AUTO (G402)";
            w.element("text", opponent, "id", "opponent-label", "x", n(labelX), "y", n(labelY), "font-size", "9",
                    "fill", MUTED_TEXT, "text-anchor", "middle", "transform", "rotate(-90 " + n(labelX) + " " + n(labelY) + ")");
            double half = textWidth(opponent, 9.0) / 2.0;
            labelBoxes.add(new double[] {labelX - 10.0, labelY - half, labelX + 3.0, labelY + half});   // route labels keep clear of it
            // tile labels: columns A-F along the bottom, rows 1-6 along the left
            for (int col = 0; col < 6; col++) {
                w.element("text", String.valueOf((char) ('A' + col)), "class", "tile-label", "x", n(sx(24.0 * col + 12)), "y",
                        n(yBottom + 16), "font-size", "13", "font-weight", "bold", "text-anchor", "middle", "fill", MUTED_TEXT);
            }
            for (int row = 0; row < 6; row++) {
                w.element("text", String.valueOf(row + 1), "class", "tile-label", "x", n(x0 - 7), "y", n(sy(24.0 * row + 12) + 4.5),
                        "font-size", "13", "font-weight", "bold", "text-anchor", "end", "fill", MUTED_TEXT);
            }
            w.empty("rect", "id", "field-border", "x", n(x0), "y", n(yTop), "width", n(fieldPx), "height", n(fieldPx), "fill",
                    "none", "stroke", BORDER_COLOR, "stroke-width", "3");
            w.close("g");
        }

        private void areaLabel(String text, double cx, double cy, String color) {
            w.element("text", text, "x", n(cx), "y", n(cy), "font-size", "9", "fill", color, "text-anchor", "middle",
                    "transform", "rotate(-90 " + n(cx) + " " + n(cy) + ")");
        }

        // ---- HIVE, FLOWERS, GARDENS, LOADING ZONES ----------------------------------------------

        private void structures() {
            w.open("g", "id", "structures");
            Rect frame = f.hiveFrame();
            w.empty("rect", "id", "hive-frame", "x", n(sx(frame.minX)), "y", n(sy(frame.maxY)), "width", n(len(frame.maxX - frame.minX)),
                    "height", n(len(frame.maxY - frame.minY)), "fill", FRAME_FILL, "fill-opacity", "0.30", "stroke", FRAME_STROKE,
                    "stroke-width", "2");
            Rect keep = f.hiveKeepOut();
            w.empty("rect", "id", "hive-keepout", "x", n(sx(keep.minX)), "y", n(sy(keep.maxY)), "width", n(len(keep.maxX - keep.minX)),
                    "height", n(len(keep.maxY - keep.minY)), "fill", "none", "stroke", FRAME_STROKE, "stroke-width", "1",
                    "stroke-dasharray", "2 3");
            w.element("text", "HIVE frame", "x", n(sx(f.model().center())), "y", n(sy(f.model().center()) + 4), "font-size", "10",
                    "fill", MUTED_TEXT, "text-anchor", "middle");
            for (Alliance h : Alliance.values()) {
                hive(h);
            }
            for (int i = 0; i < f.flowerPlates().size(); i++) {
                flower(i);
            }
            for (Alliance g : Alliance.values()) {
                garden(g);
                loadingZone(g);
            }
            w.element("text", "AUDIENCE", "id", "audience-label", "x", n(canvas / 2.0), "y", n(yBottom + 38), "font-size", "15",
                    "font-weight", "bold", "letter-spacing", "4", "text-anchor", "middle", "fill",
                    BioBuzzConstants.Auto.RENDER_TEXT_COLOR);
            double ax = canvas / 2.0;
            for (int side = -1; side <= 1; side += 2) {
                double cx = ax + side * 82;
                w.empty("polygon", "class", "compass-arrow", "points", n(cx - 6) + "," + n(yBottom + 28) + " " + n(cx + 6) + "," + n(yBottom + 28)
                        + " " + n(cx) + "," + n(yBottom + 40), "fill", BioBuzzConstants.Auto.RENDER_TEXT_COLOR);
            }
            w.element("text", "REAR WALL", "x", n(canvas - 24), "y", n(yTop - 6), "font-size", "9", "fill", MUTED_TEXT, "text-anchor", "end");
            w.close("g");
        }

        /** One HIVE: its DOWN cell (dashed) and its UP cell (filled) at match start, with tag ID ranges. */
        private void hive(Alliance h) {
            Side up = f.model().initialUpSide(h);
            String color = inkOf(h);
            w.open("g", "class", "hive", "data-alliance", h.name());
            for (int pass = 0; pass < 2; pass++) {
                boolean isUp = pass == 1;
                Side side = isUp ? up : up.opposite();
                Rect plan = f.cellPlan(h, side, isUp);
                CellSpec cell = f.model().cell(h, side);
                String tags = cell.baseTagId + "-" + (cell.baseTagId + cell.tagCount() - 1);
                w.open("g", "class", isUp ? "cell cell-up" : "cell cell-down", "data-alliance", h.name(), "data-side", side.name());
                if (isUp) {
                    w.empty("rect", "x", n(sx(plan.minX)), "y", n(sy(plan.maxY)), "width", n(len(plan.maxX - plan.minX)), "height",
                            n(len(plan.maxY - plan.minY)), "fill", color, "fill-opacity", "0.28", "stroke", color, "stroke-width", "2");
                    double lipY = side == Side.AUDIENCE ? plan.minY : plan.maxY;
                    w.empty("line", "class", "cell-opening", "x1", n(sx(plan.minX)), "y1", n(sy(lipY)), "x2", n(sx(plan.maxX)), "y2",
                            n(sy(lipY)), "stroke", color, "stroke-width", "5");
                    // the aim point at the center of the opening
                    w.empty("circle", "class", "aim-point", "cx", n(sx(cell.originX)), "cy", n(sy(cell.originY)), "r", "3.5", "fill",
                            "#ffffff", "stroke", color, "stroke-width", "1.5");
                } else {
                    w.empty("rect", "x", n(sx(plan.minX)), "y", n(sy(plan.maxY)), "width", n(len(plan.maxX - plan.minX)), "height",
                            n(len(plan.maxY - plan.minY)), "fill", "#ffffff", "fill-opacity", "0.35", "stroke", color, "stroke-width", "1.5",
                            "stroke-dasharray", "5 3");
                }
                double cy = (plan.minY + plan.maxY) / 2.0;
                double labelY = isUp ? (side == Side.AUDIENCE ? plan.minY + 7.0 : plan.maxY - 7.0) : cy;
                w.element("text", (isUp ? "UP " : "DOWN ") + tags, "x", n(sx((plan.minX + plan.maxX) / 2.0)), "y", n(sy(labelY) + 3.5),
                        "font-size", "9.5", "font-weight", "bold", "fill", color, "text-anchor", "middle");
                w.close("g");
            }
            w.close("g");
        }

        private void flower(int i) {
            Rect plate = f.flowerPlates().get(i);
            Point2 c = f.flowerCenter(i);
            w.open("g", "class", "flower", "data-flower", Integer.toString(i + 1));
            w.empty("rect", "x", n(sx(plate.minX)), "y", n(sy(plate.maxY)), "width", n(len(plate.maxX - plate.minX)), "height",
                    n(len(plate.maxY - plate.minY)), "fill", FLOWER_FILL, "stroke", FLOWER_STROKE, "stroke-width", "1.5");
            w.empty("circle", "cx", n(sx(c.x)), "cy", n(sy(c.y)), "r", n(len(1.4)), "fill", POLLEN_FILL, "stroke", POLLEN_STROKE,
                    "stroke-width", "1");
            // label on the field side of the plate
            double size = f.model().size();
            double lx = c.x;
            double ly = c.y;
            String anchor = "middle";
            double dx = 0;
            double dy = 0;
            if (c.x < 10) {
                lx = plate.maxX + 1.5;
                anchor = "start";
                dy = 3;
            } else if (c.x > size - 10) {
                lx = plate.minX - 1.5;
                anchor = "end";
                dy = 3;
            } else if (c.y < 10) {
                ly = plate.maxY + 3.0;
                dy = 8;
            } else {
                ly = plate.minY - 3.0;
                dy = -2;
            }
            w.element("text", "FLOWER " + (i + 1), "x", n(sx(lx) + dx), "y", n(sy(ly) + dy), "font-size", "9", "fill", FLOWER_STROKE,
                    "font-weight", "bold", "text-anchor", anchor);
            w.close("g");
        }

        private void garden(Alliance g) {
            Rect z = f.garden(g);
            String color = inkOf(g);
            w.open("g", "class", "garden", "data-alliance", g.name());
            w.empty("rect", "x", n(sx(z.minX)), "y", n(sy(z.maxY)), "width", n(len(z.maxX - z.minX)), "height", n(len(z.maxY - z.minY)),
                    "fill", color, "fill-opacity", "0.10", "stroke", color, "stroke-width", "1", "stroke-dasharray", "3 2");
            for (Point2 p : f.gardenPollen(g)) {
                w.empty("circle", "class", "pollen", "cx", n(sx(p.x)), "cy", n(sy(p.y)), "r", n(len(BioBuzzConstants.Auto.POLLEN_DIAMETER_IN / 2.0)),
                        "fill", POLLEN_FILL, "stroke", POLLEN_STROKE, "stroke-width", "1");
            }
            boolean red = g == Alliance.RED;
            w.element("text", "GARDEN", "x", n(sx((z.minX + z.maxX) / 2.0)), "y", red ? n(sy(z.maxY) - 5) : n(sy(z.minY) + 13), "font-size", "9",
                    "fill", color, "text-anchor", "middle", "font-weight", "bold");
            w.close("g");
        }

        private void loadingZone(Alliance g) {
            Rect z = f.loadingZone(g);
            String color = inkOf(g);
            w.open("g", "class", "loading-zone", "data-alliance", g.name());
            w.empty("rect", "x", n(sx(z.minX)), "y", n(sy(z.maxY)), "width", n(len(z.maxX - z.minX)), "height", n(len(z.maxY - z.minY)),
                    "fill", color, "fill-opacity", "0.16", "stroke", color, "stroke-width", "2");
            double cx = sx((z.minX + z.maxX) / 2.0);
            double cy = sy((z.minY + z.maxY) / 2.0);
            w.element("text", "LOADING ZONE", "x", n(cx), "y", n(cy + 3.5), "font-size", "9.5", "font-weight", "bold", "fill", color,
                    "text-anchor", "middle", "transform", "rotate(-90 " + n(cx) + " " + n(cy) + ")");
            w.close("g");
        }

        // ---- the route --------------------------------------------------------------------------

        private void route() {
            w.open("g", "id", "route", "data-ink", ink, "data-alliance", alliance.name(), "data-route", route.id());
            // pass 1: paths and implicit moves (under everything else)
            for (RouteAnalysis.StepInfo info : a.steps()) {
                if (info.path != null) {
                    driveLeg(info);
                } else if (info.approach != null) {
                    approach(info);
                }
            }
            // pass 2: robot footprints, rings and shot arrows
            for (RouteAnalysis.StepInfo info : a.steps()) {
                switch (info.step.kind()) {
                    case START:
                        startFootprint(info);
                        break;
                    case INTAKE:
                        intake(info);
                        break;
                    case SHOOT:
                        shoot(info);
                        break;
                    case PARK:
                        park(info);
                        break;
                    default:
                        break;
                }
            }
            RouteAnalysis.StepInfo last = a.steps().get(a.steps().size() - 1);
            if (last.step.kind() == StepKind.DRIVE) {
                endOutline(last);
            }
            // pass 3: numbered badges, labels and issue marks on top
            badgesAndLabels();
            w.close("g");
        }

        private String stepTitle(RouteAnalysis.StepInfo info) {
            return "step " + (info.index + 1) + ": " + info.step.describe() + String.format(Locale.ROOT, " (t = %.1f to %.1f s)",
                    info.startSec, info.endSec()) + (info.step.note().isEmpty() ? "" : " - " + info.step.note());
        }

        private void driveLeg(RouteAnalysis.StepInfo info) {
            DrivePath path = info.path;
            w.open("g", "class", "drive-leg", "data-step", Integer.toString(info.index + 1), "data-mode", ((Step.Drive) info.step).mode().name());
            w.element("title", stepTitle(info));
            w.empty("path", "class", "leg", "d", pathData(path), "fill", "none", "stroke", ink, "stroke-width", "3", "stroke-linecap", "round",
                    "stroke-linejoin", "round");
            double length = path.length();
            if (length > 1.0) {
                if (length > 24.0) {
                    arrowAt(path, length * 0.5);
                }
                arrowAt(path, length);
            }
            w.close("g");
        }

        private String pathData(DrivePath path) {
            List<Point2> c = path.controlPoints();
            StringBuilder d = new StringBuilder();
            d.append("M ").append(n(sx(c.get(0).x))).append(' ').append(n(sy(c.get(0).y)));
            if (path.degree() == 1) {
                d.append(" L ").append(n(sx(c.get(1).x))).append(' ').append(n(sy(c.get(1).y)));
            } else if (path.degree() == 2) {
                d.append(" Q ").append(n(sx(c.get(1).x))).append(' ').append(n(sy(c.get(1).y))).append(' ')
                        .append(n(sx(c.get(2).x))).append(' ').append(n(sy(c.get(2).y)));
            } else if (path.degree() == 3) {
                d.append(" C ").append(n(sx(c.get(1).x))).append(' ').append(n(sy(c.get(1).y))).append(' ')
                        .append(n(sx(c.get(2).x))).append(' ').append(n(sy(c.get(2).y))).append(' ')
                        .append(n(sx(c.get(3).x))).append(' ').append(n(sy(c.get(3).y)));
            } else {
                for (Pose2 p : path.sample(1.0)) {
                    d.append(" L ").append(n(sx(p.x))).append(' ').append(n(sy(p.y)));
                }
            }
            return d.toString();
        }

        /** An arrowhead on the path at arc length s, pointing in the direction of travel. */
        private void arrowAt(DrivePath path, double arc) {
            double ahead = Math.min(path.length(), arc + 0.5);
            double behind = Math.max(0.0, arc - 0.5);
            Point2 p = path.pointAt(arc);
            Point2 b = path.pointAt(behind);
            Point2 c = path.pointAt(ahead);
            double dx = sx(c.x) - sx(b.x);
            double dy = sy(c.y) - sy(b.y);
            double norm = Math.hypot(dx, dy);
            if (norm < 1e-9) {
                return;
            }
            arrowhead(sx(p.x), sy(p.y), dx / norm, dy / norm, 10.0, ink, "arrowhead");
        }

        /** A filled triangle whose tip is at (px, py) and which points along the pixel direction (ux, uy). */
        private void arrowhead(double px, double py, double ux, double uy, double size, String fill, String cls) {
            double bx = px - ux * size;
            double by = py - uy * size;
            double nx = -uy * size * 0.5;
            double ny = ux * size * 0.5;
            w.empty("polygon", "class", cls, "points", n(px) + "," + n(py) + " " + n(bx + nx) + "," + n(by + ny) + " " + n(bx - nx) + ","
                    + n(by - ny), "fill", fill);
        }

        private void approach(RouteAnalysis.StepInfo info) {
            double dist = info.from.distanceTo(info.to);
            if (dist < 0.5) {
                return;       // a pure turn in place has no line to draw
            }
            w.open("g", "class", "approach", "data-step", Integer.toString(info.index + 1));
            w.element("title", "step " + (info.index + 1) + ": move to the pose of " + info.step.kind());
            w.empty("line", "x1", n(sx(info.from.x)), "y1", n(sy(info.from.y)), "x2", n(sx(info.to.x)), "y2", n(sy(info.to.y)), "stroke", ink,
                    "stroke-width", "2", "stroke-dasharray", "2 5", "stroke-linecap", "round", "opacity", "0.85");
            double dx = sx(info.to.x) - sx(info.from.x);
            double dy = sy(info.to.y) - sy(info.from.y);
            double norm = Math.hypot(dx, dy);
            arrowhead(sx(info.to.x), sy(info.to.y), dx / norm, dy / norm, 8.0, ink, "arrowhead");
            w.close("g");
        }

        // ---- robot footprints -------------------------------------------------------------------

        /** True if the validator found an ERROR at this step: the robot drawn there breaks a rule and is drawn in the warning colour. */
        private boolean broken(RouteAnalysis.StepInfo info) {
            for (Issue i : issues) {
                if (i.stepIndex == info.index && i.isError()) {
                    return true;
                }
            }
            return false;
        }

        /**
         * The robot to scale, in two tones: the whole PERIMETER as a faint outline, the CHASSIS (the body) in the ink tone and
         * the two flat intake WHEELS as dark discs at the front corners; the FRONT edge (wheel tips) is marked in the front
         * colour and the BACK edge in the back colour. {@code pressing} (or null) is the wheel that presses a FLOWER: it is
         * ringed in the front colour. A footprint that {@code broken} a rule (an ERROR at its step) gets an amber outline.
         */
        private void footprint(Pose2 pose, double fillOpacity, boolean dashed, boolean broken, IntakeWheel pressing) {
            Footprint fp = Footprint.of(pose);
            w.empty("polygon", "class", broken ? "outline error" : "outline", "points", points(fp), "fill", broken ? WARN_COLOR : ink,
                    "fill-opacity", n(broken ? Math.max(fillOpacity, BROKEN_FILL_OPACITY) : fillOpacity), "stroke", broken ? WARN_COLOR : ink,
                    "stroke-width", broken ? "3.4" : "1.8", "stroke-dasharray", dashed ? "6 3" : null, "stroke-linejoin", "round");
            chassisBody(pose);
            frontAndBackEdges(fp);
            flatWheels(pose, pressing);
        }

        /** The front edge (the wheel tips) in the front colour and the back edge (the shooter) in the back colour. */
        private void frontAndBackEdges(Footprint fp) {
            w.empty("line", "class", "front-edge", "x1", n(sx(fp.corner(0).x)), "y1", n(sy(fp.corner(0).y)), "x2", n(sx(fp.corner(3).x)),
                    "y2", n(sy(fp.corner(3).y)), "stroke", BioBuzzConstants.Auto.RENDER_FRONT_COLOR, "stroke-width", "4.5");
            w.empty("line", "class", "back-edge", "x1", n(sx(fp.corner(1).x)), "y1", n(sy(fp.corner(1).y)), "x2", n(sx(fp.corner(2).x)),
                    "y2", n(sy(fp.corner(2).y)), "stroke", BioBuzzConstants.Auto.RENDER_BACK_COLOR, "stroke-width", "4.5");
        }

        /** The first tone: the chassis body, tinted in the alliance ink. */
        private void chassisBody(Pose2 pose) {
            w.empty("polygon", "class", "chassis", "points", points(Footprint.chassis(pose)), "fill", ink, "fill-opacity", CHASSIS_OPACITY,
                    "stroke", ink, "stroke-width", "0.8", "stroke-linejoin", "round");
        }

        /** The second tone: the two flat intake wheels as dark discs. Drawn last so the wheel tips show over the front edge. */
        private void flatWheels(Pose2 pose, IntakeWheel pressing) {
            for (IntakeWheel wheel : IntakeWheel.values()) {
                Point2 c = Footprint.wheelCenter(pose, wheel);
                boolean presses = wheel == pressing;
                w.empty("circle", "class", presses ? "wheel pressing" : "wheel", "data-wheel", wheel.name(), "cx", n(sx(c.x)), "cy", n(sy(c.y)),
                        "r", n(len(BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0)), "fill", WHEEL_FILL, "fill-opacity", "0.88",
                        "stroke", presses ? BioBuzzConstants.Auto.RENDER_FRONT_COLOR : WHEEL_FILL, "stroke-width", presses ? "2.2" : "0.8");
            }
        }

        private void startFootprint(RouteAnalysis.StepInfo info) {
            Pose2 p = info.to;
            w.open("g", "id", "start-footprint", "class", "footprint start", "data-step", "1");
            w.element("title", stepTitle(info));
            footprint(p, 0.14, false, broken(info), null);
            // heading arrow from the center out past the front edge
            double dirX = Math.cos(p.heading);
            double dirY = -Math.sin(p.heading);
            double cx = sx(p.x);
            double cy = sy(p.y);
            double reach = len(Footprint.frontExtentIn()) - 5.0;
            w.empty("line", "class", "heading-arrow", "x1", n(cx + dirX * 11), "y1", n(cy + dirY * 11), "x2", n(cx + dirX * (reach - 8)), "y2",
                    n(cy + dirY * (reach - 8)), "stroke", ink, "stroke-width", "2.5");
            arrowhead(cx + dirX * reach, cy + dirY * reach, dirX, dirY, 10.0, ink, "arrowhead");
            w.close("g");
        }

        private void intake(RouteAnalysis.StepInfo info) {
            Step.Intake in = (Step.Intake) info.step;
            double radius = in.source() == IntakeSource.FLOWER ? BioBuzzConstants.Auto.RENDER_INTAKE_RING_FLOWER_IN
                    : in.source() == IntakeSource.GARDEN ? BioBuzzConstants.Auto.RENDER_INTAKE_RING_GARDEN_IN
                    : BioBuzzConstants.Auto.RENDER_INTAKE_RING_FLOOR_IN;
            FlowerPress press = FlowerPress.of(in, f);
            w.open("g", "class", "intake", "data-step", Integer.toString(info.index + 1), "data-source", in.source().name());
            w.element("title", stepTitle(info));
            // a FLOWER intake is drawn where it happens: at the press pose, the wheel on the POLLEN
            footprint(press == null ? in.robotPose() : press.press, 0.07, true, broken(info), press == null ? null : press.wheel);
            w.empty("circle", "class", "intake-ring", "cx", n(sx(in.target().x)), "cy", n(sy(in.target().y)), "r", n(len(radius)), "fill",
                    "none", "stroke", ink, "stroke-width", "2.5", "stroke-dasharray", "7 4");
            if (press != null) {
                pressIn(info, press);
            }
            w.close("g");
        }

        /**
         * The press-in of a FLOWER: a short solid arrow along the path of the pressing wheel, from where the final approach
         * starts to the wheel's front tip inside the POLLEN, and a short dashed arrow for the retreat (the robot backs straight
         * out along its own center line).
         */
        private void pressIn(RouteAnalysis.StepInfo info, FlowerPress press) {
            Point2 from = Footprint.wheelCenter(press.lineUp, press.wheel);
            Point2 wheel = Footprint.wheelCenter(press.press, press.wheel);
            double radius = BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN / 2.0;
            double tipX = wheel.x + Math.cos(press.press.heading) * radius;      // the front tip of the wheel, in the POLLEN
            double tipY = wheel.y + Math.sin(press.press.heading) * radius;
            w.open("g", "class", "press-in", "data-step", Integer.toString(info.index + 1), "data-wheel", press.wheel.name(), "data-flower",
                    Integer.toString(press.flower + 1));
            w.element("title", "press-in: drive straight in, the " + press.wheel + " flat wheel presses the bottom POLLEN of FLOWER "
                    + (press.flower + 1) + ", then back straight out " + String.format(Locale.ROOT, "%.0f", press.retreatLengthIn) + " in");
            double ux = sx(tipX) - sx(from.x);
            double uy = sy(tipY) - sy(from.y);
            double norm = Math.hypot(ux, uy);
            if (norm > 1.0) {
                w.empty("line", "class", "press-arrow", "x1", n(sx(from.x)), "y1", n(sy(from.y)), "x2", n(sx(tipX) - ux / norm * 6.0), "y2",
                        n(sy(tipY) - uy / norm * 6.0), "stroke", ink, "stroke-width", "3", "stroke-linecap", "round");
                arrowhead(sx(tipX), sy(tipY), ux / norm, uy / norm, 9.0, ink, "arrowhead");
            }
            double rx = sx(press.retreat.x) - sx(press.press.x);
            double ry = sy(press.retreat.y) - sy(press.press.y);
            double rn = Math.hypot(rx, ry);
            if (rn > 1.0) {
                w.empty("line", "class", "retreat-arrow", "x1", n(sx(press.press.x)), "y1", n(sy(press.press.y)), "x2", n(sx(press.retreat.x) - rx / rn * 5.0),
                        "y2", n(sy(press.retreat.y) - ry / rn * 5.0), "stroke", ink, "stroke-width", "2", "stroke-dasharray", "2 4", "stroke-linecap", "round");
                arrowhead(sx(press.retreat.x), sy(press.retreat.y), rx / rn, ry / rn, 7.0, ink, "arrowhead");
            }
            w.close("g");
        }

        private void shoot(RouteAnalysis.StepInfo info) {
            Step.Shoot sh = (Step.Shoot) info.step;
            CellSpec cell = f.cell(info.targetSide);
            Point2 launch = ShotGeometry.launchPoint(info.shootPose);
            Point2 aim = ShotGeometry.aimPoint(cell);
            w.open("g", "class", "shoot-pose", "data-step", Integer.toString(info.index + 1), "data-nominal",
                    info.shootPoseIsNominal ? "true" : "false");
            w.element("title", stepTitle(info) + (info.shootPoseIsNominal ? " - pose drawn is the nominal standoff pose; the solver picks the real one" : ""));
            footprint(info.shootPose, 0.10, info.shootPoseIsNominal, broken(info), null);
            double lx = sx(launch.x);
            double ly = sy(launch.y);
            double ax = sx(aim.x);
            double ay = sy(aim.y);
            double dx = ax - lx;
            double dy = ay - ly;
            double norm = Math.hypot(dx, dy);
            if (norm > 1.0) {
                double ux = dx / norm;
                double uy = dy / norm;
                w.empty("line", "class", "shot", "x1", n(lx), "y1", n(ly), "x2", n(ax - ux * 9), "y2", n(ay - uy * 9), "stroke", ink,
                        "stroke-width", "2.5", "stroke-dasharray", "8 5", "stroke-linecap", "round");
                arrowhead(ax, ay, ux, uy, 10.0, ink, "arrowhead");
            }
            w.close("g");
        }

        private void park(RouteAnalysis.StepInfo info) {
            finalOutline("park-outline", info, ((Step.Park) info.step).pose());
        }

        /** The robot's last pose when the route ends on a DRIVE, so that LEAVE can be judged by eye. */
        private void endOutline(RouteAnalysis.StepInfo info) {
            finalOutline("end-outline", info, info.to);
        }

        /** The final footprint of the route: a bold dashed outline (PARK: inside the LOADING ZONE) with the front and back edges. */
        private void finalOutline(String cls, RouteAnalysis.StepInfo info, Pose2 p) {
            Footprint fp = Footprint.of(p);
            boolean broken = broken(info);
            w.open("g", "class", cls, "data-step", Integer.toString(info.index + 1));
            w.element("title", stepTitle(info));
            w.empty("polygon", "class", broken ? "outline error" : "outline", "points", points(fp), "fill", broken ? WARN_COLOR : ink,
                    "fill-opacity", n(broken ? BROKEN_FILL_OPACITY : 0.12), "stroke", broken ? WARN_COLOR : ink, "stroke-width", broken ? "3.8" : "3",
                    "stroke-dasharray", "9 4", "stroke-linejoin", "round");
            chassisBody(p);
            frontAndBackEdges(fp);
            flatWheels(p, null);
            w.close("g");
        }

        // ---- badges, labels, issue marks --------------------------------------------------------

        /** Where a step's number circle wants to sit: on its leg, or at its pose. */
        private double[] badgeAnchor(RouteAnalysis.StepInfo info) {
            if (info.path != null) {
                Point2 mid = info.path.pointAt(info.path.length() / 2.0);
                return new double[] {sx(mid.x), sy(mid.y)};
            }
            if (info.step.kind() == StepKind.SHOOT) {
                return new double[] {sx(info.shootPose.x), sy(info.shootPose.y)};
            }
            if (info.step.kind() == StepKind.INTAKE) {
                FlowerPress press = FlowerPress.of((Step.Intake) info.step, f);
                if (press != null) {
                    return new double[] {sx(press.press.x), sy(press.press.y)};     // the robot pressed in, not backed out
                }
            }
            return new double[] {sx(info.to.x), sy(info.to.y)};
        }

        /** The first free spot around the wanted one, so that badges of steps at the same place fan out. */
        private double[] placeBadge(double[] want) {
            double minGap = badgeR * 2 + 1.5;
            for (int ring = 0; ring < 6; ring++) {
                int slots = ring == 0 ? 1 : 8;
                for (int k = 0; k < slots; k++) {
                    double angle = Math.toRadians(-45.0 + 45.0 * k);
                    double r = ring * (badgeR * 2 + 2);
                    double x = want[0] + r * Math.cos(angle);
                    double y = want[1] + r * Math.sin(angle);
                    boolean free = true;
                    for (double[] b : badges) {
                        if (Math.hypot(b[0] - x, b[1] - y) < minGap) {
                            free = false;
                            break;
                        }
                    }
                    if (free) {
                        return new double[] {x, y};
                    }
                }
            }
            return want;
        }

        /** Finds a free place for a label near an anchor; returns {x, y, 0 start | 1 middle | 2 end}. */
        private double[] placeLabel(String text, double size, double ax, double ay) {
            double wd = textWidth(text, size);
            double ht = size + 2;
            double[][] candidates = {{15, 18, 0}, {15, -9, 0}, {-15, 18, 2}, {-15, -9, 2}, {0, 30, 1}, {0, -20, 1}, {32, 5, 0}, {-32, 5, 2}};
            for (int scale = 1; scale <= 3; scale++) {
                for (double[] c : candidates) {
                    double x = ax + c[0] * scale;
                    double y = ay + c[1] * scale;
                    double left = c[2] == 0 ? x : c[2] == 1 ? x - wd / 2 : x - wd;
                    double[] box = {left, y - ht + 2, left + wd, y + 3};
                    if (box[0] < 3 || box[2] > canvas - 3 || box[1] < 4 || box[3] > legendTop - FOOTER_GAP_PX) {
                        continue;
                    }
                    if (collides(box)) {
                        continue;
                    }
                    labelBoxes.add(box);
                    return new double[] {x, y, c[2]};
                }
            }
            double x = ax + 15;
            double y = ay + 18;
            labelBoxes.add(new double[] {x, y - ht + 2, x + wd, y + 3});
            return new double[] {x, y, 0};
        }

        private boolean collides(double[] box) {
            for (double[] o : labelBoxes) {
                if (box[0] < o[2] && box[2] > o[0] && box[1] < o[3] && box[3] > o[1]) {
                    return true;
                }
            }
            for (double[] b : badges) {
                if (box[0] < b[0] + badgeR && box[2] > b[0] - badgeR && box[1] < b[1] + badgeR
                        && box[3] > b[1] - badgeR) {
                    return true;
                }
            }
            return false;
        }

        private void label(String text, double ax, double ay, boolean small) {
            double size = small ? fontPx - 1.0 : fontPx;
            double[] p = placeLabel(text, size, ax, ay);
            w.element("text", text, "class", "step-label", "x", n(p[0]), "y", n(p[1]), "font-size", n(size), "font-weight", "bold", "fill", ink,
                    "text-anchor", p[2] == 0 ? "start" : p[2] == 1 ? "middle" : "end", "stroke", "#ffffff", "stroke-width", "3",
                    "stroke-linejoin", "round", "paint-order", "stroke");
        }

        private void badgesAndLabels() {
            // reserve every badge first, so labels can avoid all of them
            List<double[]> placed = new ArrayList<double[]>();
            for (RouteAnalysis.StepInfo info : a.steps()) {
                double[] p = placeBadge(badgeAnchor(info));
                badges.add(p);
                placed.add(p);
            }
            w.open("g", "id", "badges");
            for (RouteAnalysis.StepInfo info : a.steps()) {
                double[] p = placed.get(info.index);
                w.open("g", "class", "badge", "data-step", Integer.toString(info.index + 1), "data-kind", info.step.kind().name());
                w.element("title", stepTitle(info));
                w.empty("circle", "cx", n(p[0]), "cy", n(p[1]), "r", n(badgeR), "fill", HIGHLIGHT, "stroke", ink, "stroke-width", "2.5");
                w.element("text", Integer.toString(info.index + 1), "x", n(p[0]), "y", n(p[1] + 3.8), "font-size", "11", "font-weight", "bold",
                        "text-anchor", "middle", "fill", BioBuzzConstants.Auto.RENDER_TEXT_COLOR);
                w.close("g");
            }
            w.close("g");
            w.open("g", "id", "step-labels");
            for (RouteAnalysis.StepInfo info : a.steps()) {
                double[] p = placed.get(info.index);
                stepLabel(info, p);
            }
            w.close("g");
            w.open("g", "id", "issue-marks");
            for (RouteAnalysis.StepInfo info : a.steps()) {
                issueMark(info, placed.get(info.index));
            }
            w.close("g");
        }

        private void stepLabel(RouteAnalysis.StepInfo info, double[] p) {
            Step step = info.step;
            switch (step.kind()) {
                case START:
                    label("START " + route.startTile(), p[0], p[1], false);
                    break;
                case DRIVE: {
                    String name = ((Step.Drive) step).label();
                    if (!name.isEmpty()) {
                        label(name, p[0], p[1], true);
                    }
                    if (info.index == route.size() - 1) {
                        label("END", sx(info.to.x), sy(info.to.y), false);
                    }
                    break;
                }
                case INTAKE: {
                    Step.Intake in = (Step.Intake) step;
                    label("INTAKE " + in.source() + " +" + in.expectedCount(), sx(in.target().x), sy(in.target().y), false);
                    break;
                }
                case SHOOT: {
                    Step.Shoot sh = (Step.Shoot) step;
                    String text = "SHOOT " + sh.count() + (sharesPose(info) ? " (step " + (info.index + 1) + ")" : "");
                    Point2 launch = ShotGeometry.launchPoint(info.shootPose);
                    Point2 aim = ShotGeometry.aimPoint(f.cell(info.targetSide));
                    label(text, (sx(launch.x) + sx(aim.x)) / 2.0, (sy(launch.y) + sy(aim.y)) / 2.0, false);
                    if (info.shootPoseIsNominal && labelledNominal.add(n(info.shootPose.x) + "," + n(info.shootPose.y))) {
                        label("pose chosen by the solver", sx(info.shootPose.x), sy(info.shootPose.y), true);
                    }
                    if (info.tipped) {
                        label("HIVE TIP +" + BioBuzzConstants.Auto.TIP_POINTS, sx(aim.x), sy(aim.y), false);
                    }
                    break;
                }
                case WAIT: {
                    String text = "WAIT " + n(((Step.Wait) step).seconds()) + " s";
                    double[] pos = placeLabel(text, fontPx, p[0], p[1]);
                    // a small clock in front of the text
                    double cx = pos[2] == 2 ? pos[0] - textWidth(text, fontPx) - 9 : pos[2] == 1 ? pos[0] - textWidth(text, fontPx) / 2 - 9 : pos[0] - 9;
                    double cy = pos[1] - 4;
                    w.open("g", "class", "clock");
                    w.empty("circle", "cx", n(cx), "cy", n(cy), "r", "5.5", "fill", "#ffffff", "stroke", ink, "stroke-width", "1.6");
                    w.empty("polyline", "points", n(cx) + "," + n(cy - 3.5) + " " + n(cx) + "," + n(cy) + " " + n(cx + 2.8) + "," + n(cy + 1.6),
                            "fill", "none", "stroke", ink, "stroke-width", "1.4", "stroke-linecap", "round");
                    w.close("g");
                    w.element("text", text, "class", "step-label", "x", n(pos[0]), "y", n(pos[1]), "font-size", n(fontPx), "font-weight", "bold",
                            "fill", ink, "text-anchor", pos[2] == 0 ? "start" : pos[2] == 1 ? "middle" : "end", "stroke", "#ffffff",
                            "stroke-width", "3", "stroke-linejoin", "round", "paint-order", "stroke");
                    break;
                }
                case PARK:
                    label("PARK", p[0], p[1], false);
                    break;
                default:
                    break;
            }
        }

        /** True if another SHOOT stands at the same pose and fires at the same cell, so the two arrows coincide. */
        private boolean sharesPose(RouteAnalysis.StepInfo info) {
            for (RouteAnalysis.StepInfo other : a.steps()) {
                if (other.index != info.index && other.step.kind() == StepKind.SHOOT && other.targetSide == info.targetSide
                        && other.shootPose.distanceTo(info.shootPose) < SAME_POSE_IN) {
                    return true;
                }
            }
            return false;
        }

        private void issueMark(RouteAnalysis.StepInfo info, double[] p) {
            Issue.Severity worst = null;
            for (Issue i : issues) {
                if (i.stepIndex == info.index) {
                    if (i.severity == Issue.Severity.ERROR) {
                        worst = Issue.Severity.ERROR;
                        break;
                    }
                    worst = Issue.Severity.WARN;
                }
            }
            if (worst == null) {
                return;
            }
            double cx = p[0] + 12;
            double cy = p[1] - 12;
            boolean error = worst == Issue.Severity.ERROR;
            w.open("g", "class", "issue-mark", "data-step", Integer.toString(info.index + 1), "data-severity", worst.name());
            w.empty("polygon", "points", n(cx) + "," + n(cy - 8) + " " + n(cx + 8) + "," + n(cy + 6) + " " + n(cx - 8) + "," + n(cy + 6), "fill",
                    error ? ERROR_COLOR : WARN_COLOR, "stroke", "#ffffff", "stroke-width", "1");
            w.element("text", "!", "x", n(cx), "y", n(cy + 4.6), "font-size", "10", "font-weight", "bold", "text-anchor", "middle", "fill",
                    error ? "#ffffff" : "#222222");
            w.close("g");
        }

        // ---- legend and footer ------------------------------------------------------------------

        private void legend() {
            w.open("g", "id", "legend");
            double y1 = legendTop;
            double y2 = legendTop + LEGEND_ROW_SPACING_PX;
            double x = 24;
            x = legendItem(x, y1, "drive", "DRIVE leg");
            x = legendItem(x, y1, "approach", "move to a pose");
            x = legendItem(x, y1, "badge", "step number");
            x = legendItem(x, y1, "robot", "robot: body + 2 flat wheels");
            x = legendItem(x, y1, "front", "FRONT = intake");
            x = legendItem(x, y1, "back", "BACK = shooter");
            x = 24;
            x = legendItem(x, y2, "intake", "INTAKE");
            x = legendItem(x, y2, "shoot", "SHOOT: launch to aim point");
            x = legendItem(x, y2, "wait", "WAIT");
            x = legendItem(x, y2, "park", "PARK");
            x = legendItem(x, y2, "up", "UP cell");
            x = legendItem(x, y2, "down", "DOWN cell");
            legendItem(x, y2, "issue", "finding (amber robot = ERROR)");
            w.close("g");
        }

        private double legendItem(double x, double y, String icon, String text) {
            String col = BioBuzzConstants.Auto.RENDER_TEXT_COLOR;
            w.open("g", "class", "legend-item", "data-icon", icon);
            switch (icon) {
                case "drive":
                    w.empty("line", "x1", n(x), "y1", n(y), "x2", n(x + 16), "y2", n(y), "stroke", ink, "stroke-width", "3", "stroke-linecap", "round");
                    arrowhead(x + 24, y, 1, 0, 9, ink, "arrowhead");
                    break;
                case "approach":
                    w.empty("line", "x1", n(x), "y1", n(y), "x2", n(x + 16), "y2", n(y), "stroke", ink, "stroke-width", "2", "stroke-dasharray", "2 5",
                            "stroke-linecap", "round");
                    arrowhead(x + 24, y, 1, 0, 8, ink, "arrowhead");
                    break;
                case "badge":
                    w.empty("circle", "cx", n(x + 12), "cy", n(y), "r", "7", "fill", "#ffffff", "stroke", ink, "stroke-width", "2");
                    w.element("text", "1", "x", n(x + 12), "y", n(y + 3.5), "font-size", "10", "font-weight", "bold", "text-anchor", "middle", "fill", col);
                    break;
                case "robot":
                    // the outline, the body in the ink tone and the two flat wheels at the front (the right-hand end)
                    w.empty("rect", "x", n(x), "y", n(y - 6), "width", "24", "height", "12", "fill", ink, "fill-opacity", "0.14", "stroke", ink, "stroke-width", "1.4");
                    w.empty("rect", "x", n(x), "y", n(y - 6), "width", "19", "height", "12", "fill", ink, "fill-opacity", CHASSIS_OPACITY);
                    w.empty("circle", "cx", n(x + 20.5), "cy", n(y - 3.5), "r", "2.6", "fill", WHEEL_FILL);
                    w.empty("circle", "cx", n(x + 20.5), "cy", n(y + 3.5), "r", "2.6", "fill", WHEEL_FILL);
                    break;
                case "front":
                    w.empty("line", "x1", n(x), "y1", n(y), "x2", n(x + 24), "y2", n(y), "stroke", BioBuzzConstants.Auto.RENDER_FRONT_COLOR, "stroke-width", "4.5");
                    break;
                case "back":
                    w.empty("line", "x1", n(x), "y1", n(y), "x2", n(x + 24), "y2", n(y), "stroke", BioBuzzConstants.Auto.RENDER_BACK_COLOR, "stroke-width", "4.5");
                    break;
                case "intake":
                    w.empty("circle", "cx", n(x + 12), "cy", n(y), "r", "6.5", "fill", "none", "stroke", ink, "stroke-width", "2", "stroke-dasharray", "5 3");
                    break;
                case "shoot":
                    w.empty("line", "x1", n(x), "y1", n(y), "x2", n(x + 16), "y2", n(y), "stroke", ink, "stroke-width", "2.5", "stroke-dasharray", "6 4");
                    arrowhead(x + 24, y, 1, 0, 9, ink, "arrowhead");
                    break;
                case "wait":
                    w.empty("circle", "cx", n(x + 12), "cy", n(y), "r", "6", "fill", "#ffffff", "stroke", ink, "stroke-width", "1.6");
                    w.empty("polyline", "points", n(x + 12) + "," + n(y - 4) + " " + n(x + 12) + "," + n(y) + " " + n(x + 15) + "," + n(y + 2), "fill", "none",
                            "stroke", ink, "stroke-width", "1.4", "stroke-linecap", "round");
                    break;
                case "park":
                    w.empty("rect", "x", n(x), "y", n(y - 6), "width", "24", "height", "12", "fill", ink, "fill-opacity", "0.12", "stroke", ink, "stroke-width", "2.4",
                            "stroke-dasharray", "6 3");
                    break;
                case "up":
                    w.empty("rect", "x", n(x), "y", n(y - 6), "width", "24", "height", "12", "fill", ink, "fill-opacity", "0.28", "stroke", ink, "stroke-width", "1.8");
                    break;
                case "down":
                    w.empty("rect", "x", n(x), "y", n(y - 6), "width", "24", "height", "12", "fill", "#ffffff", "fill-opacity", "0.35", "stroke", ink,
                            "stroke-width", "1.4", "stroke-dasharray", "5 3");
                    break;
                case "issue":
                    // the amber triangle (the same amber outlines a robot that breaks a rule)
                    w.empty("polygon", "points", n(x + 12) + "," + n(y - 8) + " " + n(x + 20) + "," + n(y + 6) + " " + n(x + 4) + "," + n(y + 6), "fill", WARN_COLOR);
                    w.element("text", "!", "x", n(x + 12), "y", n(y + 4.6), "font-size", "10", "font-weight", "bold", "text-anchor", "middle", "fill", "#222222");
                    break;
                default:
                    break;
            }
            w.element("text", text, "x", n(x + 31), "y", n(y + 4), "font-size", "10.5", "fill", col);
            w.close("g");
            return x + 31 + textWidth(text, 10.5) + 18;
        }

        private void footer() {
            w.open("g", "id", "footer");
            double y = legendTop + LEGEND_ROW_SPACING_PX + FOOTER_GAP_PX;
            String line1 = "Estimated time " + sec(a.totalSeconds()) + " of the " + sec(BioBuzzConstants.Auto.AUTO_DURATION_SEC)
                    + " AUTO (placeholder timing)   |   Claimable AUTO points: " + a.points() + "  (LEAVE " + a.leavePoints() + ", PARK "
                    + a.parkPoints() + ", HIVE TIP " + a.tipPoints() + ")";
            w.element("text", line1, "id", "footer-metrics", "x", "24", "y", n(y), "font-size", "12", "font-weight", "bold", "fill",
                    BioBuzzConstants.Auto.RENDER_TEXT_COLOR);
            int errors = countOf(Issue.Severity.ERROR);
            int warns = countOf(Issue.Severity.WARN);
            String line2 = "POLLEN: hold " + BioBuzzConstants.Auto.PRELOAD_POLLEN + " at start, +" + a.pollenGained() + " taken, -"
                    + a.pollenShot() + " fired, " + a.pollenEnd() + " at the end (balance " + signed(a.pollenBalance()) + ")   |   Validator: "
                    + (errors + warns == 0 ? "no findings" : errors + " ERROR, " + warns + " WARN");
            w.element("text", line2, "id", "footer-pollen", "x", "24", "y", n(y + 16), "font-size", "12", "fill",
                    errors > 0 ? ERROR_COLOR : BioBuzzConstants.Auto.RENDER_TEXT_COLOR);
            int cap = BioBuzzConstants.Auto.RENDER_MAX_ISSUE_LINES;
            int shown = Math.min(issues.size(), issues.size() > cap ? cap - 1 : cap);
            for (int i = 0; i < shown; i++) {
                Issue is = issues.get(i);
                boolean namesStep = is.message.startsWith("step " + is.stepNumber() + " ");
                String where = is.stepIndex < 0 ? "route: " : namesStep ? "" : "step " + is.stepNumber() + ": ";
                String text = (is.isError() ? "ERROR " : "WARN ") + where + is.message;
                if (text.length() > ISSUE_LINE_MAX_CHARS) {
                    text = text.substring(0, ISSUE_LINE_MAX_CHARS - 3) + "...";
                }
                w.open("g", "class", "issue-line", "data-severity", is.severity.name());
                w.element("title", is.toString());
                w.element("text", text, "x", "24", "y", n(y + 29 + ISSUE_LINE_SPACING_PX * i), "font-size", "10", "fill", is.isError() ? ERROR_COLOR : "#7a5a00");
                w.close("g");
            }
            if (issues.size() > shown) {
                w.element("text", "+ " + (issues.size() - shown) + " more findings (see the gallery page)", "x", "24", "y",
                        n(y + 29 + ISSUE_LINE_SPACING_PX * shown), "font-size", "10", "fill", MUTED_TEXT);
            }
            w.close("g");
        }

        // ---- text helpers -----------------------------------------------------------------------

        private static String sec(double v) {
            return String.format(Locale.ROOT, "%.1f s", v);
        }

        private static String signed(int v) {
            return v > 0 ? "+" + v : Integer.toString(v);
        }
    }
}
