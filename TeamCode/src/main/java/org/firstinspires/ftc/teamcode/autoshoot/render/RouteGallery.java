package org.firstinspires.ftc.teamcode.autoshoot.render;

import static org.firstinspires.ftc.teamcode.autoshoot.render.SvgWriter.esc;
import static org.firstinspires.ftc.teamcode.autoshoot.render.SvgWriter.n;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.FirstTip;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteValidator;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Step;
import org.firstinspires.ftc.teamcode.autoshoot.routes.TipModel;

/**
 * Builds the route gallery: one {@code <id>_red.svg} and one {@code <id>_blue.svg} per route and an
 * {@code index.html} that lists every route with both diagrams side by side, a table of its numbers and
 * the validator's findings. The page is self-contained (its style is inline, no scripts, no fonts or
 * images from the internet) and follows the reader's light or dark setting through CSS variables and
 * {@code prefers-color-scheme}. The SVG files carry their own white background, so they read the same in
 * both themes.
 *
 * <p>The output depends only on the routes and {@link BioBuzzConstants}: no dates, no machine names, so
 * regenerating it without a change gives byte-identical files and a unit test can tell when the committed
 * copy is out of date. Pure Java (java.io only), so a PC can run it; see {@link RenderRoutesMain}.
 */
public final class RouteGallery {
    /** The page that lists everything. */
    public static final String INDEX_FILE = "index.html";

    private RouteGallery() {
    }

    /** The SVG file name of a route for an alliance, e.g. {@code red_b1_blue.svg}. */
    public static String svgFileName(AutoRoute route, Alliance alliance) {
        return route.id() + "_" + alliance.name().toLowerCase(Locale.ROOT) + ".svg";
    }

    /**
     * Every file of the gallery by file name, {@code index.html} first, in a stable order. Route ids become
     * part of file names, so they may only use letters, digits, '_' and '-', and must be unique.
     */
    public static Map<String, String> build(List<AutoRoute> routes) {
        Set<String> seen = new HashSet<String>();
        List<Entry> entries = new ArrayList<Entry>();
        for (AutoRoute route : routes) {
            if (!route.id().matches("[A-Za-z0-9_-]+")) {
                throw new IllegalArgumentException("route id '" + route.id() + "' cannot be a file name: use only letters, digits, '_' and '-'");
            }
            if (!seen.add(route.id())) {
                throw new IllegalArgumentException("two routes share the id '" + route.id() + "'");
            }
            entries.add(new Entry(route));
        }
        Map<String, String> files = new LinkedHashMap<String, String>();
        files.put(INDEX_FILE, page(entries));
        for (Entry e : entries) {
            files.put(svgFileName(e.route, Alliance.RED), RouteSvgRenderer.render(e.route, Alliance.RED, e.redIssues));
            files.put(svgFileName(e.route, Alliance.BLUE), RouteSvgRenderer.render(e.route, Alliance.BLUE, e.blueIssues));
        }
        return files;
    }

    /** The index page alone (for tests and previews); the SVG files are named as {@link #build} names them. */
    public static String indexHtml(List<AutoRoute> routes) {
        List<Entry> entries = new ArrayList<Entry>();
        for (AutoRoute route : routes) {
            entries.add(new Entry(route));
        }
        return page(entries);
    }

    /**
     * Writes the gallery into {@code dir} (created if needed), replacing files of the same name. Files that
     * were written for routes that no longer exist are left alone; {@link RenderRoutesMain} reports them.
     *
     * @return the names of the files written, in the order of {@link #build}
     */
    public static List<String> write(File dir, List<AutoRoute> routes) throws IOException {
        Map<String, String> files = build(routes);
        if (!dir.isDirectory() && !dir.mkdirs()) {
            throw new IOException("cannot create the output directory " + dir.getAbsolutePath());
        }
        List<String> written = new ArrayList<String>();
        for (Map.Entry<String, String> f : files.entrySet()) {
            OutputStream out = new FileOutputStream(new File(dir, f.getKey()));
            try {
                out.write(f.getValue().getBytes("UTF-8"));      // the text is plain ASCII, see SvgWriter.esc
            } finally {
                out.close();
            }
            written.add(f.getKey());
        }
        return written;
    }

    // ---------------------------------------------------------------------------------------------

    /** A route with everything the page says about it, worked out once. */
    private static final class Entry {
        final AutoRoute route;
        final AutoRoute red;
        final AutoRoute blue;
        final RouteAnalysis analysis;
        final List<Issue> redIssues;
        final List<Issue> blueIssues;

        Entry(AutoRoute route) {
            this.route = route;
            this.red = route.forAlliance(Alliance.RED);
            this.blue = route.forAlliance(Alliance.BLUE);
            this.analysis = red.analyze();
            this.redIssues = RouteValidator.check(red);
            this.blueIssues = RouteValidator.check(blue);
        }

        int count(List<Issue> issues, Issue.Severity severity) {
            int n = 0;
            for (Issue i : issues) {
                if (i.severity == severity) {
                    n++;
                }
            }
            return n;
        }

        boolean sameFindings() {
            if (redIssues.size() != blueIssues.size()) {
                return false;
            }
            for (int i = 0; i < redIssues.size(); i++) {
                Issue a = redIssues.get(i);
                Issue b = blueIssues.get(i);
                if (a.severity != b.severity || a.rule != b.rule || a.stepIndex != b.stepIndex) {
                    return false;
                }
            }
            return true;
        }
    }

    // ---- the page ----------------------------------------------------------------------------------

    private static final String CSS =
            ":root { color-scheme: light dark; --bg: #f6f5f1; --fg: #1d1d1b; --muted: #5d5d57; --card: #ffffff; --border: #d6d3c8;"
            + " --link: #0b57d0; --red: " + BioBuzzConstants.Auto.RENDER_INK_RED + "; --blue: " + BioBuzzConstants.Auto.RENDER_INK_BLUE + ";"
            + " --err: #b71c1c; --err-bg: #fdecea; --warn: #7a5a00; --warn-bg: #fff4d6; --ok: #1b5e20; --ok-bg: #e6f4ea; --code-bg: #ecebe4; }\n"
            + "@media (prefers-color-scheme: dark) {\n"
            + "  :root { --bg: #16161a; --fg: #e8e6df; --muted: #a3a198; --card: #202026; --border: #3a3a42; --link: #8ab4f8;"
            + " --red: #ef9a9a; --blue: #90caf9; --err: #ff8a80; --err-bg: #3b1f1f; --warn: #ffd36b; --warn-bg: #3a3017;"
            + " --ok: #81c995; --ok-bg: #1d3326; --code-bg: #2a2a31; }\n"
            + "}\n"
            + "* { box-sizing: border-box; }\n"
            + "body { margin: 0; background: var(--bg); color: var(--fg); font: 16px/1.5 system-ui, -apple-system, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; }\n"
            + "a { color: var(--link); }\n"
            + "header, main, footer { max-width: 1500px; margin: 0 auto; padding: 0 16px; }\n"
            + "header { padding-top: 24px; }\n"
            + "h1 { margin: 0 0 4px; font-size: 1.7rem; }\n"
            + "h2 { margin: 0 0 4px; font-size: 1.3rem; }\n"
            + "h3 { margin: 20px 0 6px; font-size: 1rem; }\n"
            + "p { margin: 6px 0; }\n"
            + ".muted { color: var(--muted); }\n"
            + "code { background: var(--code-bg); padding: 1px 5px; border-radius: 4px; font-size: 0.9em; }\n"
            + "section.route { background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 16px; margin: 20px 0; }\n"
            + ".tablewrap { overflow-x: auto; }\n"
            + "table { border-collapse: collapse; width: 100%; }\n"
            + "th, td { text-align: left; padding: 5px 10px; border-bottom: 1px solid var(--border); vertical-align: top; }\n"
            + "thead th { color: var(--muted); font-weight: 600; font-size: 0.85rem; }\n"
            + "table.meta { max-width: 780px; margin: 10px 0; }\n"
            + "table.meta th { width: 15rem; color: var(--muted); font-weight: 600; }\n"
            + ".pair { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; margin: 14px 0; }\n"
            + "@media (max-width: 900px) { .pair { grid-template-columns: minmax(0, 1fr); } }\n"
            + "figure { margin: 0; }\n"
            + "figcaption { font-weight: 600; margin-bottom: 4px; }\n"
            + "figure.red figcaption { color: var(--red); }\n"
            + "figure.blue figcaption { color: var(--blue); }\n"
            + "img.diagram { display: block; width: 100%; height: auto; background: #ffffff; border: 1px solid var(--border); border-radius: 6px; }\n"
            + "ol.steps, ul.tips { margin: 6px 0; padding-left: 1.6rem; }\n"
            + "ol.steps li, ul.tips li { margin: 3px 0; }\n"
            + ".kind { display: inline-block; min-width: 4.2rem; font-weight: 700; font-size: 0.85rem; }\n"
            + ".time { color: var(--muted); font-size: 0.85rem; }\n"
            + ".note { color: var(--muted); font-size: 0.9rem; }\n"
            + "ul.issues { list-style: none; margin: 6px 0; padding: 0; }\n"
            + "li.issue { margin: 5px 0; padding: 6px 10px; border-radius: 6px; border-left: 5px solid; }\n"
            + "li.error { background: var(--err-bg); border-color: var(--err); }\n"
            + "li.warn { background: var(--warn-bg); border-color: var(--warn); }\n"
            + "li.ok { background: var(--ok-bg); border-color: var(--ok); }\n"
            + "li.error strong { color: var(--err); }\n"
            + "li.warn strong { color: var(--warn); }\n"
            + "li.ok strong { color: var(--ok); }\n"
            + ".status-error { color: var(--err); font-weight: 700; }\n"
            + ".status-warn { color: var(--warn); font-weight: 700; }\n"
            + ".status-ok { color: var(--ok); font-weight: 700; }\n"
            + "footer { padding-bottom: 32px; color: var(--muted); font-size: 0.9rem; }\n";

    private static String page(List<Entry> entries) {
        StringBuilder h = new StringBuilder(64 * 1024);
        h.append("<!DOCTYPE html>\n");
        h.append("<html lang=\"en\">\n<head>\n");
        h.append("<meta charset=\"utf-8\" />\n");
        h.append("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />\n");
        h.append("<title>BIOBUZZ autonomous routes</title>\n");
        h.append("<style>\n").append(CSS).append("</style>\n</head>\n<body>\n");

        h.append("<header>\n<h1>BIOBUZZ autonomous routes</h1>\n");
        h.append("<p class=\"muted\">Team 38063. Every route of <code>RouteCatalog</code>, drawn for both alliances and checked by the validator. ");
        h.append("This page and the diagrams are generated: do not edit them by hand, change the catalog and run ");
        h.append("<code>tools/render-autos.sh</code> (or <code>tools/render-autos.bat</code>).</p>\n");
        h.append("<p class=\"muted\">How to read a diagram: top view of the ").append(n(BioBuzzConstants.Field.SIZE_IN)).append(" x ")
                .append(n(BioBuzzConstants.Field.SIZE_IN)).append(" in field, the AUDIENCE wall at the bottom, the red alliance on the left. ");
        h.append("A BLUE route is the RED route turned half a turn about the field center (not a mirror image), so a blue robot starts at the top. ");
        h.append("The robot is drawn to scale (").append(inches(BioBuzzConstants.Robot.LENGTH_IN)).append(" x ")
                .append(inches(BioBuzzConstants.Robot.WIDTH_IN)).append(", wheels included): the body is tinted, the two flat intake wheels are the dark discs at the front, ");
        h.append("and the small ringed dot is its center of rotation, the point every pose refers to (it is ").append(inches(BioBuzzConstants.Robot.FRONT_EXTENT_IN))
                .append(" behind the front edge and ").append(inches(BioBuzzConstants.Robot.REAR_EXTENT_IN)).append(" in front of the back edge, so it is not in the middle). ");
        h.append("Its FRONT edge (the wheel tips) is green and its BACK edge (the shooter) is orange. A robot with an amber outline breaks a rule at that step. ");
        h.append("A FLOWER is emptied by pressing ONE flat wheel into its bottom POLLEN: the arrow shows that straight press-in and the dotted arrow the straight retreat. ");
        h.append("Times use placeholder speeds that nobody has measured yet, so compare routes with them but do not trust them to the second.</p>\n");
        h.append("<p id=\"tip-rule\" class=\"muted\">How a HIVE tips: ").append(esc(TipModel.explainRule())).append(" Every route says who shoots the first tip: ")
                .append("<code>").append(esc(FirstTip.SELF.summary(0.0))).append("</code> (our pre-loads are the first tip: ")
                .append(TipModel.firstTipPollen()).append(" of the ").append(BioBuzzConstants.Auto.PRELOAD_POLLEN).append(" tip it) or <code>")
                .append(esc(FirstTip.PARTNER.summary(BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC)))
                .append("</code> (the alliance partner's volley is the first tip and our robot goes to the other side; the HIVE belongs to the alliance, so the partner's launches count for it). The partner's ")
                .append(BioBuzzConstants.Auto.TIP_POINTS).append(" points are shown but never counted in the points of our robot. ")
                .append("The robot goes on after a partner's tip only when its camera has SEEN the tipped cell steady for ")
                .append(seconds(BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC)).append(", never because a time has passed.</p>\n");
        h.append("</header>\n<main>\n");

        // ---- summary table
        h.append("<section id=\"summary\">\n<h2>All routes</h2>\n<div class=\"tablewrap\">\n<table class=\"summary\">\n<thead><tr>");
        for (String col : new String[] {"Route", "Start tile", "First tip", "Steps", "Estimated time", "Our AUTO points", "POLLEN balance", "Validator"}) {
            h.append("<th>").append(esc(col)).append("</th>");
        }
        h.append("</tr></thead>\n<tbody>\n");
        for (Entry e : entries) {
            RouteAnalysis a = e.analysis;
            h.append("<tr class=\"summary-row\" data-route=\"").append(esc(e.route.id())).append("\">");
            h.append("<td><a href=\"#").append(esc(e.route.id())).append("\">").append(esc(e.route.title())).append("</a><br /><code>")
                    .append(esc(e.route.id())).append("</code></td>");
            h.append("<td>").append(esc(e.route.startTile())).append("</td>");
            h.append("<td>").append(esc(e.route.firstTipSummary()));
            if (a.partnerTipAssumed()) {
                h.append("<br /><span class=\"note\">").append(esc(latestTipText(a))).append("</span>");
            }
            h.append("</td>");
            h.append("<td>").append(e.route.size()).append("</td>");
            h.append("<td>").append(seconds(a.totalSeconds())).append("</td>");
            h.append("<td>").append(a.points()).append("</td>");
            h.append("<td>").append(signed(a.pollenBalance())).append("</td>");
            h.append("<td>").append(statusHtml(e)).append("</td>");
            h.append("</tr>\n");
        }
        h.append("</tbody>\n</table>\n</div>\n</section>\n");

        // ---- one section per route
        for (Entry e : entries) {
            route(h, e);
        }

        h.append("</main>\n<footer>\n<p>Generated by <code>autoshoot.render.RenderRoutesMain</code> from <code>autoshoot.routes.RouteCatalog</code>. ");
        h.append("How to add an auto as one data entry: <code>docs/AUTOS.md</code>.</p>\n</footer>\n</body>\n</html>\n");
        return h.toString();
    }

    private static void route(StringBuilder h, Entry e) {
        AutoRoute route = e.route;
        RouteAnalysis a = e.analysis;
        h.append("<section class=\"route\" id=\"").append(esc(route.id())).append("\" data-route=\"").append(esc(route.id())).append("\">\n");
        h.append("<h2>").append(esc(route.title())).append("</h2>\n");
        h.append("<p class=\"muted\"><code>").append(esc(route.id())).append("</code></p>\n");
        if (!route.description().isEmpty()) {
            h.append("<p>").append(esc(route.description())).append("</p>\n");
        }

        h.append("<table class=\"meta\">\n");
        row(h, "Start tile", esc(route.startTile()));
        row(h, "Start pose, RED", esc(pose(e.red.startPose())));
        row(h, "Start pose, BLUE", esc(pose(e.blue.startPose())));
        row(h, "Steps", Integer.toString(route.size()));
        row(h, "First tip", esc(route.firstTipSummary()));
        row(h, "Estimated time", seconds(a.totalSeconds()) + " of the " + seconds(BioBuzzConstants.Auto.AUTO_DURATION_SEC)
                + " AUTO (placeholder timing)");
        row(h, "Our AUTO points", a.points() + " = LEAVE " + a.leavePoints() + " + PARK " + a.parkPoints() + " + our HIVE TIP "
                + a.tipPoints());
        if (a.partnerTipAssumed()) {
            row(h, "Assumed partner tip", a.partnerTips() == 1
                    ? "+" + a.partnerTipPoints() + " for the alliance at " + seconds(a.partnerTipSec())
                            + ": an assumption, shown here and NOT counted in our points"
                    : "none: this route tips the HIVE itself before " + seconds(a.partnerTipSec()) + ", so the partner's tip is not needed");
        }
        if (a.partnerTipAssumed()) {
            row(h, "Latest partner tip that works", esc(slackText(a)));
        }
        if (a.giveUpPlan() != null) {
            row(h, "If the tip is never confirmed", esc(giveUpText(a)));
        }
        row(h, "POLLEN", "hold " + BioBuzzConstants.Auto.PRELOAD_POLLEN + " at the start, +" + a.pollenGained() + " taken, -"
                + a.pollenShot() + " fired, " + a.pollenEnd() + " at the end (balance " + signed(a.pollenBalance()) + ")");
        row(h, "Our HIVE TIPs", Integer.toString(a.tips()) + (a.tips() == 0 ? "" : " (the first needs " + TipModel.firstTipPollen()
                + " POLLEN into the UP cell, each later one " + TipModel.laterTipPollen() + "; " + esc(spareText(a)) + ")"));
        row(h, "Ends", "LEAVE " + (a.leave() ? "yes" : "no" + esc(leaveLostText(a))) + ", PARK " + (a.park() ? "yes, " + inches(a.parkOverlapIn()) + " inside the LOADING ZONE" : "no")
                + ", at " + esc(pose(a.endPose())));
        row(h, "Validator", statusHtml(e));
        h.append("</table>\n");

        h.append("<div class=\"pair\">\n");
        figure(h, e, Alliance.RED);
        figure(h, e, Alliance.BLUE);
        h.append("</div>\n");

        h.append("<h3>Steps</h3>\n<ol class=\"steps\">\n");
        for (RouteAnalysis.StepInfo info : a.steps()) {
            h.append("<li class=\"step\" data-kind=\"").append(info.step.kind().name()).append("\"><span class=\"kind\">")
                    .append(info.step.kind().name()).append("</span> ").append(esc(describeRest(info))).append(" <span class=\"time\">t = ")
                    .append(oneDecimal(info.startSec)).append(" to ").append(oneDecimal(info.endSec())).append(" s</span>");
            if (info.step instanceof Step.WaitForTip) {
                h.append("<div class=\"note waitnote\">").append(esc(waitText(a, info))).append("</div>");
            }
            if (!info.step.note().isEmpty()) {
                h.append("<div class=\"note\">").append(esc(info.step.note())).append("</div>");
            }
            h.append("</li>\n");
        }
        h.append("</ol>\n");

        tipArithmetic(h, a);

        h.append("<h3>Validator findings</h3>\n");
        if (e.sameFindings()) {
            issues(h, e.redIssues, "Same for RED and BLUE.");
        } else {
            issues(h, e.redIssues, "RED:");
            issues(h, e.blueIssues, "BLUE:");
        }
        h.append("</section>\n");
    }

    /**
     * The tip arithmetic of THIS route in plain words: the partner's assumed tip (if the route assumes one) and every
     * volley with what the UP cell held and whether the HIVE tipped. The rule itself is at the top of the page.
     */
    private static void tipArithmetic(StringBuilder h, RouteAnalysis a) {
        h.append("<h3>HIVE tips</h3>\n<ul class=\"tips\">\n");
        boolean any = false;
        for (RouteAnalysis.TipEvent tip : a.tipEvents()) {
            if (tip.byPartner) {
                any = true;
                h.append("<li class=\"tipline partner\">The partner's assumed tip at ").append(seconds(tip.timeSec)).append(": the ")
                        .append(tip.releasingSide).append(" cell tips down and spills, the ").append(tip.newUpSide)
                        .append(" cell comes up with no NECTAR in it, so our robot needs ").append(TipModel.laterTipPollen())
                        .append(" POLLEN there for the next tip (plan ").append(TipModel.planningPollen(0))
                        .append(": a real field drifts). The robot sees it confirmed ").append(seconds(a.tipSettleSec()))
                        .append(" after the tip, at ").append(seconds(a.partnerTipConfirmedSec())).append(". These ").append(BioBuzzConstants.Auto.TIP_POINTS)
                        .append(" points are the alliance's, not ours.</li>\n");
            }
        }
        if (a.partnerTipVoided()) {
            any = true;
            h.append("<li class=\"tipline partner\">The partner's tip would be at ").append(seconds(a.partnerTipSec()))
                    .append(", but this route tips the HIVE itself before that, so the partner is not needed.</li>\n");
        }
        for (RouteAnalysis.StepInfo info : a.steps()) {
            if (!(info.step instanceof Step.Shoot)) {
                continue;
            }
            any = true;
            int count = ((Step.Shoot) info.step).count();
            h.append("<li class=\"tipline volley\" data-step=\"").append(info.index + 1).append("\">Step ").append(info.index + 1)
                    .append(": ");
            if (info.targetWasUp) {
                h.append(esc(TipModel.explainVolley(info.nectarInCellBefore, info.pollenInCellBefore, count)));
                if (info.firedAfterTip > 0) {
                    h.append("; this volley is TIMED and cannot stop, so the last ").append(info.firedAfterTip)
                            .append(" POLLEN would go into a cell that is already tipping down");
                }
            } else {
                h.append("fires ").append(count).append(" POLLEN at the ").append(info.targetSide)
                        .append(" cell, which faces down when they leave, so they count for nothing");
            }
            h.append(".</li>\n");
        }
        if (!any) {
            h.append("<li class=\"tipline none\">This route launches no POLLEN, so it does not tip the HIVE.</li>\n");
        }
        h.append("</ul>\n");
    }

    private static void figure(StringBuilder h, Entry e, Alliance alliance) {
        String name = alliance.name().toLowerCase(Locale.ROOT);
        String file = svgFileName(e.route, alliance);
        String alt = e.route.title() + ", " + alliance + " alliance: top view of the field with the route drawn on it";
        h.append("<figure class=\"").append(name).append("\">\n<figcaption>").append(alliance).append(" alliance</figcaption>\n");
        h.append("<a href=\"").append(esc(file)).append("\"><img class=\"diagram\" src=\"").append(esc(file)).append("\" alt=\"").append(esc(alt))
                .append("\" width=\"").append(n(BioBuzzConstants.Auto.RENDER_CANVAS_PX)).append("\" height=\"")
                .append(n(BioBuzzConstants.Auto.RENDER_CANVAS_PX)).append("\" loading=\"lazy\" /></a>\n</figure>\n");
    }

    private static void issues(StringBuilder h, List<Issue> issues, String heading) {
        h.append("<p class=\"muted\">").append(esc(heading)).append("</p>\n<ul class=\"issues\">\n");
        if (issues.isEmpty()) {
            h.append("<li class=\"issue ok\"><strong>OK</strong> no findings</li>\n");
        }
        for (Issue i : issues) {
            boolean error = i.isError();
            h.append("<li class=\"issue ").append(error ? "error" : "warn").append("\" data-severity=\"").append(i.severity.name())
                    .append("\" data-step=\"").append(i.stepNumber()).append("\"><strong>").append(i.severity.name()).append("</strong> ")
                    .append(i.stepIndex < 0 ? "route" : "step " + i.stepNumber()).append(" <code>").append(esc(i.rule.manualRule())).append("</code> ")
                    .append(esc(i.rule.title())).append(": ").append(esc(i.message)).append("</li>\n");
        }
        h.append("</ul>\n");
    }

    private static void row(StringBuilder h, String name, String htmlValue) {
        h.append("<tr><th>").append(esc(name)).append("</th><td>").append(htmlValue).append("</td></tr>\n");
    }

    // ---- small formatters ----------------------------------------------------------------------------

    private static String statusHtml(Entry e) {
        int errors = e.count(e.redIssues, Issue.Severity.ERROR);
        int warns = e.count(e.redIssues, Issue.Severity.WARN);
        if (errors > 0) {
            return "<span class=\"status-error\">" + errors + " ERROR" + (warns > 0 ? ", " + warns + " WARN" : "") + "</span>";
        }
        if (warns > 0) {
            return "<span class=\"status-warn\">" + warns + " WARN</span>";
        }
        return "<span class=\"status-ok\">no findings</span>";
    }

    /** What a step does without repeating its kind (the page shows the kind in its own column). */
    private static String describeRest(RouteAnalysis.StepInfo info) {
        String d = info.step.describe();
        String kind = info.step.kind().name();
        return d.startsWith(kind + " ") ? d.substring(kind.length() + 1) : d;
    }

    /**
     * Why LEAVE is lost, in words: the robot ends too close to a wall. LEAVE is judged at the END of AUTO and any touch of any
     * perimeter wall loses it, also the wall inside the LOADING ZONE (FIRST's Q&A QA21); a robot that parks there touching the wall
     * keeps its PARK.
     */
    private static String leaveLostText(RouteAnalysis a) {
        return " (the robot ends " + inches(Math.max(0.0, a.endWallGapIn())) + " from the " + a.endNearestWall() + "; LEAVE needs "
                + inches(BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN) + " off every wall at the end of AUTO, and touching any wall loses it, the one in the LOADING ZONE too: QA21)";
    }

    /**
     * How much spare the route's own tips have, in words: the smallest spare of its tipping volleys. "no spare" is the case
     * the validator WARNs about (a field that needs one more POLLEN than FIRST's calibration would not tip).
     */
    private static String spareText(RouteAnalysis a) {
        int spare = a.minimumTipSpare();
        if (spare < 0) {
            return "no tip, so no spare";
        }
        return spare == 0 ? "NO spare: exactly the minimum, a field that needs one more POLLEN would not tip"
                : spare + " POLLEN to spare, so a field that needs " + spare + " more still tips";
    }

    /**
     * What the robot does when the camera never confirms the tip a volley depends on (the partner is late or never tips, or the
     * camera is dead, missing or on the wrong pipeline): it waits until its give-up time, leaves the tip-dependent volleys out ("no
     * camera, no shot") and carries on, which should still earn the safe points (LEAVE, PARK).
     */
    private static String giveUpText(RouteAnalysis a) {
        RouteAnalysis g = a.giveUpPlan();
        int volleys = a.tipDependentSteps().size();
        String wait = "does not wait";
        for (RouteAnalysis.StepInfo info : a.steps()) {
            if (info.step instanceof Step.WaitForTip && a.partnerTipAssumed()) {
                wait = "waits until " + seconds(a.effectiveGiveUpSec(info));
                break;
            }
        }
        return "the robot " + wait + ", leaves " + volleys + " volley" + (volleys == 1 ? "" : "s") + " out (no camera, no shot) and carries on: it ends at "
                + pose(g.endPose()) + " after " + seconds(g.totalSeconds()) + " with LEAVE " + (g.leave() ? "yes" : "no") + ", PARK "
                + (g.park() ? "yes" : "no") + ": " + g.points() + " points";
    }

    /** Short: the latest partner tip that still works, for the summary table. */
    private static String latestTipText(RouteAnalysis a) {
        double latest = a.latestPartnerTipSec();
        return Double.isInfinite(latest) ? "works with any partner tip time" : "latest tip that works: " + seconds(latest);
    }

    /**
     * The slack of a partner route in plain words: how late the partner may tip and the route still does everything it
     * plans, and why that is the limit. The robot never shoots because of a time; the numbers only tell the drive team
     * which partner routes are safe to pick.
     */
    private static String slackText(RouteAnalysis a) {
        double latest = a.latestPartnerTipSec();
        if (Double.isInfinite(latest)) {
            return "any time: this route does not depend on when the partner tips (it never shoots at the cell that comes up)";
        }
        double slack = a.partnerTipSlackSec();
        String text = seconds(latest) + " is the latest partner tip the route can still use, " + seconds(Math.abs(slack))
                + (slack >= 0 ? " of slack after" : " BEFORE") + " the assumed " + seconds(a.partnerTipSec()) + ".";
        if (slack < 0) {
            return text + " The route does not work with the assumed tip time.";
        }
        if (slack < BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC) {
            return text + " That is less than " + seconds(BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC)
                    + ": pick this route only if the partner is as fast as agreed.";
        }
        return text + " If the partner tips later than that, the robot gives up waiting, skips the volleys that need the tip"
                + " and still does LEAVE and PARK.";
    }

    /** The give-up time of a WAIT_FOR_TIP and the latest one the rest of the route allows. */
    private static String waitText(RouteAnalysis a, RouteAnalysis.StepInfo info) {
        double giveUp = ((Step.WaitForTip) info.step).giveUpSec();
        return "The robot shoots after the camera has SEEN the HIVE tipped, never at a time. It stops waiting at " + seconds(giveUp)
                + " on the route clock; the steps after this one take " + seconds(a.totalSeconds() - info.endSec())
                + ", so it may wait until " + seconds(a.latestGiveUpSec(info)) + " at the latest.";
    }

    private static String pose(Pose2 p) {
        return String.format(Locale.ROOT, "(%.1f, %.1f) in, heading %.0f deg", p.x, p.y, p.headingDegrees());
    }

    private static String inches(double v) {
        return String.format(Locale.ROOT, "%.2f in", v);
    }

    private static String oneDecimal(double v) {
        return String.format(Locale.ROOT, "%.1f", v);
    }

    private static String seconds(double v) {
        return oneDecimal(v) + " s";
    }

    private static String signed(int v) {
        return v > 0 ? "+" + v : Integer.toString(v);
    }
}
