package org.firstinspires.ftc.teamcode.autoshoot.render;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteCatalog;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteValidator;

/**
 * Command line entry point that writes the route gallery of {@link RouteCatalog}: {@code java ...RenderRoutesMain [output directory]}.
 * Without an argument it writes {@link #DEFAULT_DIR}, relative to where it is started. The scripts
 * {@code tools/render-autos.sh} and {@code tools/render-autos.bat} compile and run it with plain javac.
 *
 * <p>It prints one line per route (time, points and validator findings) so a change to the catalog can be
 * judged without opening the page. It always exits normally when the files were written, even if routes have
 * ERROR findings: those are information about the routes, not a failure of the tool.
 */
public final class RenderRoutesMain {
    /** Where the committed diagrams live, relative to the repository root. */
    public static final String DEFAULT_DIR = "docs/autos";

    private RenderRoutesMain() {
    }

    public static void main(String[] args) throws IOException {
        File dir = new File(args.length > 0 ? args[0] : DEFAULT_DIR);
        List<AutoRoute> routes = RouteCatalog.all();
        List<String> written = RouteGallery.write(dir, routes);
        System.out.println("Wrote " + written.size() + " files to " + dir.getAbsolutePath() + " (" + RouteGallery.INDEX_FILE + " and "
                + (written.size() - 1) + " SVG diagrams).");
        System.out.println();
        System.out.println(String.format(Locale.ROOT, "%-18s %7s %7s %7s  %s", "route", "time", "points", "POLLEN", "validator"));
        for (AutoRoute route : routes) {
            RouteAnalysis a = route.analyze();
            List<Issue> issues = RouteValidator.check(route);
            int errors = 0;
            for (Issue i : issues) {
                if (i.isError()) {
                    errors++;
                }
            }
            String status = issues.isEmpty() ? "no findings" : errors + " ERROR, " + (issues.size() - errors) + " WARN";
            System.out.println(String.format(Locale.ROOT, "%-18s %6.1fs %7d %7d  %s", route.id(), a.totalSeconds(), a.points(),
                    a.pollenBalance(), status));
        }
        // routes that were removed from the catalog leave their old diagrams behind; say so instead of deleting files
        File[] existing = dir.listFiles();
        if (existing != null) {
            for (File f : existing) {
                String name = f.getName();
                if ((name.endsWith("_" + Alliance.RED.name().toLowerCase(Locale.ROOT) + ".svg")
                        || name.endsWith("_" + Alliance.BLUE.name().toLowerCase(Locale.ROOT) + ".svg")) && !written.contains(name)) {
                    System.out.println();
                    System.out.println("Note: " + name + " belongs to no route in the catalog any more; delete it if the route was removed.");
                }
            }
        }
    }
}
