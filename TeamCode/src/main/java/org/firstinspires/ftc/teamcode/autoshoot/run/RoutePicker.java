package org.firstinspires.ftc.teamcode.autoshoot.run;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.routes.AutoRoute;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteAnalysis;
import org.firstinspires.ftc.teamcode.autoshoot.routes.RouteValidator;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StartPlacement;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Step;
import org.firstinspires.ftc.teamcode.autoshoot.routes.StepKind;
import org.firstinspires.ftc.teamcode.autoshoot.routes.TipCameraPolicy;

/**
 * The logic of the INIT screen of the Auto Route Picker, without any gamepad or telemetry in it: which route and which alliance are chosen,
 * what is known about that choice (the route checker's findings, the time, the points, where to put the robot) and whether the robot may
 * start. The OpMode only feeds it the buttons, the camera's status and prints {@link #screen}.
 *
 * <p>The student picks with four buttons (the buttons are in {@code BioBuzzConstants.Run}): one goes up the list of routes, one down it
 * (the list wraps round), one chooses RED and one BLUE. The routes are the ones authored for RED in the {@code RouteCatalog}; for BLUE the
 * picker uses {@code AutoRoute.forAlliance}, the half turn of the field.
 *
 * <p><b>When the robot refuses to start</b> ({@link Readiness#refusal}):
 * <ul>
 *   <li>the route has a validator ERROR: it breaks a rule or cannot work. The message says which and how many;</li>
 *   <li>the route waits for the HIVE to tip (a WAIT_FOR_TIP) and the Limelight is not working: without the camera the robot could never
 *       see the tip, so the route could only give up; or the route's whole point is a volley after the partner's tip
 *       ({@link TipCameraPolicy} says REFUSE).</li>
 * </ul>
 * A route that only has later volleys depending on its own tip gets a WARNING instead (it still earns its first tip).
 *
 * <p>The analysis of every (route, alliance) is worked out once and kept: the validator is thorough and the INIT loop runs many times a second.
 */
public final class RoutePicker {

    /** What is known about one choice of route and alliance. */
    public static final class Choice {
        /** The route in the frame of the chosen alliance. */
        public final AutoRoute route;
        /** The route played through (poses, times, points, who tips). */
        public final RouteAnalysis analysis;
        /** Every finding of the route checker, ERRORs and WARNs. */
        public final List<Issue> issues;
        public final int errors;
        public final int warnings;
        /** Where to put the robot, in plain words for a student with a tape measure. */
        public final String placement;
        /** True if the route has a WAIT_FOR_TIP step. */
        public final boolean waitsForTip;
        private final Map<VisionFrame.Status, TipCameraPolicy.Finding> policies = new EnumMap<VisionFrame.Status, TipCameraPolicy.Finding>(VisionFrame.Status.class);

        Choice(AutoRoute route, RouteAnalysis analysis, List<Issue> issues, String placement) {
            this.route = route;
            this.analysis = analysis;
            this.issues = issues;
            int e = 0;
            for (Issue i : issues) {
                if (i.isError()) {
                    e++;
                }
            }
            this.errors = e;
            this.warnings = issues.size() - e;
            this.placement = placement;
            boolean waits = false;
            for (Step s : route.steps()) {
                waits |= s.kind() == StepKind.WAIT_FOR_TIP;
            }
            this.waitsForTip = waits;
        }

        /** What {@link TipCameraPolicy} says about this route when the camera reports {@code camera} (worked out once, then kept). */
        public TipCameraPolicy.Finding cameraPolicy(VisionFrame.Status camera) {
            TipCameraPolicy.Finding f = policies.get(camera);
            if (f == null) {
                f = TipCameraPolicy.atInit(route, camera);
                policies.put(camera, f);
            }
            return f;
        }
    }

    /** Whether the robot may start with this choice, and what to tell the drive team. */
    public static final class Readiness {
        /** Null if the robot may start; otherwise the reason it refuses, in plain words. */
        public final String refusal;
        /** Things that do not stop the robot but that the drive team should know. */
        public final List<String> warnings;

        Readiness(String refusal, List<String> warnings) {
            this.refusal = refusal;
            this.warnings = Collections.unmodifiableList(warnings);
        }

        public boolean canStart() {
            return refusal == null;
        }
    }

    private final List<AutoRoute> routes;
    private final Map<String, Choice> cache = new HashMap<String, Choice>();
    private int index;
    private Alliance alliance;
    private boolean prevPrevious;
    private boolean prevNext;
    private boolean prevRed;
    private boolean prevBlue;

    /** @param routes the routes to choose from, authored for RED (as in {@code RouteCatalog.all()}); at least one */
    public RoutePicker(List<AutoRoute> routes, Alliance initialAlliance) {
        if (routes.isEmpty()) {
            throw new IllegalArgumentException("there is no route to pick");
        }
        this.routes = Collections.unmodifiableList(new ArrayList<AutoRoute>(routes));
        this.alliance = initialAlliance;
    }

    // ---- choosing ----------------------------------------------------------------------------------------------------

    /**
     * Feed it the four buttons as they are NOW (true = held down), once per INIT loop. A button counts when it goes down, so holding one
     * does not run through the list.
     */
    public void buttons(boolean previousRoute, boolean nextRoute, boolean red, boolean blue) {
        if (previousRoute && !prevPrevious) {
            index = (index + routes.size() - 1) % routes.size();
        }
        if (nextRoute && !prevNext) {
            index = (index + 1) % routes.size();
        }
        if (red && !prevRed) {
            alliance = Alliance.RED;
        }
        if (blue && !prevBlue) {
            alliance = Alliance.BLUE;
        }
        prevPrevious = previousRoute;
        prevNext = nextRoute;
        prevRed = red;
        prevBlue = blue;
    }

    public Alliance alliance() {
        return alliance;
    }

    /** Position in the list, from 0. */
    public int routeIndex() {
        return index;
    }

    public int routeCount() {
        return routes.size();
    }

    /** What is known about the chosen route and alliance (worked out the first time it is asked for). */
    public Choice choice() {
        AutoRoute red = routes.get(index);
        String key = red.id() + "/" + alliance;
        Choice c = cache.get(key);
        if (c == null) {
            AutoRoute mine = red.forAlliance(alliance);
            c = new Choice(mine, mine.analyze(), RouteValidator.check(mine), StartPlacement.placementText(mine));
            cache.put(key, c);
        }
        return c;
    }

    // ---- may it start? -----------------------------------------------------------------------------------------------

    /** Whether the robot may start with the chosen route when the Limelight reports {@code camera}. */
    public Readiness readiness(VisionFrame.Status camera) {
        Choice c = choice();
        List<String> warnings = new ArrayList<String>();
        String refusal = null;
        if (c.errors > 0) {
            Issue first = null;
            for (Issue i : c.issues) {
                if (i.isError()) {
                    first = i;
                    break;
                }
            }
            refusal = "this route has " + c.errors + " ERROR" + (c.errors == 1 ? "" : "S") + " from the route checker, so it breaks a rule or cannot work. The first: "
                    + first + ". Fix the route (or pick another) before the match.";
        }
        TipCameraPolicy.Finding policy = c.cameraPolicy(camera);
        if (policy.verdict == TipCameraPolicy.Verdict.REFUSE) {
            String why = "the Limelight camera is not working and this route needs it: " + policy.message;
            refusal = refusal == null ? why : refusal + "  ALSO " + why;
        } else if (c.waitsForTip && camera != VisionFrame.Status.OK) {
            String why = "this route waits for the HIVE to tip, and the Limelight is not working (" + TipCameraPolicy.reason(camera)
                    + "), so it could never see the tip. Fix the camera or pick a route that does not wait.";
            refusal = refusal == null ? why : refusal + "  ALSO " + why;
        } else if (policy.verdict == TipCameraPolicy.Verdict.WARN) {
            warnings.add(policy.message);
        }
        if (c.warnings > 0) {
            warnings.add(c.warnings + " WARN" + (c.warnings == 1 ? "" : "S") + " from the route checker: probably fine, check them on the field.");
        }
        return new Readiness(refusal, warnings);
    }

    // ---- the screen --------------------------------------------------------------------------------------------------

    /**
     * The lines of the INIT screen, top to bottom. {@code placeholders} are the names of the constants still waiting for a measurement
     * (from {@code ConstantsCheck}), shown so nobody forgets them.
     */
    public List<String> screen(VisionFrame.Status camera, List<String> placeholders) {
        Choice c = choice();
        Readiness ready = readiness(camera);
        List<String> out = new ArrayList<String>();
        out.add("AUTO ROUTE PICKER   " + BioBuzzConstants.Run.MENU_PREVIOUS_ROUTE_BUTTON + " / " + BioBuzzConstants.Run.MENU_NEXT_ROUTE_BUTTON
                + " = route,  " + BioBuzzConstants.Run.MENU_RED_BUTTON + " = RED,  " + BioBuzzConstants.Run.MENU_BLUE_BUTTON + " = BLUE");
        out.add("ALLIANCE: " + alliance);
        out.add("ROUTE " + (index + 1) + " of " + routes.size() + ": " + c.route.title() + "  [" + c.route.id() + "]");
        out.add("FIRST TIP: " + c.route.firstTipSummary());
        RouteAnalysis a = c.analysis;
        String ours = "TIME about " + String.format(Locale.ROOT, "%.1f", a.totalSeconds()) + " s of " + BioBuzzConstants.Auto.AUTO_DURATION_SEC
                + " s   OUR POINTS " + a.points() + (a.partnerTipPoints() > 0 ? " (+" + a.partnerTipPoints() + " the partner's tip, not ours)" : "");
        out.add(ours);
        if (a.partnerTipAssumed()) {
            out.add(String.format(Locale.ROOT, "LATEST PARTNER TIP that still works: %.1f s (assumed %.1f s)", a.latestPartnerTipSec(), a.partnerTipSec()));
        }
        out.add("PLACE THE ROBOT: " + c.placement);
        out.add("CAMERA: " + camera + " - " + c.cameraPolicy(camera).message);
        out.add("ROUTE CHECK: " + c.errors + " ERROR, " + c.warnings + " WARN");
        int shown = 0;
        for (int pass = 0; pass < 2; pass++) {
            for (Issue i : c.issues) {
                if (i.isError() == (pass == 0)) {
                    if (shown < BioBuzzConstants.Run.MENU_MAX_ISSUE_LINES) {
                        out.add("  " + i);
                    }
                    shown++;
                }
            }
        }
        if (shown > BioBuzzConstants.Run.MENU_MAX_ISSUE_LINES) {
            out.add("  ... and " + (shown - BioBuzzConstants.Run.MENU_MAX_ISSUE_LINES) + " more findings (see docs/autos)");
        }
        if (!placeholders.isEmpty()) {
            int names = Math.min(placeholders.size(), BioBuzzConstants.Run.MENU_MAX_PLACEHOLDER_NAMES);
            StringBuilder sb = new StringBuilder("STILL TO MEASURE (" + placeholders.size() + "): ");
            for (int i = 0; i < names; i++) {
                sb.append(i == 0 ? "" : ", ").append(placeholders.get(i));
            }
            if (placeholders.size() > names) {
                sb.append(", and ").append(placeholders.size() - names).append(" more");
            }
            out.add(sb.toString());
        }
        for (String w : ready.warnings) {
            out.add("WARNING: " + w);
        }
        out.add(ready.canStart() ? ">>> READY: put the robot where it says, then press PLAY" : ">>> REFUSED, WILL NOT START: " + ready.refusal);
        return out;
    }
}
