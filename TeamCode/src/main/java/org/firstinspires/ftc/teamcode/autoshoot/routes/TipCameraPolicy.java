package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;

/**
 * What the Auto Route Picker must say at INIT about a route that depends on a HIVE tip when the Limelight camera is not working.
 *
 * <p><b>The rule (the user's fourth constraint):</b> the robot shoots at the cell that comes up after a tip only when its camera has
 * SEEN that cell UP and steady ({@code TipConfirmation}); a time never lets it shoot. With no working camera it can never confirm a
 * tip, so every volley that depends on one is skipped and the robot carries on with the safe steps (LEAVE, PARK). A drive team must
 * not find that out in the middle of a match, so at INIT:
 * <ul>
 *   <li>a route that depends on no tip (it only fires at the cell that is up from the start, or not at all) is fine without the
 *       camera: {@link Verdict#OK};</li>
 *   <li>a route that assumes the PARTNER shoots the first tip ({@link FirstTip#PARTNER}) is useless without the camera, since its
 *       whole point is the volley after the partner's tip: {@link Verdict#REFUSE}, with the reason;</li>
 *   <li>a route that shoots the first tip itself but has later volleys that depend on its own tip still earns its first tip: a
 *       {@link Verdict#WARN} with the reason and what is lost.</li>
 * </ul>
 * The plain-words reason says what is wrong with the camera ({@link VisionFrame.Status}). The picker calls {@link #atInit} with the
 * status the Limelight adapter reports; everything is pure Java, so a test can try every status.
 */
public final class TipCameraPolicy {
    private TipCameraPolicy() {
    }

    /** How serious it is that the camera is not working, for this route. */
    public enum Verdict {
        /** Nothing to say: the camera works, or the route does not need it. */
        OK,
        /** The route can still be run but loses its tip-dependent volleys: show the reason on the Driver Station. */
        WARN,
        /** The route must not be run: it exists for a volley that cannot happen without the camera. */
        REFUSE
    }

    /** The picker's finding: how serious, and the plain-words message for the Driver Station. */
    public static final class Finding {
        public final Verdict verdict;
        public final String message;

        Finding(Verdict verdict, String message) {
            this.verdict = verdict;
            this.message = message;
        }

        @Override
        public String toString() {
            return verdict + ": " + message;
        }
    }

    /**
     * The finding for {@code route} (in whichever alliance frame; the numbers do not depend on it) when the camera reports
     * {@code camera}.
     */
    public static Finding atInit(AutoRoute route, VisionFrame.Status camera) {
        RouteAnalysis a = route.analyze();
        if (!a.dependsOnTipConfirmation()) {
            return new Finding(Verdict.OK, "this route needs no camera for any tip: it only fires at the cell that is UP from the start");
        }
        if (camera == VisionFrame.Status.OK) {
            return new Finding(Verdict.OK, "the camera works: the volley after the tip waits until it has seen the HIVE tipped, never for a time");
        }
        RouteAnalysis g = a.giveUpPlan();
        int volleys = a.tipDependentSteps().size();
        String volleyText = volleys + " volley" + (volleys == 1 ? "" : "s");
        String lost = "without the camera the robot cannot confirm any tip, so it would leave " + volleyText + " out (no camera, no shot)"
                + (g.points() < a.points() ? " and claim only " + g.points() + " of the " + a.points() + " points of this route" : "")
                + " and carry on with the safe steps (LEAVE, PARK)";
        if (route.firstTip() == FirstTip.PARTNER) {
            return new Finding(Verdict.REFUSE, "do not run this route: it shoots after the alliance partner's tip, and " + reason(camera) + ". " + lost
                    + ". Pick a route that does not depend on the camera, or fix the Limelight first");
        }
        return new Finding(Verdict.WARN, "the Limelight is not working (" + reason(camera) + "): " + lost + " (its first tip does not need the camera)");
    }

    /** What is wrong with the camera, in plain words for a first-year student. */
    public static String reason(VisionFrame.Status camera) {
        switch (camera) {
            case NOT_CONNECTED:
                return "the Control Hub cannot reach the Limelight (check its cable and power, and the name in the robot configuration)";
            case NO_RESULT:
                return "the Limelight is connected but has not produced a result yet";
            case STALE:
                return "the Limelight's newest picture is too old to trust";
            case STALLED:
                return "the Limelight's link looks alive but no new picture has arrived for a while";
            case WRONG_PIPELINE:
                return "the Limelight is running a pipeline other than the AprilTag one";
            default:
                return "the camera status is " + camera.name().toLowerCase(Locale.ROOT);
        }
    }
}
