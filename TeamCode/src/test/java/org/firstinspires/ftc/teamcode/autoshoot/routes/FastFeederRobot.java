package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;

/**
 * A robot for the tests whose feeder stops the INSTANT the camera sees the HIVE tip.
 *
 * <p>The real plan is less kind: the camera sees a tip only about {@code Auto.TIP_STOP_LATENCY_SEC} after the tipping POLLEN lands, and the next POLLEN
 * of a volley leaves 0.9 s after the last, so the spare POLLEN of a 4-POLLEN volley is probably launched into the cell while it swings down and is
 * spent ({@link TipModel#launchedBeforeTheFeederStops}); the validator warns and the plan counts it as fired. A test about something ELSE (a wheel, a
 * wall, the timing, the partner's tip) that needs a clean 4-POLLEN volley that keeps its spare asks for this robot: the same everything, with a feeder
 * that stops with no delay. Nothing here changes {@code BioBuzzConstants}. The behaviour of the real numbers is tested in {@code TipStopLatencyTest}.
 */
public final class FastFeederRobot {
    private FastFeederRobot() {
    }

    /** The default timing numbers with a feeder that stops at once ({@code TIP_STOP_LATENCY_SEC} = 0, so the ball flight alone never fits a second POLLEN). */
    public static TimingModel timing() {
        return new TimingModel().withTipStopLatencySec(0.0);
    }

    /** A validator for this robot. */
    public static RouteValidator validator() {
        return new RouteValidator(new FieldModel(), timing());
    }

    /** The findings for a route, as seen by this robot. */
    public static List<Issue> check(AutoRoute route) {
        return validator().validate(route);
    }

    /** The analysis of a route, as seen by this robot. */
    public static RouteAnalysis analyze(AutoRoute route) {
        return RouteAnalysis.of(route, new FieldModel(), timing());
    }
}
