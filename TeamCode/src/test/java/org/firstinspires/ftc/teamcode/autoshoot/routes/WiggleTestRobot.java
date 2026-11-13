package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;

/**
 * A robot for the tests of the FLOWER wiggle whose flat wheels stick out far enough for it to work.
 *
 * <p>With the numbers of the goBILDA drawing (the flat wheels stick out 2.7 in in front of the chassis, which keeps 0.5 in off the FLOWER plate) the
 * wiggle does NOT reach the ball: the turn swings the chassis corner toward the plate, so the robot has to stand farther out, and the wheel then
 * falls 0.77 in short. The validator reports exactly that (see {@code FlowerWiggleTest}). A test about something else (the wheel choice, the timing, the
 * drawing, the HIVE tips) needs a pick-up that works, so it asks for this robot: the same everything, with wheels that stick out
 * {@link #REACH_IN} inches. Nothing here changes {@code BioBuzzConstants}.
 */
public final class WiggleTestRobot {
    /** How far the flat wheels stick out in front of the chassis for a robot whose wiggle works (the drawing says 2.7 in; 3.5 in is the least that works with 0.5 in of clearance). */
    public static final double REACH_IN = 4.0;

    private WiggleTestRobot() {
    }

    /** The wiggle numbers of the real robot, with wheels that reach {@link #REACH_IN}. */
    public static FlowerWiggle.Spec spec() {
        return FlowerWiggle.Spec.fromConstants().withIntakeReach(REACH_IN);
    }

    /** The default timing numbers with {@link #spec()}. */
    public static TimingModel timing() {
        return new TimingModel().withFlowerSpec(spec());
    }

    /** A validator for this robot. */
    public static RouteValidator validator() {
        return new RouteValidator(new FieldModel(), timing());
    }

    /** The findings for a route, as seen by a robot whose wheels reach {@link #REACH_IN}. */
    public static List<Issue> check(AutoRoute route) {
        return validator().validate(route);
    }

    /** The analysis of a route, as seen by a robot whose wheels reach {@link #REACH_IN}. */
    public static RouteAnalysis analyze(AutoRoute route) {
        return RouteAnalysis.of(route, new FieldModel(), timing());
    }
}
