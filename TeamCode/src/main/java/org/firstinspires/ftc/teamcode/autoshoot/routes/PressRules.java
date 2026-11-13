package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

/**
 * The rules of a FLOWER pick-up, written ONCE as "depths" that {@link Sweep} can run over any movement of the robot (a
 * straight approach, the turn of the wiggle, the retreat). Both the program that DERIVES the wiggle ({@link FlowerWiggle}: it
 * looks for the closest legal standoff) and the validator that CHECKS a route ({@link RouteValidator}) use these same functions,
 * so what the program plans is by construction what the validator accepts, and they can never drift apart.
 *
 * <p>A "depth" says how far the robot is into something it must stay out of: above 0 the rule is broken, below 0 the robot is clear by
 * that many inches (see {@link Sweep.Depth}). {@code margin} makes a rule a little STRICTER (the derivation asks for a hair of extra
 * room, so the pose it derives is clear of the validator's rule by more than rounding noise); the validator passes 0.
 *
 * <p>The robot is the chassis (the body) plus two round flat wheels in front of it. Around the FLOWER being emptied:
 * <ul>
 *   <li>the CHASSIS stays off the plate and its clearance zone: {@link #chassisOffPlate};</li>
 *   <li>the PRESSING wheel may enter the plate but never the stop zone, the part nearer the wall than the POLLEN's center line:
 *       {@link #wheelShortOfStopLine};</li>
 *   <li>the OTHER wheel stays out of the plate: {@link #wheelOffPlate};</li>
 *   <li>nothing leaves the field: {@link #insideField}.</li>
 * </ul>
 */
final class PressRules {
    /** A guard against rounding, not a tuning value: a rule is broken only when the robot is more than this far into the obstacle. */
    static final double EPS = 1e-9;

    private PressRules() {
    }

    /** The chassis (the perimeter without the {@code reach} inches where only the two wheels stick out) must stay off {@code keepOut}. */
    static Sweep.Depth chassisOffPlate(final Rect keepOut, final double reach, final double margin) {
        return new Sweep.Depth() {
            @Override
            public double at(Pose2 p) {
                return -Footprint.chassis(p, reach).signedGap(keepOut) - EPS + margin;
            }
        };
    }

    /** The round pressing {@code wheel} must stay out of {@code zone} (the plate between the wall and the stop line). */
    static Sweep.Depth wheelShortOfStopLine(final IntakeWheel wheel, final Rect zone, final double margin) {
        return wheelOffPlate(wheel, zone, margin);
    }

    /** The round {@code wheel} must stay out of {@code keepOut}. */
    static Sweep.Depth wheelOffPlate(final IntakeWheel wheel, final Rect keepOut, final double margin) {
        return new Sweep.Depth() {
            @Override
            public double at(Pose2 p) {
                return -Footprint.wheelDiscGap(p, wheel, keepOut) - EPS + margin;
            }
        };
    }

    /** The whole footprint must stay inside {@code field}, to within {@code tolerance} inches. */
    static Sweep.Depth insideField(final Rect field, final double tolerance, final double margin) {
        return new Sweep.Depth() {
            @Override
            public double at(Pose2 p) {
                return Footprint.of(p).signedOvershoot(field) - tolerance - EPS + margin;
            }
        };
    }
}
