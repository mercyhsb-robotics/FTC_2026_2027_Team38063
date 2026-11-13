package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.HiveSolid;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

/**
 * Every geometric rule of one leg of a route, swept over EVERY pose of that leg ({@link Sweep}): inside the field, out of the HIVE, out of every
 * FLOWER plate, on the own side of the center line, and, for a FLOWER pick-up, the three rules that replace "stay out of the plate" for the one
 * FLOWER being emptied (the chassis keeps its clearance off the plate, the pressing wheel never goes past the stop line, the other wheel never
 * enters the plate). The routes validator turns the findings into messages; {@link WheelChoice} asks only whether anything is broken. They share
 * this one place so a wheel is never called legal by one and illegal by the other.
 *
 * <p>The "leg" can be a drive, an implicit move to a pose, the straight approach to a FLOWER, the TURN ON THE SPOT of the wiggle, or the retreat.
 */
final class LegSweeps {
    final Sweep.Result bounds;
    final Sweep.Result hive;
    final Sweep.Result side;
    /** One result per FLOWER plate; empty (nothing broken) for the plate of the FLOWER being pressed, which the next three rules judge instead. */
    final Sweep.Result[] flowers;
    final Sweep.Result chassis;
    final Sweep.Result pressing;
    final Sweep.Result otherWheel;
    /** True if the leg was checked against the exact HIVE solid (the lane under the HIVE) instead of the whole keep-out box. */
    final boolean underHive;

    private LegSweeps(Sweep.Result bounds, Sweep.Result hive, Sweep.Result side, Sweep.Result[] flowers, Sweep.Result chassis,
                      Sweep.Result pressing, Sweep.Result otherWheel, boolean underHive) {
        this.bounds = bounds;
        this.hive = hive;
        this.side = side;
        this.flowers = flowers;
        this.chassis = chassis;
        this.pressing = pressing;
        this.otherWheel = otherWheel;
        this.underHive = underHive;
    }

    /**
     * Sweeps every rule over one leg. With {@code press == null} the whole footprint must stay out of every FLOWER plate. With a press, the plate of
     * THAT FLOWER is judged by the pick-up rules instead: the chassis may not come within the clearance of it (even while the robot turns), the pressing
     * wheel may enter it only as far as the stop line, and the other wheel may not enter it at all. Walls, the HIVE keep-out, the other plates and the
     * center line apply to the whole robot as always. {@code underHive} lets the leg use the lane under the HIVE (only the planned path of a DRIVE that
     * says so may).
     */
    static LegSweeps run(final AllianceField f, DrivePath leg, FlowerPress press, final boolean underHive) {
        final HiveSolid solid = f.model().hiveSolid();
        final double laneMargin = BioBuzzConstants.Auto.HIVE_LANE_MARGIN_IN;
        // A wall may be TOUCHED, never entered: one tolerance for every step, wall-contact legs and GARDEN/FLOWER intakes included.
        final double fieldTol = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
        final Rect field = f.fieldRect();
        final Rect hiveKeepOut = f.hiveKeepOut();
        final double eps = PressRules.EPS;

        // Each rule is a "depth": how far the robot is into what it must stay out of (above 0 = rule broken). Sweep tests it at every sample of the
        // leg AND proves nothing slips between two samples (see Sweep).
        Sweep.Result bounds = Sweep.run(leg, PressRules.insideField(field, fieldTol, 0.0));
        Sweep.Result hive = Sweep.run(leg, new Sweep.Depth() {
            @Override
            public double at(Pose2 p) {
                if (underHive) {
                    // the exact solid: the foot bars and the leg stubs, with the lane margin on both sides (the lane itself is open)
                    return laneMargin - solid.signedGap(Footprint.of(p)) - eps;
                }
                return -Footprint.of(p).signedGap(hiveKeepOut) - eps;
            }
        });
        Sweep.Result side = Sweep.run(leg, new Sweep.Depth() {
            @Override
            public double at(Pose2 p) {
                return f.pastCenterLine(p.x) - eps;
            }
        });
        Sweep.Result[] flowers = new Sweep.Result[f.flowerPlates().size()];
        for (int i = 0; i < flowers.length; i++) {
            final Rect keepOut = f.flowerKeepOut(i);
            flowers[i] = (press != null && press.flower == i) ? new Sweep.Result() : Sweep.run(leg, new Sweep.Depth() {
                @Override
                public double at(Pose2 p) {
                    return -Footprint.of(p).signedGap(keepOut) - eps;
                }
            });
        }
        Sweep.Result chassis = new Sweep.Result();
        Sweep.Result pressing = new Sweep.Result();
        Sweep.Result otherWheel = new Sweep.Result();
        if (press != null) {
            // I2: the body stays off the plate (plus the clearance), even while the robot turns with the wheel in the plate's air space
            chassis = Sweep.run(leg, PressRules.chassisOffPlate(f.flowerChassisKeepOut(press.flower), press.spec.intakeReachIn, 0.0));
            // I3: the pressing wheel may be over the plate, but never nearer the wall than the POLLEN's center line
            pressing = Sweep.run(leg, PressRules.wheelShortOfStopLine(press.wheel, f.flowerPressZone(press.flower), 0.0));
            // I3: the other wheel never enters the plate
            otherWheel = Sweep.run(leg, PressRules.wheelOffPlate(press.wheel.other(), f.flowerKeepOut(press.flower), 0.0));
        }
        return new LegSweeps(bounds, hive, side, flowers, chassis, pressing, otherWheel, underHive);
    }

    /**
     * The pose where a sweep gave up because the leg's heading jumps (see {@link Sweep.Result#unprovable}), or null if every sweep of the leg was proven.
     * A jump is a fault of the PATH, whichever rule noticed it, so the first one found is enough.
     */
    Pose2 unprovablePose() {
        java.util.List<Sweep.Result> all = new java.util.ArrayList<Sweep.Result>(java.util.Arrays.asList(bounds, hive, side, chassis, pressing, otherWheel));
        all.addAll(java.util.Arrays.asList(flowers));
        for (Sweep.Result r : all) {
            if (r.unprovable) {
                return r.unprovablePose;
            }
        }
        return null;
    }

    /** True if any rule is broken anywhere on the leg. */
    boolean broken() {
        if (bounds.broken() || hive.broken() || side.broken() || chassis.broken() || pressing.broken() || otherWheel.broken()) {
            return true;
        }
        for (Sweep.Result r : flowers) {
            if (r.broken()) {
                return true;
            }
        }
        return false;
    }

    /** The first broken rule in plain words (for "why is this wheel not allowed"), or an empty string if none is broken. */
    String firstProblem() {
        if (bounds.broken()) {
            return "the robot would leave the field (by up to " + oneDecimal(bounds.worst + BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN) + " in)";
        }
        if (hive.broken()) {
            return "the robot would run into the HIVE keep-out";
        }
        if (side.broken()) {
            return "the robot center would cross the center line into the opponent half (G402)";
        }
        for (int i = 0; i < flowers.length; i++) {
            if (flowers[i].broken()) {
                return "the robot would run into the plate of FLOWER " + (i + 1);
            }
        }
        if (chassis.broken()) {
            return "the chassis would come " + oneDecimal(chassis.worst) + " in too close to the plate";
        }
        if (pressing.broken()) {
            return "the pressing wheel would go " + oneDecimal(pressing.worst) + " in past the stop line";
        }
        if (otherWheel.broken()) {
            return "the other wheel would enter the plate";
        }
        return "";
    }

    private static String oneDecimal(double v) {
        return String.format(java.util.Locale.ROOT, "%.1f", v);
    }
}
