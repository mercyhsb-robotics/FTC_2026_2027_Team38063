package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;

/**
 * Plays a route through, step by step, WITHOUT any physics: where the robot is after each step (the
 * effective pose, which for the tangent heading modes follows from the geometry), how long each step
 * takes ({@link TimingModel}), how many POLLEN the robot holds, which cell of the HIVE is UP, and what
 * the route scores. The validator, the summary methods of {@link AutoRoute} and the diagram renderer
 * all read this one result, so they can never disagree.
 *
 * <p>The route is analysed in the frame of its own alliance ({@link AutoRoute#alliance()}).
 */
public final class RouteAnalysis {

    /** What the simulation knows about one step. */
    public static final class StepInfo {
        public final int index;
        public final Step step;
        /** Robot pose before the step. */
        public final Pose2 from;
        /** Robot pose after the step. */
        public final Pose2 to;
        /** The DRIVE leg's path, else null. */
        public final DrivePath path;
        /**
         * The implicit straight move to the pose of an INTAKE, SHOOT or PARK step (for a FLOWER: to its line-up pose), or
         * null if the robot is already there.
         */
        public final DrivePath approach;
        public final double startSec;
        public final double durationSec;
        public final int pollenBefore;
        public final int pollenAfter;
        public final Side upSideBefore;
        public final Side upSideAfter;

        /** SHOOT: the cell side fired at (the UP side for an UP-cell target). Else null. */
        public final Side targetSide;
        /** SHOOT: true if the targeted cell was UP at that moment. */
        public final boolean targetWasUp;
        /** SHOOT: the shooting pose used (explicit, or the nominal standoff pose for an AUTO pose). Else null. */
        public final Pose2 shootPose;
        /** SHOOT: true if {@link #shootPose} is the nominal pose because the step leaves it to the solver. */
        public final boolean shootPoseIsNominal;
        /** SHOOT: true if this volley tipped the HIVE. */
        public final boolean tipped;
        /** SHOOT: POLLEN in the targeted cell after this volley (0 right after a tip). */
        public final int pollenInCellAfter;

        /** INTAKE: what the robot takes from: "FLOWER n", "own GARDEN", "other GARDEN" or "FLOOR"; null if nothing matched. */
        public final String sourceName;
        /** INTAKE: POLLEN left in the source before this step (-1 if there is no source or it is the floor). */
        public final int sourceRemainingBefore;
        /** INTAKE: true if the source belongs to the other alliance (off limits in AUTO, rule G402). */
        public final boolean sourceOffLimits;

        StepInfo(int index, Step step, Pose2 from, Pose2 to, DrivePath path, DrivePath approach, double startSec,
                 double durationSec, int pollenBefore, int pollenAfter, Side upSideBefore, Side upSideAfter,
                 Side targetSide, boolean targetWasUp, Pose2 shootPose, boolean shootPoseIsNominal, boolean tipped,
                 int pollenInCellAfter, String sourceName, int sourceRemainingBefore, boolean sourceOffLimits) {
            this.index = index;
            this.step = step;
            this.from = from;
            this.to = to;
            this.path = path;
            this.approach = approach;
            this.startSec = startSec;
            this.durationSec = durationSec;
            this.pollenBefore = pollenBefore;
            this.pollenAfter = pollenAfter;
            this.upSideBefore = upSideBefore;
            this.upSideAfter = upSideAfter;
            this.targetSide = targetSide;
            this.targetWasUp = targetWasUp;
            this.shootPose = shootPose;
            this.shootPoseIsNominal = shootPoseIsNominal;
            this.tipped = tipped;
            this.pollenInCellAfter = pollenInCellAfter;
            this.sourceName = sourceName;
            this.sourceRemainingBefore = sourceRemainingBefore;
            this.sourceOffLimits = sourceOffLimits;
        }

        public double endSec() {
            return startSec + durationSec;
        }
    }

    private final AutoRoute route;
    private final AllianceField field;
    private final List<StepInfo> infos;
    private final double totalSeconds;
    private final int pollenGained;
    private final int pollenShot;
    private final int tips;
    private final boolean leave;
    private final boolean park;
    private final double parkOverlapIn;

    private RouteAnalysis(AutoRoute route, AllianceField field, List<StepInfo> infos, double totalSeconds,
                          int pollenGained, int pollenShot, int tips) {
        this.route = route;
        this.field = field;
        this.infos = Collections.unmodifiableList(infos);
        this.totalSeconds = totalSeconds;
        this.pollenGained = pollenGained;
        this.pollenShot = pollenShot;
        this.tips = tips;

        Pose2 end = infos.get(infos.size() - 1).to;
        Footprint fp = Footprint.of(end);
        Rect b = fp.bounds();
        double m = BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN;
        double s = field.fieldRect().maxX;
        this.leave = b.minX >= m && b.minY >= m && b.maxX <= s - m && b.maxY <= s - m;
        Rect clipped = fp.clippedBounds(field.ownLoadingZone());
        this.park = clipped != null;
        this.parkOverlapIn = clipped == null ? 0.0 : Math.min(clipped.maxX - clipped.minX, clipped.maxY - clipped.minY);
    }

    /** Analyses a route with the default field model and timing model. */
    public static RouteAnalysis of(AutoRoute route) {
        return of(route, new FieldModel(), new TimingModel());
    }

    public static RouteAnalysis of(AutoRoute route, FieldModel model, TimingModel timing) {
        Alliance alliance = route.alliance();
        AllianceField field = new AllianceField(model, alliance);
        List<StepInfo> out = new ArrayList<StepInfo>();

        Pose2 pose = route.startPose();
        double t = 0.0;
        int held = BioBuzzConstants.Auto.PRELOAD_POLLEN;
        Side up = field.initialUpSide();
        int inCell = 0;
        int gained = 0;
        int shot = 0;
        int tips = 0;
        int[] flowerLeft = new int[BioBuzzConstants.Field.FLOWER_CENTERS_IN.length];
        for (int i = 0; i < flowerLeft.length; i++) {
            flowerLeft[i] = BioBuzzConstants.Auto.FLOWER_POLLEN;
        }
        int[] gardenLeft = new int[Alliance.values().length];      // indexed by alliance ordinal
        for (int i = 0; i < gardenLeft.length; i++) {
            gardenLeft[i] = BioBuzzConstants.Auto.GARDEN_POLLEN;
        }

        for (int i = 0; i < route.size(); i++) {
            Step step = route.step(i);
            Pose2 from = pose;
            Pose2 to = pose;
            DrivePath path = null;
            DrivePath approach = null;
            double duration = 0.0;
            int heldBefore = held;
            Side upBefore = up;
            Side targetSide = null;
            boolean targetWasUp = false;
            Pose2 shootPose = null;
            boolean nominal = false;
            boolean tipped = false;
            String sourceName = null;
            int sourceLeft = -1;
            boolean offLimits = false;

            switch (step.kind()) {
                case START:
                    to = ((Step.Start) step).pose();
                    break;
                case DRIVE: {
                    Step.Drive d = (Step.Drive) step;
                    path = DrivePath.of(from, d);
                    duration = timing.legSeconds(path, d.speedFraction());
                    to = path.end();
                    break;
                }
                case INTAKE: {
                    Step.Intake in = (Step.Intake) step;
                    // A FLOWER is a press-in: the robot pose is the line-up pose, then the robot presses in and backs out,
                    // so the step ends at the retreat pose and costs the slow approach and retreat as well.
                    FlowerPress press = FlowerPress.of(in, field);
                    approach = approachOrNull(from, in.robotPose());
                    duration = (approach == null ? 0.0 : timing.moveSeconds(from, in.robotPose()))
                            + (press == null ? timing.intakeSeconds(in) : timing.flowerSeconds(in, press));
                    to = press == null ? in.robotPose() : press.retreat;
                    held += in.expectedCount();
                    gained += in.expectedCount();
                    switch (in.source()) {
                        case FLOWER: {
                            int f = field.nearestFlower(in.target());
                            if (f >= 0) {
                                sourceName = "FLOWER " + (f + 1);
                                sourceLeft = flowerLeft[f];
                                offLimits = !field.isOwnFlower(f);
                                flowerLeft[f] = Math.max(0, flowerLeft[f] - in.expectedCount());
                            }
                            break;
                        }
                        case GARDEN: {
                            Alliance g = field.nearestGarden(in.target());
                            if (g != null) {
                                sourceName = g == alliance ? "own GARDEN" : "other GARDEN";
                                sourceLeft = gardenLeft[g.ordinal()];
                                offLimits = g != alliance;
                                gardenLeft[g.ordinal()] = Math.max(0, gardenLeft[g.ordinal()] - in.expectedCount());
                            }
                            break;
                        }
                        default:
                            sourceName = "FLOOR";
                            offLimits = !field.isOwnSide(in.target().x);
                            break;
                    }
                    break;
                }
                case SHOOT: {
                    Step.Shoot sh = (Step.Shoot) step;
                    targetSide = sh.target().isUpCell() ? up : sh.target().fixedSideOrNull();
                    targetWasUp = targetSide == up;
                    if (sh.isAutoPose()) {
                        shootPose = field.nominalShootPose(targetSide);
                        nominal = true;
                    } else {
                        shootPose = sh.poseOrNull();
                    }
                    to = shootPose;
                    approach = approachOrNull(from, to);
                    duration = (approach == null ? 0.0 : timing.moveSeconds(from, to)) + timing.shootSeconds(sh);
                    held -= sh.count();
                    shot += sh.count();
                    if (targetWasUp) {
                        inCell += sh.count();
                        if (inCell >= BioBuzzConstants.Auto.TIP_POLLEN_THRESHOLD) {
                            tipped = true;
                            tips++;
                            up = up.opposite();
                            inCell = 0;
                        }
                    }
                    break;
                }
                case WAIT:
                    duration = timing.waitSeconds((Step.Wait) step);
                    break;
                case PARK: {
                    to = ((Step.Park) step).pose();
                    approach = approachOrNull(from, to);
                    duration = approach == null ? 0.0 : timing.moveSeconds(from, to);
                    break;
                }
                default:
                    throw new IllegalStateException("unhandled step kind " + step.kind());
            }

            out.add(new StepInfo(i, step, from, to, path, approach, t, duration, heldBefore, held, upBefore, up,
                    targetSide, targetWasUp, shootPose, nominal, tipped, inCell, sourceName, sourceLeft, offLimits));
            t += duration;
            pose = to;
        }
        return new RouteAnalysis(route, field, out, t, gained, shot, tips);
    }

    private static DrivePath approachOrNull(Pose2 from, Pose2 to) {
        boolean sameSpot = from.distanceTo(to) < 1e-6;
        boolean sameHeading = Math.abs(Angles.error(from.heading, to.heading)) < 1e-9;
        return sameSpot && sameHeading ? null : DrivePath.straight(from, to);
    }

    // ---- results ---------------------------------------------------------------------------------------

    public AutoRoute route() {
        return route;
    }

    public AllianceField field() {
        return field;
    }

    public List<StepInfo> steps() {
        return infos;
    }

    public StepInfo step(int index) {
        return infos.get(index);
    }

    /** Estimated duration of the whole route in seconds. */
    public double totalSeconds() {
        return totalSeconds;
    }

    /** The robot pose after the last step. */
    public Pose2 endPose() {
        return infos.get(infos.size() - 1).to;
    }

    /** POLLEN the route takes in (sum of the INTAKE counts). */
    public int pollenGained() {
        return pollenGained;
    }

    /** POLLEN the route fires (sum of the SHOOT counts). */
    public int pollenShot() {
        return pollenShot;
    }

    /** POLLEN held after the last step (starts at the 4 pre-loads). */
    public int pollenEnd() {
        return infos.get(infos.size() - 1).pollenAfter;
    }

    /** Net change in POLLEN held over the route: taken in minus fired. */
    public int pollenBalance() {
        return pollenGained - pollenShot;
    }

    /** Number of HIVE TIPs the shots reach (each needs {@code TIP_POLLEN_THRESHOLD} POLLEN into the UP cell). */
    public int tips() {
        return tips;
    }

    /** LEAVE: the whole robot ends off every wall by the margin. */
    public boolean leave() {
        return leave;
    }

    /** PARK: the robot ends overlapping the own LOADING ZONE. */
    public boolean park() {
        return park;
    }

    /** How far into the zone the end pose reaches (smaller side of the overlap box, inches); 0 if not parked. */
    public double parkOverlapIn() {
        return parkOverlapIn;
    }

    public int leavePoints() {
        return leave ? BioBuzzConstants.Auto.LEAVE_POINTS : 0;
    }

    public int parkPoints() {
        return park ? BioBuzzConstants.Auto.PARK_POINTS : 0;
    }

    public int tipPoints() {
        return tips * BioBuzzConstants.Auto.TIP_POINTS;
    }

    /**
     * The AUTO points the route can claim if everything goes to plan: LEAVE and PARK from the end pose,
     * 20 per HIVE TIP its shots reach. It assumes every POLLEN lands and the robot is not late, so it is a
     * ceiling, not a forecast.
     */
    public int points() {
        return leavePoints() + parkPoints() + tipPoints();
    }
}
