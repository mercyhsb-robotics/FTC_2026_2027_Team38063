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
        /** SHOOT: true if the targeted cell was UP at the moment the first POLLEN left ({@link #launchSec}). */
        public final boolean targetWasUp;
        /** SHOOT: the shooting pose used (explicit, or the nominal standoff pose for an AUTO pose). Else null. */
        public final Pose2 shootPose;
        /** SHOOT: true if {@link #shootPose} is the nominal pose because the step leaves it to the solver. */
        public final boolean shootPoseIsNominal;
        /** SHOOT: true if this volley tipped the HIVE. */
        public final boolean tipped;
        /** SHOOT: POLLEN in the targeted cell after this volley (0 right after a tip). */
        public final int pollenInCellAfter;
        /** SHOOT: route time in seconds at which the first POLLEN leaves the shooter (after the move to the pose); NaN otherwise. */
        public final double launchSec;
        /** SHOOT that tipped the HIVE: route time in seconds of the POLLEN that tipped it; NaN otherwise. */
        public final double tipSec;
        /**
         * SHOOT that tipped the HIVE: POLLEN of the same volley that are still launched AFTER the tip, i.e. into a cell that
         * is already tipping down. A volley that is gated by the camera (the normal one) stops at the tip and keeps the rest,
         * so this is 0 for it; only a TIMED volley (an old OpMode that feeds for a fixed time) cannot stop and fires them
         * all. 0 when the last POLLEN of the volley is the one that tips.
         */
        public final int firedAfterTip;
        /**
         * SHOOT that tipped the HIVE: how many POLLEN of this volley the cell needed (the one that tips it is the
         * {@code tipNeeded}-th); 0 for a volley that did not tip. The POLLEN of earlier volleys into the same cell count.
         */
        public final int tipNeeded;
        /**
         * SHOOT that tipped the HIVE: how many POLLEN the volley plans beyond what the cell needed, the route's SPARE (a
         * field that is stiffer by that many POLLEN still tips). The camera-gated volley keeps the spare in the robot. 0
         * means the plan has exactly the minimum; 0 also for a volley that did not tip.
         */
        public final int tipSpare;
        /** SHOOT: NECTAR sitting in the targeted cell when the volley starts (3 in the starting UP cell, else 0). */
        public final int nectarInCellBefore;
        /** SHOOT: POLLEN of this route that are already in the targeted cell when the volley starts (0 for a cell that is DOWN). */
        public final int pollenInCellBefore;
        /**
         * SHOOT: seconds the robot stands at its pose before the first POLLEN leaves, waiting for its camera to confirm that OUR OWN
         * earlier tip is complete (the camera must see the cell that came up steady for the settle time, {@code RouteAnalysis.tipSettleSec()});
         * 0 if the robot was busy long enough anyway, or if the cell did not come up with a tip of ours.
         */
        public final double confirmWaitSec;
        /**
         * SHOOT: true if this volley goes into a cell that is UP only because the HIVE has tipped (ours or the partner's), or that
         * will come up with the partner's assumed tip. On the robot such a volley needs the CAMERA to confirm the tip first and is
         * skipped when the camera cannot ({@link #giveUpPlan()}); a volley at the cell that is UP at the start is not tip-dependent.
         */
        public final boolean tipDependent;
        /** True in a {@link #giveUpPlan()} for a volley that the robot skips because the tip it depends on is never confirmed. */
        public final boolean skipped;
        /**
         * WAIT_FOR_TIP: the route time in seconds at which the partner's tip is ASSUMED to happen (it is only a planning
         * guess: the robot goes on when its camera sees the tip); NaN when there is no tip to wait for (a route that
         * assumes SELF, or the HIVE has tipped already). Else NaN.
         */
        public final double tipWaitSec;

        /** INTAKE: what the robot takes from: "FLOWER n", "own GARDEN", "other GARDEN" or "FLOOR"; null if nothing matched. */
        public final String sourceName;
        /** INTAKE: POLLEN left in the source before this step (-1 if there is no source or it is the floor). */
        public final int sourceRemainingBefore;
        /** INTAKE: true if the source belongs to the other alliance (off limits in AUTO, rule G402). */
        public final boolean sourceOffLimits;
        /**
         * FLOWER INTAKE: the pick-up the robot makes, with the wheel {@link WheelChoice} chose for it (see {@link FlowerPress}); null for any other
         * step, and for a FLOWER step whose target point is near no FLOWER. The validator, the renderer and the timing all read this one object.
         */
        public final FlowerPress press;
        /** FLOWER INTAKE: how the wheel was chosen (every option with its cost and legality); null when {@link #press} is null. */
        public final WheelChoice.Choice choice;
        /** FLOWER INTAKE: seconds of the pick-up itself (slow approach, the wiggle cycles and the slow retreat), without the move to the line-up pose; 0 otherwise. */
        public final double pickupSec;

        StepInfo(int index, Step step, Pose2 from, Pose2 to, DrivePath path, DrivePath approach, double startSec,
                 double durationSec, int pollenBefore, int pollenAfter, Side upSideBefore, Side upSideAfter,
                 Side targetSide, boolean targetWasUp, Pose2 shootPose, boolean shootPoseIsNominal, boolean tipped,
                 int pollenInCellAfter, String sourceName, int sourceRemainingBefore, boolean sourceOffLimits,
                 double launchSec, double tipSec, int firedAfterTip, int nectarInCellBefore, double tipWaitSec,
                 int pollenInCellBefore, int tipNeeded, int tipSpare, double confirmWaitSec, boolean tipDependent, boolean skipped,
                 FlowerPress press, WheelChoice.Choice choice, double pickupSec) {
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
            this.launchSec = launchSec;
            this.tipSec = tipSec;
            this.firedAfterTip = firedAfterTip;
            this.tipNeeded = tipNeeded;
            this.tipSpare = tipSpare;
            this.confirmWaitSec = confirmWaitSec;
            this.tipDependent = tipDependent;
            this.skipped = skipped;
            this.nectarInCellBefore = nectarInCellBefore;
            this.tipWaitSec = tipWaitSec;
            this.pollenInCellBefore = pollenInCellBefore;
            this.press = press;
            this.choice = choice;
            this.pickupSec = pickupSec;
        }

        /**
         * FLOWER INTAKE in plain words for the gallery and the diagram: "FLOWER pick-up: 4 POLLEN in about 5.2 s with the LEFT wheel", and how the wheel was
         * chosen. An empty string for any other step.
         */
        public String pickupText() {
            if (press == null) {
                return "";
            }
            return "FLOWER pick-up: " + ((Step.Intake) step).expectedCount() + " POLLEN in about " + String.format(java.util.Locale.ROOT, "%.1f", pickupSec)
                    + " s with the " + press.wheel + " wheel";
        }

        public double endSec() {
            return startSec + durationSec;
        }
    }

    /** One HIVE tip during the route: when, which cell tipped down, and who caused it. */
    public static final class TipEvent {
        /** Route time in seconds at which the tip happens. */
        public final double timeSec;
        /** The cell that WAS up and tips down now: what it holds spills out on its side of the HIVE (rule G409). */
        public final Side releasingSide;
        /** The cell that is UP from now on. */
        public final Side newUpSide;
        /** True for the alliance partner's assumed tip, false for a tip caused by a volley of this route. */
        public final boolean byPartner;
        /** The SHOOT step that tipped the HIVE; -1 for the partner's tip. */
        public final int stepIndex;

        TipEvent(double timeSec, Side releasingSide, Side newUpSide, boolean byPartner, int stepIndex) {
            this.timeSec = timeSec;
            this.releasingSide = releasingSide;
            this.newUpSide = newUpSide;
            this.byPartner = byPartner;
            this.stepIndex = stepIndex;
        }
    }

    /**
     * The HIVE while a route is played: which cell is UP, how many NECTAR it holds (only the cell that is UP at the
     * start holds the staged ones), how many of OUR POLLEN went into it since it came up, and the partner's tip if
     * the route assumes one ({@link FirstTip#PARTNER}) that has not happened yet.
     */
    private static final class Hive {
        Side up;
        int nectar = BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL;
        int pollen;
        final double partnerSec;
        boolean partnerPending;
        boolean partnerVoided;
        /** Route time at which the camera has confirmed OUR latest tip (its time plus the settle time); NaN if we have not tipped it. */
        double ownConfirmedSec = Double.NaN;
        final List<TipEvent> events = new ArrayList<TipEvent>();

        Hive(Side up, double partnerSec) {
            this.up = up;
            this.partnerSec = partnerSec;
            this.partnerPending = !Double.isNaN(partnerSec);
        }

        /** Lets time pass: the partner's tip happens once the clock reaches it (a hair of tolerance for rounding). */
        void advanceTo(double sec) {
            if (partnerPending && sec >= partnerSec - 1e-9) {
                events.add(new TipEvent(partnerSec, up, up.opposite(), true, -1));
                flip();
                partnerPending = false;
                ownConfirmedSec = Double.NaN;      // the partner's tip is confirmed by a WAIT_FOR_TIP, not by us
            }
        }

        /** A tip: the other cell comes up empty (Fig 10-2 shows NECTAR only in the cell that is UP at the start). */
        void flip() {
            up = up.opposite();
            nectar = 0;
            pollen = 0;
        }
    }

    private final AutoRoute route;
    private final AllianceField field;
    private final TimingModel timing;
    private final List<StepInfo> infos;
    private final double totalSeconds;
    private final int pollenGained;
    private final int pollenShot;
    private final int tips;
    private final List<TipEvent> tipEvents;
    private final boolean partnerTipVoided;
    private final double partnerTipSec;
    private final boolean leave;
    private final boolean park;
    private final double parkOverlapIn;
    private final double endWallGapIn;
    private final String endNearestWall;
    /** The plan for "the tip is never confirmed", built the first time it is asked for (see {@link #giveUpPlan()}). */
    private RouteAnalysis giveUp;
    private boolean giveUpBuilt;

    private RouteAnalysis(AutoRoute route, AllianceField field, TimingModel timing, List<StepInfo> infos, double totalSeconds,
                          int pollenGained, int pollenShot, int tips, Hive hive, double partnerTipSec) {
        this.route = route;
        this.partnerTipSec = partnerTipSec;
        this.field = field;
        this.timing = timing;
        this.infos = Collections.unmodifiableList(infos);
        this.totalSeconds = totalSeconds;
        this.pollenGained = pollenGained;
        this.pollenShot = pollenShot;
        this.tips = tips;
        this.tipEvents = Collections.unmodifiableList(new ArrayList<TipEvent>(hive.events));
        this.partnerTipVoided = hive.partnerVoided;

        Pose2 end = infos.get(infos.size() - 1).to;
        Footprint fp = Footprint.of(end);
        Rect b = fp.bounds();
        double m = BioBuzzConstants.Auto.LEAVE_WALL_MARGIN_IN;
        double s = field.fieldRect().maxX;
        this.leave = b.minX >= m && b.minY >= m && b.maxX <= s - m && b.maxY <= s - m;
        // how far the robot's edge is from each wall: the red (left) wall, the blue (right) wall, the audience wall, the rear wall
        double[] gaps = {b.minX, s - b.maxX, b.minY, s - b.maxY};
        String[] walls = {"red wall (x = 0)", "blue wall (x = " + String.format(java.util.Locale.ROOT, "%.1f", s) + ")", "audience wall (y = 0)",
                "rear wall (y = " + String.format(java.util.Locale.ROOT, "%.1f", s) + ")"};
        int nearest = 0;
        for (int i = 1; i < gaps.length; i++) {
            nearest = gaps[i] < gaps[nearest] ? i : nearest;
        }
        this.endWallGapIn = gaps[nearest];
        this.endNearestWall = walls[nearest];
        Rect clipped = fp.clippedBounds(field.ownLoadingZone());
        this.park = clipped != null;
        this.parkOverlapIn = clipped == null ? 0.0 : Math.min(clipped.maxX - clipped.minX, clipped.maxY - clipped.minY);
    }

    /** Analyses a route with the default field model and timing model. */
    public static RouteAnalysis of(AutoRoute route) {
        return of(route, new FieldModel(), new TimingModel());
    }

    public static RouteAnalysis of(AutoRoute route, FieldModel model, TimingModel timing) {
        return of(route, model, timing, route.partnerTipSec());
    }

    /**
     * A "what if": the route played through with the partner's tip at {@code partnerTipSec} instead of the assumed
     * {@code Auto.PARTNER_FIRST_TIP_DONE_SEC} (NaN: nobody tips the HIVE by itself). The numbers for "how late may the
     * partner be?" ({@link #latestPartnerTipSec()}) are checked against this in the tests.
     */
    public static RouteAnalysis ofWithPartnerTipAt(AutoRoute route, double partnerTipSec) {
        return of(route, new FieldModel(), new TimingModel(), partnerTipSec);
    }

    private static RouteAnalysis of(AutoRoute route, FieldModel model, TimingModel timing, double partnerTipSec) {
        return play(route, model, timing, partnerTipSec, null, null);
    }

    /**
     * Plays the route through. With {@code skip == null} that is the plan. With {@code skip} (and {@code waitUntil}) it is the
     * plan for "the tip is never confirmed": a step with {@code skip[i]} is a volley the robot leaves out, a WAIT_FOR_TIP lasts until
     * {@code waitUntil[i]} on the route clock (its give-up time), and an INTAKE takes only what fits in the robot (it stops at 4, G407).
     */
    private static RouteAnalysis play(AutoRoute route, FieldModel model, TimingModel timing, double partnerTipSec, boolean[] skip,
                                      double[] waitUntil) {
        Alliance alliance = route.alliance();
        AllianceField field = new AllianceField(model, alliance);
        List<StepInfo> out = new ArrayList<StepInfo>();

        Pose2 pose = route.startPose();
        double t = 0.0;
        int held = BioBuzzConstants.Auto.PRELOAD_POLLEN;
        // The HIVE: which cell is UP, what it holds, and the partner's tip if the route assumes one.
        Hive hive = new Hive(field.initialUpSide(), partnerTipSec);
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
            hive.advanceTo(t);      // the partner's tip happens when its time comes, whatever this robot is doing
            Step step = route.step(i);
            if (skip != null && skip[i]) {
                // a volley that needs a tip the robot never confirms: it stays where it is and goes on with the next step
                out.add(new StepInfo(i, step, pose, pose, null, null, t, 0.0, held, held, hive.up, hive.up, null, false, null, false,
                        false, hive.pollen, null, -1, false, Double.NaN, Double.NaN, 0, 0, Double.NaN, 0, 0, 0, 0.0, true, true, null, null, 0.0));
                continue;
            }
            Pose2 from = pose;
            Pose2 to = pose;
            DrivePath path = null;
            DrivePath approach = null;
            double duration = 0.0;
            int heldBefore = held;
            Side upBefore = hive.up;
            Side targetSide = null;
            boolean targetWasUp = false;
            Pose2 shootPose = null;
            boolean nominal = false;
            boolean tipped = false;
            String sourceName = null;
            int sourceLeft = -1;
            boolean offLimits = false;
            double launchSec = Double.NaN;
            double tipSec = Double.NaN;
            int firedAfterTip = 0;
            int tipNeeded = 0;
            int tipSpare = 0;
            double confirmWait = 0.0;
            boolean tipDependent = false;
            int nectarBefore = 0;
            int pollenBeforeInCell = 0;
            double tipWaitSec = Double.NaN;
            FlowerPress press = null;
            WheelChoice.Choice choice = null;
            double pickupSec = 0.0;

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
                    int flowerIndex = in.source() == IntakeSource.FLOWER ? field.nearestFlower(in.target()) : -1;
                    if (flowerIndex >= 0) {
                        // A FLOWER is a wiggle pick-up: the robot goes to a line-up pose it works out itself (the wheel is chosen by cost, looking at the legs
                        // before and after), drives in, wiggles and backs out, so the step ends at the retreat pose.
                        choice = WheelChoice.choose(field, timing, in, flowerIndex, from, lookahead(route, i, field, hive));
                        press = choice.press();
                        pickupSec = timing.flowerSeconds(press);
                        approach = approachOrNull(from, press.lineUp);
                        duration = (approach == null ? 0.0 : timing.moveSeconds(from, press.lineUp)) + pickupSec;
                        to = press.retreat;
                    } else if (in.source() == IntakeSource.FLOWER) {
                        // the target point is near no FLOWER: there is nothing to take and nowhere to go (the validator warns)
                        to = from;
                    } else {
                        approach = approachOrNull(from, in.robotPose());
                        duration = (approach == null ? 0.0 : timing.moveSeconds(from, in.robotPose())) + timing.intakeSeconds(in);
                        to = in.robotPose();
                    }
                    // in the give-up plan a volley was left out, so the robot may hold more than planned: it stops taking at 4 (G407)
                    int taken = skip == null ? in.expectedCount() : Math.min(in.expectedCount(), Math.max(0, BioBuzzConstants.Auto.POLLEN_CAPACITY - held));
                    held += taken;
                    gained += taken;
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
                    // "The UP cell" is the one that is UP when the step starts (that is what the solver sees and drives
                    // to); a fixed side is the cell the route names.
                    targetSide = sh.target().isUpCell() ? hive.up : sh.target().fixedSideOrNull();
                    if (sh.isAutoPose()) {
                        shootPose = field.nominalShootPose(targetSide);
                        nominal = true;
                    } else {
                        shootPose = sh.poseOrNull();
                    }
                    to = shootPose;
                    approach = approachOrNull(from, to);
                    double approachSec = approach == null ? 0.0 : timing.moveSeconds(from, to);
                    double shootSec = timing.shootSeconds(sh);
                    launchSec = t + approachSec;
                    // Whether that cell is still UP when the first POLLEN leaves decides if the launch is legal (G417):
                    // the partner's tip may happen while the robot is still driving to the pose.
                    hive.advanceTo(launchSec);
                    // A cell that came up with OUR own tip is shot at only after the camera has seen it steady for the settle time,
                    // never because a time has passed: the robot stands at its pose until then.
                    if (targetSide == hive.up && !Double.isNaN(hive.ownConfirmedSec) && launchSec < hive.ownConfirmedSec) {
                        confirmWait = hive.ownConfirmedSec - launchSec;
                        launchSec = hive.ownConfirmedSec;
                        hive.advanceTo(launchSec);
                    }
                    targetWasUp = targetSide == hive.up;
                    // The volley needs the camera to confirm a tip when its cell is up only because the HIVE has tipped, or is the
                    // one that comes up with the partner's tip that is still to come.
                    tipDependent = (targetWasUp && !hive.events.isEmpty()) || (hive.partnerPending && targetSide == hive.up.opposite());
                    nectarBefore = targetWasUp ? hive.nectar : 0;
                    pollenBeforeInCell = targetWasUp ? hive.pollen : 0;
                    int fired = sh.count();
                    if (targetWasUp) {
                        // How many POLLEN of THIS volley does the cell need? (see TipModel for the arithmetic.) At
                        // least 1: a cell cannot tip before something is launched into it.
                        int needed = Math.max(1, TipModel.pollenStillNeeded(hive.nectar, hive.pollen));
                        // The POLLEN leave one every timePerShot after the aim and spin-up; the tip happens when the
                        // needed one lands.
                        double wouldTipAt = launchSec + timing.shootSeconds(sh, needed);
                        boolean partnerFirst = hive.partnerPending && hive.partnerSec <= wouldTipAt;
                        if (sh.count() >= needed && !partnerFirst) {
                            tipped = true;
                            tips++;
                            tipSec = wouldTipAt;
                            tipNeeded = needed;
                            tipSpare = sh.count() - needed;
                            if (sh.hasFixedSeconds()) {
                                // A timed volley cannot stop: it goes on launching into the cell that is tipping down.
                                firedAfterTip = tipSpare;
                            } else {
                                // A camera-gated volley stops the moment the camera sees the tip (the cell's tags vanish and
                                // the feeder pauses, G417), so only the needed POLLEN leave and the spare stays in the robot.
                                fired = needed;
                                shootSec = timing.shootSeconds(sh, needed);
                            }
                            hive.events.add(new TipEvent(tipSec, hive.up, hive.up.opposite(), false, i));
                            hive.ownConfirmedSec = tipSec + timing.tipSettleSec();
                            if (hive.partnerPending) {
                                // we tipped the HIVE before the partner did: the partner's tip is no longer coming
                                hive.partnerPending = false;
                                hive.partnerVoided = true;
                            }
                            hive.flip();
                        } else {
                            hive.pollen += sh.count();
                        }
                    }
                    duration = approachSec + confirmWait + shootSec;
                    held -= fired;
                    shot += fired;
                    break;
                }
                case WAIT:
                    duration = timing.waitSeconds((Step.Wait) step);
                    break;
                case WAIT_FOR_TIP: {
                    // Wait for the partner's tip if one is still to come; with nothing to wait for the cell is UP
                    // already and the wait takes no time.
                    tipWaitSec = hive.partnerPending ? hive.partnerSec : Double.NaN;
                    duration = timing.waitForTipSeconds(t, tipWaitSec, ((Step.WaitForTip) step).giveUpSec());
                    if (waitUntil != null && !Double.isNaN(waitUntil[i])) {
                        // the tip never comes: the robot waits until its deadline and then goes on
                        duration = Math.max(0.0, waitUntil[i] - t);
                    }
                    break;
                }
                case PARK: {
                    to = ((Step.Park) step).pose();
                    approach = approachOrNull(from, to);
                    duration = approach == null ? 0.0 : timing.moveSeconds(from, to);
                    break;
                }
                default:
                    throw new IllegalStateException("unhandled step kind " + step.kind());
            }

            hive.advanceTo(t + duration);
            out.add(new StepInfo(i, step, from, to, path, approach, t, duration, heldBefore, held, upBefore, hive.up,
                    targetSide, targetWasUp, shootPose, nominal, tipped, hive.pollen, sourceName, sourceLeft, offLimits,
                    launchSec, tipSec, firedAfterTip, nectarBefore, tipWaitSec, pollenBeforeInCell, tipNeeded, tipSpare, confirmWait,
                    tipDependent, false, press, choice, pickupSec));
            t += duration;
            pose = to;
        }
        // A partner tip that is assumed after the route has ended still happens (the robot stands where it ended).
        hive.advanceTo(Double.POSITIVE_INFINITY);
        return new RouteAnalysis(route, field, timing, out, t, gained, shot, tips, hive, partnerTipSec);
    }

    /**
     * What the robot does right after step {@code i}, for the wheel choice of a FLOWER pick-up: the next step that moves the robot (a WAIT does not). A SHOOT
     * that leaves its pose to the solver counts with the nominal pose in front of the cell that is UP now.
     */
    private static WheelChoice.Lookahead lookahead(AutoRoute route, int i, AllianceField field, Hive hive) {
        for (int j = i + 1; j < route.size(); j++) {
            Step next = route.step(j);
            switch (next.kind()) {
                case WAIT:
                case WAIT_FOR_TIP:
                    continue;
                case DRIVE:
                    return WheelChoice.Lookahead.drive((Step.Drive) next);
                case PARK:
                    return WheelChoice.Lookahead.pose(((Step.Park) next).pose());
                case INTAKE: {
                    Step.Intake n = (Step.Intake) next;
                    return n.source() == IntakeSource.FLOWER ? WheelChoice.Lookahead.flower(n) : WheelChoice.Lookahead.pose(n.robotPose());
                }
                case SHOOT: {
                    Step.Shoot sh = (Step.Shoot) next;
                    if (sh.poseOrNull() != null) {
                        return WheelChoice.Lookahead.pose(sh.poseOrNull());
                    }
                    Side side = sh.target().isUpCell() ? hive.up : sh.target().fixedSideOrNull();
                    return WheelChoice.Lookahead.pose(field.nominalShootPose(side));
                }
                default:
                    return WheelChoice.Lookahead.none();
            }
        }
        return WheelChoice.Lookahead.none();
    }

    private static DrivePath approachOrNull(Pose2 from, Pose2 to) {
        boolean sameSpot = from.distanceTo(to) < 1e-6;
        boolean sameHeading = Math.abs(Angles.error(from.heading, to.heading)) < 1e-9;
        return sameSpot && sameHeading ? null : DrivePath.straight(from, to);
    }

    // ---- where is the robot at a given time? -------------------------------------------------------------

    private static final double TINY = 1e-9;

    /**
     * The poses the robot has during one step between two route times (both inclusive), for checks that ask "where is
     * the robot in the 3 seconds after the HIVE tips?". Empty if the step is not running in that window.
     *
     * <p>While the robot drives, its place follows the speed profile of the {@link TimingModel} (speed up, cruise,
     * brake), sampled every {@code PATH_SAMPLE_STEP_IN}; the turn on the spot at the start of a leg is sampled too;
     * after the move the robot stands at the end pose of the step (settling, taking POLLEN, shooting, waiting).
     */
    public List<Pose2> posesDuring(StepInfo info, double fromSec, double toSec) {
        List<Pose2> out = new ArrayList<Pose2>();
        double a = Math.max(fromSec, info.startSec);
        double b = Math.min(toSec, info.endSec());
        if (a > b + TINY) {
            return out;
        }
        DrivePath leg = info.path != null ? info.path : info.approach;
        if (leg == null) {
            out.add(info.to);
            return out;
        }
        double speed = info.step instanceof Step.Drive ? ((Step.Drive) info.step).speedFraction() : 1.0;
        double travel = timing.legTravelSeconds(leg, speed);
        double tauA = a - info.startSec;
        double tauB = b - info.startSec;
        if (tauA <= TINY) {
            out.add(info.from);
            out.addAll(leg.turnInPlaceSamples(Math.toRadians(BioBuzzConstants.Auto.TURN_SAMPLE_STEP_DEG)));
        }
        if (tauA < travel) {
            double arcA = arcAfter(leg, speed, travel, tauA);
            double arcB = arcAfter(leg, speed, travel, Math.min(tauB, travel));
            for (double s = arcA; s < arcB; s += BioBuzzConstants.Auto.PATH_SAMPLE_STEP_IN) {
                out.add(leg.poseAt(s));
            }
            out.add(leg.poseAt(arcB));
        }
        if (tauB >= travel - TINY) {
            out.add(info.to);
        }
        return out;
    }

    /** Arc length reached {@code tau} seconds into a leg whose movement takes {@code travelSec}, turning included. */
    private double arcAfter(DrivePath leg, double speed, double travelSec, double tau) {
        double driveSec = timing.translationSeconds(leg.length(), speed);
        if (travelSec <= TINY || driveSec <= TINY) {
            return tau <= TINY ? 0.0 : leg.length();
        }
        // when turning takes longer than driving, the driving is spread over the whole travel time
        return timing.distanceAfter(leg.length(), speed, tau * driveSec / travelSec);
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

    /**
     * Number of HIVE TIPs the route's own volleys cause (the alliance partner's assumed tip is NOT among them, see
     * {@link #partnerTips()}). A cell tips by the rule of {@link TipModel} (FIRST's calibration): the first tip needs 3
     * POLLEN (the 3 staged NECTAR count as 5 POLLEN-equivalents), every later tip needs 8 POLLEN into the cell that came up.
     */
    public int tips() {
        return tips;
    }

    /**
     * The smallest spare of the route's own tips: for every volley that tips the HIVE, how many POLLEN it plans beyond the
     * minimum ({@link StepInfo#tipSpare}); the smallest of them, because the route is only as safe as its tightest tip.
     * 0 means a tip with exactly the minimum (a field that needs one more POLLEN would not tip, the validator WARNs); -1
     * if the route tips nothing itself.
     */
    public int minimumTipSpare() {
        int smallest = -1;
        for (StepInfo info : infos) {
            if (info.tipped && (smallest < 0 || info.tipSpare < smallest)) {
                smallest = info.tipSpare;
            }
        }
        return smallest;
    }

    /**
     * Every HIVE tip of the route in time order: the ones this route's volleys cause and the partner's assumed tip
     * (if the route assumes one and our own volleys did not tip the HIVE before it).
     */
    public List<TipEvent> tipEvents() {
        return tipEvents;
    }

    /** True if the route assumes the alliance partner shoots the first tip ({@link FirstTip#PARTNER}). */
    public boolean partnerTipAssumed() {
        return !Double.isNaN(partnerTipSec);
    }

    /** Route time in seconds at which the partner's tip is assumed to be done; NaN if the route assumes SELF. */
    public double partnerTipSec() {
        return partnerTipSec;
    }

    /**
     * How long, in seconds, the camera must see the tipped cell's tags steadily before the robot calls the HIVE FULLY tipped
     * ({@code Vision.HIVE_STABLE_SEEN_SEC}). A tip is complete only when the damper touches the frame, and launching at a
     * HIVE that is still rocking can spoil it (G417), so the robot goes on this long AFTER the tip, never at it.
     */
    public double tipSettleSec() {
        return timing.tipSettleSec();
    }

    /**
     * Route time in seconds at which the robot would be SURE the partner's tip is done: the assumed tip plus the settle
     * time. This, not the tip itself, is when a WAIT_FOR_TIP ends and the first volley at the cell that comes up may start.
     * NaN if the route assumes SELF.
     */
    public double partnerTipConfirmedSec() {
        return Double.isNaN(partnerTipSec) ? Double.NaN : timing.confirmedAtSec(partnerTipSec);
    }

    /** The last moment, on the route clock, by which the route must be over: the AUTO limit minus the safety margin. */
    public static double timeBudgetSec() {
        return BioBuzzConstants.Auto.AUTO_DURATION_SEC - BioBuzzConstants.Auto.TIME_SAFETY_MARGIN_SEC;
    }

    /**
     * The steps (0-based indexes) that are volleys which need the camera to confirm a tip first: a SHOOT at the cell that is UP only
     * because the HIVE has tipped (the partner's assumed tip or one of ours), or at the cell that comes up with the partner's tip.
     * On the robot each of them waits for the camera to see the cell UP and steady, and is skipped when the camera cannot.
     */
    public List<Integer> tipDependentSteps() {
        List<Integer> out = new ArrayList<Integer>();
        for (StepInfo info : infos) {
            if (info.step instanceof Step.Shoot && info.tipDependent) {
                out.add(info.index);
            }
        }
        return out;
    }

    /** True if some volley of the route needs the camera to confirm a tip first ({@link #tipDependentSteps()} is not empty). */
    public boolean dependsOnTipConfirmation() {
        return !tipDependentSteps().isEmpty();
    }

    /**
     * WAIT_FOR_TIP: the moment on the route clock at which the robot really stops waiting if the tip has not been confirmed: the
     * give-up time of the step, or the latest time the rest of the route allows ({@link #latestGiveUpSec}) if that is earlier. The
     * robot skips the tip-dependent volleys then and goes on with the safe steps (LEAVE, PARK). NaN for any other step.
     */
    public double effectiveGiveUpSec(StepInfo wait) {
        if (!(wait.step instanceof Step.WaitForTip)) {
            return Double.NaN;
        }
        return Math.min(((Step.WaitForTip) wait.step).giveUpSec(), latestGiveUpSec(wait));
    }

    /**
     * What the route does when the tip it depends on is NEVER confirmed (the partner is too late or never tips, or the camera is dead,
     * missing or on the wrong pipeline): every WAIT_FOR_TIP waits until {@link #effectiveGiveUpSec}, every tip-dependent volley is left
     * out ("no camera, no shot"), and the robot carries on with the other steps (an INTAKE takes only what fits in the robot, G407),
     * which should end with the safe steps (LEAVE, PARK). It is a worst case in time: a robot that knows its camera is dead at the
     * start would not wait. {@code null} if no volley needs a confirmed tip.
     */
    public RouteAnalysis giveUpPlan() {
        if (!giveUpBuilt) {
            giveUpBuilt = true;
            boolean[] skip = new boolean[infos.size()];
            double[] waitUntil = new double[infos.size()];
            boolean any = false;
            for (StepInfo info : infos) {
                skip[info.index] = info.step instanceof Step.Shoot && info.tipDependent;
                any |= skip[info.index];
                waitUntil[info.index] = partnerTipAssumed() ? effectiveGiveUpSec(info) : Double.NaN;
            }
            giveUp = any ? play(route, field.model(), timing, Double.NaN, skip, waitUntil) : null;
        }
        return giveUp;
    }

    /**
     * WAIT_FOR_TIP: the latest route time, in seconds, at which the robot may stop waiting and still do the rest of
     * the route inside {@link #timeBudgetSec()}: the budget minus what the steps after the wait take. A give-up time
     * later than this could make the route overrun the AUTO limit. NaN for any other step.
     */
    public double latestGiveUpSec(StepInfo wait) {
        if (!(wait.step instanceof Step.WaitForTip)) {
            return Double.NaN;
        }
        return timeBudgetSec() - (totalSeconds - wait.endSec());
    }

    /**
     * The latest time, in seconds on the route clock, at which the partner's tip may happen so that this route still
     * does everything it plans: the robot has not given up waiting yet, and the whole route (LEAVE and PARK after the
     * volleys included) still ends inside {@link #timeBudgetSec()}. It is how much a slow partner can be tolerated.
     *
     * <p>Why a formula and not a search: the steps after a WAIT_FOR_TIP take the same time whenever the tip comes, and
     * the wait ends when the camera has seen the tip steadily for the settle time ({@code RouteAnalysis.tipSettleSec()}) or at its
     * give-up time, so a tip at T ends the route at {@code max(wait start, T + settle) + the time of the rest}. That is
     * within the budget until T + settle = budget - rest, and the robot gives up at the deadline of the wait. The
     * smaller of the two, minus the settle time, is the answer.
     *
     * <p>{@code +infinity}: the route does not depend on the partner's tip at all (no volley at the cell that comes up,
     * no wait for it), or our own volley tipped the HIVE first. NaN: the route does not assume a partner tip. A route
     * with a volley at the cell that comes up but no WAIT_FOR_TIP before it (that is an ERROR in the checker) can only
     * cope with a tip before the volley starts, so the answer is that start time.
     */
    public double latestPartnerTipSec() {
        if (!partnerTipAssumed()) {
            return Double.NaN;
        }
        TipEvent partner = null;
        for (TipEvent e : tipEvents) {
            if (e.byPartner) {
                partner = e;
            }
        }
        if (partner == null) {
            return Double.POSITIVE_INFINITY;
        }
        StepInfo wait = null;
        StepInfo dependent = null;
        for (StepInfo info : infos) {
            if (wait == null && info.step instanceof Step.WaitForTip) {
                wait = info;
            }
            if (dependent == null && info.step instanceof Step.Shoot && info.targetSide == partner.newUpSide) {
                dependent = info;
            }
        }
        if (wait == null) {
            return dependent == null ? Double.POSITIVE_INFINITY : dependent.launchSec - timing.tipSettleSec();
        }
        double giveUp = ((Step.WaitForTip) wait.step).giveUpSec();
        return Math.min(giveUp, latestGiveUpSec(wait)) - timing.tipSettleSec();
    }

    /**
     * Seconds of slack the route has: the latest partner tip that still works minus the assumed one. Large is robust
     * (a slow partner is fine), small means the route only works with a quick partner, below zero means it does not
     * even work with the assumed time. {@code +infinity} if the route does not depend on the partner's tip; NaN for a
     * route that does not assume one.
     */
    public double partnerTipSlackSec() {
        return latestPartnerTipSec() - partnerTipSec;
    }

    /**
     * True if the partner's tip is assumed but never happens in the simulation because this route tipped the HIVE
     * first: the assumption no longer holds for this route.
     */
    public boolean partnerTipVoided() {
        return partnerTipVoided;
    }

    /** 1 if the partner's tip happens in the simulation (assumed, and not pre-empted by this route's own tip), else 0. */
    public int partnerTips() {
        int n = 0;
        for (TipEvent e : tipEvents) {
            if (e.byPartner) {
                n++;
            }
        }
        return n;
    }

    /**
     * LEAVE: the whole robot ends off every wall by {@code Auto.LEAVE_WALL_MARGIN_IN}. The manual (10.5.4) and FIRST's Q&amp;A QA21
     * judge LEAVE at the END of AUTO: a robot that touches ANY perimeter wall then loses it, also the wall inside the LOADING
     * ZONE. A robot that parks in the LOADING ZONE touching its wall keeps its PARK ({@link #park()}) but not its LEAVE. Only the
     * robot counts: a POLLEN squeezed between the robot and the wall does not make the robot "touch" it (QA47), so no POLLEN is
     * looked at here. (The manual does not say whether touching a FLOWER or the HIVE frame counts as touching "the wall"; they are
     * not the perimeter wall, so they do not.)
     */
    public boolean leave() {
        return leave;
    }

    /**
     * How far, in inches, the robot's footprint is from the NEAREST perimeter wall when the route ends: 0 means touching, a negative
     * number means through the wall. LEAVE needs at least {@code Auto.LEAVE_WALL_MARGIN_IN}.
     */
    public double endWallGapIn() {
        return endWallGapIn;
    }

    /** Which wall {@link #endWallGapIn()} is measured to, in words: "red wall (x = 0)", "audience wall (y = 0)" and so on. */
    public String endNearestWall() {
        return endNearestWall;
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

    /** Points of the HIVE tips OUR volleys cause. */
    public int tipPoints() {
        return tips * BioBuzzConstants.Auto.TIP_POINTS;
    }

    /**
     * Points the alliance gets from the partner's assumed tip. They are shown next to the route's own points but are
     * NOT part of {@link #points()}: the partner's volley is an assumption, not something this robot does.
     */
    public int partnerTipPoints() {
        return partnerTips() * BioBuzzConstants.Auto.TIP_POINTS;
    }

    /**
     * The AUTO points OUR robot can claim if everything goes to plan: LEAVE and PARK from the end pose, 20 per HIVE
     * TIP its own volleys cause. It assumes every POLLEN lands and the robot is not late, so it is a ceiling, not a
     * forecast. The partner's assumed tip is reported separately by {@link #partnerTipPoints()}.
     */
    public int points() {
        return leavePoints() + parkPoints() + tipPoints();
    }
}
