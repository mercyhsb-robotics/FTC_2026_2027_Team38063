package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * The route checks that are about a HIVE TIP: what happens to the POLLEN of a volley around the moment the HIVE
 * tips, who is assumed to shoot the first tip, and where the robot may stand while the tipped cell spills.
 * {@link RouteValidator} calls {@link #check} once per route; the rules live here so that the validator does not
 * grow without end. They all read the one simulation, {@link RouteAnalysis}.
 *
 * <p>The rules (manual numbers):
 * <ul>
 *   <li><b>The tip arithmetic and the spare ({@link TipModel}).</b> The simulation says on which POLLEN of a volley the
 *       HIVE tips (FIRST's calibration: 3 POLLEN for the cell that holds the 3 staged NECTAR, 8 for an empty cell). A
 *       volley that tips with POLLEN to spare is normal: the camera sees the tip, the feeder stops and the spare stays in
 *       the robot. A route that puts in exactly the minimum has no spare, and a field that drifts (QA46: one needed 9
 *       instead of 8) would not tip: WARN.</li>
 *   <li><b>G417, a timed volley that is too big.</b> A volley that feeds for a fixed time (an old OpMode) cannot stop when
 *       the camera sees the tip. POLLEN that such a volley would still launch after the tip go into a cell that is
 *       tipping down: WARN (QA12 allows adding to a moving HIVE, but G417 allows launching only into an upward-facing
 *       CELL and a launch at a rocking HIVE can spoil the tip).</li>
 *   <li><b>G417, launching into a cell that faces down.</b> Only an upward-facing cell may be launched into. A SHOOT at
 *       a cell that is DOWN when the first POLLEN leaves is a WARN, except in a route that assumes the PARTNER shoots
 *       the first tip. There the cell that comes UP with the partner's tip is special, because partner teams tip at
 *       different times and the robot must not guess: every SHOOT at that cell needs a WAIT_FOR_TIP before it, even
 *       one that starts after the assumed tip time (a time never lets the robot shoot, only its camera does), and the
 *       wait must last until the tip is CONFIRMED: the tip plus the settle time the camera needs to see the cell steady
 *       ({@code Vision.HIVE_STABLE_SEEN_SEC}), an ERROR otherwise. A volley that is still firing at the old cell when the
 *       partner's tip happens is a WARN.</li>
 *   <li><b>G409, the spill.</b> When a HIVE tips, the cell that was UP tips down and everything in it falls out of its
 *       opening onto the floor on that side of the HIVE: its 3 staged NECTAR (if it is the first tip) and the POLLEN
 *       launched into it (field_layout.verified.md 2.1 and 5.2: the red audience cell is x [49.25, 69.25],
 *       y [52.3, 69.9] and opens toward the audience wall). G409 says a robot may not catch or deflect an element
 *       released by a tipped HIVE until it has touched something else (rules.verified.md, G409 p109). So for
 *       {@code TIP_SPILL_CLEAR_SEC} after every tip, ours and the partner's, the robot footprint must stay out of
 *       the spill zone: that cell's plan footprint grown by {@code TIP_SPILL_MARGIN_IN}. The other cell's side of the
 *       HIVE is not in the zone; it is where a robot waiting for the partner's tip stands. ERROR. (This is the
 *       planning version, with times. On the robot the same rule is "stay out until the camera has confirmed the tip,
 *       plus {@code TIP_SPILL_CLEAR_SEC}", because the real time of the partner's tip is not known.)</li>
 *   <li><b>G409 / G417, under the HIVE.</b> A DRIVE that is allowed under the HIVE (the lane between the foot bars, see
 *       {@code HiveSolid}) gets a WARN when the robot is over the HIVE frame while the HIVE could still tip: that is, unless the
 *       camera has confirmed it at rest (a WAIT_FOR_TIP has ended before it) and none of our volleys went into it since.</li>
 *   <li><b>The assumption must hold.</b> WARN: a WAIT_FOR_TIP in a route that assumes we shoot the first tip (there is
 *       nothing to wait for); a WAIT_FOR_TIP that gives up before the tip it waits for, or later than the rest of the
 *       route can afford; our own volley tipping the HIVE before the partner's assumed tip (the route then does not
 *       need the partner); and a route with less than {@code PARTNER_TIP_MIN_SLACK_SEC} of slack, which only works if
 *       the partner is about as fast as assumed ({@link RouteAnalysis#partnerTipSlackSec()}).</li>
 * </ul>
 */
final class TipRules {
    private static final double EPS = 1e-9;

    private TipRules() {
    }

    static void check(RouteAnalysis a, List<Issue> out) {
        for (RouteAnalysis.StepInfo info : a.steps()) {
            checkVolleyAfterTip(info, out);
            checkSpare(info, out);
            checkTarget(a, info, out);
            checkWaitForTip(a, info, out);
        }
        checkPartnerAssumption(a, out);
        checkSpill(a, out);
        checkUnderTheHive(a, out);
    }

    private static Issue error(int step, Rule rule, String message) {
        return new Issue(Severity.ERROR, step, rule, message);
    }

    private static Issue warn(int step, Rule rule, String message) {
        return new Issue(Severity.WARN, step, rule, message);
    }

    /** The partner's assumed tip, or null if the route assumes SELF or our own volley tipped the HIVE first. */
    private static RouteAnalysis.TipEvent partnerEvent(RouteAnalysis a) {
        for (RouteAnalysis.TipEvent e : a.tipEvents()) {
            if (e.byPartner) {
                return e;
            }
        }
        return null;
    }

    // ---- G417: a TIMED volley that goes on after the tip ------------------------------------------------------

    /**
     * Only a timed volley can launch POLLEN after the tip: the camera-gated volley stops when the cell's tags vanish and
     * keeps the rest (see {@link RouteAnalysis.StepInfo#firedAfterTip}).
     */
    private static void checkVolleyAfterTip(RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Shoot) || !info.tipped || info.firedAfterTip <= 0) {
            return;
        }
        int count = ((Step.Shoot) info.step).count();
        out.add(warn(info.index, Rule.SHOOT_TARGET, RouteValidator.describe(info) + ": this volley is TIMED (it feeds for "
                + RouteValidator.sec(((Step.Shoot) info.step).fixedSeconds()) + " whatever happens, it cannot stop when the camera sees the tip) and the HIVE tips with POLLEN "
                + info.tipNeeded + " of the " + count + " (" + info.nectarInCellBefore + " NECTAR in the cell; "
                + TipModel.ruleShort() + "), so the last "
                + info.firedAfterTip + " would be launched into a cell that is already tipping down. FIRST's Q&A QA12 lets you add elements to a HIVE"
                + " that is moving, but G417 allows launching only into an upward-facing CELL and a launch at a rocking HIVE can spoil the tip."
                + " Drop launchSeconds() so the volley is gated by the camera (it stops at the tip and keeps the spare), or fire only "
                + info.tipNeeded));
    }

    // ---- the spare: plan one POLLEN more than the minimum ------------------------------------------------------

    /**
     * A volley of OUR route that tips the HIVE with no POLLEN to spare plans exactly the minimum: on a field that needs one
     * more (FIRST's Q&A QA46: a HIVE that needed 9 POLLEN instead of 8 after 8 to 10 matches) it would not tip. WARN, and
     * the message says what to plan.
     */
    private static void checkSpare(RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Shoot) || !info.tipped || info.tipSpare >= BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN) {
            return;
        }
        int count = ((Step.Shoot) info.step).count();
        int planned = info.pollenInCellBefore + count;
        int minimum = TipModel.minimumPollen(info.nectarInCellBefore);
        int wanted = TipModel.planningPollen(info.nectarInCellBefore);
        out.add(warn(info.index, Rule.TIP_MARGIN, RouteValidator.describe(info) + ": the route puts " + planned + " POLLEN into the "
                + info.targetSide + " cell, exactly the " + minimum + " that tip it (" + info.nectarInCellBefore + " NECTAR in the cell), with "
                + "no spare. A field drifts: FIRST's Q&A QA46 tells of a HIVE that needed 9 POLLEN instead of 8, and on such a field this "
                + "route would not tip. Plan " + wanted + " POLLEN (a spare of " + BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN
                + ", Auto.TIP_PLANNING_MARGIN_POLLEN)" + (info.pollenBefore < wanted - info.pollenInCellBefore
                ? "; the robot holds only " + info.pollenBefore + " here, so fetch more before this volley" : "")));
    }

    // ---- G417: the target cell must face up when the POLLEN leave -----------------------------------------------

    private static void checkTarget(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Shoot)) {
            return;
        }
        RouteAnalysis.TipEvent partner = partnerEvent(a);
        if (partner != null && info.targetSide == partner.newUpSide) {
            // The cell that only comes UP when the partner's tip is done. The robot must SEE it up before it shoots: a
            // time never decides, because partner teams tip at different times.
            int waitIndex = lastWaitBefore(a, info.index);
            // The wait must last until the tip is CONFIRMED: the tip plus the time the camera needs to see the cell steady.
            double confirmed = a.partnerTipConfirmedSec();
            if (waitIndex < 0) {
                out.add(error(info.index, Rule.SHOOT_TARGET, noWaitMessage(info, partner)));
            } else if (a.step(waitIndex).endSec() < confirmed - EPS || info.launchSec < confirmed - EPS) {
                out.add(error(info.index, Rule.SHOOT_TARGET, waitEndsTooEarlyMessage(a, info, partner, waitIndex)));
            }
            return;
        }
        if (!info.targetWasUp) {
            Side upNow = info.targetSide.opposite();      // a HIVE has two cells, so the other one is the UP one
            out.add(warn(info.index, Rule.SHOOT_TARGET, RouteValidator.describe(info) + ": fires at the " + info.targetSide
                    + " cell, but at this point the " + upNow + " cell is the UP one; launching into a cell that faces"
                    + " down is a G417 violation"));
        } else if (partner != null && info.targetSide == partner.releasingSide && info.launchSec < partner.timeSec - EPS
                && info.endSec() > partner.timeSec + EPS) {
            out.add(warn(info.index, Rule.SHOOT_TARGET, RouteValidator.describe(info) + ": the volley at the "
                    + info.targetSide + " cell starts at " + RouteValidator.sec(info.launchSec) + " and is still firing at "
                    + RouteValidator.sec(info.endSec()) + ", but the partner is assumed to tip that cell at "
                    + RouteValidator.sec(partner.timeSec) + "; launching at a cell that is tipping down may spoil the tip"
                    + " (G417)"));
        }
    }

    /** Index of the last WAIT_FOR_TIP before step {@code index}, or -1 if there is none. */
    private static int lastWaitBefore(RouteAnalysis a, int index) {
        int waitIndex = -1;
        for (int i = 0; i < index; i++) {
            if (a.step(i).step.kind() == StepKind.WAIT_FOR_TIP) {
                waitIndex = i;
            }
        }
        return waitIndex;
    }

    private static String noWaitMessage(RouteAnalysis.StepInfo info, RouteAnalysis.TipEvent partner) {
        return RouteValidator.describe(info) + ": aims at the " + info.targetSide + " cell, which only comes up when the partner's"
                + " tip is done (assumed at " + RouteValidator.sec(partner.timeSec) + "), with no WAIT_FOR_TIP before it. Partner"
                + " teams tip at different times, so the robot shoots at that cell only after its camera has SEEN it UP, never"
                + " because a time has passed (a cell that faces down must not be launched into, G417). Put a WAIT_FOR_TIP"
                + " before this SHOOT";
    }

    private static String waitEndsTooEarlyMessage(RouteAnalysis a, RouteAnalysis.StepInfo info, RouteAnalysis.TipEvent partner,
                                                  int waitIndex) {
        RouteAnalysis.StepInfo wait = a.step(waitIndex);
        return RouteValidator.describe(info) + ": launches into the " + info.targetSide + " cell at "
                + RouteValidator.sec(info.launchSec) + ", but that cell only comes up when the partner's tip is done, assumed at "
                + RouteValidator.sec(partner.timeSec) + " and confirmed " + RouteValidator.sec(a.tipSettleSec())
                + " later (the camera must see the cell steady first), at " + RouteValidator.sec(a.partnerTipConfirmedSec())
                + ", and a cell that faces down must not be launched into (G417). The"
                + " WAIT_FOR_TIP at step " + (waitIndex + 1) + " gives up at " + RouteValidator.sec(wait.endSec())
                + ", before the tip is confirmed (its give-up time is " + RouteValidator.sec(((Step.WaitForTip) wait.step).giveUpSec())
                + "), so the robot would skip this volley: raise the give-up time to at least "
                + RouteValidator.sec(a.partnerTipConfirmedSec()) + " or plan the route for a later tip";
    }

    // ---- WAIT_FOR_TIP and the first-tip assumption ----------------------------------------------------------

    private static void checkWaitForTip(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.WaitForTip)) {
            return;
        }
        if (!a.partnerTipAssumed()) {
            out.add(warn(info.index, Rule.TIP_ASSUMPTION, RouteValidator.describe(info) + ": this route assumes that WE shoot the"
                    + " first tip, so there is no partner tip to wait for and the step does nothing in the plan; remove it, or"
                    + " write the route with partnerTipsFirst()"));
            return;
        }
        double giveUp = ((Step.WaitForTip) info.step).giveUpSec();
        if (!Double.isNaN(info.tipWaitSec) && info.endSec() < a.partnerTipConfirmedSec() - EPS) {
            out.add(warn(info.index, Rule.TIP_ASSUMPTION, RouteValidator.describe(info) + ": gives up at "
                    + RouteValidator.sec(info.endSec()) + ", before the partner's tip assumed at " + RouteValidator.sec(info.tipWaitSec)
                    + " is confirmed (the camera needs " + RouteValidator.sec(a.tipSettleSec()) + " of a steady view after the tip, so "
                    + RouteValidator.sec(a.partnerTipConfirmedSec()) + " at the earliest); the cell it waits for is not known to be up then."
                    + " Raise the give-up time to at least " + RouteValidator.sec(a.partnerTipConfirmedSec())));
        }
        double latest = a.latestGiveUpSec(info);
        if (giveUp > latest + EPS) {
            out.add(warn(info.index, Rule.TIP_ASSUMPTION, RouteValidator.describe(info) + ": gives up at "
                    + RouteValidator.sec(giveUp) + ", but the steps after it take " + RouteValidator.sec(a.totalSeconds() - info.endSec())
                    + " and the route must be over by " + RouteValidator.sec(RouteAnalysis.timeBudgetSec())
                    + ", so waiting any later than " + RouteValidator.sec(latest) + " would run past the AUTO limit. Set the"
                    + " give-up time to " + RouteValidator.sec(latest) + " or less"));
        }
    }

    private static void checkPartnerAssumption(RouteAnalysis a, List<Issue> out) {
        if (!a.partnerTipAssumed()) {
            return;
        }
        checkSlack(a, out);
        if (a.partnerTipVoided()) {
            for (RouteAnalysis.TipEvent e : a.tipEvents()) {
                if (!e.byPartner) {
                    out.add(warn(e.stepIndex, Rule.TIP_ASSUMPTION, RouteValidator.describe(a.step(e.stepIndex))
                            + ": this volley tips the HIVE at " + RouteValidator.sec(e.timeSec) + ", before the partner's tip"
                            + " assumed at " + RouteValidator.sec(a.partnerTipSec()) + ", so the route does not need the partner"
                            + " (and the partner's own volley would then come too late); write it as a route that shoots the"
                            + " first tip itself"));
                    break;
                }
            }
        }
    }

    /**
     * A route that waits for the partner's tip works only if the tip is not too late. The slack is how late it may be
     * beyond the assumed time ({@link RouteAnalysis#partnerTipSlackSec()}); a small one is a WARN, so the drive team knows
     * the route needs a partner that is about as fast as agreed.
     */
    private static void checkSlack(RouteAnalysis a, List<Issue> out) {
        int waitIndex = -1;
        for (RouteAnalysis.StepInfo info : a.steps()) {
            if (info.step instanceof Step.WaitForTip) {
                waitIndex = info.index;
                break;
            }
        }
        double slack = a.partnerTipSlackSec();
        if (waitIndex < 0 || Double.isNaN(slack) || Double.isInfinite(slack) || slack >= BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC) {
            return;
        }
        RouteAnalysis.StepInfo wait = a.step(waitIndex);
        double giveUp = ((Step.WaitForTip) wait.step).giveUpSec();
        String why = giveUp <= a.latestGiveUpSec(wait) ? "the WAIT_FOR_TIP gives up at " + RouteValidator.sec(giveUp)
                : "the steps after the wait take " + RouteValidator.sec(a.totalSeconds() - wait.endSec()) + " and the route must be over by "
                + RouteValidator.sec(RouteAnalysis.timeBudgetSec());
        out.add(warn(waitIndex, Rule.TIP_ASSUMPTION, RouteValidator.describe(wait) + ": the partner's tip is assumed at "
                + RouteValidator.sec(a.partnerTipSec()) + ", but the latest tip this route can still use is "
                + RouteValidator.sec(a.latestPartnerTipSec()) + " (" + why + "): " + String.format(Locale.ROOT, "%.1f", slack)
                + " s of slack, less than the " + RouteValidator.sec(BioBuzzConstants.Auto.PARTNER_TIP_MIN_SLACK_SEC)
                + " a slower partner needs. The route only works with a partner that is about as fast as assumed; give up later,"
                + " or make the steps after the wait shorter"));
    }

    // ---- G409 / G417: do not be under a HIVE that can still tip ----------------------------------------------------

    /**
     * A DRIVE that is allowed under the HIVE ({@code allowUnderHive}) may drive along the lane between its foot bars, but the HIVE can tip
     * while the robot is there: the cells swing through 60 degrees over its head and drop what they hold (G409), and a robot under the HIVE
     * is where nobody wants to be when a volley lands (G417). The HIVE is known to be AT REST only after the camera has confirmed a tip fully
     * (a WAIT_FOR_TIP ends exactly then) and until the next volley of ours goes into it. So for every such DRIVE whose robot footprint is
     * over the HIVE frame at some moment, WARN unless a WAIT_FOR_TIP has completed before it with no SHOOT in between. The WARN also says when
     * a tip is assumed during the move.
     */
    private static void checkUnderTheHive(RouteAnalysis a, List<Issue> out) {
        Rect frame = a.field().hiveFrame();
        boolean atRest = false;
        for (RouteAnalysis.StepInfo info : a.steps()) {
            if (info.step instanceof Step.Drive && ((Step.Drive) info.step).allowsUnderHive() && !atRest) {
                Pose2 first = null;
                for (Pose2 p : a.posesDuring(info, info.startSec, info.endSec())) {
                    if (Footprint.of(p).overlaps(frame)) {
                        first = p;
                        break;
                    }
                }
                if (first != null) {
                    out.add(warn(info.index, Rule.UNDER_HIVE, RouteValidator.describe(info) + ": the robot drives under the HIVE (first at "
                            + RouteValidator.pose(first) + ", the step runs from " + RouteValidator.sec(info.startSec) + " to " + RouteValidator.sec(info.endSec())
                            + ") while the HIVE could still tip: the camera has not confirmed it at rest" + tipDuring(a, info) + ". The cells swing through 60"
                            + " degrees over the robot and drop what they hold (G409), and launching into a tipping HIVE spoils the tip (G417). Put a"
                            + " WAIT_FOR_TIP before this DRIVE, with no SHOOT between them, or go around the HIVE"));
                }
            }
            if (info.step instanceof Step.WaitForTip) {
                atRest = true;
            } else if (info.step instanceof Step.Shoot) {
                atRest = false;
            }
        }
    }

    /** ", and a tip is assumed at T s, during this move" for every tip of the simulation that falls inside the step, or "". */
    private static String tipDuring(RouteAnalysis a, RouteAnalysis.StepInfo info) {
        StringBuilder sb = new StringBuilder();
        for (RouteAnalysis.TipEvent tip : a.tipEvents()) {
            if (tip.timeSec >= info.startSec - EPS && tip.timeSec <= info.endSec() + EPS) {
                sb.append(", and ").append(tip.byPartner ? "the partner's" : "our own").append(" tip is assumed at ")
                        .append(RouteValidator.sec(tip.timeSec)).append(", during this move");
            }
        }
        return sb.toString();
    }

    // ---- G409: do not stand where the tipped cell spills ----------------------------------------------------

    private static void checkSpill(RouteAnalysis a, List<Issue> out) {
        double clear = BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC;
        double margin = BioBuzzConstants.Auto.TIP_SPILL_MARGIN_IN;
        for (RouteAnalysis.TipEvent tip : a.tipEvents()) {
            Rect zone = a.field().cellPlan(a.field().alliance(), tip.releasingSide, true).inflate(margin);
            double from = tip.timeSec;
            double to = tip.timeSec + clear;
            int last = a.steps().size() - 1;
            boolean lastStepReported = false;
            for (RouteAnalysis.StepInfo info : a.steps()) {
                for (Pose2 p : a.posesDuring(info, from, to)) {
                    if (Footprint.of(p).overlaps(zone)) {
                        out.add(error(info.index, Rule.SPILL_G409, spillMessage(info, tip, zone, p)));
                        lastStepReported |= info.index == last;
                        break;
                    }
                }
            }
            // after the last step the robot stands where the route ended
            if (!lastStepReported && to > a.totalSeconds() + EPS && Footprint.of(a.endPose()).overlaps(zone)) {
                out.add(error(last, Rule.SPILL_G409, spillMessage(a.step(last), tip, zone, a.endPose())));
            }
        }
    }

    private static String spillMessage(RouteAnalysis.StepInfo info, RouteAnalysis.TipEvent tip, Rect zone, Pose2 at) {
        String who = tip.byPartner ? "the partner's assumed tip" : "the HIVE tip of step " + (tip.stepIndex + 1);
        return RouteValidator.describe(info) + ": the robot footprint at " + RouteValidator.pose(at) + " is inside the spill zone of the "
                + tip.releasingSide + " cell " + zone + " within " + RouteValidator.sec(BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC)
                + " of " + who + " at " + RouteValidator.sec(tip.timeSec) + "; the cell that tips releases its NECTAR and POLLEN"
                + " there and a robot may not catch or deflect them (G409). Stay out of the zone until "
                + String.format(Locale.ROOT, "%.1f s", tip.timeSec + BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC);
    }
}
