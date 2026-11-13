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
 *   <li><b>G417, a volley that is too big.</b> The simulation says on which POLLEN of a volley the HIVE tips. A cell
 *       that is tipping down must not be launched into, so POLLEN that would still be fired after the tip is an
 *       ERROR: fire fewer POLLEN in that volley.</li>
 *   <li><b>G417, launching into a cell that faces down.</b> Only an upward-facing cell may be launched into. A SHOOT at
 *       a cell that is DOWN when the first POLLEN leaves is a WARN, except in a route that assumes the PARTNER shoots
 *       the first tip: there, a SHOOT at the cell that comes UP with the partner's tip, which starts BEFORE that tip
 *       is done, is an ERROR (with a WAIT_FOR_TIP before it that lasts until the tip the SHOOT is fine). A volley
 *       that is still firing at the old cell when the partner's tip happens is a WARN.</li>
 *   <li><b>G409, the spill.</b> When a HIVE tips, the cell that was UP tips down and everything in it falls out of its
 *       opening onto the floor on that side of the HIVE: its 3 staged NECTAR (if it is the first tip) and the POLLEN
 *       launched into it (field_layout.verified.md 2.1 and 5.2: the red audience cell is x [49.25, 69.25],
 *       y [52.3, 69.9] and opens toward the audience wall). G409 says a robot may not catch or deflect an element
 *       released by a tipped HIVE until it has touched something else (rules.verified.md, G409 p109). So for
 *       {@code TIP_SPILL_CLEAR_SEC} after every tip, ours and the partner's, the robot footprint must stay out of
 *       the spill zone: that cell's plan footprint grown by {@code TIP_SPILL_MARGIN_IN}. The other cell's side of the
 *       HIVE is not in the zone; it is where a robot waiting for the partner's tip stands. ERROR.</li>
 *   <li><b>The assumption must hold.</b> WARN: a WAIT_FOR_TIP in a route that assumes we shoot the first tip (there is
 *       nothing to wait for); a WAIT_FOR_TIP whose timeout ends before the tip it waits for; our own volley tipping
 *       the HIVE before the partner's assumed tip (the route then does not need the partner).</li>
 * </ul>
 */
final class TipRules {
    private static final double EPS = 1e-9;

    private TipRules() {
    }

    static void check(RouteAnalysis a, List<Issue> out) {
        for (RouteAnalysis.StepInfo info : a.steps()) {
            checkVolleyAfterTip(info, out);
            checkTarget(a, info, out);
            checkWaitForTip(a, info, out);
        }
        checkPartnerAssumption(a, out);
        checkSpill(a, out);
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

    // ---- G417: a volley that goes on after the tip -----------------------------------------------------------

    private static void checkVolleyAfterTip(RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Shoot) || !info.tipped || info.firedAfterTip <= 0) {
            return;
        }
        int count = ((Step.Shoot) info.step).count();
        int needed = count - info.firedAfterTip;
        out.add(error(info.index, Rule.SHOOT_TARGET, RouteValidator.describe(info) + ": the HIVE tips with POLLEN "
                + needed + " of the " + count + " (" + info.nectarInCellBefore + " NECTAR in the cell, "
                + BioBuzzConstants.Auto.NECTAR_TIP_THRESHOLD + " NECTAR or " + BioBuzzConstants.Auto.POLLEN_TIP_THRESHOLD
                + " POLLEN tip it), so the last " + info.firedAfterTip + " would be launched into a cell that is already"
                + " tipping down; launching into a cell that does not face up is a G417 violation. Fire only " + needed
                + " at this cell"));
    }

    // ---- G417: the target cell must face up when the POLLEN leave -----------------------------------------------

    private static void checkTarget(RouteAnalysis a, RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Shoot)) {
            return;
        }
        RouteAnalysis.TipEvent partner = partnerEvent(a);
        if (!info.targetWasUp) {
            Side upNow = info.targetSide.opposite();      // a HIVE has two cells, so the other one is the UP one
            boolean beforePartnerTip = partner != null && info.targetSide == partner.newUpSide
                    && info.launchSec < partner.timeSec - EPS;
            if (beforePartnerTip) {
                out.add(error(info.index, Rule.SHOOT_TARGET, beforePartnerTipMessage(a, info, partner)));
            } else {
                out.add(warn(info.index, Rule.SHOOT_TARGET, RouteValidator.describe(info) + ": fires at the " + info.targetSide
                        + " cell, but at this point the " + upNow + " cell is the UP one; launching into a cell that faces"
                        + " down is a G417 violation"));
            }
        } else if (partner != null && info.targetSide == partner.releasingSide && info.launchSec < partner.timeSec - EPS
                && info.endSec() > partner.timeSec + EPS) {
            out.add(warn(info.index, Rule.SHOOT_TARGET, RouteValidator.describe(info) + ": the volley at the "
                    + info.targetSide + " cell starts at " + RouteValidator.sec(info.launchSec) + " and is still firing at "
                    + RouteValidator.sec(info.endSec()) + ", but the partner is assumed to tip that cell at "
                    + RouteValidator.sec(partner.timeSec) + "; launching at a cell that is tipping down may spoil the tip"
                    + " (G417)"));
        }
    }

    private static String beforePartnerTipMessage(RouteAnalysis a, RouteAnalysis.StepInfo info, RouteAnalysis.TipEvent partner) {
        int waitIndex = -1;
        for (int i = 0; i < info.index; i++) {
            if (a.step(i).step.kind() == StepKind.WAIT_FOR_TIP) {
                waitIndex = i;
            }
        }
        String start = RouteValidator.describe(info) + ": launches into the " + info.targetSide + " cell at "
                + RouteValidator.sec(info.launchSec) + ", but that cell only comes up when the partner's tip is done, assumed at "
                + RouteValidator.sec(partner.timeSec) + ", and a cell that faces down must not be launched into (G417). ";
        if (waitIndex < 0) {
            return start + "Put a WAIT_FOR_TIP before this SHOOT so the robot waits for the tip";
        }
        RouteAnalysis.StepInfo wait = a.step(waitIndex);
        return start + "The WAIT_FOR_TIP at step " + (waitIndex + 1) + " ends at " + RouteValidator.sec(wait.endSec())
                + ", before the tip (its timeout is " + RouteValidator.sec(((Step.WaitForTip) wait.step).timeoutSeconds())
                + "): raise the timeout or start the SHOOT later";
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
        if (!Double.isNaN(info.tipWaitSec) && info.endSec() < info.tipWaitSec - EPS) {
            out.add(warn(info.index, Rule.TIP_ASSUMPTION, RouteValidator.describe(info) + ": gives up at "
                    + RouteValidator.sec(info.endSec()) + ", before the partner's tip assumed at " + RouteValidator.sec(info.tipWaitSec)
                    + "; the cell it waits for is still facing down then. Raise the timeout to at least "
                    + RouteValidator.sec(info.tipWaitSec - info.startSec)));
        }
    }

    private static void checkPartnerAssumption(RouteAnalysis a, List<Issue> out) {
        if (!a.partnerTipAssumed()) {
            return;
        }
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
