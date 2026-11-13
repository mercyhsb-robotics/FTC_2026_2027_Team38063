package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * The route checks that are about a HIVE TIP: what happens to the POLLEN of a volley around the moment the HIVE
 * tips. {@link RouteValidator} calls {@link #check} once per route; the rules live here so that the validator
 * does not grow without end.
 *
 * <p>Rules so far (manual numbers):
 * <ul>
 *   <li><b>G417, a volley that is too big.</b> The simulation ({@link RouteAnalysis}) says on which POLLEN of a
 *       volley the HIVE tips. A cell that is tipping down must not be launched into, so POLLEN that would still
 *       be fired after the tip is an ERROR: fire fewer POLLEN in that volley.</li>
 * </ul>
 */
final class TipRules {
    private TipRules() {
    }

    static void check(RouteAnalysis a, List<Issue> out) {
        for (RouteAnalysis.StepInfo info : a.steps()) {
            checkVolleyAfterTip(info, out);
        }
    }

    // ---- G417: a volley that goes on after the tip -----------------------------------------------------------

    private static void checkVolleyAfterTip(RouteAnalysis.StepInfo info, List<Issue> out) {
        if (!(info.step instanceof Step.Shoot) || !info.tipped || info.firedAfterTip <= 0) {
            return;
        }
        int count = ((Step.Shoot) info.step).count();
        int needed = count - info.firedAfterTip;
        out.add(new Issue(Severity.ERROR, info.index, Rule.SHOOT_TARGET, RouteValidator.describe(info) + ": the HIVE tips with POLLEN "
                + needed + " of the " + count + " (" + info.nectarInCellBefore + " NECTAR in the cell, "
                + BioBuzzConstants.Auto.NECTAR_TIP_THRESHOLD + " NECTAR or " + BioBuzzConstants.Auto.POLLEN_TIP_THRESHOLD
                + " POLLEN tip it), so the last " + info.firedAfterTip + " would be launched into a cell that is already"
                + " tipping down; launching into a cell that does not face up is a G417 violation. Fire only " + needed
                + " at this cell"));
    }
}
