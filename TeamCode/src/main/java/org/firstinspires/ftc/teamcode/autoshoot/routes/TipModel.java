package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The arithmetic of tipping a HIVE, in whole numbers, for the route planner.
 *
 * <p><b>Where the numbers come from.</b> A HIVE cell holds NECTAR (the big balls, 3.6 in) and POLLEN (the small balls,
 * 2.8 in). The game manual never says how much it takes to tip the HIVE, but FIRST's field documents do (the Event Field
 * Setup Guide section 12.3, the Field Acceptance Checklist and the Field Mitigation Guide): every cell of every field
 * is calibrated so that
 * <ul>
 *   <li>an EMPTY cell tips with the 8th POLLEN (7 POLLEN do not tip it), and</li>
 *   <li>a cell holding the 3 NECTAR that are staged in it at the start of the match tips with the 3rd POLLEN (3 NECTAR
 *       + 2 POLLEN do not tip it).</li>
 * </ul>
 *
 * <p><b>The rule.</b> To plan with those two facts (and with any mix in between) the planner counts every POLLEN as 1 and
 * every NECTAR as 5/3 of a POLLEN (a NECTAR is a bit heavier than a POLLEN; 5/3 is the weight that reproduces the table
 * above, and it also says "5 NECTAR tip, 4 do not", the team's rule of thumb). A cell tips when what it holds adds up to
 * 8 POLLEN-equivalents:
 *
 * <pre>
 *   pollen  +  nectar x 5/3  &gt;=  8
 * </pre>
 *
 * <p>A fraction like 5/3 cannot be written exactly as a decimal (1.666... in code would sometimes round a sum of exactly
 * 8 down to 7.999 and say "no tip"), so the whole inequality is multiplied by the bottom of the fraction (3), which
 * leaves whole numbers only:
 *
 * <pre>
 *   nectar x 5  +  pollen x 3  &gt;=  8 x 3          (thirds of a POLLEN)
 * </pre>
 *
 * <p>The official table, checked with that rule (the tests do exactly this):
 * <pre>
 *   0 NECTAR + 7 POLLEN:  0 + 21 = 21  &lt; 24   no tip          0 NECTAR + 8 POLLEN:  0 + 24 = 24  &gt;= 24   tips
 *   3 NECTAR + 2 POLLEN: 15 +  6 = 21  &lt; 24   no tip          3 NECTAR + 3 POLLEN: 15 + 9 = 24  &gt;= 24   tips
 *   4 NECTAR:            20            &lt; 24   no tip          5 NECTAR:            25            &gt;= 24   tips
 * </pre>
 *
 * <p><b>What it means for a route.</b> The cell that is UP at the start holds the 3 staged NECTAR, so the FIRST tip needs
 * only 3 POLLEN (the robot starts with 4 pre-loads, so one is spare). The cell that comes up after a tip is empty, so every
 * LATER tip needs 8 POLLEN. A real field drifts (FIRST's Q&amp;A QA46: one HIVE needed 9 instead of 8 after 8 to 10 matches),
 * so a route should plan {@code BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN} more than the minimum: 4 for the first
 * tip and 9 for a later one. Those extra POLLEN are the route's <i>spare</i>.
 *
 * <p>The figures live in {@link BioBuzzConstants.Auto} (VERIFY: they are FIRST's calibration targets, not a measurement of
 * this season's real HIVE); this class only does the arithmetic with them.
 */
public final class TipModel {
    private TipModel() {
    }

    /** True if a cell holding {@code nectar} NECTAR and {@code pollen} POLLEN tips (the figures of {@link BioBuzzConstants.Auto}). */
    public static boolean tips(int nectar, int pollen) {
        return tips(nectar, pollen, BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR,
                BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR, BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD);
    }

    /**
     * The same rule with the figures given, so a test can try other ones: one NECTAR counts as
     * {@code weightNumerator / weightDenominator} POLLEN and the cell tips at {@code thresholdPollen} POLLEN-equivalents. In
     * whole numbers: {@code nectar x weightNumerator + pollen x weightDenominator >= thresholdPollen x weightDenominator}.
     */
    public static boolean tips(int nectar, int pollen, int weightNumerator, int weightDenominator, int thresholdPollen) {
        return nectar * weightNumerator + pollen * weightDenominator >= thresholdPollen * weightDenominator;
    }

    /**
     * How many MORE POLLEN a cell needs to tip: the smallest {@code k >= 0} so that the cell with
     * {@code pollenAlready + k} POLLEN tips. 0 means it tips already (or would tip without any more).
     */
    public static int pollenStillNeeded(int nectar, int pollenAlready) {
        int k = 0;
        while (!tips(nectar, pollenAlready + k)) {
            k++;
        }
        return k;
    }

    /** The least POLLEN that tip a cell holding {@code nectar} NECTAR (and nothing else): 3 POLLEN for the 3 staged NECTAR, 8 for an empty cell. */
    public static int minimumPollen(int nectar) {
        return pollenStillNeeded(nectar, 0);
    }

    /** POLLEN that tip the cell that is UP at the start of the match (it holds the staged NECTAR): 3 with FIRST's calibration. */
    public static int firstTipPollen() {
        return minimumPollen(BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL);
    }

    /** POLLEN that tip a cell with no NECTAR in it, which is every cell after the first tip: 8 with FIRST's calibration. */
    public static int laterTipPollen() {
        return minimumPollen(0);
    }

    /**
     * What a route should PLAN to put into a cell holding {@code nectar} NECTAR so that a field that drifts still tips it: the
     * minimum plus {@code Auto.TIP_PLANNING_MARGIN_POLLEN} (4 for the first tip, 9 for a later one).
     */
    public static int planningPollen(int nectar) {
        return minimumPollen(nectar) + BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN;
    }

    /**
     * The spare of a plan: how many POLLEN {@code pollenPlanned} (all the POLLEN the route puts into a cell, the volley
     * under discussion included) are more than the minimum that tip it. 0 means "exactly the minimum": a field that needs
     * one more would not tip. Negative: the plan does not even reach the minimum.
     */
    public static int spare(int nectar, int pollenPlanned) {
        return pollenPlanned - minimumPollen(nectar);
    }

    /**
     * What a cell holds, in POLLEN-equivalents, as text that is exact: "8" for 24 thirds, "7 2/3" for 23 thirds. The decision
     * itself never uses this text, only the whole-number test {@link #tips(int, int)}.
     */
    static String equivalentsText(int nectar, int pollen) {
        int den = BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR;
        int total = nectar * BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR + pollen * den;
        int whole = total / den;
        int rest = total % den;
        if (rest == 0) {
            return Integer.toString(whole);
        }
        return (whole == 0 ? "" : whole + " ") + rest + "/" + den;
    }

    /** The weight of one NECTAR as text: "5/3". */
    static String weightText() {
        return BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR + "/" + BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR;
    }

    /**
     * How many POLLEN of a camera-gated volley are still launched AFTER the one that tips the HIVE, before the feeder can stop. The POLLEN
     * leave the shooter {@code secondsPerShot} apart. The one that tips lands {@code flightSec} after it left, and the feeder has stopped
     * {@code stopLatencySec} after that landing. So the j-th POLLEN after the tipping one (it leaves j x secondsPerShot later) is launched if
     * {@code j x secondsPerShot < flightSec + stopLatencySec}. At most {@code spare}, the POLLEN the volley plans beyond what the cell needed.
     *
     * <p>Example with the shipped numbers (1.0 s between POLLEN, 0.5 s flight, 0.85 s to stop): the next POLLEN leaves 1.0 s after the tipping
     * one, the feeder stops 1.35 s after it, so one POLLEN is launched into the cell that is swinging down; the one after that (2.0 s) is not. With a
     * feeder that stops within 0.5 s of the landing, none is.
     */
    public static int launchedBeforeTheFeederStops(int spare, double secondsPerShot, double flightSec, double stopLatencySec) {
        if (spare <= 0 || !(secondsPerShot > 0.0)) {
            return 0;
        }
        // j x secondsPerShot < window  <=>  j < window / secondsPerShot: the largest such whole j is ceil(window / secondsPerShot) - 1
        // (the 1e-9 keeps a window that is exactly j x secondsPerShot from counting the j-th POLLEN)
        int fed = (int) Math.ceil((flightSec + stopLatencySec) / secondsPerShot - 1e-9) - 1;
        return Math.max(0, Math.min(spare, fed));
    }

    /**
     * The exact chance that at least {@code needed} of {@code shots} POLLEN land in the cell when each one lands with the chance
     * {@code chanceEach} (0 to 1) and the shots do not depend on each other: the binomial sum
     * {@code C(shots, i) x p^i x (1 - p)^(shots - i)} over every {@code i} from {@code needed} to {@code shots}. For example 3 of 4 at 0.6 is
     * 4 x 0.216 x 0.4 + 0.1296 = 0.4752, and 8 of 8 is 0.6^8 = 0.0168. Needing nothing is certain; needing more than are fired is impossible.
     */
    public static double chanceAtLeast(int shots, int needed, double chanceEach) {
        if (needed <= 0) {
            return 1.0;
        }
        if (needed > shots) {
            return 0.0;
        }
        double total = 0.0;
        double ways = 1.0;                                   // C(shots, i), starting at i = 0
        for (int i = 0; i <= shots; i++) {
            if (i >= needed) {
                total += ways * Math.pow(chanceEach, i) * Math.pow(1.0 - chanceEach, shots - i);
            }
            ways = ways * (shots - i) / (i + 1);
        }
        return Math.min(1.0, total);
    }

    /**
     * One volley worked out in plain words: what the UP cell holds and what the volley does to it, for example
     * {@code "the UP cell holds 3 NECTAR and 0 POLLEN of ours; this volley fires up to 4: 3 NECTAR x 5/3 + 3 POLLEN = 8
     * POLLEN-equivalents, the 8 that tip a cell, so the HIVE tips on POLLEN 3 of the 4 and the volley stops there; the last 1
     * stays in the robot as the spare"}.
     */
    public static String explainVolley(int nectar, int pollenBefore, int count) {
        return explainVolley(nectar, pollenBefore, count, true);
    }

    /**
     * The same, for a volley that may not be able to stop at the tip: a volley gated by the camera ({@code stopsAtTip}) keeps the
     * POLLEN it did not need, a TIMED one (an old OpMode that feeds for a fixed time) fires them all, into a cell that is already
     * tipping down.
     */
    public static String explainVolley(int nectar, int pollenBefore, int count, boolean stopsAtTip) {
        return explainVolley(nectar, pollenBefore, count, stopsAtTip, 0);
    }

    /**
     * The same, saying how many of the POLLEN after the tipping one the camera-gated volley still launches before its feeder can stop
     * ({@code launchedAfterTip}, see {@link #launchedBeforeTheFeederStops}): those go into the cell that is swinging down and are spent, so
     * only the rest of the spare stays in the robot. The camera needs about {@code Auto.TIP_STOP_LATENCY_SEC} to see the tip, which is a guess to be measured.
     */
    public static String explainVolley(int nectar, int pollenBefore, int count, boolean stopsAtTip, int launchedAfterTip) {
        int threshold = BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD;
        int pollenAfter = pollenBefore + count;
        StringBuilder s = new StringBuilder();
        s.append("the UP cell holds ").append(nectar).append(" NECTAR and ").append(pollenBefore)
                .append(" POLLEN of ours; this volley fires up to ").append(count).append(": ");
        if (tips(nectar, pollenAfter)) {
            int needed = Math.max(1, pollenStillNeeded(nectar, pollenBefore));
            int onTheTip = pollenBefore + needed;
            s.append(nectar).append(" NECTAR x ").append(weightText()).append(" + ").append(onTheTip).append(" POLLEN = ")
                    .append(equivalentsText(nectar, onTheTip)).append(" POLLEN-equivalents, the ").append(threshold)
                    .append(" that tip a cell, so the HIVE tips on POLLEN ").append(needed).append(" of the ").append(count);
            int spare = count - needed;
            if (spare > 0 && stopsAtTip && launchedAfterTip > 0) {
                int kept = spare - launchedAfterTip;
                s.append(" and the camera stops the volley, but only about ").append(String.format(java.util.Locale.ROOT, "%.2f",
                        BioBuzzConstants.Auto.TIP_STOP_LATENCY_SEC)).append(" s after the tip, so the next ").append(launchedAfterTip)
                        .append(" POLLEN ").append(launchedAfterTip == 1 ? "is" : "are").append(" probably launched into the cell while it swings down and ")
                        .append(launchedAfterTip == 1 ? "is" : "are").append(" spent")
                        .append(kept > 0 ? "; the last " + kept + " stay" + (kept == 1 ? "s" : "") + " in the robot" : "; none stays in the robot as a spare")
                        .append(" (the spare is for a field that needs one more POLLEN, not a POLLEN to carry on)");
            } else if (spare > 0 && stopsAtTip) {
                s.append(" and the volley stops there; the last ").append(spare).append(" stay").append(spare == 1 ? "s" : "")
                        .append(" in the robot as the spare");
            } else if (spare > 0) {
                s.append("; this volley is TIMED and cannot stop, so the last ").append(spare).append(spare == 1 ? " goes" : " go")
                        .append(" into a cell that is already tipping down");
            } else {
                s.append(": NO spare, a field that needs one more POLLEN would not tip");
            }
        } else {
            s.append(nectar).append(" NECTAR x ").append(weightText()).append(" + ").append(pollenAfter).append(" POLLEN = ")
                    .append(equivalentsText(nectar, pollenAfter)).append(" POLLEN-equivalents, short of the ").append(threshold)
                    .append(" that tip a cell, so no tip yet (").append(pollenStillNeeded(nectar, pollenAfter))
                    .append(" more POLLEN into this cell would tip it)");
        }
        return s.toString();
    }

    /** The rule in one short phrase with the real numbers: "a cell tips at 8 POLLEN-equivalents (each POLLEN is 1, each NECTAR 5/3)". */
    public static String ruleShort() {
        return "a cell tips at " + BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD + " POLLEN-equivalents (each POLLEN counts 1, each NECTAR "
                + weightText() + ")";
    }

    // English ordinals: the last two digits decide whether the number is one of the "-teens" (11th, 12th, 13th), then the last digit does.
    // These are spelling rules, not tuning.
    private static final int ORDINAL_HUNDRED = 100;
    private static final int ORDINAL_TEN = 10;
    private static final int ORDINAL_TEENS_FROM = 11;
    private static final int ORDINAL_TEENS_TO = 13;

    /** 1 becomes "1st", 2 "2nd", 3 "3rd", 8 "8th", 11 "11th", 21 "21st". */
    static String ordinal(int n) {
        int lastTwo = n % ORDINAL_HUNDRED;
        int last = n % ORDINAL_TEN;
        String suffix = lastTwo >= ORDINAL_TEENS_FROM && lastTwo <= ORDINAL_TEENS_TO ? "th" : last == 1 ? "st" : last == 2 ? "nd" : last == 3 ? "rd" : "th";
        return n + suffix;
    }

    /** The plain-words version of the rule with the real numbers, for the gallery page and the documentation. */
    public static String explainRule() {
        int threshold = BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD;
        int staged = BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL;
        int first = firstTipPollen();
        int later = laterTipPollen();
        int margin = BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN;
        StringBuilder s = new StringBuilder();
        s.append("A HIVE cell tips when its POLLEN plus ").append(weightText()).append(" POLLEN for every NECTAR add up to ")
                .append(threshold).append(" POLLEN-equivalents (FIRST's own calibration, Event Field Setup Guide section 12.3: an empty cell tips with the ")
                .append(ordinal(later)).append(" POLLEN and ").append(later - 1).append(" do not; a cell with the ").append(staged)
                .append(" staged NECTAR tips with the ").append(ordinal(first)).append(" POLLEN and ").append(first - 1)
                .append(" do not). The cell that is UP at the start holds those ").append(staged)
                .append(" NECTAR, so the FIRST tip needs only ").append(first).append(" POLLEN (the robot starts with ")
                .append(BioBuzzConstants.Auto.PRELOAD_POLLEN).append(", so ").append(BioBuzzConstants.Auto.PRELOAD_POLLEN - first)
                .append(" is spare). The cell that comes up after a tip is empty, so each LATER tip needs ").append(later)
                .append(" POLLEN. A real field drifts (FIRST's Q&A QA46: one HIVE needed 9 instead of 8), so a route plans ")
                .append(margin).append(" POLLEN more than the minimum, ").append(planningPollen(staged)).append(" and ")
                .append(planningPollen(0)).append(", and the page shows each route's spare.");
        return s.toString();
    }
}
