package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The arithmetic of tipping a HIVE, in whole numbers, for the route planner.
 *
 * <p><b>The rule.</b> A HIVE cell holds NECTAR (the big balls, 3.6 in) and POLLEN (the small balls, 2.8 in). The
 * manual does not say how much it takes to tip the HIVE, so the planner uses the team's figures: 5 NECTAR, or 8
 * POLLEN, or a mix of both. A mix works like a recipe: every NECTAR is worth 1/5 of a tip and every POLLEN 1/8 of
 * a tip, and the cell tips when the shares add up to 1 or more:
 *
 * <pre>
 *   nectar / 5  +  pollen / 8  &gt;=  1
 * </pre>
 *
 * <p>Fractions are awkward in code (0.6 + 0.5 and rounding), so the whole inequality is multiplied by 5 x 8 = 40,
 * which gives the same answer using whole numbers only:
 *
 * <pre>
 *   nectar x 8  +  pollen x 5  &gt;=  5 x 8
 * </pre>
 *
 * <p>Example with the 3 NECTAR that sit in the UP cell when the match starts: 3 NECTAR + 4 POLLEN is
 * 3 x 8 + 4 x 5 = 44, which is at least 40, so ONE volley of 4 POLLEN tips the first HIVE; 3 NECTAR + 3 POLLEN is
 * 24 + 15 = 39, which is not, so 3 POLLEN do not. The cell that comes up after a tip holds no NECTAR, so it needs
 * 8 POLLEN (0 x 8 + 8 x 5 = 40), two volleys of 4.
 *
 * <p>The two thresholds live in {@link BioBuzzConstants.Auto} (they are the team's figures, to be confirmed on
 * the real HIVE); this class only does the arithmetic with them.
 */
public final class TipModel {
    private TipModel() {
    }

    /** True if a cell holding {@code nectar} NECTAR and {@code pollen} POLLEN tips (the thresholds of {@link BioBuzzConstants.Auto}). */
    public static boolean tips(int nectar, int pollen) {
        return tips(nectar, pollen, BioBuzzConstants.Auto.NECTAR_TIP_THRESHOLD, BioBuzzConstants.Auto.POLLEN_TIP_THRESHOLD);
    }

    /**
     * The same rule with the two thresholds given, so a test can try other figures:
     * {@code nectar x pollenThreshold + pollen x nectarThreshold >= nectarThreshold x pollenThreshold}.
     */
    public static boolean tips(int nectar, int pollen, int nectarThreshold, int pollenThreshold) {
        return nectar * pollenThreshold + pollen * nectarThreshold >= nectarThreshold * pollenThreshold;
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

    /** POLLEN that tip the cell that is UP at the start of the match (it holds the staged NECTAR): 4 with the team's figures. */
    public static int firstTipPollen() {
        return pollenStillNeeded(BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL, 0);
    }

    /** POLLEN that tip a cell with no NECTAR in it, which is every cell after the first tip: 8 with the team's figures. */
    public static int laterTipPollen() {
        return pollenStillNeeded(0, 0);
    }

    /** The plain-words version of the rule with the real numbers, for the gallery page and the documentation. */
    public static String explainRule() {
        int n = BioBuzzConstants.Auto.NECTAR_TIP_THRESHOLD;
        int p = BioBuzzConstants.Auto.POLLEN_TIP_THRESHOLD;
        int staged = BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL;
        int first = firstTipPollen();
        StringBuilder s = new StringBuilder();
        s.append("A HIVE cell tips when NECTAR x ").append(p).append(" + POLLEN x ").append(n).append(" reaches ").append(n * p)
                .append(" (").append(n).append(" NECTAR, or ").append(p).append(" POLLEN, or a mix). The cell that is UP at the start holds ")
                .append(staged).append(" NECTAR, so ").append(first).append(" POLLEN tip it (").append(staged).append(" x ").append(p)
                .append(" + ").append(first).append(" x ").append(n).append(" = ").append(staged * p + first * n).append(")");
        if (first > 0) {
            s.append(" but ").append(first - 1).append(" do not (").append(staged).append(" x ").append(p).append(" + ")
                    .append(first - 1).append(" x ").append(n).append(" = ").append(staged * p + (first - 1) * n).append(")");
        }
        s.append(". The cell that comes up after a tip holds no NECTAR, so each later tip needs ").append(laterTipPollen())
                .append(" POLLEN.");
        return s.toString();
    }
}
