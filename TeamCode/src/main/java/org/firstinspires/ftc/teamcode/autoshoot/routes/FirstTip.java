package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.Locale;

/**
 * WHO shoots the first tip of our HIVE in the AUTO period, as an assumption a route is written under. The drive
 * team picks the auto that matches what the alliance partner's auto does, so every route says which one it is.
 *
 * <p>Why it matters: the HIVE has two cells and only the one that is UP may be launched into (rule G417). The
 * first tip needs one volley of 4 POLLEN (the staged NECTAR help, see {@link TipModel}). After it the OTHER cell
 * is UP and holds no NECTAR, so the next tip needs 8 POLLEN. If the partner robot shoots the first tip, our robot
 * should go to the side that comes up and start there.
 */
public enum FirstTip {
    /**
     * OUR robot shoots the first tip: its 4 pre-loads, together with the staged NECTAR, tip the HIVE. This is the
     * default and what every route did before the partner family existed.
     */
    SELF,
    /**
     * The alliance PARTNER shoots the first tip. The HIVE is assumed to tip by itself at
     * {@code BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC} seconds into the match (the drive team must agree that
     * time with the partner team). The other cell comes up then, and our robot goes to that side. The partner's
     * tip is worth 20 points to the alliance but is NOT counted as ours.
     */
    PARTNER;

    /** One line in plain words, for the gallery and the diagrams; {@code partnerTipSec} is only used by PARTNER. */
    public String summary(double partnerTipSec) {
        if (this == SELF) {
            return "we shoot the first tip";
        }
        return String.format(Locale.ROOT, "partner tips first, assumed at %.1f s", partnerTipSec);
    }
}
