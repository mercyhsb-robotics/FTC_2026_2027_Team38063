package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.Locale;

/**
 * WHO shoots the first tip of our HIVE in the AUTO period, as an assumption a route is written under. The drive
 * team picks the auto that matches what the alliance partner's auto does, so every route says which one it is.
 *
 * <p>Why it matters: the HIVE has two cells and only the one that is UP may be launched into (rule G417). The
 * first tip needs only 3 POLLEN (the 3 staged NECTAR help, see {@link TipModel}). After it the OTHER cell
 * is UP and holds no NECTAR, so the next tip needs 8 POLLEN. If the partner robot shoots the first tip, our robot
 * should go to the side that comes up and start there. A HIVE belongs to the ALLIANCE, and a TIP is defined only by what the HIVE
 * does (the cells swap and the damper touches the frame), so whose POLLEN tipped it does not matter: the partner's launches count
 * for our HIVE and the 20 points are the alliance's.
 */
public enum FirstTip {
    /**
     * OUR robot shoots the first tip: the 3rd of its 4 pre-loads, together with the staged NECTAR, tips the HIVE (the 4th is the
     * spare). This is the default and what every route did before the partner family existed.
     */
    SELF,
    /**
     * The alliance PARTNER shoots the first tip. The HIVE is assumed to tip by itself at
     * {@code BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC} seconds into the match (the drive team must agree that
     * time with the partner team, and the robot never relies on it: it goes on only when its camera has SEEN the tipped cell
     * steady). The other cell comes up then, and our robot goes to that side. The partner's tip is worth 20 points to the alliance
     * (the HIVE is the alliance's, so the partner's launches count for it) but is NOT counted as ours.
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
