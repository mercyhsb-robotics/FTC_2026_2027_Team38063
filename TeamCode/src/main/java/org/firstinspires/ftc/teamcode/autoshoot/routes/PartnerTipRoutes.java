package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The autonomous routes that assume the alliance PARTNER shoots the first tip of the HIVE ({@link FirstTip#PARTNER}):
 * the HIVE tips by itself at {@code Auto.PARTNER_FIRST_TIP_DONE_SEC} and the OTHER cell comes up, so our robot goes to
 * that side of the HIVE (for the red alliance the REAR side) and shoots there. That cell holds no NECTAR, so our own
 * tip needs 8 POLLEN (plan 9, see {@link TipModel}): two volleys of 4 and a spare, with a fetch from a FLOWER in between.
 *
 * <p>To add a route, write ONE entry in {@link #routes()} that starts with {@code partnerTipsFirst()}, uses
 * {@code waitForTip(giveUpSec)} before the first shot at the cell that comes up, and stays out of the spill zone of
 * the cell that tips (rule G409; the validator checks both, and refuses a shot at that cell without the wait: the
 * robot shoots after its camera has SEEN the cell UP, never because of a time). {@code giveUpSec} is a time on the
 * route clock; the gallery shows the latest one that still lets the route finish and the latest partner tip that
 * still works. The drive team must agree the assumed tip time with the partner team before the match. Routes that
 * shoot the first tip themselves go to {@link SelfTipRoutes}.
 * {@link RouteCatalog#all()} lists the routes of this file last; route ids must be unique across the catalog.
 *
 * <p><b>What fits, and what does not.</b> The route the family was asked for is: partner tips, we shoot 4 into the empty cell that comes up, fetch 4 more
 * from the rear FLOWER and shoot 4 again for the SECOND tip (+20). With the shipped numbers it cannot fit in the 30 s of AUTO, even with no driving at
 * all: the partner's tip at 12 s plus the camera's 0.5 s settle, a volley of 4 (1.5 s aim and spin-up + 4 x 1.0 s), a FLOWER pick-up of 4 (about 7.3 s) and a
 * second volley of 4 add up to about 30.8 s, and 12 s is already twice what our own first volley takes. It needs a partner that tips by about 6 s AND
 * wheels that really reach the POLLEN (the validator says the shipped robot's wheels fall 0.8 in short, see {@code FlowerWiggleTest}), neither of which is known
 * yet; the assumed 12 s is the team's own guess ({@code Auto.PARTNER_FIRST_TIP_DONE_SEC}, VERIFY with the partner team) and is not changed here. So the
 * route below is the part that does fit: wait for the tip, put 4 POLLEN toward the second tip, LEAVE and PARK. The second tip (8 POLLEN in all) is then
 * for TELEOP. Once the partner's real time and the robot's wheel reach are measured, the full route is one more entry here.
 *
 * <p><b>Where the robot starts, and why not straight on the wall.</b> A robot standing straight on the rear wall, in line with the rear cell, would have
 * the gap between the rear-wall FLOWER (its bracket ends at x = 50.6 in) and the center line (x = 70.67 in) to stand in: about 20 in for a robot that is
 * 17.8 in wide. No place in it survives a hand that is 1 in and 3 degrees off ({@code StartRobustness}). The start search ({@code tools/start-shortcuts.sh})
 * found a start TURNED in the corner left of the FLOWER that does, and is also a shooting pose, so that is the one used. Its numbers are below.
 */
public final class PartnerTipRoutes {
    /** Id of the route that waits at the rear wall for the partner's tip, shoots 4 at the cell that comes up, then leaves and parks. */
    public static final String PARTNER_REAR_SHOOT_PARK = "partner_rear_shoot_park";

    /**
     * Where the robot is set down: the front-right corner on the rear wall, turned 35 degrees counter-clockwise from facing it, in tile B6. The start
     * search found it (the best start that keeps the 1 in and 3 degree placement margin); {@code StartPlacement} turns it into tape-measure words.
     */
    private static final double START_X_IN = 36.25;
    private static final double START_Y_IN = 128.24;
    private static final double START_HEADING_DEG = 125.0;

    /**
     * Where the robot fires from: the start moved about 1 in straight toward the cell it shoots at (along its own back direction). From the start itself the
     * launch point is 38.5 in from the aim point, 0.5 in outside the 30 to 38 in band of the validator; one inch closer it is 37.5 in, the standoff of the
     * only shot the team has measured (37.7 in).
     */
    private static final double SHOOT_X_IN = 36.8;
    private static final double SHOOT_Y_IN = 127.4;

    /**
     * The give-up time of the wait for the partner's tip, seconds on the route clock. It is the latest the rest of the route allows, rounded down to a
     * tenth ({@code RouteAnalysis.latestGiveUpSec}; a test keeps the two together): a partner that tips later than this gets no volley from us, and the
     * robot still leaves and parks.
     */
    static final double GIVE_UP_SEC = 19.4;

    private PartnerTipRoutes() {
    }

    /** Every route of this family, authored for the RED alliance (BLUE is the half turn). A new list on every call. */
    public static List<AutoRoute> routes() {
        List<AutoRoute> routes = new ArrayList<AutoRoute>();

        // the park pose of the route: the numbers the park note quotes come from the constants, so they cannot go stale
        double parkX = 12.0;
        double backEdgeFromWall = parkX - BioBuzzConstants.Robot.REAR_EXTENT_IN;
        double overlapDeep = BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - backEdgeFromWall;

        routes.add(AutoRoute.builder(PARTNER_REAR_SHOOT_PARK)
                .partnerTipsFirst()
                .title("Partner tips first: wait beside the cell that comes up, shoot 4 into it, park")
                .description("Assumes the alliance PARTNER shoots the first tip (assumed done at 12 s, agree the real time with the partner team). The robot starts "
                        + "TURNED in a corner of its own half, on the wall behind the cell that comes up with the partner's tip (for RED that is the rear wall, for BLUE "
                        + "the audience wall), with its back (the shooter) toward that cell, and WAITS: it never shoots because a time has passed, only after its camera "
                        + "has SEEN that cell come up and stay steady. Then it fires its 4 pre-loads into it (the cell holds no NECTAR, so 4 POLLEN do not tip it: they "
                        + "are 4 of the 8 the second tip needs, which is for TELEOP), drives off the wall and parks in its own LOADING ZONE. LEAVE 3 + PARK 5 are ours; "
                        + "the partner's 20 is an assumption, not counted. The second tip does NOT fit in AUTO with the shipped numbers (see PartnerTipRoutes): a partner "
                        + "that tips by about 6 s and wheels that reach the FLOWER are needed first. If the tip never comes the robot gives up waiting at "
                        + seconds(GIVE_UP_SEC) + ", leaves the volley out and still leaves and parks.")
                .startTile("B6")
                .start(START_X_IN, START_Y_IN, START_HEADING_DEG)
                    .note("Set down TURNED in the corner left of the wall FLOWER (rule G304 has no required heading): its front-right corner touches the wall, and its "
                            + "back (the shooter) already points at the cell that comes up, so there is no drive before the volley. A robot standing straight on that wall "
                            + "cannot be placed safely here: it is squeezed between the FLOWER and the center line, so a hand that is 1 in and 3 degrees off puts it over "
                            + "the line or onto the FLOWER. This turned start keeps that 1 in and 3 degree placement margin. The gallery prints the tape-measure "
                            + "words for each alliance.")
                .waitForTip(GIVE_UP_SEC)
                    .note("Waits for the CAMERA to see the cell up and steady (0.5 s). " + seconds(GIVE_UP_SEC) + " on the route clock is the give-up time, nearly the latest "
                            + "the rest of the route allows (the validator and the gallery say the exact latest): a partner that tips later than that gets no volley from "
                            + "us, the robot skips it and goes on to LEAVE and PARK.")
                .shootUp(SHOOT_X_IN, SHOOT_Y_IN, START_HEADING_DEG, 4)
                    .note("Steps about 1 in toward the cell first (the start itself is 0.5 in too far for the 30 to 38 in band), so the launch point is 37.5 in from the "
                            + "aim point: as near as we get to the 37.7 in of the only measured shot. 4 POLLEN into an empty cell do not tip it (it needs 8); they are "
                            + "banked for the second tip.")
                .driveConstant(START_X_IN, 112.0, START_HEADING_DEG).label("off the wall")
                    .note("Straight away from the wall toward the open floor beside the HIVE, keeping the heading and clear of the FLOWER; the whole robot ends more than "
                            + "1.5 in off every wall (LEAVE).")
                .driveLinear(parkX, 108.0, 0.0).label("to zone")
                    .note("Across the open floor between the HIVE keep-out and the side wall while turning to face the zone, into the own LOADING ZONE.")
                .park(parkX, 108.0, 0.0)
                    .note(String.format(Locale.ROOT, "Overlaps the LOADING ZONE by %.1f in (the zone is %.0f in deep), standing %.1f in off the wall so the LEAVE is kept too (QA21).",
                            overlapDeep, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, backEdgeFromWall))
                .build());

        return routes;
    }

    private static String seconds(double v) {
        return String.format(Locale.ROOT, "%.1f s", v);
    }
}
