package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;

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
 * all: the partner's tip at 12 s plus the camera's 0.5 s settle, a volley of 4 (1.9 s aim and spin-up + 4 x 0.9 s), a FLOWER pick-up of 4 (about 7.4 s) and a
 * second volley of 4 add up to about 30.9 s, and 12 s is already twice what our own first volley takes. It needs a partner that tips by about 6 s AND
 * wheels that really reach the POLLEN (the validator says the shipped robot's wheels fall 0.8 in short, see {@code FlowerWiggleTest}), neither of which is known
 * yet; the assumed 12 s is the team's own guess ({@code Auto.PARTNER_FIRST_TIP_DONE_SEC}, VERIFY with the partner team) and is not changed here. So the
 * route below is the part that does fit: wait for the tip, put 4 POLLEN toward the second tip, LEAVE and PARK. The second tip (8 POLLEN in all) is then
 * for TELEOP. Once the partner's real time and the robot's wheel reach are measured, the full route is one more entry here.
 */
public final class PartnerTipRoutes {
    /** Id of the route that waits at the rear wall for the partner's tip, shoots 4 at the cell that comes up, then leaves and parks. */
    public static final String PARTNER_REAR_SHOOT_PARK = "partner_rear_shoot_park";

    private PartnerTipRoutes() {
    }

    /** Every route of this family, authored for the RED alliance (BLUE is the half turn). A new list on every call. */
    public static List<AutoRoute> routes() {
        List<AutoRoute> routes = new ArrayList<AutoRoute>();

        routes.add(AutoRoute.builder(PARTNER_REAR_SHOOT_PARK)
                .partnerTipsFirst()
                .title("Partner tips first: wait at the rear wall, shoot 4 at the cell that comes up, park")
                .description("Assumes the alliance PARTNER shoots the first tip (assumed done at 12 s, agree the real time with the partner team). Starts on the "
                        + "rear wall in tile C6, front edge on the wall, so the back (the shooter) faces the HIVE, and WAITS: the robot never shoots because a time "
                        + "has passed, only after its camera has SEEN the REAR cell come up and stay steady. Then it fires its 4 pre-loads into that cell from where it "
                        + "stands (it holds no NECTAR, so 4 POLLEN do not tip it: they are 4 of the 8 the second tip needs, which is for TELEOP), drives off the rear "
                        + "wall and parks in the red LOADING ZONE. LEAVE 3 + PARK 5 are ours; the partner's 20 is an assumption, not counted. The second tip does "
                        + "NOT fit in AUTO with the shipped numbers (see PartnerTipRoutes): a partner that tips by about 6 s and wheels that reach the FLOWER "
                        + "are needed first. If the tip never comes the robot gives up waiting, leaves the volley out and still leaves and parks.")
                .startTile("C6")
                .start(61.5, BioBuzzConstants.Field.SIZE_IN - BioBuzzConstants.Robot.FRONT_EXTENT_IN, 90)
                    .note("Front edge ON the rear wall (the center of rotation is " + BioBuzzConstants.Robot.FRONT_EXTENT_IN + " in from it), facing the wall, the middle of "
                            + "the robot over x = 61.5: its right edge is 1.7 in clear of the rear-wall FLOWER plate (G304.D), and the shooter end points at the HIVE.")
                .waitForTip(16.0)
                    .note("Waits for the CAMERA to see the REAR cell up and steady (0.5 s). 16 s is the give-up time on the route clock: later than that the rest "
                            + "(the volley and the drive to the zone) would not fit in the 30 s, so the robot skips the volley and goes on to LEAVE and PARK.")
                .shootUp(61.5, 130.0, 90, 4)
                    .note("From where it stands (about 3 in from the start): the back toward the rear cell, 35 in from its aim point, inside the 30 to 38 in band. "
                            + "4 POLLEN into an empty cell do not tip it (it needs 8); they are banked for the second tip.")
                .driveConstant(61.5, 112.0, 90).label("off the wall")
                    .note("Straight toward the HIVE side, away from the rear wall and the FLOWER; the whole robot ends more than 1.5 in off every wall (LEAVE).")
                .driveLinear(12.0, 108.0, 0.0).label("to zone")
                    .note("Along the corridor above the HIVE (clear of its keep-out) while turning to face +x, into the red LOADING ZONE.")
                .park(12.0, 108.0, 0.0)
                    .note("Overlaps the LOADING ZONE by 9.5 x 17.8 in, standing 4 in off the red wall so the LEAVE is kept too (QA21).")
                .build());

        return routes;
    }
}
