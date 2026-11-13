package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;

/**
 * The autonomous routes that assume the alliance PARTNER shoots the first tip of the HIVE ({@link FirstTip#PARTNER}):
 * the HIVE tips by itself at {@code Auto.PARTNER_FIRST_TIP_DONE_SEC} and the OTHER cell comes up, so our robot goes to
 * that side of the HIVE (for the red alliance the REAR side) and shoots there. That cell holds no NECTAR, so our own
 * tip needs 8 POLLEN: two volleys of 4, with a fetch from a FLOWER in between.
 *
 * <p>To add a route, write ONE entry in {@link #routes()} that starts with {@code partnerTipsFirst()}, uses
 * {@code waitForTip(timeoutSeconds)} before the first shot at the cell that comes up, and stays out of the spill zone
 * of the cell that tips (rule G409; the validator checks both). The drive team must agree the assumed tip time with
 * the partner team before the match. Routes that shoot the first tip themselves go to {@link SelfTipRoutes}.
 * {@link RouteCatalog#all()} lists the routes of this file last; route ids must be unique across the catalog.
 *
 * <p>No route is in this file yet.
 */
public final class PartnerTipRoutes {
    private PartnerTipRoutes() {
    }

    /** Every route of this family, authored for the RED alliance (BLUE is the half turn). A new list on every call. */
    public static List<AutoRoute> routes() {
        List<AutoRoute> routes = new ArrayList<AutoRoute>();
        // add routes here: routes.add(AutoRoute.builder("my_route").partnerTipsFirst()....build());
        return routes;
    }
}
