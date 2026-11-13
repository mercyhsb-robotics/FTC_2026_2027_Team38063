package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;

/**
 * The autonomous routes that assume OUR robot shoots the first tip of the HIVE ({@link FirstTip#SELF}): the 4
 * pre-loads, together with the 3 NECTAR already in the UP cell, tip it (the 3rd POLLEN tips it and the 4th is the spare, see
 * {@link TipModel}),
 * and the rest of the route is planned from there. These routes work whatever the alliance partner's auto does with
 * our HIVE, as long as it does not shoot into it first.
 *
 * <p>To add a route, write ONE entry in {@link #routes()} (the same builder calls as in {@link RouteCatalog}; do not
 * call {@code partnerTipsFirst()}, a route in this file says SELF) and run {@code tools/render-autos.sh}. Routes
 * that need the partner to shoot first go to {@link PartnerTipRoutes}. {@link RouteCatalog#all()} lists the routes
 * of this file after its own; route ids must be unique across all of them.
 *
 * <p>No route is in this file yet.
 */
public final class SelfTipRoutes {
    private SelfTipRoutes() {
    }

    /** Every route of this family, authored for the RED alliance (BLUE is the half turn). A new list on every call. */
    public static List<AutoRoute> routes() {
        List<AutoRoute> routes = new ArrayList<AutoRoute>();
        // add routes here: routes.add(AutoRoute.builder("my_route")....build());
        return routes;
    }
}
