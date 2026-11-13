package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Every autonomous route the team has, as data. Each route is written ONCE, for the RED alliance;
 * the BLUE version is {@link AutoRoute#forAlliance} (a half turn about the field center).
 *
 * <p>The catalog is the routes written here (the three conversions and the two demonstrations), then the routes of
 * {@link SelfTipRoutes} (we shoot the first tip) and {@link PartnerTipRoutes} (the partner shoots it), in that order.
 * Every route says in its data which of the two it assumes ({@link AutoRoute#firstTip()}).
 *
 * <p><b>To add an auto, add one entry to {@link #build()}</b> and re-run {@code tools/render-autos.sh}
 * (or {@code .bat}): the validator checks it, the timing model estimates it and the diagram of
 * it appears in {@code docs/autos/}. See {@code docs/AUTOS.md}.
 *
 * <p>The first three entries are faithful conversions of the existing OpModes
 * ({@code RedAlliance_B1}, {@code RedAlliance_C1}, {@code RedAlliance_LeavePark}); the numbers
 * are copied from their source, including the headings that Pedro ignores for reverse-tangent legs.
 * They are NOT improved here, so the validator reports their real problems honestly. The last two are
 * small demonstration routes that pass the validator. Every route here assumes that OUR robot shoots the first
 * tip of the HIVE (the 3 NECTAR staged in the UP cell make one volley of 4 POLLEN enough).
 */
public final class RouteCatalog {
    public static final String RED_B1 = "red_b1";
    public static final String RED_C1 = "red_c1";
    public static final String RED_LEAVE_PARK = "red_leave_park";
    public static final String WALL_SHOOT_PARK = "wall_shoot_park";
    public static final String GARDEN_CYCLE = "garden_cycle";

    private static final List<AutoRoute> ALL = Collections.unmodifiableList(build());

    private RouteCatalog() {
    }

    /** All routes, in a stable order, authored in the RED frame. */
    public static List<AutoRoute> all() {
        return ALL;
    }

    /** The route with this id. Throws IllegalArgumentException (naming the known ids) if there is none. */
    public static AutoRoute byId(String id) {
        for (AutoRoute r : ALL) {
            if (r.id().equals(id)) {
                return r;
            }
        }
        List<String> ids = new ArrayList<String>();
        for (AutoRoute r : ALL) {
            ids.add(r.id());
        }
        throw new IllegalArgumentException("no auto route with id '" + id + "'; known ids: " + ids);
    }

    private static List<AutoRoute> build() {
        List<AutoRoute> routes = new ArrayList<AutoRoute>();

        // -----------------------------------------------------------------------------------------
        // RedAlliance_B1.java
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(RED_B1)
                .title("Red B1: shoot, then drive up the left side to the LOADING ZONE")
                .description("Conversion of RedAlliance_B1.java. Starts in tile B1, turns to -115 deg and fires the 4 "
                        + "pre-loads at the UP cell, waits for the launcher, then drives backwards up the left corridor "
                        + "to (35.8, 90) and sideways to (12, 90), ending partly in the red LOADING ZONE.")
                .startTile("B1")
                .start(35.0, 9.2, -90)
                .shootUp(35.0, 9.2, -115, 4).launchSeconds(5.0)
                    .note("hold(follower, (35.0, 9.2, -115)) then aimAndLaunch(): spin up, feed, waitMs(5000). hold() is an "
                            + "instant command, so the shot really starts without waiting for the turn.")
                .waitSeconds(4.5)
                    .note("waitMs(4500) before launcher.stop(): the volley holds the launcher on for 9.5 s in total.")
                .driveReverseTangent(35.8, 90, -90).label("path1")
                    .note("line(start, point1).reverseTangent(): shooter-first up the left corridor.")
                .driveReverseTangent(12, 90, 0).label("path2")
                    .note("line(point1, point2).reverseTangent(): the 0 deg heading in the source is ignored; the robot "
                            + "turns to face +x by itself because it drives toward -x backwards.")
                .park(12, 90, 0)
                    .note("The source never says PARK, but this pose is the red LOADING ZONE corner: it overlaps the zone "
                            + "by only 2.9 in (the 17.80 in square robot reaches y 98.9; the zone starts at y 96).")
                .build());

        // -----------------------------------------------------------------------------------------
        // RedAlliance_C1.java
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(RED_C1)
                .title("Red C1: shoot, then drive past the HIVE to the LOADING ZONE")
                .description("Conversion of RedAlliance_C1.java. Starts in tile C1 with the back toward the UP cell, "
                        + "fires the 4 pre-loads, waits, then drives (58, 9.2) -> (62, 118.1) -> (58, 118.1) -> (12, 118.1), "
                        + "all backwards. The first leg is a straight line through the HIVE footprint.")
                .startTile("C1")
                .start(58.0, 9.2, -90)
                .shootUp(58.0, 9.2, -90, 4).launchSeconds(5.0)
                    .note("hold(follower, (58.0, 9.2, -90)) then aimAndLaunch(): fixed waitMs(5000).")
                .waitSeconds(4.5)
                    .note("waitMs(4500) before launcher.stop().")
                .driveReverseTangent(62.0, 118.1, -90).label("path1")
                    .note("line(start, point1).reverseTangent(): a straight line across the HIVE frame footprint.")
                .driveReverseTangent(58.0, 118.1, -90).label("path2")
                    .note("4 in long: the robot has to turn about 90 deg inside it (reverse tangent now points to +x).")
                .driveReverseTangent(12.0, 118.1, -90).label("path3")
                    .note("The -90 deg heading in the source is ignored: the robot ends facing 0 deg.")
                .park(12.0, 118.1, 0)
                    .note("The source never says PARK; this pose overlaps the red LOADING ZONE by 7.9 x 10.8 in.")
                .build());

        // -----------------------------------------------------------------------------------------
        // RedAlliance_LeavePark.java
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(RED_LEAVE_PARK)
                .title("Red A3: leave the wall and park (no shooting)")
                .description("Conversion of RedAlliance_LeavePark.java. Starts on the left wall in tile A3 facing 180 deg, "
                        + "drives to (24.86, 83.62) turning to 135 deg, then to (16, 91.5): LEAVE and a shallow PARK. "
                        + "The 4 pre-loads are never fired.")
                .startTile("A3")
                .start(9.2112, 70.2702, 180)
                .driveLinear(24.8608, 83.6152, 135).label("path1")
                    .note("Paths.line(start, path1).linear(start, path1)")
                .driveLinear(16.0, 91.5, 135).label("path2")
                    .note("Paths.line(point2Start, point2).linear(point2Start, point2)")
                .park(16.0, 91.5, 135)
                    .note("Only a corner of the robot reaches the LOADING ZONE, 3.1 in deep; the source does not call it PARK.")
                .build());

        // -----------------------------------------------------------------------------------------
        // new demonstration routes (both pass the validator)
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(WALL_SHOOT_PARK)
                .title("Wall shoot and park: C1, shoot 4 (tips the HIVE), left corridor to the LOADING ZONE")
                .description("Starts in tile C1, fires the 4 pre-loads at the UP cell from the standoff pose the solver "
                        + "picks, then drives out through the left corridor (x < 47, clear of the HIVE frame) and "
                        + "parks deep inside the red LOADING ZONE. The 4 POLLEN land on top of the NECTAR that already "
                        + "sit in the UP cell and tip the HIVE, so the route claims LEAVE 3 + PARK 5 + our TIP 20.")
                .startTile("C1")
                .start(58.0, 9.2, -90)
                .shootUp(4)
                    .note("AUTO pose: the solver stands about 34 in in front of the cell; the diagram shows the nominal pose. "
                            + "The staged NECTAR in the cell plus these 4 POLLEN tip the HIVE: one volley is enough for the first tip.")
                .driveReverseTangent(30.0, 45.0).label("to corridor").via(30.0, 12.6)
                    .note("A curve: leaves the shooting pose sideways along the wall, then bends up into the corridor.")
                .driveReverseTangent(30.0, 100.0).label("corridor")
                    .note("Straight up the left corridor, 21 in clear of the HIVE frame keep-out.")
                .driveLinear(12.0, 108.0, 0.0).label("to zone")
                    .note("Turns to face +x while sliding into the zone.")
                .park(12.0, 108.0, 0.0)
                .build());

        routes.add(AutoRoute.builder(GARDEN_CYCLE)
                .title("Garden cycle: shoot 4 to tip the HIVE, take the 4 GARDEN POLLEN, step off the wall")
                .description("Starts in tile B1 and fires the 4 pre-loads at the UP cell. With the NECTAR already in that "
                        + "cell they tip the HIVE (+20). After a tip the UP cell is on the OTHER side of the HIVE, about "
                        + "111 in away around it, and it holds no NECTAR, so a second volley of 4 there would not tip it "
                        + "(that needs 8 POLLEN). So the route does not shoot again: it sweeps up the 4 POLLEN of the red "
                        + "GARDEN along the audience wall to carry into TELEOP and steps off the wall. LEAVE 3 + our TIP 20.")
                .startTile("B1")
                .start(35.0, 9.2, -90)
                .shootUp(4)
                    .note("AUTO pose: the solver moves from B1 to the standoff pose in front of the UP cell. The staged "
                            + "NECTAR plus these 4 POLLEN tip the HIVE, and the REAR cell is the UP one from now on.")
                .driveConstant(9.0, 18.0, -90).label("to GARDEN")
                    .note("Slides along the audience wall with the intake still facing it.")
                .driveConstant(9.0, 9.7, -90).label("into GARDEN").wallContact()
                    .note("Front edge ends 0.8 in from the wall, on the row of POLLEN (x 0..12).")
                .intakeGarden(5.9, 1.4, 9.0, 9.7, -90, 4)
                    .note("The 4 GARDEN POLLEN lie along the wall at x 1.4, 4.5, 7.3 and 10.3. The robot now holds 4 for TELEOP.")
                .driveConstant(20.0, 24.0, -90).label("off the wall")
                    .note("Ends with the whole footprint more than 1.5 in off every wall: LEAVE.")
                .build());

        // -----------------------------------------------------------------------------------------
        // the two families of routes, each in its own file (see their documentation)
        // -----------------------------------------------------------------------------------------
        routes.addAll(SelfTipRoutes.routes());
        routes.addAll(PartnerTipRoutes.routes());

        return routes;
    }

    /**
     * Describes every route of a family file that does not assume {@code expected} (the files are
     * {@link SelfTipRoutes}, which must be SELF, and {@link PartnerTipRoutes}, which must be PARTNER), so that a route
     * cannot sit in the wrong file and print the wrong assumption. Empty when the family is fine.
     */
    static List<String> familyProblems(String fileName, List<AutoRoute> routes, FirstTip expected) {
        List<String> problems = new ArrayList<String>();
        for (AutoRoute r : routes) {
            if (r.firstTip() != expected) {
                problems.add("route '" + r.id() + "' is in " + fileName + " but assumes " + r.firstTip() + "; every route in that file"
                        + " must assume " + expected + (expected == FirstTip.PARTNER ? " (start its entry with partnerTipsFirst())"
                        : " (remove partnerTipsFirst() or move the route to PartnerTipRoutes)"));
            }
        }
        return problems;
    }
}
