package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Every autonomous route the team has, as data. Each route is written ONCE, for the RED alliance;
 * the BLUE version is {@link AutoRoute#forAlliance} (a half turn about the field center).
 *
 * <p><b>To add an auto, add one entry to {@link #build()}</b> and re-run {@code tools/render-autos.sh}
 * (or {@code .bat}): the validator checks it, the timing model estimates it and the diagram of
 * it appears in {@code docs/autos/}. See {@code docs/AUTOS.md}.
 *
 * <p>The first three entries are faithful conversions of the existing OpModes
 * ({@code pedro/RedAlliance_B1}, {@code RedAlliance_C1}, {@code RedAlliance_LeavePark}); the numbers
 * are copied from their source, including the headings that Pedro ignores for reverse-tangent legs.
 * They are NOT improved here, so the validator reports their real problems honestly. The last two are
 * small demonstration routes that pass the validator.
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
        // pedro/RedAlliance_B1.java
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
                            + "by about 2.4 in only.")
                .build());

        // -----------------------------------------------------------------------------------------
        // pedro/RedAlliance_C1.java
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
                    .note("The source never says PARK; this pose overlaps the red LOADING ZONE by 7.9 x 9.8 in.")
                .build());

        // -----------------------------------------------------------------------------------------
        // pedro/RedAlliance_LeavePark.java
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
                    .note("Overlaps the LOADING ZONE corner by about 3 in; the source does not call it PARK.")
                .build());

        // -----------------------------------------------------------------------------------------
        // new demonstration routes (both pass the validator)
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(WALL_SHOOT_PARK)
                .title("Wall shoot and park: C1, shoot 4, left corridor to the LOADING ZONE")
                .description("Starts in tile C1, fires the 4 pre-loads at the UP cell from the standoff pose the solver "
                        + "picks, then drives out through the left corridor (x < 47, clear of the HIVE frame) and "
                        + "parks deep inside the red LOADING ZONE. LEAVE 3 + PARK 5; 4 POLLEN cannot tip the HIVE.")
                .startTile("C1")
                .start(58.0, 9.2, -90)
                .shootUp(4)
                    .note("AUTO pose: the solver stands about 34 in in front of the cell; the diagram shows the nominal pose.")
                .driveReverseTangent(30.0, 45.0).label("to corridor").via(30.0, 12.6)
                    .note("A curve: leaves the shooting pose sideways along the wall, then bends up into the corridor.")
                .driveReverseTangent(30.0, 100.0).label("corridor")
                    .note("Straight up the left corridor, 21 in clear of the HIVE frame keep-out.")
                .driveLinear(12.0, 108.0, 0.0).label("to zone")
                    .note("Turns to face +x while sliding into the zone.")
                .park(12.0, 108.0, 0.0)
                .build());

        routes.add(AutoRoute.builder(GARDEN_CYCLE)
                .title("Garden cycle: shoot 4, take the 4 GARDEN POLLEN, shoot 4 again")
                .description("Starts in tile B1, fires the 4 pre-loads, sweeps up the 4 POLLEN of the red GARDEN along "
                        + "the audience wall and fires them too. 8 POLLEN into the same UP cell is the HIVE TIP threshold, "
                        + "so the route claims LEAVE 3 + TIP 20.")
                .startTile("B1")
                .start(35.0, 9.2, -90)
                .shootUp(4)
                    .note("AUTO pose: the solver moves from B1 to the standoff pose in front of the UP cell.")
                .driveConstant(9.0, 18.0, -90).label("to GARDEN")
                    .note("Slides along the audience wall with the intake still facing it.")
                .driveConstant(9.0, 9.7, -90).label("into GARDEN").wallContact()
                    .note("Front edge ends 0.8 in from the wall, on the row of POLLEN (x 0..12).")
                .intakeGarden(5.9, 1.4, 9.0, 9.7, -90, 4)
                    .note("The 4 GARDEN POLLEN lie along the wall at x 1.4, 4.5, 7.3 and 10.3.")
                .shootUp(4)
                    .note("The same UP cell now holds 8 POLLEN: the HIVE tips.")
                .build());

        return routes;
    }
}
