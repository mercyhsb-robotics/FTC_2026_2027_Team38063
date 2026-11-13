package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;

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
 * tip of the HIVE: the 3 NECTAR staged in the UP cell mean the 3rd POLLEN tips it (FIRST's calibration, see
 * {@link TipModel}), so the 4 pre-loads tip it with 1 spare, and a volley that tips stops there and keeps the rest.
 *
 * <p><b>Where the robot stands at the start.</b> A pose is the robot's CENTER OF ROTATION, which is not the middle of
 * its 17.80 x 17.62 in outline: the front edge is 9.77 in ahead of it, the back edge 8.03 in behind it
 * ({@code Robot.FRONT_EXTENT_IN} and {@code REAR_EXTENT_IN}, measured by the team). So a robot that faces the
 * audience wall with its FRONT edge on the wall has its pose 9.77 in from it ({@link #FRONT_ON_WALL_IN}); the old OpModes
 * wrote 9.2 in, a number from before the center of rotation was known, and the validator reports honestly that the real
 * footprint would then reach 0.57 in through the wall. Place the real robot with its front edge ON the wall.
 */
public final class RouteCatalog {
    /**
     * How far the robot's center of rotation is from a wall that its FRONT edge is touching, inches: the robot faces the wall.
     * The new demonstration routes start (and stand at the GARDEN) at this distance.
     */
    static final double FRONT_ON_WALL_IN = BioBuzzConstants.Robot.FRONT_EXTENT_IN;

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
                .title("Copy of RedAlliance_B1: shoot 4, then drive along the side wall to the LOADING ZONE")
                .description("Conversion of RedAlliance_B1.java. Starts in tile B1, turns to -115 deg and fires the 4 "
                        + "pre-loads at the UP cell, waits for the launcher, then drives backwards up the left corridor "
                        + "to (35.8, 90) and sideways to (12, 90), ending partly in the red LOADING ZONE. The staged NECTAR plus the "
                        + "3rd POLLEN tip the HIVE (4 pre-loads, 1 spare), but the old OpMode feeds for a fixed time and cannot stop, "
                        + "so its 4th POLLEN goes into a cell that is already tipping: the validator says so.")
                .startTile("B1")
                .start(35.0, 9.2, -90)
                    .note("The OpMode's number, copied. The robot's center of rotation is 9.2 in from the audience wall, but its "
                            + "front edge is " + FRONT_ON_WALL_IN + " in ahead of the center, so the real footprint would reach "
                            + "0.57 in through the wall: place the real robot with its FRONT ON the wall (center " + FRONT_ON_WALL_IN
                            + " in out) and change the OpMode's y to match once the center of rotation is measured.")
                .shootUp(35.0, 9.2, -115, 4).launchSeconds(5.0)
                    .note("hold(follower, (35.0, 9.2, -115)) then aimAndLaunch(): spin up, feed, waitMs(5000). hold() is an "
                            + "instant command, so the shot really starts without waiting for the turn. The model charges the flat 5 s only: the OpMode first waits for the "
                            + "flywheel to reach speed (waitUntil isAtTargetRpm, about 1.9 s in the model's own numbers), so the real tip and finish times are about that much later "
                            + "than shown.")
                .waitSeconds(4.5)
                    .note("waitMs(4500) before launcher.stop(): the volley holds the launcher on for 9.5 s in total.")
                .driveReverseTangent(35.8, 90, -90).label("path1")
                    .note("line(start, point1).reverseTangent(): shooter-first up the left corridor.")
                .driveReverseTangent(12, 90, 0).label("path2")
                    .note("line(point1, point2).reverseTangent(): the 0 deg heading in the source is ignored; the robot "
                            + "turns to face +x by itself because it drives toward -x backwards.")
                .park(12, 90, 0)
                    .note(String.format(Locale.ROOT, "The source never says PARK, but this pose is the red LOADING ZONE corner: it overlaps the zone "
                            + "by %.1f in (the robot reaches %.2f in to its left, y %.1f; the zone, taken from the CAD tape, starts at y %.2f), "
                            + "deep enough for PARK. With the old 144 in field numbers the zone started at y 96 and this was only 2.9 in.",
                            parkDepth(12, 90, 0), BioBuzzConstants.Robot.LEFT_EXTENT_IN, 90 + BioBuzzConstants.Robot.LEFT_EXTENT_IN,
                            BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN))
                .build());

        // -----------------------------------------------------------------------------------------
        // RedAlliance_C1.java
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(RED_C1)
                .title("Copy of RedAlliance_C1: shoot 4, then through the HIVE to the LOADING ZONE (do not run)")
                .description("Conversion of RedAlliance_C1.java. Starts in tile C1 with the back toward the UP cell, "
                        + "fires the 4 pre-loads, waits, then drives (58, 9.2) -> (62, 118.1) -> (58, 118.1) -> (12, 118.1), "
                        + "all backwards. The first leg is a straight line through the HIVE footprint. The staged NECTAR plus the "
                        + "3rd POLLEN tip the HIVE (4 pre-loads, 1 spare), but the old OpMode feeds for a fixed time and cannot stop, "
                        + "so its 4th POLLEN goes into a cell that is already tipping: the validator says so.")
                .startTile("C1")
                .start(58.0, 9.2, -90)
                    .note("The OpMode's number, copied. The center of rotation is 9.2 in from the audience wall but the front edge is "
                            + FRONT_ON_WALL_IN + " in ahead of it: the real footprint would reach 0.57 in through the wall. Place the "
                            + "real robot with its FRONT ON the wall (center " + FRONT_ON_WALL_IN + " in out) and change the OpMode's y to match.")
                .shootUp(58.0, 9.2, -90, 4).launchSeconds(5.0)
                    .note("hold(follower, (58.0, 9.2, -90)) then aimAndLaunch(): fixed waitMs(5000), after a wait for the flywheel to reach speed that the model does not charge (about 1.9 s), so the real times are later than shown.")
                .waitSeconds(4.5)
                    .note("waitMs(4500) before launcher.stop().")
                .driveReverseTangent(62.0, 118.1, -90).label("path1")
                    .note("line(start, point1).reverseTangent(): a straight line across the HIVE frame footprint.")
                .driveReverseTangent(58.0, 118.1, -90).label("path2")
                    .note("4 in long: the robot has to turn about 90 deg inside it (reverse tangent now points to +x).")
                .driveReverseTangent(12.0, 118.1, -90).label("path3")
                    .note("The -90 deg heading in the source is ignored: the robot ends facing 0 deg.")
                .park(12.0, 118.1, 0)
                    .note(String.format(Locale.ROOT, "The source never says PARK; this pose overlaps the red LOADING ZONE by %s in (the back edge is 8.03 in behind the center of rotation; the zone ends at y 117.27).",
                            parkSizes(12.0, 118.1, 0)))
                .build());

        // -----------------------------------------------------------------------------------------
        // RedAlliance_LeavePark.java
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(RED_LEAVE_PARK)
                .title("Copy of RedAlliance_LeavePark: leave the wall and park (no shooting)")
                .description("Conversion of RedAlliance_LeavePark.java. Starts on the left wall in tile A3 facing 180 deg, "
                        + "drives to (24.86, 83.62) turning to 135 deg, then to (16, 91.5): LEAVE and a PARK with only a corner in the zone. "
                        + "The 4 pre-loads are never fired.")
                .startTile("A3")
                .start(9.2112, 70.2702, 180)
                    .note("The OpMode's number, copied (its comment says the front is about 0.3 in from the red wall). The center of "
                            + "rotation is 9.21 in from the wall but the front edge is " + FRONT_ON_WALL_IN + " in ahead of it, so the real "
                            + "footprint would reach 0.56 in through the wall: place the real robot with its FRONT ON the wall (center "
                            + FRONT_ON_WALL_IN + " in out) and change the OpMode's x to match once the center of rotation is measured.")
                .driveLinear(24.8608, 83.6152, 135).label("path1")
                    .note("Paths.line(start, path1).linear(start, path1)")
                .driveLinear(16.0, 91.5, 135).label("path2")
                    .note("Paths.line(point2Start, point2).linear(point2Start, point2)")
                .park(16.0, 91.5, 135)
                    .note("Only a corner of the robot reaches the LOADING ZONE, 5.7 in deep; the source does not call it PARK.")
                .build());

        // -----------------------------------------------------------------------------------------
        // new demonstration routes (both pass the validator)
        // -----------------------------------------------------------------------------------------
        routes.add(AutoRoute.builder(WALL_SHOOT_PARK)
                .title("Shoot 4 from the start wall (tips the HIVE), side corridor to the LOADING ZONE, park")
                .description("Starts in tile C1, fires the 4 pre-loads at the UP cell from the standoff pose the solver "
                        + "picks, then drives out through the left corridor (x < 43, clear of the HIVE frame keep-out) and "
                        + "parks deep inside the red LOADING ZONE. The 3 NECTAR already in the UP cell plus the 3rd POLLEN tip "
                        + "the HIVE (FIRST's calibration), so the 4 pre-loads tip it with 1 spare for a HIVE that needs one more POLLEN. "
                        + "The camera sees the tip only about 0.85 s after the 3rd POLLEN lands, so the 4th is probably launched into the cell "
                        + "while it swings down and is spent (the validator says so; measure the real time on day one). The route "
                        + "claims LEAVE 3 + PARK 5 + our TIP 20: it parks standing about 4 in off the red wall, because a robot that "
                        + "touches ANY wall at the end of AUTO, the one inside the LOADING ZONE too, loses its LEAVE (QA21).")
                .startTile("C1")
                .start(58.0, FRONT_ON_WALL_IN, -90)
                    .note("Front edge ON the audience wall (the center of rotation is " + FRONT_ON_WALL_IN + " in out). Place the robot with the "
                            + "wheel tips touching the wall and the middle of the robot over tile C1.")
                .shootUp(4)
                    .note("AUTO pose: the solver stands about 34 in in front of the cell; the diagram shows the nominal pose. "
                            + "The 3 staged NECTAR plus the 3rd POLLEN tip the HIVE: one volley is the first tip, and the 4th "
                            + "pre-load is the spare for a stiff HIVE (the camera cannot stop the feeder before the 4th is launched, so it "
                            + "is probably spent).")
                .driveConstant(30.0, 14.8, -90.0).label("along the wall")
                    .note(String.format(Locale.ROOT, "Slides sideways along the audience wall, STAYING LOW and keeping the front toward the wall (at the nominal shooting pose "
                            + "the center is %.1f in from the wall, only %.1f in more than the %.1f in a robot needs to turn on the spot there). ",
                            nominalAudienceY(), nominalAudienceY() - Footprint.circumscribedRadiusIn(), Footprint.circumscribedRadiusIn())
                            + "Staying low matters for the tip: what falls out of the tipped cell lands up to 40 in out from the HIVE's middle line, "
                            + "and the robot's rear edge (14.8 + 8.03 = 22.8 in from the wall) is 47.8 in out, so it is clear of the spill (G409) "
                            + "until it is beside the cell, where it turns up the corridor.")
                .driveReverseTangent(30.0, 100.0).label("corridor")
                    .note("Straight up the left corridor, backwards (the shooter end first), 12.9 in (center to keep-out) clear of the HIVE frame "
                            + "keep-out at x 42.94 and clear of the spill zone (x 41.39 and more) beside it.")
                .driveLinear(12.0, 108.0, 0.0).label("to zone")
                    .note("Turns to face +x while sliding into the zone.")
                .park(12.0, 108.0, 0.0)
                .build());

        routes.add(AutoRoute.builder(GARDEN_CYCLE)
                .title("Shoot 4 (tips the HIVE, 1 spare), take 3 GARDEN POLLEN, step off the wall")
                .description("Starts in tile B1 and fires the 4 pre-loads at the UP cell. With the 3 NECTAR already in that "
                        + "cell the 3rd POLLEN tips the HIVE (+20); the 4th pre-load is the spare for a stiff HIVE, and because the camera "
                        + "sees the tip only about 0.85 s after the 3rd POLLEN lands it is probably launched into the swinging cell and spent. After a "
                        + "tip the UP cell is on the OTHER side of the HIVE, about 170 in of driving away around it, and it holds no NECTAR, "
                        + "so it needs 8 POLLEN (plan 9): far more than this route can carry there. So the route does not shoot "
                        + "again: it sweeps up 3 of the 4 POLLEN of the red GARDEN along the audience wall (the robot holds 0 + 3 = 3; "
                        + "if the feeder does stop in time it holds 1 + 3 = 4, the most it may hold, G407, so never take a 4th) to carry "
                        + "into TELEOP and steps off the wall. LEAVE 3 + our TIP 20.")
                .startTile("B1")
                .start(35.0, FRONT_ON_WALL_IN, -90)
                    .note("Front edge ON the audience wall (the center of rotation is " + FRONT_ON_WALL_IN + " in out). Place the robot with the "
                            + "wheel tips touching the wall and the middle of the robot over tile B1.")
                .shootUp(4)
                    .note("AUTO pose: the solver moves from B1 to the standoff pose in front of the UP cell. The 3 staged "
                            + "NECTAR plus the 3rd POLLEN tip the HIVE and the 4th pre-load is probably spent (the feeder stops only "
                            + "about 0.85 s after the tip); the REAR cell is the UP one from now on.")
                .driveConstant(9.0, 18.0, -90).label("to GARDEN")
                    .note("Slides along the audience wall with the intake still facing it. The robot's side edge is only " + String.format(Locale.ROOT, "%.1f", 9.0 - BioBuzzConstants.Robot.LEFT_EXTENT_IN) + " in from the red wall all the way to the GARDEN and "
                            + "back (by design, so that the wheel can reach the POLLEN): touching a wall is legal, but a heading error of about half a degree would press a corner into it.")
                .driveConstant(9.0, FRONT_ON_WALL_IN + 0.8, -90).label("into GARDEN").wallContact()
                    .note("Front edge ends 0.8 in from the wall, on the row of POLLEN (x 0..12).")
                .intakeGarden(5.7, 1.4, 9.0, FRONT_ON_WALL_IN + 0.8, -90, 3)
                    .note("NOT MEASURED YET (day one): at this pose a flat wheel disc sits over two GARDEN POLLEN (its middle is 1.6 to 1.7 in from them, and a wheel and a POLLEN "
                            + "need 2.8 in between their middles), so which 3 of the 4 are taken, the +3 and the 2.5 s are placeholders until the intake is tried on the real GARDEN. "
                            + "The 4 GARDEN POLLEN lie along the wall at x 1.40, 4.29, 7.18 and 10.07 (CAD). The robot holds 0 (the spare "
                            + "pre-load was probably spent) or 1 (the feeder stopped in time) and takes 3, so it carries 3 or 4 into TELEOP; "
                            + "4 is the most it may hold (G407), so the intake never takes a 4th.")
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

    // ---- numbers the notes quote, worked out from the constants so they cannot go stale -----------------------------------------------

    /** How deep (the smaller of the two overlaps) the robot parked at this pose reaches into the red LOADING ZONE, inches. */
    private static double parkDepth(double x, double y, double headingDeg) {
        Rect c = Footprint.of(Pose2.ofDegrees(x, y, headingDeg)).clippedBounds(new AllianceField(new FieldModel(), Alliance.RED).ownLoadingZone());
        return Math.min(c.maxX - c.minX, c.maxY - c.minY);
    }

    /** The two overlaps of the robot parked at this pose with the red LOADING ZONE, as "7.0 x 8.0". */
    private static String parkSizes(double x, double y, double headingDeg) {
        Rect c = Footprint.of(Pose2.ofDegrees(x, y, headingDeg)).clippedBounds(new AllianceField(new FieldModel(), Alliance.RED).ownLoadingZone());
        return String.format(Locale.ROOT, "%.1f x %.1f", c.maxX - c.minX, c.maxY - c.minY);
    }

    /** How far from the audience wall the center of rotation stands at the nominal shooting pose of the audience cell, inches. */
    private static double nominalAudienceY() {
        CellSpec cell = new FieldModel().cell(Alliance.RED, Side.AUDIENCE);
        return ShotGeometry.nominalPose(cell).y;
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
