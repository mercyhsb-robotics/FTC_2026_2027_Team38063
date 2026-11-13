package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
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
import org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid;

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
 * 17.62 in wide. No place in it survives a hand that is 1 in and 3 degrees off ({@code StartRobustness}). The start search ({@code tools/start-shortcuts.sh})
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
     * Where the robot fires from: the start moved about 2 in straight toward the cell it shoots at (along its own back direction). From the start itself the
     * launch point is 41.1 in from the aim point, 1.6 in outside the 30 to 39.5 in band of the validator; two inches closer it is 39.0 in, the range of the anchor
     * spot (front on the audience wall, in line with the HIVE: 39.0 in), which is as near as the corner allows to the one shot the team saw work.
     * (A test keeps these numbers and the texts that quote them true.)
     */
    private static final double SHOOT_X_IN = 37.5;
    private static final double SHOOT_Y_IN = 126.5;

    /**
     * The give-up time of the wait for the partner's tip, seconds on the route clock. It is the latest the rest of the route allows, rounded down to a
     * tenth ({@code RouteAnalysis.latestGiveUpSec}; a test keeps the two together): a partner that tips later than this gets no volley from us, and the
     * robot still leaves and parks.
     */
    static final double GIVE_UP_SEC = 20.0;

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
                .title("Standard: start turned in a corner, wait beside the cell that comes up, shoot 4 into it, park")
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
                            + "back (the shooter) already points at the cell that comes up, so only a short step toward it comes before the volley. A robot standing straight on that wall "
                            + "cannot be placed safely here: it is squeezed between the FLOWER and the center line, so a hand that is 1 in and 3 degrees off puts it over "
                            + "the line or onto the FLOWER. This turned start keeps that 1 in and 3 degree placement margin. The gallery prints the tape-measure "
                            + "words for each alliance.")
                .driveLinear(SHOOT_X_IN, SHOOT_Y_IN, START_HEADING_DEG).label("step toward the cell")
                    .note(String.format(Locale.ROOT, "Steps about %.0f in toward the cell first (the start itself is %s in from the aim point, outside the %.1f in far end of the band), "
                            + "so the launch point is %s in from the aim point: the range of the anchor spot, where the team saw a shot work (%.1f in). 4 POLLEN into an empty cell "
                            + "do not tip it (it needs 8); they are banked for the second tip.",
                            Math.hypot(SHOOT_X_IN - START_X_IN, SHOOT_Y_IN - START_Y_IN), standoffText(START_X_IN, START_Y_IN, START_HEADING_DEG),
                            BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN, standoffText(SHOOT_X_IN, SHOOT_Y_IN, START_HEADING_DEG), BioBuzzConstants.AutoShoot.PREFERRED_RANGE_IN))
                .waitForTip(GIVE_UP_SEC)
                    .note("Waits for the CAMERA to see the cell up and steady (0.5 s). " + seconds(GIVE_UP_SEC) + " on the route clock is the give-up time, nearly the latest "
                            + "the rest of the route allows (the validator and the gallery say the exact latest): a partner that tips later than that gets no volley from "
                            + "us, the robot skips it and goes on to LEAVE and PARK.")
                .shootUp(SHOOT_X_IN, SHOOT_Y_IN, START_HEADING_DEG, 4)
                    .note("4 POLLEN into an empty cell do not tip it (it needs 8); they are banked for the second tip.")
                .driveConstant(START_X_IN, 112.0, START_HEADING_DEG).label("off the wall")
                    .note("Straight away from the wall toward the open floor beside the HIVE, keeping the heading and clear of the FLOWER; the whole robot ends more than "
                            + "1.5 in off every wall (LEAVE).")
                .driveLinear(parkX, 108.0, 0.0).label("to zone")
                    .note("Across the open floor between the HIVE keep-out and the side wall while turning to face the zone, into the own LOADING ZONE.")
                .park(parkX, 108.0, 0.0)
                    .note(String.format(Locale.ROOT, "Overlaps the LOADING ZONE by %.1f in (the zone is %.0f in deep), standing %.1f in off the wall so the LEAVE is kept too (QA21).",
                            overlapDeep, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, backEdgeFromWall))
                .build());

        addRoutesFromEveryStart(routes);
        return routes;
    }

    // =====================================================================================================================================
    // The routes that start somewhere else. Every one is written the same way: drive to a shooting pose at the rear cell WHILE the partner shoots (the
    // wait for the tip hides the drive), wait there until the camera has seen the cell up and steady, shoot 4, leave and park.
    // =====================================================================================================================================

    public static final String SAFE_REAR_WALL_B6 = "partner_safe_rear_wall_b6";
    public static final String SAFE_RED_CORNER_A6 = "partner_safe_red_corner_a6";
    public static final String STANDARD_RED_WALL_A4 = "partner_standard_red_wall_a4";
    public static final String STANDARD_RED_WALL_A2 = "partner_standard_red_wall_a2";
    public static final String STANDARD_B1_CORRIDOR = "partner_standard_b1_corridor";
    public static final String STANDARD_C1_CORRIDOR = "partner_standard_c1_corridor";
    public static final String STANDARD_B1_BACK_FIRST = "partner_standard_b1_back_first";
    public static final String AGGRESSIVE_REAR_EDGE = "partner_aggressive_rear_edge";

    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final double REAR_WALL_FRONT_ON_Y = SIZE - BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double AXIS_X = BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN;
    private static final double SIDE_ON_WALL_X = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
    /** The y of the long drive east along the rear side: the front edge then stays about 3.5 in below the FLOWER plate, so a start that is a little off still clears it. */
    private static final double ACROSS_Y = 123.0;
    /** The x of the drive up the left corridor: its right edge is 8 in short of the HIVE keep-out, its left edge 17 in off the wall. */
    private static final double CORRIDOR_X = 26.0;
    /** Where the drive away from the rear wall after the volley ends, before the turn toward the zone. */
    private static final double EXIT_Y = 112.0;

    /** Shooting pose 1: on the rear wall right of the FLOWER (its left edge 0.9 in past the FLOWER keep-out, its right edge 1.4 in short of the center line), facing the wall, 38.5 in from the aim point (inside the band: 30 to 39.5 in). */
    private static final double[] SHOOT_1 = {60.5, REAR_WALL_FRONT_ON_Y - 0.6, 90.0};
    /** Shooting pose 2: in line with the cell, 32.8 in from the aim point (the band is 30 to 39.5 in), well below the FLOWER plate. */
    private static final double[] SHOOT_2 = {AXIS_X, 125.4, 90.0};
    /** Shooting pose 3: 10 degrees to the left of the cell's axis (its back still points at the cell), 33.0 in out, on the LOADING ZONE side of the cell and inside the 20 degree cone. */
    private static final double[] SHOOT_3 = {51.27, 124.94, 100.0};
    /** The x of the Aggressive route's start: the nearest straight place to the rear FLOWER from which a start that is 1 in and 3 degrees off still clears it on the first drive. */
    private static final double AGGRESSIVE_START_X = 32.5;
    /** Park pose 1: facing +x, 4 in off the wall, 7 in into the zone. Park pose 2: facing the red wall (front first), 2.7 in off it, 8.3 in into the zone. */
    private static final double[] PARK_1 = {12.0, 108.0, 0.0};
    private static final double[] PARK_2 = {12.5, 106.0, 180.0};

    /** The steps of a route from the START on; the give-up time is the one thing a route learns after it is built once (see {@link #make}). */
    private interface Plan {
        AutoRoute.Builder steps(AutoRoute.Builder shell, double giveUpSec);
    }

    private static void addRoutesFromEveryStart(List<AutoRoute> routes) {
        // ---- Safe -------------------------------------------------------------------------------------------------------------------------
        routes.add(make(SAFE_REAR_WALL_B6, "Safe: start straight on the wall behind the cell, beside the FLOWER, roll to the cell, wait, shoot 4, park",
                "WHY THIS START: a plain, straight placement. The robot stands on the rear wall in tile B6, left of the rear-wall FLOWER, front edge on the wall and "
                        + "facing it. Nothing is turned, so it is the easiest start to measure and place, and it keeps the 1 in and 3 degree placement margin.",
                "WE DO: 1) drive off the wall, past the FLOWER and into a straight shooting pose on the rear wall just right of it, while the partner shoots; "
                        + "2) wait there until the camera has seen the rear cell up and steady; 3) shoot 4; 4) drive off the wall across the open floor and park in the LOADING ZONE.",
                "WHAT COULD GO WRONG: the roll passes the rear-wall FLOWER about " + oneDecimal(plateGapAcross()) + " in below its plate and ends " + oneDecimal(keepOutGap1())
                        + " in from the FLOWER keep-out and " + oneDecimal(centerGap1()) + " in from the center line, so a robot that slides sideways while it drives can touch the FLOWER (G415) or cross the line (G402): watch the first practice run.",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(28.0, REAR_WALL_FRONT_ON_Y, 90.0)
                                    .note("Front edge ON the rear wall, facing it, in tile B6. Straight, so the 3 degree margin is easy.")
                                .driveConstant(28.0, ACROSS_Y, 90.0).label("off the wall")
                                    .note("Straight away from the wall, keeping the heading, so the front leaves the FLOWER's side before the robot goes east.")
                                .driveConstant(SHOOT_1[0], ACROSS_Y, 90.0).label("past the FLOWER")
                                    .note("East along the rear side, below the FLOWER plate.")
                                .driveConstant(SHOOT_1[0], SHOOT_1[1], 90.0).label("to the shooting pose")
                                    .note("Up beside the FLOWER to the wall: the robot's left edge is " + oneDecimal(keepOutGap1()) + " in clear of the FLOWER keep-out and its right edge "
                                            + oneDecimal(centerGap1()) + " in short of the center line.")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_1[0], SHOOT_1[1], SHOOT_1[2], 4).note(shootNote())
                                .driveConstant(SHOOT_1[0], EXIT_Y, 90.0).label("off the wall").note(exitNote())
                                .driveLinear(PARK_1[0], PARK_1[1], PARK_1[2]).label("to zone").note(zoneNote())
                                .park(PARK_1[0], PARK_1[1], PARK_1[2]).note(parkNote(PARK_1));
                    }
                }));

        routes.add(make(SAFE_RED_CORNER_A6, "Safe: start in the corner behind the cell, roll to the cell, wait, shoot 4, park",
                "WHY THIS START: it keeps the robot out of the middle of the rear side. It stands with its LEFT side on the red wall in tile A6, in the corner behind the "
                        + "LOADING ZONE, facing the rear wall, so a partner or a third robot that sets up near the rear cell is not in its way, and the wall is a long straight "
                        + "edge to square the robot against.",
                "WE DO: 1) drive away from the corner and east along the rear side, below the FLOWER, into a shooting pose in line with the rear cell, while the partner shoots; "
                        + "2) wait there until the camera has seen the cell up and steady; 3) shoot 4 from the close end of the shooting band; 4) drive off the wall and park "
                        + "facing the red wall (front first).",
                "WHAT COULD GO WRONG: the shooting pose is " + standoffText(SHOOT_2[0], SHOOT_2[1], SHOOT_2[2]) + " in from the aim point (the band is "
                        + oneDecimal(BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN) + " to " + oneDecimal(BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN) + " in), the nearest of the three poses to the HIVE, "
                        + "and the robot parks front first, so its front intake wheels end 2.7 in from the red wall.",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(SIDE_ON_WALL_X, 128.0, 90.0)
                                    .note("Left side ON the red wall, facing the rear wall, in tile A6. It is not in the LOADING ZONE and clear of the rear FLOWER.")
                                .driveConstant(30.0, ACROSS_Y, 90.0).label("out of the corner")
                                    .note("Diagonally away from the corner, keeping the heading.")
                                .driveConstant(SHOOT_2[0], ACROSS_Y, 90.0).label("past the FLOWER")
                                    .note("East along the rear side, below the FLOWER plate.")
                                .driveConstant(SHOOT_2[0], SHOOT_2[1], 90.0).label("to the shooting pose")
                                    .note("Two inches up into the pose in line with the cell.")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_2[0], SHOOT_2[1], SHOOT_2[2], 4).note(shootNote())
                                .driveConstant(SHOOT_2[0], EXIT_Y, 90.0).label("off the wall").note(exitNote())
                                .driveLinear(PARK_2[0], PARK_2[1], PARK_2[2]).label("to zone").note(zoneNote())
                                .park(PARK_2[0], PARK_2[1], PARK_2[2]).note(parkNote(PARK_2));
                    }
                }));

        // ---- Standard ---------------------------------------------------------------------------------------------------------------------
        routes.add(make(STANDARD_RED_WALL_A4, "Standard: start on the side wall just below the LOADING ZONE, go up the corridor, wait, shoot 4, park",
                "WHY THIS START: the robot sets up on the red wall in tile A4, just below the LOADING ZONE, with its left side on the wall and facing the rear wall. It is a short, "
                        + "straight run up the left corridor to the rear side, and it leaves the audience wall free for the partner's shooting position.",
                "WE DO: 1) drive straight up the left corridor, then east along the rear side to a shooting pose on the rear wall right of the FLOWER, while the partner shoots; "
                        + "2) wait there until the camera has seen the rear cell up and steady; 3) shoot 4; 4) drive off the wall and park facing the red wall.",
                "WHAT COULD GO WRONG: the run up the corridor passes the LOADING ZONE and the HIVE's keep-out on the way; the shooting pose is " + oneDecimal(keepOutGap1()) + " in from the FLOWER keep-out "
                        + "and " + oneDecimal(centerGap1()) + " in from the center line, as in the other routes that shoot from it.",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(SIDE_ON_WALL_X, 80.0, 90.0)
                                    .note("Left side ON the red wall, facing the rear wall, in tile A4, below the LOADING ZONE (a start inside the zone is not allowed, G304.E).")
                                .driveConstant(CORRIDOR_X, ACROSS_Y, 90.0).label("up the corridor")
                                    .note("Front first up the corridor, keeping the heading, clear of the HIVE keep-out and the red wall.")
                                .driveConstant(SHOOT_1[0], ACROSS_Y, 90.0).label("past the FLOWER")
                                    .note("East along the rear side, below the FLOWER plate.")
                                .driveConstant(SHOOT_1[0], SHOOT_1[1], 90.0).label("to the shooting pose")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_1[0], SHOOT_1[1], SHOOT_1[2], 4).note(shootNote())
                                .driveConstant(SHOOT_1[0], EXIT_Y, 90.0).label("off the wall").note(exitNote())
                                .driveLinear(PARK_2[0], PARK_2[1], PARK_2[2]).label("to zone").note(zoneNote())
                                .park(PARK_2[0], PARK_2[1], PARK_2[2]).note(parkNote(PARK_2));
                    }
                }));

        routes.add(make(STANDARD_RED_WALL_A2, "Standard: start on the side wall below its FLOWER, go up the corridor, wait, shoot 4, park",
                "WHY THIS START: the robot sets up on the red wall in tile A2, below the red-wall FLOWER, with its left side on the wall, so the drive team can square it against the "
                        + "wall they use for every other start and it is as far as it can be from the rear side where a partner might also be. The drive up the corridor is the longest of "
                        + "the red-wall starts, and the wait hides all of it.",
                "WE DO: 1) drive straight up the left corridor, past the HIVE, and east along the rear side to the in-line shooting pose below the rear FLOWER, while the partner shoots; "
                        + "2) wait there until the camera has seen the rear cell up and steady; 3) shoot 4 from the close end of the band; 4) drive off the wall and park.",
                "WHAT COULD GO WRONG: it is the farthest start from the cell, so a partner that tips much earlier than the robot arrives gets nothing from us until we are there; "
                        + "and the run goes along the HIVE keep-out for 45 in, so a robot that drifts 8 in toward the HIVE is in trouble.",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(SIDE_ON_WALL_X, 28.0, 90.0)
                                    .note("Left side ON the red wall, facing the rear wall, in tile A2, below the red-wall FLOWER.")
                                .driveConstant(CORRIDOR_X, 36.0, 90.0).label("away from the wall")
                                    .note("Sideways first, so the robot is well clear of the red-wall FLOWER (its plate is 5 in deep and at y 44 to 50) when it goes up.")
                                .driveConstant(CORRIDOR_X, ACROSS_Y, 90.0).label("up the corridor")
                                    .note("Front first up the corridor past the red-wall FLOWER and the HIVE, keeping the heading.")
                                .driveConstant(SHOOT_2[0], ACROSS_Y, 90.0).label("past the FLOWER")
                                .driveConstant(SHOOT_2[0], SHOOT_2[1], 90.0).label("to the shooting pose")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_2[0], SHOOT_2[1], SHOOT_2[2], 4).note(shootNote())
                                .driveConstant(SHOOT_2[0], EXIT_Y, 90.0).label("off the wall").note(exitNote())
                                .driveLinear(PARK_1[0], PARK_1[1], PARK_1[2]).label("to zone").note(zoneNote())
                                .park(PARK_1[0], PARK_1[1], PARK_1[2]).note(parkNote(PARK_1));
                    }
                }));

        routes.add(make(STANDARD_B1_CORRIDOR, "Standard: start where RedAlliance_B1 does, go up the side corridor, wait, shoot 4, park",
                "WHY THIS START: the usual one. The robot stands in tile B1 with its FRONT edge on the audience wall, exactly as in RedAlliance_B1 and the TeleOp default, so nobody "
                        + "learns a new placement. It goes the long way round the HIVE (up the left corridor) while the partner shoots.",
                "WE DO: 1) drive back-first up the left corridor, turn around at the top and go east to the shooting pose on the rear wall, while the partner shoots; "
                        + "2) wait there until the camera has seen the rear cell up and steady; 3) shoot 4; 4) drive off the wall and park.",
                "WHAT COULD GO WRONG: the partner's own shooting position is on the audience side of the HIVE, so keep the corridor free of it; the robot must be past the corner of "
                        + "the HIVE before the tip drops the audience cell (G409: the spill zone starts 41 in from the red wall), which it is long before the assumed tip.",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(35.0, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90.0)
                                    .note("Front edge ON the audience wall, facing it, in tile B1 (the pose is 9.77 in from the wall, not the 9.2 of the old OpMode).")
                                .driveConstant(CORRIDOR_X + 1.0, 118.0, -90.0).label("up the left corridor")
                                    .note("Backwards up the corridor (the intake still faces the audience wall), so the robot does not have to turn next to the wall.")
                                .driveConstant(SHOOT_1[0], ACROSS_Y, 90.0).label("across")
                                    .note("Turns around at the top of the corridor, where there is room, then east along the rear side below the FLOWER plate.")
                                .driveConstant(SHOOT_1[0], SHOOT_1[1], 90.0).label("to the shooting pose")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_1[0], SHOOT_1[1], SHOOT_1[2], 4).note(shootNote())
                                .driveConstant(SHOOT_1[0], EXIT_Y, 90.0).label("off the wall").note(exitNote())
                                .driveLinear(PARK_1[0], PARK_1[1], PARK_1[2]).label("to zone").note(zoneNote())
                                .park(PARK_1[0], PARK_1[1], PARK_1[2]).note(parkNote(PARK_1));
                    }
                }));

        routes.add(make(STANDARD_C1_CORRIDOR, "Standard: start where RedAlliance_C1 does, slide over, go up the side corridor, wait, shoot 4, park",
                "WHY THIS START: the other usual one. The robot stands in tile C1, in line with the HIVE, with its FRONT edge on the audience wall, as in RedAlliance_C1. "
                        + "It first slides left, because the straight line up the field is the HIVE, then goes up the left corridor. Use it when B1 is taken by the partner.",
                "WE DO: 1) slide left to the corridor, drive back-first up it, turn around at the top and go east to the in-line shooting pose, while the partner shoots; "
                        + "2) wait there until the camera has seen the rear cell up and steady; 3) shoot 4; 4) drive off the wall and park front first.",
                "WHAT COULD GO WRONG: a partner that shoots from the C1 spot (in line with the HIVE, the usual place) is where this robot starts: set up first, or use B1. The slide "
                        + "left must clear the partner robot too.",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(58.0, BioBuzzConstants.Robot.FRONT_EXTENT_IN, -90.0)
                                    .note("Front edge ON the audience wall, facing it, in tile C1.")
                                .driveConstant(CORRIDOR_X + 1.0, 30.0, -90.0).label("slide left")
                                    .note("Sideways and back away from the wall to the corridor, still facing the audience wall, well below the HIVE keep-out.")
                                .driveConstant(CORRIDOR_X + 1.0, 118.0, -90.0).label("up the left corridor")
                                .driveConstant(SHOOT_2[0], ACROSS_Y, 90.0).label("across")
                                    .note("Turns around at the top of the corridor, then east along the rear side below the FLOWER plate.")
                                .driveConstant(SHOOT_2[0], SHOOT_2[1], 90.0).label("to the shooting pose")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_2[0], SHOOT_2[1], SHOOT_2[2], 4).note(shootNote())
                                .driveConstant(SHOOT_2[0], EXIT_Y, 90.0).label("off the wall").note(exitNote())
                                .driveLinear(PARK_2[0], PARK_2[1], PARK_2[2]).label("to zone").note(zoneNote())
                                .park(PARK_2[0], PARK_2[1], PARK_2[2]).note(parkNote(PARK_2));
                    }
                }));

        routes.add(make(STANDARD_B1_BACK_FIRST, "Standard: start where RedAlliance_B1 does but turned round, drive straight up the side corridor, wait, shoot 4, park",
                "WHY THIS START: the same tile as the old autos, but turned the other way round: the robot's back is on the audience wall and it faces the rear wall already, so it "
                        + "drives forward up the corridor and arrives with the heading it needs. No turn is needed anywhere. Placement is also easy (a straight robot on a wall).",
                "WE DO: 1) drive forward up the left corridor and east along the rear side to the shooting pose, while the partner shoots; 2) wait there until the camera has seen "
                        + "the rear cell up and steady; 3) shoot 4; 4) drive off the wall and park front first.",
                "WHAT COULD GO WRONG: the robot starts with its shooter end on the audience wall, so the 4 pre-loads must be seated as rule G304 says (the robot touches exactly "
                        + "4 pre-load POLLEN), and the camera must still see the cell tags from the shooting pose (the same pose as the other routes that use it).",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(35.0, BioBuzzConstants.Robot.REAR_EXTENT_IN, 90.0)
                                    .note("BACK edge ON the audience wall, facing up the field, in tile B1 (the pose is 8.03 in from the wall).")
                                .driveConstant(CORRIDOR_X + 1.0, 118.0, 90.0).label("up the left corridor")
                                    .note("Forward and a little left, keeping the heading, clear of the HIVE keep-out.")
                                .driveConstant(SHOOT_1[0], ACROSS_Y, 90.0).label("across")
                                    .note("East along the rear side below the FLOWER plate.")
                                .driveConstant(SHOOT_1[0], SHOOT_1[1], 90.0).label("to the shooting pose")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_1[0], SHOOT_1[1], SHOOT_1[2], 4).note(shootNote())
                                .driveConstant(SHOOT_1[0], EXIT_Y, 90.0).label("off the wall").note(exitNote())
                                .driveLinear(PARK_2[0], PARK_2[1], PARK_2[2]).label("to zone").note(zoneNote())
                                .park(PARK_2[0], PARK_2[1], PARK_2[2]).note(parkNote(PARK_2));
                    }
                }));

        // ---- Aggressive -------------------------------------------------------------------------------------------------------------------
        routes.add(make(AGGRESSIVE_REAR_EDGE, "Aggressive: start as near the cell as the placement margin allows, shoot from beside its axis, park by the shortest way",
                "WHY THIS START: it is the closest straight start to the cell that still keeps the 1 in and 3 degree placement margin, on the rear wall, as near the rear FLOWER as a robot placed by hand can safely stand. The "
                        + "robot is therefore ready sooner than from any other straight start in these routes, which matters for a partner that tips EARLY. It then shoots from 10 degrees "
                        + "left of the cell's axis (inside the 20 degree cone), the point of the allowed shots nearest the LOADING ZONE, so the way out is the shortest too.",
                "WE DO: 1) roll a short way east along the rear side into a shooting pose left of the cell's axis, while the partner shoots; 2) wait there until the camera has "
                        + "seen the rear cell up and steady; 3) shoot 4; 4) drive off the wall and park front first, with the shortest drive of any route here.",
                "WHAT COULD GO WRONG: everything is tight. The start is at the edge of what a hand can place; the shooting pose is " + standoffText(SHOOT_3[0], SHOOT_3[1], SHOOT_3[2]) + " in from the aim point (the band is " + oneDecimal(BioBuzzConstants.Auto.SHOOT_STANDOFF_MIN_IN)
                        + " to " + oneDecimal(BioBuzzConstants.Auto.SHOOT_STANDOFF_MAX_IN) + " in) and "
                        + "1.3 in from the FLOWER plate, and a shot from the side sees a narrower window in the cell opening (the TeleOp solver allows 20 degrees, this uses 10).",
                new Plan() {
                    @Override
                    public AutoRoute.Builder steps(AutoRoute.Builder b, double g) {
                        return b.start(AGGRESSIVE_START_X, REAR_WALL_FRONT_ON_Y, 90.0)
                                    .note("Front edge ON the rear wall, facing it, as near the FLOWER as the placement margin allows: a hand that is 1 in and 3 degrees off still clears it on the first drive.")
                                .driveConstant(AGGRESSIVE_START_X, ACROSS_Y, 90.0).label("off the wall")
                                .driveConstant(SHOOT_3[0], SHOOT_3[1], SHOOT_3[2]).label("to the shooting pose")
                                    .note("Turns 10 degrees on the way, so the back points at the cell from the left of its axis.")
                                .waitForTip(g).note(waitNote(g))
                                .shootUp(SHOOT_3[0], SHOOT_3[1], SHOOT_3[2], 4).note(shootNote())
                                .driveLinear(PARK_2[0], PARK_2[1], PARK_2[2]).label("to zone").note(zoneNote())
                                .park(PARK_2[0], PARK_2[1], PARK_2[2]).note(parkNote(PARK_2));
                    }
                }));
    }

    // ---- the machinery: build once to learn the numbers, then again with the words ------------------------------------------------------------------

    private static AutoRoute.Builder shell(String id, String title) {
        return AutoRoute.builder(id).partnerTipsFirst().title(title);
    }

    /**
     * Builds one route in three passes, so that no number in its text or its give-up time can be wrong: (1) a draft, to read the latest give-up time the rest of the
     * route allows ({@code RouteAnalysis.latestGiveUpSec}) and round it down to a tenth; (2) the route with that time; (3) the same route with its description, written from the
     * analysis of pass 2 (when the robot is ready, the latest partner tip that still works, the slack, the points).
     */
    private static AutoRoute make(String id, String title, String why, String does, String wrong, Plan plan) {
        AutoRoute draft = plan.steps(shell(id, title), BioBuzzConstants.Auto.AUTO_DURATION_SEC).build();
        double giveUp = Math.floor(latestGiveUp(draft.analyze()) * 10.0) / 10.0;
        AutoRoute timed = plan.steps(shell(id, title), giveUp).build();
        RouteAnalysis a = timed.analyze();
        String text = why + " " + readyText(a) + " " + partnerText(a, giveUp) + " " + does + " " + pointsText(a) + " " + secondTipText() + " " + wrong;
        String tile = TileGrid.nameAt(timed.startPose().x, timed.startPose().y);
        return plan.steps(shell(id, title).description(text).startTile(tile), giveUp).build();
    }

    private static double latestGiveUp(RouteAnalysis a) {
        for (RouteAnalysis.StepInfo info : a.steps()) {
            if (info.step instanceof Step.WaitForTip) {
                return a.latestGiveUpSec(info);
            }
        }
        throw new IllegalStateException("a partner route needs a WAIT_FOR_TIP");
    }

    private static double readySec(RouteAnalysis a) {
        for (RouteAnalysis.StepInfo info : a.steps()) {
            if (info.step instanceof Step.WaitForTip) {
                return info.startSec;
            }
        }
        throw new IllegalStateException("a partner route needs a WAIT_FOR_TIP");
    }

    private static String readyText(RouteAnalysis a) {
        return String.format(Locale.ROOT, "READY: the robot is at its shooting pose %.1f s after AUTO starts, long before the partner's assumed tip (%.0f s), so the wait hides the drive: "
                + "where the robot starts does not change when the route ends, it changes how early we are ready (a partner that tips before %.1f s makes the robot shoot when it arrives, not sooner) and "
                + "how easy the robot is to place. The route ends at %.1f s.", readySec(a), a.partnerTipSec(), readySec(a), a.totalSeconds());
    }

    private static String partnerText(RouteAnalysis a, double giveUp) {
        return String.format(Locale.ROOT, "THE PARTNER MUST shoot into the audience cell of our HIVE until it tips (the cell holds our 3 staged NECTAR, so 3 POLLEN tip it), "
                + "done by about %.0f s: agree that time with the partner team before the match. A partner that tips as late as %.1f s still works, which is %.1f s of slack "
                + "over %.0f s. If the tip comes later than that, never comes, or the camera is not working, the robot gives up waiting at %.1f s, skips the volley and still leaves "
                + "and parks.", a.partnerTipSec(), a.latestPartnerTipSec(), a.partnerTipSlackSec(), a.partnerTipSec(), giveUp);
    }

    private static String pointsText(RouteAnalysis a) {
        return String.format(Locale.ROOT, "OUR POINTS: LEAVE %d + PARK %d = %d (the partner's tip is 20 points for the alliance, never counted as ours). Our 4 POLLEN are not wasted: "
                + "the cell that comes up is empty and needs 8, so they are 4 of the 8 and TELEOP starts with the second tip half done.",
                BioBuzzConstants.Auto.LEAVE_POINTS, BioBuzzConstants.Auto.PARK_POINTS, a.points());
    }

    private static String secondTipText() {
        return String.format(Locale.ROOT, "WHY NO SECOND TIP: our own tip needs 8 POLLEN landed in that cell, our 4 and 4 more fetched from the rear FLOWER and shot in a second volley. "
                + "After a partner that tips at %.0f s that is more than the %.0f s of AUTO in the timing estimate, and real robots land only about 60 percent of their shots, "
                + "so 8 landed out of 8 shot is unlikely. The safe alternative, which this route takes: bank 4, and take the sure 8 points.",
                BioBuzzConstants.Auto.PARTNER_FIRST_TIP_DONE_SEC, BioBuzzConstants.Auto.AUTO_DURATION_SEC);
    }

    private static String waitNote(double giveUp) {
        return "Waits for the CAMERA to see the rear cell up and steady (0.5 s), never for a time. " + seconds(giveUp) + " on the route clock is the give-up time, the latest the rest "
                + "of the route allows rounded down to a tenth: a partner that tips later gets no volley from us, the robot skips it and goes on to LEAVE and PARK.";
    }

    private static String shootNote() {
        return "4 POLLEN into the empty cell do not tip it (it needs 8): they are banked for the second tip in TELEOP, so there is no spill here (G409).";
    }

    private static String exitNote() {
        return "Straight away from the rear wall toward the open floor, keeping the heading, so the robot does not turn next to the FLOWER.";
    }

    private static String zoneNote() {
        return "Across the open floor above the HIVE keep-out while turning to face the zone, into the own LOADING ZONE.";
    }

    /** The park note, from the footprint of the pose: how far into the zone it reaches and how far off the wall it stands (the LEAVE needs 1.5 in, QA21). */
    private static String parkNote(double[] park) {
        Rect bounds = Footprint.of(Pose2.ofDegrees(park[0], park[1], park[2])).bounds();
        return String.format(Locale.ROOT, "Overlaps the LOADING ZONE by %.1f in (the zone is %.0f in deep), standing %.1f in off the wall so the LEAVE is kept too (QA21).",
                BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - bounds.minX,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN - BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, bounds.minX);
    }

    private static String seconds(double v) {
        return String.format(Locale.ROOT, "%.1f s", v);
    }

    private static String oneDecimal(double v) {
        return String.format(Locale.ROOT, "%.1f", v);
    }

    /** How far the ball exit of a robot at this pose is from the aim point of the RED REAR cell, as the validator measures it, in inches with one decimal. */
    static String standoffText(double x, double y, double headingDeg) {
        CellSpec rear = new AllianceField(new FieldModel(), Alliance.RED).cell(Side.REAR);
        return oneDecimal(ShotGeometry.standoff(Pose2.ofDegrees(x, y, headingDeg), rear));
    }

    /** How far the left edge of shooting pose 1 is past the keep-out of the rear-wall FLOWER (its plate plus {@code Auto.FLOWER_KEEPOUT_MARGIN_IN}), inches. */
    static double keepOutGap1() {
        Rect plate = new AllianceField(new FieldModel(), Alliance.RED).flowerPlate(1);
        return (SHOOT_1[0] - BioBuzzConstants.Robot.LEFT_EXTENT_IN) - (plate.maxX + BioBuzzConstants.Auto.FLOWER_KEEPOUT_MARGIN_IN);
    }

    /** How far the right edge of shooting pose 1 is short of the center line, inches. */
    static double centerGap1() {
        return BioBuzzConstants.Field.CENTER_IN - (SHOOT_1[0] + BioBuzzConstants.Robot.RIGHT_EXTENT_IN);
    }

    /** How far the front edge of the long drive east along the rear side ({@link #ACROSS_Y}) stays below the plate of the rear-wall FLOWER, inches. */
    static double plateGapAcross() {
        Rect plate = new AllianceField(new FieldModel(), Alliance.RED).flowerPlate(1);
        return plate.minY - (ACROSS_Y + BioBuzzConstants.Robot.FRONT_EXTENT_IN);
    }
}
