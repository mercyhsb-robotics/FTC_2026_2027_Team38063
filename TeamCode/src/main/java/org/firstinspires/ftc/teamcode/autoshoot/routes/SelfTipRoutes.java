package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;
import org.firstinspires.ftc.teamcode.autoshoot.core.TileGrid;

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
 * <p><b>What these routes can and cannot do in 30 s.</b> AUTO scores only LEAVE 3, PARK 5 and a HIVE TIP 20. Our own first volley tips the HIVE
 * only if about 3 of its 4 POLLEN land, which real teams manage less than half of the time (6 in 10 shots: 47 in 100), and it takes about 5.5 s. A LATER
 * tip needs 8 POLLEN into the cell that comes up (it holds no NECTAR) and the robot holds only 4, so it needs the GARDEN and both FLOWERS: that is about
 * 36 s of work with the shipped numbers, so no route here tries a second tip. No route takes POLLEN from a FLOWER either: every FLOWER pick-up is an ERROR until
 * the real wheel reach is measured (the validator says the flat wheels fall 0.8 in short). What the time that is left CAN buy is
 * <i>insurance</i> (a second volley at the same cell when the first one did not tip it: the "top-up", with the GARDEN's 4 POLLEN) and <i>a head start for TELEOP</i>
 * (POLLEN carried, or 4 POLLEN already banked in the cell that came up).
 *
 * <p><b>The tiers</b> (every title starts with one of these words and a colon, so the drive team can pick by the word):
 * <ul>
 *   <li><b>Safe</b>: no camera needed, no pick-up, over in about 11 s or less, the easiest placement.</li>
 *   <li><b>Standard</b>: one extra job (the GARDEN) or a start that is a little harder to place; over in about 16 s or less.</li>
 *   <li><b>Aggressive</b>: a second volley, 20 to 23 s: the robot fires again only when its camera shows the cell still UP.</li>
 *   <li><b>Advanced</b>: a volley at the cell that comes up after our own tip, from the far side of the HIVE (needs the camera to see that cell UP), 22 s.</li>
 * </ul>
 *
 * <p><b>The starts</b> come from the start search ({@link StartPoseSearch}, rule G304; see {@link StartShortcuts}): the <i>anchor start</i> (front on the
 * audience wall, in line with the HIVE: the robot already stands where it shoots from, and it is the one shot the team has measured; 7 of the 8 routes use it)
 * and the <i>turned start</i> (a front corner on the audience wall, the robot turned 20 degrees: still a shooting pose, 13 in nearer the GARDEN and the left
 * corridor). The turned start saves only 0.4 s on a route with one trip along the wall, so no route is written just for it (it would be the same plan with a
 * harder placement); it saves 1.0 s on the top-up route that makes two trips to the GARDEN, and that route uses it.
 * The search's best start (turned 40 degrees, x = 35) was left out on purpose: its shot is 38.8 in from the aim point, outside the 30 to 38 in band, and
 * 28 degrees off the direction the cell faces, outside the 20 degree cone of the TeleOp planner ({@code AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG}). The turned
 * start used here is 17 degrees off, inside the cone. The route validator does not check that cone (it checks the standoff band and that the back points
 * at the cell), so this file keeps to it by hand: every volley below is within 20 degrees of the cell's own direction. For the same reason no route
 * shoots at the rear cell from the LOADING ZONE, however short the drive would be: from the side the cell's opening is seen edge-on.
 */
public final class SelfTipRoutes {
    public static final String ANCHOR_SHOOT_PARK = "self_anchor_shoot_park";
    public static final String SHOOT_STEP_OFF = "self_shoot_step_off";
    public static final String SHOOT_ONLY = "self_shoot_only";
    public static final String GARDEN_CARRY_PARK = "self_garden_carry_park";
    public static final String GARDEN_TOPUP_PARK = "self_garden_topup_park";
    public static final String GARDEN_TOPUP_STEP_OFF = "self_garden_topup_step_off";
    public static final String GARDEN_REFILL_READY = "self_garden_refill_ready";
    public static final String REAR_BANK = "self_rear_bank";

    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;

    // ---- the starts and poses the routes share (route data: poses an author types in) ---------------------------------------------

    /** Front edge on the audience wall, in line with the HIVE: the measured shot (the ball leaves about 36 in from the aim point), no drive before the first volley. */
    static final Pose2 ANCHOR_START = Pose2.ofDegrees(BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, FRONT, -90.0);
    /** A front corner on the audience wall, turned 20 degrees: a shooting pose (36 in from the aim point, 17 degrees off the cell's direction) 13 in nearer the corridor. */
    static final Pose2 TURNED_START = StartPoseSearch.seatedAt(Wall.AUDIENCE, 44.5, -110.0);
    /** The parking pose in the lower half of the LOADING ZONE: it overlaps the zone by 6.5 in (PARK needs 3) and stands 4 in off the red wall (LEAVE needs 1.5). */
    static final Pose2 PARK_LOW = Pose2.ofDegrees(12.0, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN - BioBuzzConstants.Robot.LEFT_EXTENT_IN + 6.5, 0.0);
    /** The shooting pose in front of the rear cell: the back toward it, 32.8 in from its aim point, in line with it, clear of the rear-wall FLOWER (x 44 to 51). */
    static final Pose2 REAR_SHOT = Pose2.ofDegrees(61.5, 128.0, 90.0);

    /** Where the robot stands to take the GARDEN POLLEN, its front 0.8 in off the audience wall (as in the catalog's garden_cycle). */
    private static final double GARDEN_ROBOT_X = 9.0;
    private static final double GARDEN_ROBOT_Y = FRONT + 0.8;
    /** The middle of the 4 GARDEN POLLEN (they lie along the audience wall). */
    private static final double GARDEN_TARGET_X = (BioBuzzConstants.Auto.RED_GARDEN_POLLEN_X_IN[0]
            + BioBuzzConstants.Auto.RED_GARDEN_POLLEN_X_IN[BioBuzzConstants.Auto.RED_GARDEN_POLLEN_X_IN.length - 1]) / 2.0;
    /** The left corridor: the robot's center stays at this x, 13 in clear of the HIVE keep-out (x 42.94) and of the spill zone (x 41.39). */
    private static final double CORRIDOR_X = 30.0;
    /** The height at which the robot slides along the audience wall: its front edge is 5 in off the wall (a robot that turns there stays inside the field). */
    private static final double SLIDE_Y = 14.8;
    /** How far up the corridor the robot drives before it turns into the LOADING ZONE. */
    private static final double CORRIDOR_END_Y = 84.0;

    private SelfTipRoutes() {
    }

    /** Every route of this family, authored for the RED alliance (BLUE is the half turn). A new list on every call. */
    public static List<AutoRoute> routes() {
        List<AutoRoute> routes = new ArrayList<AutoRoute>();
        routes.add(anchorShootPark());
        routes.add(shootStepOff());
        routes.add(shootOnly());
        routes.add(gardenCarryPark());
        routes.add(gardenRefillReady());
        routes.add(gardenTopUpStepOff());
        routes.add(gardenTopUpPark());
        routes.add(rearBank());
        return routes;
    }

    // ---- the pieces of a route ---------------------------------------------------------------------------------------------------

    private static AutoRoute.Builder opening(String id, Pose2 s, String title, String description, String startNote) {
        return AutoRoute.builder(id).title(title).description(description).startTile(TileGrid.nameAt(s.x, s.y))
                .start(s.x, s.y, s.headingDegrees()).note(startNote);
    }

    /** The first volley, fired from the start itself: the robot stands where it shoots from, so no drive comes first. */
    private static AutoRoute.Builder firstVolley(AutoRoute.Builder b, Pose2 s) {
        return b.shootUp(s.x, s.y, s.headingDegrees(), 4)
                .note("The first tip. The UP cell holds 3 NECTAR, so the 3rd POLLEN tips the HIVE (+20) and the 4th is the spare for a field that needs one more. "
                        + "The camera stops the feeder only about 0.85 s after the tip, so the 4th is probably launched too and spent: that is the WARN on this step, and "
                        + "it is fine, it is only the spare. The robot fires from where it was put, with no drive first. Real teams land only about 6 shots in 10, so "
                        + "plan on this volley tipping the HIVE less than half of the time (the top-up routes are for that).");
    }

    /** Slides along the audience wall to the corridor and drives up it, backwards (shooter first), to the LOADING ZONE; ends parked in the lower half. */
    private static AutoRoute.Builder toTheZoneAndPark(AutoRoute.Builder b) {
        return b.driveConstant(CORRIDOR_X, SLIDE_Y, -90.0).label("along the wall")
                .note("Slides sideways along the audience wall with the front still toward it, 5 in off the wall, until it is beside the left corridor.")
                .driveReverseTangent(CORRIDOR_X, CORRIDOR_END_Y).label("corridor")
                .note("Backwards (the shooter end first) up the left corridor, 13 in clear of the HIVE keep-out and of the spill zone of the cell that tipped.")
                .driveLinear(PARK_LOW.x, PARK_LOW.y, PARK_LOW.headingDegrees()).label("to the zone")
                .note("Turns to face +x while it slides into the LOADING ZONE.")
                .park(PARK_LOW.x, PARK_LOW.y, PARK_LOW.headingDegrees())
                .note("Parks in the LOWER half of the LOADING ZONE, overlapping it by 6.5 in (PARK needs 3), the whole robot 4 in off the red wall so the LEAVE is kept too "
                        + "(touching ANY wall at the end of AUTO loses it, QA21). The upper half of the zone stays free for the partner.");
    }

    /** Drives into the GARDEN and takes its 4 POLLEN (the robot holds 0 after the first volley, so 4 fit). */
    private static AutoRoute.Builder takeTheGarden(AutoRoute.Builder b) {
        return b.driveConstant(GARDEN_ROBOT_X, GARDEN_ROBOT_Y, -90.0).label("into the GARDEN").wallContact()
                .note("Slides along the audience wall to the GARDEN with the intake toward it and stops with the front edge 0.8 in off the wall, in front of the row of POLLEN.")
                .intakeGarden(GARDEN_TARGET_X, BioBuzzConstants.Auto.RED_GARDEN_POLLEN_Y_IN, GARDEN_ROBOT_X, GARDEN_ROBOT_Y, -90.0, 4)
                .note("The 4 GARDEN POLLEN lie in a row along the wall. After the first volley the robot holds 0 (the spare was probably spent), so it can take all 4 (G407: never "
                        + "more than 4). If the feeder did stop in time the robot holds 1 and the intake must stop at 3: the runner stops at 4.");
    }

    /** Back to the shooting pose and a second volley at the SAME cell: it only fires if the camera still shows that cell UP (the first volley did not tip it). */
    private static AutoRoute.Builder topUp(AutoRoute.Builder b, Pose2 s) {
        return b.driveLinear(s.x, s.y, s.headingDegrees()).label("back to the shot")
                .note("Back to the pose of the first volley, turning evenly while it slides away from the wall.")
                .shootSide(Side.AUDIENCE, s.x, s.y, s.headingDegrees(), 4)
                .note("THE TOP-UP. The plan says the first volley tipped the HIVE, so on paper the audience cell is already DOWN: that is the WARN on this step. On the field the "
                        + "robot fires only if its camera still shows the audience cell UP (tags in view, steady), which means the first volley did not tip it; the cell then "
                        + "still holds the 3 NECTAR and whatever landed, so 1 to 3 more POLLEN tip it. If the camera shows the cell DOWN (the HIVE tipped), or sees nothing, "
                        + "the volley is skipped and the robot goes on. The plan counts the volley's time (5.5 s) as if it fired, so the finish time is the worst case.");
    }

    // ---- 1. Safe: anchor start, shoot 4, park -------------------------------------------------------------------------------------

    private static AutoRoute anchorShootPark() {
        return toTheZoneAndPark(firstVolley(opening(ANCHOR_SHOOT_PARK, ANCHOR_START,
                "Safe: shoot 4 from the start, drive up the left side and park in the LOADING ZONE",
                "THE BASIC ROUTE. Put the robot with its front edge on the audience wall, in line with the HIVE (the placement words above). It already stands where it shoots "
                        + "from, so it fires the 4 pre-loads at once with no drive first, and this is the one shot the team has measured; that start saves about 0.7 s against tile C1 and "
                        + "1.5 s against tile B1, which must drive to a shooting pose first. Then it slides along the wall, goes up the left corridor and parks in the LOWER half of the "
                        + "LOADING ZONE. POINTS: LEAVE 3 + PARK 5 + the HIVE tip 20 if 3 of the 4 shots land = 28 (8 if the tip does not happen). SPARE POLLEN: 1 in the volley "
                        + "(probably spent, see the WARN). It needs no camera and no pick-up and is over in about 11 s. WHAT CAN GO WRONG: fewer than 3 shots land (real teams land about "
                        + "6 in 10, so this happens about half of the time): the tip is missed and nothing tries again; use a top-up route if you want a second chance. THE PARTNER: "
                        + "it may park in the upper half of the zone; it must not shoot into our audience cell first.",
                "Front edge ON the audience wall (the center of rotation is " + FRONT + " in out), facing the wall, the middle of the robot over the HIVE's line (x "
                        + BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN + " in). Chosen because it is a shooting pose already (the ball leaves about 36 in from the aim point, inside the "
                        + "30 to 38 in band, straight at the cell) and the only start where the measured shot applies."), ANCHOR_START)).build();
    }

    // ---- 2. Safe: anchor start, shoot 4, step off -------------------------------------------------------------------------------

    private static AutoRoute shootStepOff() {
        return firstVolley(opening(SHOOT_STEP_OFF, ANCHOR_START,
                "Safe: shoot 4 from the start and step off the wall (stay out of the LOADING ZONE)",
                "THE SHORTEST ROUTE. The same start and the same first volley as the basic route (front on the audience wall in line with the HIVE, 4 pre-loads fired at once), then "
                        + "it slides 5 in off the wall and stops: LEAVE only. Use it when the partner is going to park in the LOADING ZONE and the zone is too crowded for two robots, "
                        + "or when you do not trust a long drive. It is over in about 7 s, so the robot is free for TELEOP at once. POINTS: LEAVE 3 + the HIVE tip 20 if 3 of the 4 "
                        + "shots land = 23 (3 if the tip does not happen). SPARE POLLEN: 1 in the volley (probably spent). WHAT CAN GO WRONG: the tip is missed and nothing tries "
                        + "again; no PARK (5 points). THE PARTNER: it may take the whole LOADING ZONE.",
                "Front edge ON the audience wall (the center of rotation is " + FRONT + " in out), facing the wall, in line with the HIVE. The same start as the basic route: "
                        + "a shooting pose, so no drive comes before the volley."), ANCHOR_START)
                .driveConstant(CORRIDOR_X, SLIDE_Y, -90.0).label("off the wall")
                .note("Slides sideways along the audience wall, ending 5 in off it: the whole robot is more than 1.5 in off every wall, so the LEAVE is claimed. The robot is "
                        + "far below the cell that tipped by then, outside its spill zone (the rear edge is 23 in out, the zone starts 30.7 in out).")
                .build();
    }

    // ---- 3. Safe: anchor start, shoot 4, stay put ------------------------------------------------------------------------------------

    private static AutoRoute shootOnly() {
        return firstVolley(opening(SHOOT_ONLY, ANCHOR_START,
                "Safe: shoot 4 from the start and do not move (the drive or the odometry cannot be trusted)",
                "THE NO-DRIVE FALLBACK. Use it only when the drive base or the odometry (the Pinpoint) is broken or not trusted, so you do not want the robot to move at all. The robot "
                        + "stands where it is put (front on the audience wall, in line with the HIVE), fires the 4 pre-loads and stops: nothing in the route depends on where it thinks it is "
                        + "after the start, so only the placement matters. It is over in about 5.5 s. POINTS: the HIVE tip 20 if 3 of the 4 shots land (0 if not). No LEAVE: the robot ends "
                        + "touching the audience wall (QA21 takes the LEAVE away from a robot that touches any wall at the end), and no PARK. SPARE POLLEN: 1 in the volley (probably spent). "
                        + "WHAT CAN GO WRONG: the tip is missed and nothing tries again; it gives up 8 points that a working robot would take (see the basic route). THE PARTNER: it may take "
                        + "the whole LOADING ZONE; keep clear of the audience cell's spill (it falls in front of the HIVE) for 3.5 s after the tip.",
                "Front edge ON the audience wall (the center of rotation is " + FRONT + " in out), facing the wall, in line with the HIVE: a shooting pose, so the robot fires "
                        + "where it stands, with nothing to drive."), ANCHOR_START)
                .build();
    }

    // ---- 4. Standard: anchor start, shoot 4, take the GARDEN, park with 4 -----------------------------------------------------------

    private static AutoRoute gardenCarryPark() {
        return toTheZoneAndPark(takeTheGarden(firstVolley(opening(GARDEN_CARRY_PARK, ANCHOR_START,
                "Standard: shoot 4, take the 4 GARDEN POLLEN and park in the LOADING ZONE with them",
                "POLLEN FOR TELEOP. From the anchor start (front on the audience wall, in line with the HIVE) the robot fires the 4 pre-loads at once, slides 49 in along the "
                        + "wall to the GARDEN, takes all 4 of its POLLEN, goes up the left corridor and parks in the lower half of the LOADING ZONE holding them, so TELEOP starts with "
                        + "4 POLLEN loaded. POINTS: LEAVE 3 + PARK 5 + the tip 20 if 3 of the 4 shots land = 28; the GARDEN POLLEN score nothing in AUTO. SPARE POLLEN: 1 in the "
                        + "volley (probably spent). It is over in about 16 s, so about 12 s are left: a slow GARDEN pick-up (the plan says 2.5 s) does not hurt. WHAT CAN GO WRONG: it "
                        + "does not shoot again, so a missed tip stays missed; if the feeder did stop in time the robot still holds 1 POLLEN and the intake must stop at 3 (never more "
                        + "than 4 held, G407). THE PARTNER: the GARDEN is ours alone; the partner may park in the upper half of the zone.",
                "Front edge ON the audience wall (the center of rotation is " + FRONT + " in out), facing the wall, in line with the HIVE: a shooting pose already. The turned start "
                        + "would save only about 0.3 s here, which is not worth the harder placement."), ANCHOR_START))).build();
    }

    // ---- 5. Standard: anchor start, shoot 4, take the GARDEN, wait ready ------------------------------------------------------------------

    private static AutoRoute gardenRefillReady() {
        return takeTheGarden(firstVolley(opening(GARDEN_REFILL_READY, ANCHOR_START,
                "Standard: shoot 4, take the 4 GARDEN POLLEN and wait 5 in off the wall at the shooting spot",
                "A LOADED ROBOT AT THE SHOOTING SPOT. The anchor start fires the 4 pre-loads, the robot takes the 4 GARDEN POLLEN and comes back to the spot it shot from, 5 in off the wall "
                        + "(still 31 in from the audience cell: still a shooting pose) and stops there: LEAVE, no PARK, no second volley in AUTO. At the start of TELEOP the robot is loaded "
                        + "and aimed: if the first volley did not tip the HIVE (the audience cell is still UP) the drivers can fire at once; if it did, the REAR cell is the UP one and they "
                        + "drive round. It is the same plan as the top-up routes without the AUTO volley, so it needs no camera in AUTO. POINTS: LEAVE 3 + the tip 20 if 3 of the 4 shots land "
                        + "= 23. SPARE POLLEN: 1 in the first volley (probably spent). WHAT CAN GO WRONG: it gives up the PARK (5) and the second chance in AUTO; the volley is not "
                        + "taken until TELEOP starts, after the 8 s pause in which no robot may move (G403). THE PARTNER: it may use the whole LOADING ZONE.",
                "Front edge ON the audience wall (the center of rotation is " + FRONT + " in out), facing the wall, in line with the HIVE: the same start as the basic route, "
                        + "a shooting pose, so the first volley needs no drive."), ANCHOR_START))
                .driveLinear(ANCHOR_START.x, SLIDE_Y, -90.0).label("back to the shot")
                .note("Back to the spot of the first volley but 5 in off the wall: the whole robot is more than 1.5 in off every wall (LEAVE), and the ball would leave about 31 in from the "
                        + "audience cell's aim point, inside the 30 to 38 in band, straight at it.")
                .build();
    }

    // ---- 6. Aggressive: anchor start, shoot 4, GARDEN, top-up, step off -----------------------------------------------------------------------

    private static AutoRoute gardenTopUpStepOff() {
        return topUp(takeTheGarden(firstVolley(opening(GARDEN_TOPUP_STEP_OFF, ANCHOR_START,
                "Aggressive: shoot 4, refill at the GARDEN, shoot again only if the HIVE did not tip, step off (no park)",
                "THE SECOND CHANCE WITHOUT THE LONG DRIVE. The anchor start (front on the audience wall in line with the HIVE, the measured shot) fires the 4 pre-loads at once. "
                        + "The robot then slides to the GARDEN, takes its 4 POLLEN, comes BACK to the anchor pose and fires again at the audience cell, only if its camera still shows "
                        + "that cell UP (the first volley did not tip it). Then it steps 5 in off the wall and stops: it does NOT drive to the LOADING ZONE, so it is over at about 20 s, "
                        + "3 s earlier than the parking version, and it never has to share the zone. If the first volley DID tip, the robot sees the cell DOWN, skips the second volley "
                        + "and is done about 5 s sooner. With both volleys the chance of the tip goes from about 47 in 100 to about 95 in 100 (if 6 shots in 10 land). POINTS: LEAVE 3 + "
                        + "the tip 20 = 23 (the PARK's 5 points are given up for time and for the zone; the second volley can only replace the first volley's tip, never add one). "
                        + "SPARE POLLEN: 1 in the first volley (probably spent). WHAT CAN GO WRONG: with no camera, or on a bad read, the second volley is not fired; the GARDEN trip is 49 in "
                        + "each way. THE PARTNER: it may use the whole LOADING ZONE; it must NOT shoot into our audience cell in AUTO.",
                "Front edge ON the audience wall, facing it, in line with the HIVE: a shooting pose. Chosen because the SAME measured shot is used twice: the robot returns to this exact "
                        + "spot for the second volley, and it is the easiest start to place. The turned start would save about 1.1 s on the two trips."), ANCHOR_START)), ANCHOR_START)
                .driveConstant(CORRIDOR_X, SLIDE_Y, -90.0).label("off the wall")
                .note("Slides along the audience wall to 5 in off it and stops: LEAVE (the whole robot is more than 1.5 in off every wall).")
                .build();
    }

    // ---- 7. Aggressive: turned start, shoot 4, GARDEN, top-up, park --------------------------------------------------------------------------

    private static AutoRoute gardenTopUpPark() {
        return gardenTopUpPark(TURNED_START);
    }

    /** The top-up route that parks, from any start that is a shooting pose (the route of the catalog uses {@link #TURNED_START}; the test asks what the anchor start would cost). */
    static AutoRoute gardenTopUpPark(Pose2 start) {
        return toTheZoneAndPark(topUp(takeTheGarden(firstVolley(opening(GARDEN_TOPUP_PARK, start,
                "Aggressive: shoot 4, refill at the GARDEN, shoot again only if the HIVE did not tip, then park",
                "THE SECOND CHANCE, WITH THE PARK. Real teams land only about 6 shots in 10, so the first volley tips the HIVE less than half of the time (3 of 4 must land). This route "
                        + "plans the insurance and still parks: from the turned start it fires the 4 pre-loads, slides to the GARDEN, takes its 4 POLLEN, comes back to the same pose and "
                        + "fires AGAIN at the audience cell, but only if the camera still shows that cell UP (the HIVE did not tip). Then it goes up the corridor and parks in the lower "
                        + "half of the LOADING ZONE. With both volleys the chance of the tip goes from about 47 in 100 to about 95 in 100 (if 6 shots in 10 land). If the first volley DID tip, the "
                        + "robot sees the cell DOWN, skips the second volley and finishes about 5 s early. POINTS: LEAVE 3 + PARK 5 + the tip 20 = 28 (the second volley can only replace "
                        + "the first volley's tip, never add one). SPARE POLLEN: 1 in the first volley (probably spent). WHAT CAN GO WRONG: it ends at about 22.5 s on the plan, the least "
                        + "time to spare of the routes with a park: a slow robot may not make it, and then the PARK is lost; no camera means no second volley. THE PARTNER: "
                        + "it must NOT shoot into our audience cell in AUTO; the upper half of the zone is free for it.",
                "A front corner ON the audience wall, turned 20 degrees clockwise, back toward the audience cell. Chosen because the robot comes back to it for the second volley: it is a "
                        + "shooting pose AND 13 in closer to the GARDEN than the anchor start, which saves about 1.0 s on the two trips to it, against the anchor start. "
                        + "The same pose is used twice, so the aim only has to be checked once. (On a route with only one trip the same start saves just 0.4 s, which is not worth "
                        + "the harder placement.)"), start)), start))
                .build();
    }

    // ---- 8. Advanced: anchor start, shoot 4, GARDEN, drive round to the rear cell and bank 4 -----------------------------------------------------

    private static AutoRoute rearBank() {
        return takeTheGarden(firstVolley(opening(REAR_BANK, ANCHOR_START,
                "Advanced: shoot 4, take the GARDEN, drive round the HIVE and shoot 4 into the cell that came up",
                "BANK FOR THE SECOND TIP. After our own tip the OTHER (rear) cell comes up and holds no NECTAR, so it needs 8 POLLEN (two volleys of 4); there is no time for both in "
                        + "AUTO, but the first 4 can be put in, so TELEOP needs only 4 more (POLLEN left in a cell are worth 2 each at the end of the match if nothing tips them). The robot "
                        + "fires the 4 pre-loads from the anchor start (the tip), takes the GARDEN's 4 POLLEN, drives up the left corridor and across above the HIVE to the REAR cell's "
                        + "shooting pose (33 in from it, straight at it) and fires there ONLY after its camera has seen that cell UP and steady. It stops there, 3.6 in off the rear wall, "
                        + "next to the rear FLOWER: TELEOP can fetch the other 4 from it. POINTS: LEAVE 3 + the tip 20 if 3 of the 4 first shots land = 23; the 4 banked POLLEN score "
                        + "nothing in AUTO. SPARE POLLEN: 1 in the first volley (probably spent). WHAT CAN GO WRONG: it works only if the first volley tipped the HIVE (about half of the time): "
                        + "otherwise the rear cell never comes up, the camera never sees it, and the robot just stands there with 4 POLLEN (use a top-up route if the tip matters more); "
                        + "it ends at about 22 s on the plan, with several seconds to spare; the PARK (5) does not fit: with it the route needs about 26 s. THE PARTNER: if the partner ALSO shoots into "
                        + "the rear cell, our 4 only help; keep out of its way in the corridor.",
                "Front edge ON the audience wall (the center of rotation is " + FRONT + " in out), facing the wall, in line with the HIVE: the same start as the basic route, "
                        + "a shooting pose. A start nearer the GARDEN saves only 0.3 s on this route, so the easy start is used."), ANCHOR_START))
                .driveConstant(CORRIDOR_X, 22.0, -90.0).label("along the wall")
                .note("Slides along the audience wall and out of the GARDEN to the left corridor.")
                .driveReverseTangent(CORRIDOR_X, 112.0).label("up the left corridor")
                .note("Backwards up the left corridor, past the HIVE (13 in clear of its keep-out) and above it, out of the spill zone of the cell that tipped (it has been clear for a long time).")
                .driveLinear(REAR_SHOT.x, REAR_SHOT.y, REAR_SHOT.headingDegrees()).label("to the rear shot")
                .note("Across above the HIVE to the rear cell's shooting pose, turning to face the rear wall with the back toward the HIVE; the robot's left edge is 2.3 in clear of the rear-wall FLOWER's plate (x 44 to 50.3).")
                .shootUp(REAR_SHOT.x, REAR_SHOT.y, REAR_SHOT.headingDegrees(), 4)
                .note("THE BANK. After our own tip the REAR cell is the one that is UP, and it is empty, so these 4 POLLEN do not tip it (it needs 8): they wait in it for TELEOP. The robot "
                        + "fires ONLY after its camera has seen the rear cell UP and steady (0.5 s). With no camera, or if our first volley did not tip the HIVE (then the audience cell is "
                        + "still the UP one), the volley is skipped and the robot just stays here with the 4 POLLEN. From this pose the ball leaves 32.8 in from the cell's aim point, in line "
                        + "with it, and the robot ends 3.6 in off the rear wall, so the LEAVE is kept.")
                .build();
    }
}
