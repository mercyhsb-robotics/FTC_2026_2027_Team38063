package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Checks {@link BioBuzzConstants} for values that cannot work, so a typo shows up at INIT as a clear
 * message instead of a crash or a strange robot later. Errors must be fixed; warnings point at values that
 * work but look suspicious; placeholders are the values still waiting for a measurement.
 *
 * <p>Pure Java: it runs on the robot at INIT and in a unit test on a PC.
 *
 * <p>Why it exists: every tunable number lives in one file, so one slipped decimal point (a negative time, a window
 * wider than 100 %, two functions on one button) can break the whole feature. This class is the "spell checker"
 * for that file. It only READS the constants; it never changes a value.
 *
 * <p>Who calls it: the TeleOp ({@code BioBuzzRobot.init}) runs it first thing and refuses to start if
 * {@link Report#ok()} is false; it shows the warnings and the placeholder list on the Driver Station screen. A unit
 * test (ConstantsDisciplineTest) runs it too and fails if the shipped constants produce any error or warning, so a
 * bad edit is caught on a PC before it reaches the robot.
 *
 * <p>To add a check: write the condition below, put a plain-language message in the matching list
 * ({@code errors} if the robot cannot work, {@code warnings} if it probably works but looks wrong), and say which
 * constant to fix in the message.
 */
public final class ConstantsCheck {

    /** The result of a check. */
    public static final class Report {
        /** Problems that make the robot unsafe or unable to run; INIT stops if there is any. */
        public final List<String> errors = new ArrayList<String>();
        /** Suspicious-but-legal values; shown to the team, does not stop the robot. */
        public final List<String> warnings = new ArrayList<String>();
        /** Names ("Group.NAME") of constants still marked ADD, i.e. waiting for a real measurement. */
        public final List<String> placeholders = new ArrayList<String>();

        /** True if there are no errors (warnings and placeholders do not count). */
        public boolean ok() {
            return errors.isEmpty();
        }
    }

    private ConstantsCheck() {
    }

    /** Runs every check against the current {@link BioBuzzConstants} and returns what it found. */
    public static Report run() {
        Report r = new Report();
        r.placeholders.addAll(Arrays.asList(BioBuzzConstants.PLACEHOLDERS));

        // ---- the shot table and the planner band
        // Building the table also validates its rows (positive, strictly increasing range); a bad table throws.
        ShotTable table = null;
        try {
            table = ShotTable.fromConstants();
        } catch (IllegalArgumentException e) {
            r.errors.add("Shooter.SHOT_TABLE: " + e.getMessage());
        }
        if (table != null) {
            // The planner only stands where the range is at least RANGE_MARGIN_IN inside both ends of the table.
            // If the margins eat the whole table there is nowhere left to shoot from.
            double lo = table.minRange() + BioBuzzConstants.Shooter.RANGE_MARGIN_IN;
            double hi = table.maxRange() - BioBuzzConstants.Shooter.RANGE_MARGIN_IN;
            if (hi <= lo) {
                r.errors.add("Shooter.RANGE_MARGIN_IN is too large for the shot table: no range is left to shoot from.");
            }
            // A preferred range outside the usable band is only a warning: the planner still works, it just can never
            // pick the range it says it prefers.
            double preferred = BioBuzzConstants.AutoShoot.PREFERRED_RANGE_IN;
            if (preferred < lo || preferred > hi) {
                r.warnings.add("AutoShoot.PREFERRED_RANGE_IN (" + preferred + ") is outside the usable shot range ["
                        + lo + ", " + hi + "].");
            }
        }

        // ---- shooter timing and windows
        // The "at speed" window is a fraction of the target speed (0.06 = 6 %). It must be above 0 and below 0.5:
        // a window of 50 % or more is far too loose to mean "at speed".
        if (BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC <= 0 || BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC >= 0.5
                || BioBuzzConstants.Shooter.AT_SPEED_ABOVE_FRAC <= 0 || BioBuzzConstants.Shooter.AT_SPEED_ABOVE_FRAC >= 0.5) {
            r.errors.add("Shooter.AT_SPEED_BELOW_FRAC and AT_SPEED_ABOVE_FRAC must be between 0 and 0.5.");
        }
        // A ball is counted when the speed dips by SHOT_DIP_FRAC. If that dip is as deep as the "at speed" window,
        // the window closes (the feeder pauses) before the dip is seen, so shots can go uncounted.
        if (BioBuzzConstants.Shooter.SHOT_DIP_FRAC <= 0 || BioBuzzConstants.Shooter.SHOT_DIP_FRAC >= 0.5) {
            r.errors.add("Shooter.SHOT_DIP_FRAC must be between 0 and 0.5.");
        } else if (BioBuzzConstants.Shooter.SHOT_DIP_FRAC >= BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC) {
            r.warnings.add("Shooter.SHOT_DIP_FRAC is not smaller than AT_SPEED_BELOW_FRAC: the speed window will close"
                    + " before a dip is counted, so shots may be missed by the counter.");
        }
        if (BioBuzzConstants.Shooter.AT_SPEED_RISE_SEC < 0 || BioBuzzConstants.Shooter.AT_SPEED_FALL_SEC < 0
                || BioBuzzConstants.Shooter.SHOT_REFRACTORY_SEC < 0) {
            r.errors.add("Shooter times (AT_SPEED_RISE_SEC, AT_SPEED_FALL_SEC, SHOT_REFRACTORY_SEC) cannot be negative.");
        }
        // Rule G407: a robot may control at most 4 POLLEN, so a volley of more than 4 shots is never legal.
        if (BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY < 1 || BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY > 4) {
            r.errors.add("Shooter.MAX_SHOTS_PER_VOLLEY must be 1 to 4 (rule G407: a robot may hold at most 4 POLLEN).");
        }
        // The volley time limit should be long enough for every shot to be fed at the per-shot feed time.
        if (BioBuzzConstants.Shooter.FEED_SECONDS_PER_SHOT <= 0 || BioBuzzConstants.Shooter.VOLLEY_MAX_SEC <= 0) {
            r.errors.add("Shooter.FEED_SECONDS_PER_SHOT and VOLLEY_MAX_SEC must be positive.");
        } else if (BioBuzzConstants.Shooter.VOLLEY_MAX_SEC
                < BioBuzzConstants.Shooter.FEED_SECONDS_PER_SHOT * BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY) {
            r.warnings.add("Shooter.VOLLEY_MAX_SEC is shorter than FEED_SECONDS_PER_SHOT x MAX_SHOTS_PER_VOLLEY: a full"
                    + " volley may be cut short.");
        }
        // Servo power is limited to -1..1 (full speed each way); anything larger is a typo.
        if (Math.abs(BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER) > 1.0 || Math.abs(BioBuzzConstants.Shooter.FEEDER_REVERSE_POWER) > 1.0) {
            r.errors.add("Shooter.FEEDER_*_POWER must be between -1 and 1.");
        }

        // ---- robot size (rule R102: the start configuration fits an 18 in cube)
        if (BioBuzzConstants.Robot.LENGTH_IN <= 0 || BioBuzzConstants.Robot.WIDTH_IN <= 0) {
            r.errors.add("Robot.LENGTH_IN and WIDTH_IN must be positive.");
        } else {
            if (BioBuzzConstants.Robot.LENGTH_IN > 18.0 || BioBuzzConstants.Robot.WIDTH_IN > 18.0
                    || BioBuzzConstants.Robot.HEIGHT_IN > 18.0) {
                r.warnings.add("The robot is larger than the 18 in start cube (rule R102).");
            }
            checkRobotEdges(r, BioBuzzConstants.Robot.FRONT_EXTENT_IN, BioBuzzConstants.Robot.REAR_EXTENT_IN,
                    BioBuzzConstants.Robot.LEFT_EXTENT_IN, BioBuzzConstants.Robot.RIGHT_EXTENT_IN,
                    BioBuzzConstants.Robot.LENGTH_IN, BioBuzzConstants.Robot.WIDTH_IN, BioBuzzConstants.Robot.INTAKE_REACH_IN,
                    BioBuzzConstants.Robot.INTAKE_WHEEL_OFFSET_IN, BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN,
                    BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN, BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN);
        }
        // Forward is the intake's direction; the shooter fires out the back, so the ball exit is at negative forward.
        if (BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN >= 0) {
            r.warnings.add("Robot.LAUNCH_POINT_FORWARD_IN is not negative, but the shooter fires out the BACK.");
        }

        // ---- the FLOWER wiggle (autoshoot.routes.FlowerWiggle)
        checkFlowerWiggle(r);

        // ---- planner
        // The cone half-angle is the angle allowed either side of the opening's center line. 60 degrees is already
        // extreme: from there the opening looks only about half as wide as it does head-on.
        if (BioBuzzConstants.AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG < 0 || BioBuzzConstants.AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG > 60) {
            r.errors.add("AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG must be between 0 and 60.");
        }
        // The planner searches in steps of this size; a step of zero or less would make that search run (almost)
        // forever.
        if (BioBuzzConstants.AutoShoot.SHOOT_CONE_STEP_DEG <= 0 || BioBuzzConstants.AutoShoot.SHOOT_RANGE_STEP_IN <= 0) {
            r.errors.add("AutoShoot.SHOOT_CONE_STEP_DEG and SHOOT_RANGE_STEP_IN must be positive.");
        }
        if (BioBuzzConstants.AutoShoot.ALIGN_POS_TOL_IN <= 0 || BioBuzzConstants.AutoShoot.ALIGN_HEADING_TOL_DEG <= 0) {
            r.errors.add("AutoShoot.ALIGN_*_TOL must be positive.");
        }
        // The robot must be allowed to stay settled for ALIGN_SETTLE_SEC before the align step times out, or it
        // could never finish aligning.
        if (BioBuzzConstants.AutoShoot.ALIGN_TIMEOUT_SEC <= BioBuzzConstants.AutoShoot.ALIGN_SETTLE_SEC) {
            r.errors.add("AutoShoot.ALIGN_TIMEOUT_SEC must be longer than ALIGN_SETTLE_SEC.");
        }
        // A stick value is 0..1, so a threshold of 0 or less would cancel the drive on stick noise, and more
        // than 1 could never be reached.
        checkReapproachHeading(r, BioBuzzConstants.AutoShoot.REAPPROACH_HEADING_DEG, BioBuzzConstants.AutoShoot.APPROACH_HANDOFF_DEG);
        checkSpillZone(r, BioBuzzConstants.Auto.TIP_SPILL_REACH_IN, BioBuzzConstants.Auto.DOWN_CELL_PLAN_OUT_FAR_IN,
                BioBuzzConstants.Auto.TIP_SPILL_MARGIN_IN, BioBuzzConstants.Auto.TIP_SPILL_CLEAR_SEC);
        checkTipStop(r, BioBuzzConstants.Auto.BALL_FLIGHT_SEC, BioBuzzConstants.Auto.TIP_STOP_LATENCY_SEC, BioBuzzConstants.Auto.TIME_PER_SHOT_SEC);
        if (BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_STICK <= 0 || BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_STICK > 1) {
            r.errors.add("AutoShoot.DRIVER_OVERRIDE_STICK must be between 0 and 1.");
        }

        // ---- route wall-safety check (RouteSweep): how finely the whole robot is checked along a drive
        // A step of zero or less would make the check never finish; the finest step cannot be coarser than the
        // ordinary step it refines.
        if (BioBuzzConstants.AutoShoot.PATH_SWEEP_STEP_IN <= 0 || BioBuzzConstants.AutoShoot.PATH_SWEEP_STEP_DEG <= 0
                || BioBuzzConstants.AutoShoot.PATH_SWEEP_MIN_STEP_IN <= 0 || BioBuzzConstants.AutoShoot.PATH_MIN_LEG_IN <= 0) {
            r.errors.add("AutoShoot.PATH_SWEEP_STEP_IN, PATH_SWEEP_STEP_DEG, PATH_SWEEP_MIN_STEP_IN and PATH_MIN_LEG_IN"
                    + " must be positive.");
        } else if (BioBuzzConstants.AutoShoot.PATH_SWEEP_MIN_STEP_IN > BioBuzzConstants.AutoShoot.PATH_SWEEP_STEP_IN) {
            r.errors.add("AutoShoot.PATH_SWEEP_MIN_STEP_IN must not be larger than PATH_SWEEP_STEP_IN.");
        }
        if (BioBuzzConstants.AutoShoot.PATH_MAX_TURN_DEG_PER_IN <= 0) {
            r.errors.add("AutoShoot.PATH_MAX_TURN_DEG_PER_IN must be positive (the planner could never turn the robot"
                    + " away from a wall).");
        }
        if (BioBuzzConstants.AutoShoot.PATH_TURN_SETTLE_IN < 0) {
            r.errors.add("AutoShoot.PATH_TURN_SETTLE_IN cannot be negative.");
        }
        // The detour search walks around a ring in steps of this many degrees; it must make progress and fit in a circle.
        if (BioBuzzConstants.AutoShoot.PATH_DETOUR_ANGLE_STEP_DEG <= 0
                || BioBuzzConstants.AutoShoot.PATH_DETOUR_ANGLE_STEP_DEG > 180) {
            r.errors.add("AutoShoot.PATH_DETOUR_ANGLE_STEP_DEG must be between 0 (exclusive) and 180 degrees.");
        }
        if (BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN < 0) {
            r.errors.add("AutoShoot.PATH_SWEEP_TOLERANCE_IN cannot be negative.");
        } else if (BioBuzzConstants.AutoShoot.PATH_SWEEP_TOLERANCE_IN >= BioBuzzConstants.AutoShoot.WALL_CLEARANCE_IN) {
            // The forgiven amount is taken off the wall clearance, so it must be smaller than it, or a route could be
            // allowed to touch (or enter) the wall.
            r.warnings.add("AutoShoot.PATH_SWEEP_TOLERANCE_IN is not smaller than WALL_CLEARANCE_IN: a route that passes"
                    + " the check may touch the wall.");
        }

        // ---- vision
        if (BioBuzzConstants.Vision.TAG_MIN_RANGE_IN >= BioBuzzConstants.Vision.TAG_MAX_RANGE_IN) {
            r.errors.add("Vision.TAG_MIN_RANGE_IN must be smaller than TAG_MAX_RANGE_IN.");
        }
        // Pitch is the camera's upward tilt in degrees (a negative number looks down). -20..80 is the believable
        // range for a camera mounted on this robot; it catches typos such as 350 or -35.
        if (BioBuzzConstants.Vision.CAMERA_PITCH_UP_DEG < -20 || BioBuzzConstants.Vision.CAMERA_PITCH_UP_DEG > 80) {
            r.errors.add("Vision.CAMERA_PITCH_UP_DEG is outside -20..80 degrees.");
        }
        if (BioBuzzConstants.Vision.CAMERA_HEIGHT_IN <= 0) {
            r.errors.add("Vision.CAMERA_HEIGHT_IN must be positive.");
        }
        // The Limelight can be polled between 1 and 250 times per second.
        if (BioBuzzConstants.Vision.POLL_RATE_HZ < 1 || BioBuzzConstants.Vision.POLL_RATE_HZ > 250) {
            r.errors.add("Vision.POLL_RATE_HZ must be between 1 and 250.");
        }
        if (BioBuzzConstants.Vision.HIVE_CONFIRM_FRAMES < 1) {
            r.errors.add("Vision.HIVE_CONFIRM_FRAMES must be at least 1.");
        }

        // ---- controls
        // Compare every pair of buttons: two functions on one button would fire together.
        Button[] buttons = {BioBuzzConstants.Controls.SHOOT_BUTTON, BioBuzzConstants.Controls.TOGGLE_HIVE_BUTTON,
                BioBuzzConstants.Controls.RESET_POSE_BUTTON, BioBuzzConstants.Controls.ALLIANCE_RED_BUTTON,
                BioBuzzConstants.Controls.ALLIANCE_BLUE_BUTTON};
        for (int i = 0; i < buttons.length; i++) {
            for (int j = i + 1; j < buttons.length; j++) {
                if (buttons[i] == buttons[j]) {
                    r.errors.add("BioBuzzConstants.Controls assigns the button " + buttons[i] + " to two functions.");
                }
            }
        }
        // The direction signs only flip a direction, so each must be exactly +1 or -1 (never 0 or 2).
        double[] signs = {BioBuzzConstants.Controls.MANUAL_FORWARD_SIGN, BioBuzzConstants.Controls.MANUAL_STRAFE_SIGN,
                BioBuzzConstants.Controls.MANUAL_TURN_SIGN};
        for (double sgn : signs) {
            if (Math.abs(sgn) != 1.0) {
                r.errors.add("Controls.MANUAL_*_SIGN must be +1 or -1.");
                break;
            }
        }

        // ---- the field numbers must fit each other (size, tiles, HIVE, FLOWERS, zones)
        checkFieldNumbers(r, BioBuzzConstants.Field.SIZE_IN, BioBuzzConstants.Field.TILE_PITCH_IN, BioBuzzConstants.Field.TILES_PER_SIDE,
                BioBuzzConstants.Field.HIVE_FRAME_WIDTH_X_IN, BioBuzzConstants.Field.HIVE_FRAME_DEPTH_Y_IN,
                BioBuzzConstants.Field.RED_HIVE_AXIS_X_IN, BioBuzzConstants.Field.CELL_OPENING_WIDTH_IN,
                BioBuzzConstants.Field.FLOWER_CENTERS_IN, BioBuzzConstants.Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_X_MAX_IN,
                BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MIN_IN, BioBuzzConstants.Auto.RED_LOADING_ZONE_Y_MAX_IN);

        checkHiveSolid(r, BioBuzzConstants.Auto.HIVE_SOLID_HEIGHT_IN, Math.max(BioBuzzConstants.Robot.HEIGHT_IN, BioBuzzConstants.Vision.CAMERA_HEIGHT_IN),
                BioBuzzConstants.Field.HIVE_LOWEST_CELL_POINT_IN, BioBuzzConstants.Auto.HIVE_LANE_MARGIN_IN, BioBuzzConstants.Robot.WIDTH_IN);
        checkClusterGeometry(r, BioBuzzConstants.Field.UP_CELL_TAG_ROW_OUT_IN, BioBuzzConstants.Field.UP_CELL_TAG_ROW_Z_IN,
                BioBuzzConstants.Field.UP_CELL_ORIGIN_OUT_IN, BioBuzzConstants.Field.UP_CELL_ORIGIN_Z_IN,
                BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_OUT_IN, BioBuzzConstants.Field.DOWN_CELL_TAG_ROW_Z_IN,
                BioBuzzConstants.Field.HIVE_PIVOT_Z_IN, BioBuzzConstants.Field.HIVE_TIP_ANGLE_DEG, BioBuzzConstants.Field.CELL_TILT_DEG,
                BioBuzzConstants.Field.TAG_ROW_BEHIND_CELL_FRONT_IN, BioBuzzConstants.Field.CLUSTER_ORIGIN_ABOVE_STICKER_IN);

        // ---- the HIVE tip arithmetic must reproduce FIRST's own calibration table
        checkTipModel(r, BioBuzzConstants.Auto.NECTAR_STAGED_IN_UP_CELL, BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR,
                BioBuzzConstants.Auto.NECTAR_POLLEN_EQUIVALENT_DENOMINATOR, BioBuzzConstants.Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD,
                BioBuzzConstants.Auto.TIP_PLANNING_MARGIN_POLLEN, BioBuzzConstants.Auto.PRELOAD_POLLEN);

        // ---- the field model must hold together
        // Build the field from the constants and make sure every aim point is on the field; this catches a typo in
        // the HIVE geometry (for example a wrong sign or a missing digit) before the planner trusts it.
        try {
            FieldModel f = new FieldModel();
            for (Alliance a : Alliance.values()) {
                for (Side s : Side.values()) {
                    CellSpec c = f.cell(a, s);
                    if (!f.fieldRect().contains(c.originX, c.originY)) {
                        r.errors.add("Field geometry puts the " + c + " aim point outside the field.");
                    }
                }
            }
            checkStartPose(r, f, "Controls.DEFAULT_START_*", Pose2.ofDegrees(BioBuzzConstants.Controls.DEFAULT_START_X_IN,
                    BioBuzzConstants.Controls.DEFAULT_START_Y_IN, BioBuzzConstants.Controls.DEFAULT_START_HEADING_DEG));
        } catch (RuntimeException e) {
            r.errors.add("Field geometry is invalid: " + e.getMessage());
        }
        return r;
    }

    // FIRST's own calibration of every HIVE cell (Event Field Setup Guide V1.0 section 12.3, Field Acceptance Checklist 2.3 to 2.6, Field
    // Mitigation Guide Appendix A): an EMPTY cell tips with the 8th POLLEN and not with 7, and a cell holding the 3 staged NECTAR tips with
    // the 3rd POLLEN and not with 2. They are facts of the field, not tuning, so they are written here as the reference the tip constants
    // are checked against.
    private static final int OFFICIAL_STAGED_NECTAR = 3;
    private static final int OFFICIAL_EMPTY_CELL_NO_TIP_POLLEN = 7;
    private static final int OFFICIAL_EMPTY_CELL_TIP_POLLEN = 8;
    private static final int OFFICIAL_STAGED_CELL_NO_TIP_POLLEN = 2;
    private static final int OFFICIAL_STAGED_CELL_TIP_POLLEN = 3;

    /**
     * The G409 spill zone of a tipped cell ({@code AllianceField.spillZone} of the route checker) must be a real zone: it has to reach at least
     * as far out as the tipped cell itself hangs ({@code downCellFarIn}, the far end of the DOWN cell's footprint measured from the pivot line), because
     * what falls out lands beyond the cell's own mouth, the margin and the time cannot be negative. A reach that stops short of the cell would put
     * the zone under the cell and leave the floor where the balls land unguarded, which is the first version's mistake.
     */
    static void checkSpillZone(Report r, double reachIn, double downCellFarIn, double marginIn, double clearSec) {
        if (!(reachIn > downCellFarIn)) {
            r.errors.add("Auto.TIP_SPILL_REACH_IN (" + reachIn + " in) must be more than the far end of the tipped cell (Auto.DOWN_CELL_PLAN_OUT_FAR_IN, "
                    + downCellFarIn + " in): what falls out of a tipped cell lands beyond its own mouth, so a zone that stops at the cell guards nothing.");
        }
        if (marginIn < 0.0 || clearSec < 0.0) {
            r.errors.add("Auto.TIP_SPILL_MARGIN_IN (" + marginIn + ") and TIP_SPILL_CLEAR_SEC (" + clearSec + ") cannot be negative.");
        }
    }

    /**
     * A robot that has just been handed over from APPROACH to ALIGN is up to {@code handoffDeg} off its hold heading. The heading at which ALIGN
     * (and FEED) plan a NEW route for a robot that was turned ({@code reapproachDeg}) must be larger than that. If it were not, ALIGN would send
     * the robot straight back to PLAN, and PLAN straight back to ALIGN, for ever.
     */
    /**
     * The numbers that decide whether the spare POLLEN of a volley is kept or spent: how long a POLLEN is in the air, how long the camera-gated feeder
     * takes to stop after the tipping POLLEN lands, and the time between two POLLEN. The flight and the time between POLLEN must be positive, the
     * stop latency cannot be negative; a latency of 0 means a feeder that stops the instant the tip lands.
     */
    static void checkTipStop(Report r, double flightSec, double stopLatencySec, double secondsPerShot) {
        if (!(flightSec > 0.0) || !(secondsPerShot > 0.0)) {
            r.errors.add("Auto.BALL_FLIGHT_SEC (" + flightSec + ") and TIME_PER_SHOT_SEC (" + secondsPerShot + ") must be positive: they are the seconds a POLLEN is in the"
                    + " air and the seconds between two POLLEN of a volley.");
        }
        if (stopLatencySec < 0.0) {
            r.errors.add("Auto.TIP_STOP_LATENCY_SEC (" + stopLatencySec + ") cannot be negative: it is the time from the tipping POLLEN landing until the feeder has stopped.");
        }
    }

    static void checkReapproachHeading(Report r, double reapproachDeg, double handoffDeg) {
        if (reapproachDeg <= handoffDeg) {
            r.errors.add("AutoShoot.REAPPROACH_HEADING_DEG (" + reapproachDeg + ") must be larger than APPROACH_HANDOFF_DEG (" + handoffDeg
                    + "), or a robot handed over from APPROACH would be re-planned at once and the controller would loop between PLAN and ALIGN.");
        }
    }

    /**
     * The tip arithmetic of the route planner must agree with FIRST's calibration: with {@code staged} NECTAR a cell tips with 3 POLLEN and
     * not with 2, and an empty cell tips with 8 POLLEN and not with 7. One NECTAR counts as {@code numerator / denominator} POLLEN and a
     * cell tips at {@code threshold} POLLEN-equivalents (whole numbers: {@code nectar x numerator + pollen x denominator >= threshold x
     * denominator}, the same test as {@code TipModel.tips} of the route planner, which this class may not call because the planner depends
     * on this package and not the other way round). {@code margin} is the spare POLLEN a route plans beyond the minimum and
     * {@code preloads} the POLLEN the robot starts with.
     *
     * <p>Why: the planner decides from these numbers which routes tip the HIVE and how many POLLEN a route must fetch. A weight that
     * does not reproduce the table (for example the old 1/5 + 1/8 rule, which said 3 NECTAR + 3 POLLEN do not tip) would make every
     * route's points and POLLEN plan wrong, and nothing else would notice.
     */
    static void checkTipModel(Report r, int staged, int numerator, int denominator, int threshold, int margin, int preloads) {
        if (numerator <= 0 || denominator <= 0 || threshold <= 0) {
            r.errors.add("Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR / DENOMINATOR and TIP_POLLEN_EQUIVALENT_THRESHOLD must be positive whole numbers"
                    + " (a NECTAR counts numerator / denominator POLLEN, a cell tips at threshold POLLEN).");
            return;
        }
        if (staged != OFFICIAL_STAGED_NECTAR) {
            r.errors.add("Auto.NECTAR_STAGED_IN_UP_CELL (" + staged + ") must be " + OFFICIAL_STAGED_NECTAR
                    + ": the manual (Fig 10-2) puts 3 NECTAR in the UP cell of each HIVE, and FIRST's tip table is written for them.");
        }
        String weight = "NECTAR = " + numerator + "/" + denominator + " POLLEN, a cell tips at " + threshold + " POLLEN-equivalents";
        if (cellTips(0, OFFICIAL_EMPTY_CELL_NO_TIP_POLLEN, numerator, denominator, threshold)) {
            r.errors.add("The tip constants (" + weight + ") say that " + OFFICIAL_EMPTY_CELL_NO_TIP_POLLEN
                    + " POLLEN tip an empty HIVE cell, but FIRST's field calibration says they do not (the 8th POLLEN tips it).");
        }
        if (!cellTips(0, OFFICIAL_EMPTY_CELL_TIP_POLLEN, numerator, denominator, threshold)) {
            r.errors.add("The tip constants (" + weight + ") say that " + OFFICIAL_EMPTY_CELL_TIP_POLLEN
                    + " POLLEN do not tip an empty HIVE cell, but FIRST's field calibration says they do.");
        }
        if (cellTips(OFFICIAL_STAGED_NECTAR, OFFICIAL_STAGED_CELL_NO_TIP_POLLEN, numerator, denominator, threshold)) {
            r.errors.add("The tip constants (" + weight + ") say that 3 NECTAR + " + OFFICIAL_STAGED_CELL_NO_TIP_POLLEN
                    + " POLLEN tip a HIVE cell, but FIRST's field calibration says they do not (the 3rd POLLEN tips it).");
        }
        if (!cellTips(OFFICIAL_STAGED_NECTAR, OFFICIAL_STAGED_CELL_TIP_POLLEN, numerator, denominator, threshold)) {
            r.errors.add("The tip constants (" + weight + ") say that 3 NECTAR + " + OFFICIAL_STAGED_CELL_TIP_POLLEN
                    + " POLLEN do not tip a HIVE cell, but FIRST's field calibration says they do (the first tip needs only 3 POLLEN).");
        }
        if (margin < 0) {
            r.errors.add("Auto.TIP_PLANNING_MARGIN_POLLEN (" + margin + ") cannot be negative: it is the spare POLLEN beyond the minimum.");
        } else if (OFFICIAL_STAGED_CELL_TIP_POLLEN + margin > preloads) {
            r.warnings.add("Auto.TIP_PLANNING_MARGIN_POLLEN (" + margin + ") asks for " + (OFFICIAL_STAGED_CELL_TIP_POLLEN + margin)
                    + " POLLEN into the first cell, more than the " + preloads + " pre-loads: no route can have that spare on the first tip.");
        }
    }

    /** The same test as {@code TipModel.tips}: whole numbers only, so a sum of exactly the threshold is never lost to rounding. */
    private static boolean cellTips(int nectar, int pollen, int numerator, int denominator, int threshold) {
        return nectar * numerator + pollen * denominator >= threshold * denominator;
    }

    /**
     * The field numbers must fit each other, whatever the field turns out to measure: the tiles must fit between the walls, the HIVE
     * frame must stand inside the field with its two cells' openings between its feet, each FLOWER must stand the POLLEN-center
     * distance from a wall, and the red LOADING ZONE must touch the red wall and lie on the red half. The values are parameters (the
     * shipped constants are passed in by {@link #run()}) so a unit test can feed in bad ones. All lengths are inches.
     *
     * <p>Why: the field is one frame (the wall-corner frame of the official CAD) and every number is given in it. A number copied
     * from the old 144 in frame (or typed in the HIVE-centered frame) breaks one of these relations at once.
     */
    static void checkFieldNumbers(Report r, double size, double tilePitch, int tiles, double frameWidth, double frameDepth,
                                  double redAxisX, double openingWidth, double[][] flowerCenters, double pollenFromWall,
                                  double zoneXMin, double zoneXMax, double zoneYMin, double zoneYMax) {
        final double tolerance = 0.01;
        if (size <= 0 || tilePitch <= 0 || tiles <= 0) {
            r.errors.add("Field.SIZE_IN, TILE_PITCH_IN and TILES_PER_SIDE must be positive.");
            return;
        }
        // The tiles lie edge to edge inside the walls: they can leave a sliver of the outer tiles cut away (the CAD: 141.0 of tile in a
        // 141.348 in field) but never more than a couple of inches, and they cannot be longer than the field.
        double tiled = tilePitch * tiles;
        if (tiled > size + tolerance || size - tiled > 2.0) {
            r.errors.add("Field.TILE_PITCH_IN (" + tilePitch + ") x TILES_PER_SIDE (" + tiles + ") = " + String.format(Locale.ROOT, "%.2f", tiled)
                    + " in does not fit Field.SIZE_IN (" + size + " in): the tiles must cover the floor between the walls to within about 2 in.");
        }
        double center = size / 2.0;
        // The HIVE frame stands in the middle of the field and the red HIVE hangs on the left half of it.
        if (frameWidth <= 0 || frameDepth <= 0 || frameWidth >= size || frameDepth >= size) {
            r.errors.add("Field.HIVE_FRAME_WIDTH_X_IN and HIVE_FRAME_DEPTH_Y_IN (" + frameWidth + " x " + frameDepth
                    + " in) must be positive and smaller than the field.");
        } else {
            double frameMinX = center - frameWidth / 2.0;
            if (redAxisX - openingWidth / 2.0 < frameMinX - tolerance || redAxisX + openingWidth / 2.0 > center + tolerance) {
                r.errors.add("Field.RED_HIVE_AXIS_X_IN (" + redAxisX + ") puts the red cell opening (" + openingWidth + " in wide) outside the red half"
                        + " of the HIVE frame (x " + String.format(Locale.ROOT, "%.2f", frameMinX) + " to " + String.format(Locale.ROOT, "%.2f", center)
                        + "): was a number typed in the old 144 in frame, or measured from the field center instead of the wall corner?");
            }
        }
        // Every FLOWER stands against one wall: its POLLEN center is pollenFromWall in from that wall and inside the field.
        for (int i = 0; i < flowerCenters.length; i++) {
            double x = flowerCenters[i][0];
            double y = flowerCenters[i][1];
            double[] fromWall = {x, size - x, y, size - y};
            boolean onAWall = false;
            for (double d : fromWall) {
                onAWall |= Math.abs(d - pollenFromWall) < tolerance;
            }
            if (!onAWall || x < 0 || y < 0 || x > size || y > size) {
                r.errors.add("Field.FLOWER_CENTERS_IN[" + i + "] = (" + x + ", " + y + ") is not " + pollenFromWall
                        + " in (Field.FLOWER_POLLEN_CENTER_FROM_WALL_IN) from a wall of the " + size + " in field.");
            }
        }
        // The red LOADING ZONE is a box against the red wall, inside the red half.
        if (Math.abs(zoneXMin) > tolerance || zoneXMax <= zoneXMin || zoneXMax >= center || zoneYMin < 0 || zoneYMax <= zoneYMin || zoneYMax > size) {
            r.errors.add("The red LOADING ZONE (Auto.RED_LOADING_ZONE_*: x " + zoneXMin + " to " + zoneXMax + ", y " + zoneYMin + " to " + zoneYMax
                    + ") must touch the red wall (x = 0), lie on the red half and inside the field.");
        }
    }

    /**
     * The exact HIVE solid is only right for a robot that is at least as tall as its tallest part (so no leg stub is missing) and lower
     * than the lowest cell (the solid has no cells in it); and the lane between the feet must be wide enough for the robot plus the lane margin
     * on both sides, or a route that is allowed under the HIVE could never fit (a WARN: the default routes never go there).
     */
    static void checkHiveSolid(Report r, double solidHeight, double robotTallest, double lowestCell, double laneMargin, double robotWidth) {
        if (solidHeight < robotTallest) {
            r.errors.add("Auto.HIVE_SOLID_HEIGHT_IN (" + solidHeight + " in) is lower than the tallest part of the robot (" + robotTallest
                    + " in, the camera included): the legs of the HIVE would look shorter than the robot can reach them.");
        }
        if (solidHeight >= lowestCell) {
            r.errors.add("Auto.HIVE_SOLID_HEIGHT_IN (" + solidHeight + " in) reaches the lowest cell of the HIVE (" + lowestCell
                    + " in): the exact solid has no cells in it.");
            return;
        }
        double[] lane = HiveSolid.of(0.0, solidHeight).narrowestLane();
        double room = lane[1] - lane[0] - robotWidth - 2.0 * laneMargin;
        if (room <= 0.0) {
            r.warnings.add("The lane under the HIVE (" + String.format(Locale.ROOT, "%.1f", lane[1] - lane[0]) + " in) is not wide enough for the robot ("
                    + robotWidth + " in) with Auto.HIVE_LANE_MARGIN_IN (" + laneMargin + " in) on both sides: no route may use allowUnderHive.");
        }
    }

    /**
     * The AprilTag cluster numbers must follow from the HIVE's mechanics: the cluster origin (the aim point) is the tag row moved 7.1874 in
     * up the cell's axis (tilted {@code tiltDeg} from horizontal) and 5.622 in out of the sticker plane, and a DOWN cell is its UP self turned
     * by {@code tipDeg} about the pivot axle. All of this is in the CAD to 0.02 in, so a typo in one of the six numbers (a tag row, an origin or
     * a DOWN row) breaks a relation and is reported here. Distances out are measured from the pivot plane toward the cell's own side.
     */
    static void checkClusterGeometry(Report r, double tagRowOut, double tagRowZ, double originOut, double originZ, double downOut,
                                     double downZ, double pivotZ, double tipDeg, double tiltDeg, double rowBehindFront, double originAbove) {
        final double tolerance = 0.02;
        double tilt = Math.toRadians(tiltDeg);
        double expectedOriginOut = tagRowOut + rowBehindFront * Math.cos(tilt) - originAbove * Math.sin(tilt);
        double expectedOriginZ = tagRowZ + rowBehindFront * Math.sin(tilt) + originAbove * Math.cos(tilt);
        if (Math.abs(expectedOriginOut - originOut) > tolerance || Math.abs(expectedOriginZ - originZ) > tolerance) {
            r.errors.add("Field.UP_CELL_ORIGIN_* (the aim point: " + originOut + " in out, " + originZ + " in up) does not follow from the tag row ("
                    + tagRowOut + " in out, " + tagRowZ + " in up): moving " + rowBehindFront + " in up the cell's " + tiltDeg + " degree axis and "
                    + originAbove + " in out of the sticker plane gives " + String.format(Locale.ROOT, "%.3f out, %.3f up", expectedOriginOut, expectedOriginZ) + ".");
        }
        // A tip turns the cell by tipDeg about the pivot axle: the UP tag row, seen from the axle, is turned that far to the DOWN row.
        double tip = Math.toRadians(tipDeg);
        double up = tagRowZ - pivotZ;
        double expectedDownOut = tagRowOut * Math.cos(tip) + up * Math.sin(tip);
        double expectedDownZ = pivotZ - tagRowOut * Math.sin(tip) + up * Math.cos(tip);
        if (Math.abs(expectedDownOut - downOut) > tolerance || Math.abs(expectedDownZ - downZ) > tolerance) {
            r.errors.add("Field.DOWN_CELL_TAG_ROW_* (" + downOut + " in out, " + downZ + " in up) is not the UP tag row turned " + tipDeg
                    + " degrees about the pivot axle (" + pivotZ + " in up): that gives " + String.format(Locale.ROOT, "%.3f out, %.3f up", expectedDownOut, expectedDownZ) + ".");
        }
    }

    /** The FLOWER wiggle numbers of {@link BioBuzzConstants.Auto}, passed to the checker below. */
    private static void checkFlowerWiggle(Report r) {
        checkFlowerWiggle(r, BioBuzzConstants.Robot.INTAKE_WHEEL_DIAMETER_IN, BioBuzzConstants.Auto.POLLEN_DIAMETER_IN,
                BioBuzzConstants.Auto.FLOWER_WHEEL_START_OFFSET_IN, BioBuzzConstants.Auto.FLOWER_WHEEL_PRESS_OFFSET_IN,
                BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_TURN_DEG, BioBuzzConstants.Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC,
                BioBuzzConstants.Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC, BioBuzzConstants.Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN,
                BioBuzzConstants.Auto.FLOWER_WIGGLE_MAX_CYCLES, BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_SEC,
                BioBuzzConstants.Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC, BioBuzzConstants.Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG,
                BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN, BioBuzzConstants.Auto.FLOWER_APPROACH_SPEED_FRACTION,
                BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN, BioBuzzConstants.Auto.FLOWER_CHASSIS_CLEARANCE_IN);
    }

    /**
     * The numbers of the FLOWER wiggle must make sense together. The two offsets are distances between the MIDDLE of the pressing wheel
     * and the middle of the POLLEN: at the start the wheel has to pass the ball without touching it (more than the two radii added
     * up), at the end it has to overlap the ball (less than the two radii added up) but not by so much that it is on top of it. The
     * values are parameters (the shipped constants are passed in by {@link #run()}) so a unit test can feed in bad ones.
     */
    static void checkFlowerWiggle(Report r, double wheelDiameter, double pollenDiameter, double start, double press, double maxTurnDeg,
                                  double pushHold, double releaseHold, int cyclesPerPollen, int maxCycles, double turnSec,
                                  double turnTimeout, double headingTolDeg, double approachLen, double approachSpeed,
                                  double retreatLen, double clearance) {
        double touching = wheelDiameter / 2.0 + pollenDiameter / 2.0;       // centers this far apart: wheel and POLLEN just touch
        if (!(start > 0.0) || !(press > 0.0)) {
            r.errors.add("Auto.FLOWER_WHEEL_START_OFFSET_IN (" + start + ") and FLOWER_WHEEL_PRESS_OFFSET_IN (" + press
                    + ") are distances between the middle of the wheel and the middle of the POLLEN and must be positive.");
        } else {
            if (press >= touching) {
                r.errors.add("Auto.FLOWER_WHEEL_PRESS_OFFSET_IN (" + press + " in) is not less than the wheel radius plus the POLLEN radius ("
                        + touching + " in): at the end of the push the wheel would only touch the ball, not press into it.");
            }
            if (start <= press) {
                r.errors.add("Auto.FLOWER_WHEEL_START_OFFSET_IN (" + start + " in) must be more than FLOWER_WHEEL_PRESS_OFFSET_IN (" + press
                        + " in): the wheel starts beside the ball and the push brings it closer.");
            } else if (start < touching) {
                r.warnings.add("Auto.FLOWER_WHEEL_START_OFFSET_IN (" + start + " in) is less than the wheel radius plus the POLLEN radius ("
                        + touching + " in): the wheel would already touch the ball on the way in and could drag it out of its hole.");
            }
        }
        if (!(maxTurnDeg > 0.0) || maxTurnDeg > 45.0) {
            r.errors.add("Auto.FLOWER_WIGGLE_MAX_TURN_DEG (" + maxTurnDeg + ") must be more than 0 and at most 45 degrees: a bigger turn"
                    + " would swing the robot's corners into the FLOWER.");
        } else if (!(headingTolDeg > 0.0) || headingTolDeg >= maxTurnDeg) {
            r.errors.add("Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG (" + headingTolDeg + ") must be more than 0 and less than the biggest turn ("
                    + maxTurnDeg + " degrees).");
        }
        if (pushHold < 0.0 || releaseHold < 0.0 || !(turnSec > 0.0)) {
            r.errors.add("Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC and RELEASE_HOLD_SEC cannot be negative and FLOWER_WIGGLE_TURN_SEC must be positive.");
        } else if (turnTimeout <= turnSec) {
            r.warnings.add("Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC (" + turnTimeout + " s) is not longer than FLOWER_WIGGLE_TURN_SEC (" + turnSec
                    + " s): a normal turn would be called stuck.");
        }
        if (cyclesPerPollen < 1 || maxCycles < cyclesPerPollen) {
            r.errors.add("Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN (" + cyclesPerPollen + ") must be at least 1 and no more than FLOWER_WIGGLE_MAX_CYCLES ("
                    + maxCycles + ").");
        } else if (maxCycles < BioBuzzConstants.Auto.FLOWER_POLLEN * cyclesPerPollen) {
            // The cap would cut a pick-up of a whole FLOWER short: 4 POLLEN x 2 cycles = 8 cycles, but the limit says 6, so only 3 POLLEN come out.
            int needed = BioBuzzConstants.Auto.FLOWER_POLLEN * cyclesPerPollen;
            r.errors.add("Auto.FLOWER_WIGGLE_MAX_CYCLES (" + maxCycles + ") is less than a whole FLOWER needs (" + BioBuzzConstants.Auto.FLOWER_POLLEN
                    + " POLLEN x " + cyclesPerPollen + " cycles per POLLEN = " + needed + "): a pick-up of all " + BioBuzzConstants.Auto.FLOWER_POLLEN
                    + " POLLEN would stop after " + (maxCycles / cyclesPerPollen) + ". Raise FLOWER_WIGGLE_MAX_CYCLES to at least " + needed + ".");
        }
        if (!(approachLen > 0.0) || !(retreatLen > 0.0) || !(approachSpeed > 0.0) || approachSpeed > 1.0 || clearance < 0.0) {
            r.errors.add("Auto.FLOWER_APPROACH_LENGTH_IN and FLOWER_RETREAT_LENGTH_IN must be positive, FLOWER_APPROACH_SPEED_FRACTION must be"
                    + " in (0, 1] and FLOWER_CHASSIS_CLEARANCE_IN cannot be negative.");
        }
    }

    /**
     * The robot's edges around its center of rotation (see {@link RobotShape}): the four distances must be positive and
     * must add up to the overall size, the intake wheels must fit inside them, and the ball exit must lie on the robot.
     * The tolerance (0.01 in) only forgives the rounding of a drawing dimension typed with two decimals. The values are
     * parameters (the shipped constants are passed in by {@link #run()}) so a unit test can feed in bad ones.
     */
    static void checkRobotEdges(Report r, double front, double rear, double left, double right, double length, double width,
                                double reach, double wheelOffset, double wheelDiameter, double lf, double ll) {
        final double sumTolerance = 0.01;
        if (front <= 0 || rear <= 0 || left <= 0 || right <= 0) {
            r.errors.add("Robot.FRONT_EXTENT_IN, REAR_EXTENT_IN, LEFT_EXTENT_IN and RIGHT_EXTENT_IN (how far each edge of the"
                    + " robot is from its center of rotation) must all be positive.");
            return;
        }
        if (Math.abs(front + rear - length) > sumTolerance) {
            r.errors.add("Robot.FRONT_EXTENT_IN (" + front + ") + REAR_EXTENT_IN (" + rear + ") = " + String.format(Locale.ROOT, "%.2f", front + rear)
                    + " in, but Robot.LENGTH_IN is " + length + " in: the distances from the center"
                    + " of rotation to the front and rear edges must add up to the robot's length. Re-measure the center of"
                    + " rotation, or fix the typo.");
        }
        if (Math.abs(left + right - width) > sumTolerance) {
            r.errors.add("Robot.LEFT_EXTENT_IN (" + left + ") + RIGHT_EXTENT_IN (" + right + ") = " + String.format(Locale.ROOT, "%.2f", left + right)
                    + " in, but Robot.WIDTH_IN is " + width + " in: the distances from the center"
                    + " of rotation to the left and right edges must add up to the robot's width. Re-measure the center of"
                    + " rotation, or fix the typo.");
        }
        // The flat intake wheels sit at the front: the frame has to reach behind the wheel tips, and each wheel disc
        // has to fit inside the side edge it sits near.
        if (reach <= 0 || reach >= front) {
            r.errors.add("Robot.INTAKE_REACH_IN (" + reach + ") must be more than 0 and less than FRONT_EXTENT_IN (" + front
                    + "): it is how far the flat wheels stick out in front of the chassis frame, so the frame's front edge"
                    + " would not be in front of the center of rotation.");
        }
        double wheelOuter = wheelOffset + wheelDiameter / 2.0;
        if (wheelOuter > Math.min(left, right) + sumTolerance) {
            r.warnings.add("A flat intake wheel (centered " + wheelOffset + " in from the center"
                    + " line, " + wheelDiameter + " in across) would reach " + wheelOuter
                    + " in to the side, past the robot's side edge (" + Math.min(left, right) + " in).");
        }
        // The ball leaves the robot, so the exit must lie on it. This also catches a mix-up of units (millimeters typed
        // as inches) and a sign error (the shooter is at the back, which is NEGATIVE forward).
        double eps = 1e-9;
        if (lf < -rear - eps || lf > front + eps || ll > left + eps || ll < -right - eps) {
            r.warnings.add("Robot.LAUNCH_POINT_* (" + lf + " in forward, " + ll + " in left) is outside the robot's perimeter"
                    + " (" + (-rear) + " to " + front + " in forward, " + (-right) + " to " + left + " in left of the center of"
                    + " rotation).");
        }
    }

    /**
     * A start pose (RED frame) must be a pose the REAL footprint can stand on: inside the field (the robot's edges,
     * turned to its heading, may not poke through a wall) and clear of the HIVE frame and the FLOWER plates. A start
     * whose center of rotation is closer to a wall than the robot's edge toward that wall would start the match with the
     * robot "inside" the wall, and the odometry would be wrong from the first loop. {@link #run()} uses it for the TeleOp
     * default start; it takes the pose as a parameter so a unit test can try bad ones.
     *
     * @param what the name to put in the message, for example {@code Controls.DEFAULT_START_*}
     */
    static void checkStartPose(Report r, FieldModel field, String what, Pose2 start) {
        Footprint fp = Footprint.of(start);
        double tolerance = BioBuzzConstants.Auto.FIELD_BOUNDS_TOLERANCE_IN;
        double outside = fp.overshootOutside(field.fieldRect());
        if (outside > tolerance) {
            RobotShape shape = RobotShape.fromConstants();
            r.errors.add(what + " puts the robot's real footprint " + String.format(Locale.ROOT, "%.2f", outside) + " in through a field wall (the start pose is "
                    + start + "). The pose is the robot's center of rotation, which must be at least as far from a wall as the"
                    + " robot's edge that faces it: front " + shape.frontIn() + " in, rear " + shape.rearIn() + " in, each side "
                    + shape.leftIn() + " in. A robot that stands with its FRONT on the audience wall (heading -90 degrees)"
                    + " has its center of rotation " + shape.frontIn() + " in from it.");
        }
        for (Rect o : field.hardObstacles()) {
            if (fp.overlaps(o)) {
                r.errors.add(what + " puts the robot on top of " + o + " (the HIVE frame or a FLOWER plate).");
            }
        }
    }
}
