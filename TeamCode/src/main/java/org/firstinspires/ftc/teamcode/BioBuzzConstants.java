package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.autoshoot.core.Button;
import org.firstinspires.ftc.teamcode.autoshoot.core.Side;

/**
 * THE ONE CONSTANTS FILE for Team 38063's BIOBUZZ robot.
 *
 * <p>Every number that can change how the robot behaves on the field lives here, so the Limelight,
 * HIVE-tracking, driving and shooting classes never need to be edited to retune the robot. If a
 * value is needed by the logic and is not in this file, that is a bug: add it here.
 * (The only exception is {@code pedro/Constants.java}, which Pedro Pathing's tuners generate and
 * which owns the drivetrain, Pinpoint and Foresight settings.)
 *
 * <p>Units are in the name or the comment: {@code _IN} inches, {@code _DEG} degrees,
 * {@code _SEC} seconds, {@code _MS} milliseconds, {@code _TPS} encoder ticks per second,
 * {@code _FRAC} a 0..1 fraction. Field coordinates are the Pedro frame: origin at the corner where
 * the audience wall meets the red wall, +x toward blue, +y away from the audience. Everything
 * field-related is written for the RED alliance and converted for BLUE by a 180 degree rotation
 * about the field center.
 *
 * <p>Every value carries one of these markers:
 * <ul>
 *   <li><b>FIXED</b> - from the game manual or the CAD. Change only if the rules/field change.</li>
 *   <li><b>ADD</b> - a PLACEHOLDER waiting for a real measurement. Replace it, then change the
 *       marker to VERIFY or TUNE.</li>
 *   <li><b>VERIFY</b> - believed right, but must be confirmed on the real robot or field once.</li>
 *   <li><b>TUNE</b> - a starting value that is expected to be adjusted from testing.</li>
 * </ul>
 * The symptom-to-knob table in docs/TUNING.md says which constant to reach for when the robot
 * misbehaves.
 */
public final class BioBuzzConstants {
    private BioBuzzConstants() {
    }

    /**
     * The name ("Group.NAME") of every value that is still marked ADD, i.e. a placeholder waiting for a real
     * measurement. The TeleOp lists these at INIT so nothing is forgotten. When you replace a value, change its
     * marker to VERIFY or TUNE AND delete its name here; a unit test fails if this list and the markers disagree.
     */
    public static final String[] PLACEHOLDERS = {
            "Robot.LAUNCH_POINT_FORWARD_IN",
            "Robot.LAUNCH_POINT_LEFT_IN",
            "Shooter.SHOT_TABLE",
            "Vision.CAMERA_FORWARD_IN",
            "Vision.CAMERA_LEFT_IN",
            "Vision.CAMERA_HEIGHT_IN",
            "Vision.CAMERA_PITCH_UP_DEG",
            "Auto.CRUISE_SPEED_IN_PER_SEC",
            "Auto.ACCEL_IN_PER_SEC2",
            "Auto.TURN_RATE_DEG_PER_SEC",
            "Auto.SETTLE_SEC",
            "Auto.INTAKE_DWELL_FLOOR_SEC",
            "Auto.INTAKE_DWELL_FLOWER_SEC",
            "Auto.INTAKE_DWELL_GARDEN_SEC",
            "Auto.AIM_SPINUP_SEC",
            "Auto.TIME_PER_SHOT_SEC"};

    // =========================================================================================
    // HARDWARE NAMES (must match the robot configuration on the Driver Station exactly)
    // =========================================================================================
    public static final class Hardware {
        private Hardware() {
        }

        // Mecanum drive motors. (pedro/Constants.java uses these same names for the Pedro drivetrain.)
        public static final String FRONT_LEFT_MOTOR = "frontLeft";
        public static final String BACK_LEFT_MOTOR = "backLeft";
        public static final String FRONT_RIGHT_MOTOR = "frontRight";
        public static final String BACK_RIGHT_MOTOR = "backRight";

        /** goBILDA Pinpoint odometry computer (an I2C device). */
        public static final String PINPOINT = "pinpoint";
        /** The Control Hub's built-in IMU (used only by the legacy MecanumDrive class). */
        public static final String IMU = "imu";

        // Launcher: one flywheel motor and one continuous-rotation feeder ("windmill") servo.
        public static final String FLYWHEEL = "flywheel";
        public static final String FEEDER = "feeder";

        // Intake: one roller motor and two continuous-rotation servos that pull POLLEN out of corners.
        public static final String INTAKE_MOTOR = "intakeMotor";
        public static final String LEFT_INTAKE_SERVO = "servoLeft";
        public static final String RIGHT_INTAKE_SERVO = "servoRight";

        /** The Limelight 3A (a USB device named in the robot configuration). */
        public static final String LIMELIGHT = Vision.DEVICE_NAME;
    }

    // =========================================================================================
    // AUTONOMOUS ROUTE FRAMEWORK (declarative autos: data model, validator, timing, diagrams)
    // Used by autoshoot.routes and autoshoot.render only. Field geometry is written for RED.
    // =========================================================================================
    public static final class Auto {
        private Auto() {
        }

        // ---- match rules and scoring (manual TU09 sections 10.1-10.5, G304, G402, G407) -------

        /** FIXED (manual 10.1, G401): the AUTONOMOUS period lasts 30 s. */
        public static final double AUTO_DURATION_SEC = 30.0;
        /** FIXED (G407): a robot may CONTROL at most 4 SCORING ELEMENTS at once. */
        public static final int POLLEN_CAPACITY = 4;
        /** FIXED (G304.G / 10.3.4): the robot starts in contact with exactly 4 POLLEN pre-loads. */
        public static final int PRELOAD_POLLEN = 4;
        /** FIXED (manual 10.3.1): every FLOWER starts with 4 POLLEN in it. */
        public static final int FLOWER_POLLEN = 4;
        /** FIXED (manual 10.3.1): every GARDEN starts with a line of 4 POLLEN. */
        public static final int GARDEN_POLLEN = 4;
        /** FIXED (manual 9.8): POLLEN diameter, drawn to scale by the diagrams. */
        public static final double POLLEN_DIAMETER_IN = 2.8;
        /**
         * FIXED (manual Fig 10-2): NECTAR that sit in the HIVE cell that is UP when the match starts (3 of the
         * alliance's own colour). Only that cell holds them: after the HIVE has tipped, the cell that comes up
         * is empty, so it has no NECTAR to help the next tip.
         */
        public static final int NECTAR_STAGED_IN_UP_CELL = 3;
        /**
         * VERIFY: the team's figure for tipping a HIVE cell with NECTAR alone: 5 NECTAR. The manual gives no
         * tip threshold at all, so this and {@link #POLLEN_TIP_THRESHOLD} are the team's figures. A cell tips
         * when {@code nectar / NECTAR_TIP_THRESHOLD + pollen / POLLEN_TIP_THRESHOLD >= 1}: every NECTAR is worth
         * 1/5 of a tip and every POLLEN 1/8 of a tip, so 5 NECTAR, or 8 POLLEN, or a mix tips it. The code
         * multiplies the whole inequality out so it uses whole numbers only (see TipModel).
         */
        public static final int NECTAR_TIP_THRESHOLD = 5;
        /** VERIFY: the team's figure for tipping a cell with POLLEN alone: 8 POLLEN (two volleys of 4). See {@link #NECTAR_TIP_THRESHOLD}. */
        public static final int POLLEN_TIP_THRESHOLD = 8;
        /** FIXED (manual Table 10-2): LEAVE is worth 3 points in AUTO. */
        public static final int LEAVE_POINTS = 3;
        /** FIXED (manual Table 10-2): PARK is worth 5 points in AUTO. */
        public static final int PARK_POINTS = 5;
        /** FIXED (manual Table 10-2): a HIVE TIP is worth 20 points in AUTO. */
        public static final int TIP_POINTS = 20;

        // ---- field zones and layout used by the validator and the diagrams (RED frame) --------

        /** FIXED (manual 9.3, tile A5): the red LOADING ZONE is x [0, 11] ... */
        public static final double RED_LOADING_ZONE_X_MIN_IN = 0.0;
        public static final double RED_LOADING_ZONE_X_MAX_IN = 11.0;
        /** FIXED (manual 9.3, tile A5): ... y [96, 120]. The blue one is its half-turn image. */
        public static final double RED_LOADING_ZONE_Y_MIN_IN = 96.0;
        public static final double RED_LOADING_ZONE_Y_MAX_IN = 120.0;

        /** FIXED (manual 9.3): the red GARDEN is about 23 x 2 in in the corner of tile A1. */
        public static final double RED_GARDEN_X_MIN_IN = 0.0;
        public static final double RED_GARDEN_X_MAX_IN = 23.0;
        public static final double RED_GARDEN_Y_MIN_IN = 0.0;
        public static final double RED_GARDEN_Y_MAX_IN = 2.0;
        /** VERIFY (manual Fig 9-17, +/- 0.5 in): centers of the 4 red GARDEN POLLEN along the audience wall. */
        public static final double[] RED_GARDEN_POLLEN_X_IN = {1.4, 4.5, 7.3, 10.3};
        /** VERIFY: y of those centers (one POLLEN radius off the wall). */
        public static final double RED_GARDEN_POLLEN_Y_IN = 1.4;

        /**
         * VERIFY (field report 2.1, +/- 1 in): plan footprint of the UP cell, as the distance OUT from the
         * HIVE pivot line (y = 72) to its inner and outer ends. Red audience UP cell = y [52.3, 69.9].
         */
        public static final double UP_CELL_PLAN_OUT_NEAR_IN = 2.1;
        public static final double UP_CELL_PLAN_OUT_FAR_IN = 19.7;
        /**
         * VERIFY (field report 2.1, +/- 1 in): the same for the DOWN cell, which overhangs the frame
         * (red rear DOWN cell = y [79.4, 96.9]).
         */
        public static final double DOWN_CELL_PLAN_OUT_NEAR_IN = 7.4;
        public static final double DOWN_CELL_PLAN_OUT_FAR_IN = 24.9;

        /**
         * TUNE: an INTAKE step belongs to a FLOWER or GARDEN when its target point is within this
         * distance of the FLOWER center / the GARDEN zone.
         */
        public static final double INTAKE_TARGET_MATCH_RADIUS_IN = 12.0;

        // ---- robot and drive model for the timing estimate (all ADD: the robot is unmeasured) --

        /** ADD: cruise speed along a path. Pedro's provisional maximum in Constants.java is ~72 in/s. */
        public static final double CRUISE_SPEED_IN_PER_SEC = 40.0;
        /** ADD: acceleration and deceleration while following a path. */
        public static final double ACCEL_IN_PER_SEC2 = 60.0;
        /** ADD: how fast the robot turns (a turn overlaps the drive and costs only what is longer). */
        public static final double TURN_RATE_DEG_PER_SEC = 180.0;
        /** ADD: time to settle inside the end tolerance after every stop. */
        public static final double SETTLE_SEC = 0.3;
        /** ADD: default time to scoop POLLEN off the floor (INTAKE step data may override it). */
        public static final double INTAKE_DWELL_FLOOR_SEC = 2.0;
        /** ADD: default time to empty a FLOWER's POLLEN through its retrieval opening. */
        public static final double INTAKE_DWELL_FLOWER_SEC = 4.0;
        /** ADD: default time to sweep up the GARDEN POLLEN line. */
        public static final double INTAKE_DWELL_GARDEN_SEC = 2.5;
        /** ADD: aim with the Limelight and spin the flywheel up before the first shot of a volley. */
        public static final double AIM_SPINUP_SEC = 1.9;
        /** ADD: feeding one POLLEN through the shooter. */
        public static final double TIME_PER_SHOT_SEC = 0.9;

        // ---- validator limits ------------------------------------------------------------------

        /** TUNE: finish this long before the 30 s limit; an estimate inside the margin is a WARN. */
        public static final double TIME_SAFETY_MARGIN_SEC = 2.0;
        /** FIXED: spacing of the samples that the validator checks along every leg. */
        public static final double PATH_SAMPLE_STEP_IN = 1.0;
        /** FIXED: angular spacing of the samples checked while the robot turns in place. */
        public static final double TURN_SAMPLE_STEP_DEG = 5.0;
        /** TUNE (G417 design guard): extra clearance kept around the HIVE frame footprint. */
        public static final double HIVE_KEEPOUT_MARGIN_IN = 3.0;
        /** TUNE: extra clearance kept around a FLOWER plate (the plate itself is in Field.FLOWER_KEEPOUT_*). */
        public static final double FLOWER_KEEPOUT_MARGIN_IN = 0.5;
        /** TUNE (G304.C): a start footprint within this distance of a perimeter wall counts as touching it. */
        public static final double START_WALL_CONTACT_TOLERANCE_IN = 0.75;
        /** TUNE: how far the footprint may poke past the field edge on a step marked wall-contact. */
        public static final double WALL_CONTACT_ALLOWANCE_IN = 1.0;
        /** TUNE: footprint overshoot past a wall that is ignored everywhere (rounding, field tolerance). */
        public static final double FIELD_BOUNDS_TOLERANCE_IN = 0.1;
        /** TUNE (field report 3.3): LEAVE needs the whole footprint at least this far off every wall. */
        public static final double LEAVE_WALL_MARGIN_IN = 1.5;
        /** TUNE (manual Fig 10-7, field tolerance +/- 1 in): a PARK overlapping the zone by less than this is a WARN. */
        public static final double PARK_MIN_OVERLAP_IN = 3.0;
        /** TUNE: an explicit INTAKE/SHOOT/PARK pose farther than this from where the robot is gets a WARN (add a DRIVE). */
        public static final double IMPLICIT_MOVE_WARN_IN = 3.0;
        /** VERIFY (hive_tags report 6d, 70-75 deg launcher): closest launch-point distance to the cell aim point. */
        public static final double SHOOT_STANDOFF_MIN_IN = 30.0;
        /** VERIFY (hive_tags report 6d): farthest launch-point distance (the audience wall limits the near side). */
        public static final double SHOOT_STANDOFF_MAX_IN = 38.0;
        /** TUNE: standoff used for the nominal pose that is drawn when a SHOOT leaves its pose to the solver. */
        public static final double SHOOT_STANDOFF_NOMINAL_IN = 34.0;
        /** TUNE (hive_tags report 6d: +/-13..17 deg in line): largest aim error of the back toward the cell before a WARN. */
        public static final double SHOOT_AIM_TOLERANCE_DEG = 15.0;

        // ---- route diagrams (autoshoot.render; SVG, top view, audience at the bottom) ---------------

        /** TUNE: the SVG is this many pixels wide and tall; the diagrams must stay readable at this size. */
        public static final double RENDER_CANVAS_PX = 900.0;
        /** TUNE: pixels per field inch. 144 in x 4.6 = 662 px leaves room for the title above and the footer below. */
        public static final double RENDER_FIELD_SCALE_PX_PER_IN = 4.6;
        /** TUNE: distance from the top of the SVG to the rear wall of the field. */
        public static final double RENDER_FIELD_TOP_PX = 64.0;
        /** TUNE: an ALLIANCE AREA is really 54 in deep outside the field wall (manual 9.5); only this much of it is drawn (faded out), to leave the field room in a square image. */
        public static final double RENDER_ALLIANCE_AREA_SHOWN_IN = 20.0;
        /** VERIFY (field report D7): the ALLIANCE AREA spans y 23 to 121 along the wall. */
        public static final double RENDER_ALLIANCE_AREA_Y_MIN_IN = 23.0;
        public static final double RENDER_ALLIANCE_AREA_Y_MAX_IN = 121.0;
        /** TUNE: ink of a RED alliance route. */
        public static final String RENDER_INK_RED = "#c62828";
        /** TUNE: ink of a BLUE alliance route. */
        public static final String RENDER_INK_BLUE = "#1565c0";
        /** TUNE: colour of the FRONT (intake) end of a robot footprint. */
        public static final String RENDER_FRONT_COLOR = "#2e7d32";
        /** TUNE: colour of the BACK (shooter) end of a robot footprint. */
        public static final String RENDER_BACK_COLOR = "#ef6c00";
        /** TUNE: light and dark floor tile fills (they alternate like the real checkerboard of tiles). */
        public static final String RENDER_TILE_LIGHT = "#f3f1ea";
        public static final String RENDER_TILE_DARK = "#e0ddd2";
        /** TUNE: page background and text colour of the diagram. */
        public static final String RENDER_BACKGROUND = "#ffffff";
        public static final String RENDER_TEXT_COLOR = "#222222";
        /** TUNE: base font size of labels, in SVG pixels (smaller items scale from it). */
        public static final double RENDER_FONT_PX = 11.0;
        /** TUNE: average glyph width as a fraction of the font size; only used to keep labels from overlapping, not to measure text. */
        public static final double RENDER_LABEL_CHAR_WIDTH_EM = 0.58;
        /** TUNE: radius of the numbered circle that marks each step, in SVG pixels. */
        public static final double RENDER_BADGE_RADIUS_PX = 8.5;
        /** TUNE: radius, in field inches, of the dashed ring drawn around an INTAKE target of each source. */
        public static final double RENDER_INTAKE_RING_FLOOR_IN = 5.0;
        public static final double RENDER_INTAKE_RING_FLOWER_IN = 7.5;
        public static final double RENDER_INTAKE_RING_GARDEN_IN = 7.0;
        /** TUNE: validator findings that fit under a diagram; with more, the last line says how many are left out. */
        public static final int RENDER_MAX_ISSUE_LINES = 5;
        /** TUNE: gap between the bottom of the field (below its tile letters and the AUDIENCE label) and the legend, in SVG pixels. */
        public static final double RENDER_LEGEND_GAP_PX = 56.0;
    }

    // =========================================================================================
    // ROBOT GEOMETRY (the goBILDA StarterBot converted to mecanum)
    // =========================================================================================
    public static final class Robot {
        private Robot() {
        }

        /** FIXED (goBILDA mecanum StarterBot drawing 3200-2627-0004): overall length, front to back, 452 mm. */
        public static final double LENGTH_IN = 17.80;
        /** FIXED (same drawing): overall width, side to side, 452 mm. The robot is square, so it fits the 18 in start cube (rule R102) with 0.2 in to spare. */
        public static final double WIDTH_IN = 17.80;
        /** FIXED (same drawing): height of the tallest part, 308 mm. */
        public static final double HEIGHT_IN = 12.13;
        /** FIXED (same drawing): mecanum wheel diameter, 104 mm. */
        public static final double WHEEL_DIAMETER_IN = 4.09;

        /**
         * ADD: where the ball leaves the shooter, measured from the robot's rotation center. The
         * shooter fires out the BACK, so forward is NEGATIVE. Placeholder = the rear face.
         */
        public static final double LAUNCH_POINT_FORWARD_IN = -8.90;
        /** ADD: sideways offset of the exit from the center line (positive = robot's left). */
        public static final double LAUNCH_POINT_LEFT_IN = 0.0;

        /**
         * VERIFY: height of the ball exit above the floor, used only for the diagnostic ballistics
         * print-out. 7.89 in is the 200.5 mm launch-deck dimension on the goBILDA side-view drawing;
         * measure the real exit.
         */
        public static final double LAUNCH_HEIGHT_IN = 7.89;
        /**
         * VERIFY: the fixed launch angle above horizontal, about 68.5 degrees (team measurement, 2026-11-13). The manual's
         * geometry needs roughly 67 degrees or more for a good entry into the opening from 38 in. Used only for the
         * physics check on the Shooter Tuner screen.
         */
        public static final double LAUNCH_ANGLE_DEG = 68.5;
    }

    // =========================================================================================
    // SHOOTER (single fixed-angle flywheel + a feeder servo that pushes POLLEN into it)
    // =========================================================================================
    public static final class Shooter {
        private Shooter() {
        }

        /** How the number of POLLEN fired is decided. */
        public enum ShotCountMode {
            /** Feed for a fixed time per ball, capped at {@link #MAX_SHOTS_PER_VOLLEY}. Never faults. */
            TIMED,
            /** Count the speed dip each ball causes. Stops at the cap; faults if no dip is ever seen. */
            DIP,
            /** Count dips AND keep a timeout: stops at the cap or when the timeout expires. Never faults. */
            HYBRID
        }

        /** How the feeder follows the flywheel. */
        public enum FeedMode {
            /** Once the first ball is released, keep the feeder running until the volley ends. */
            CONTINUOUS,
            /** Feeder runs only while the flywheel is inside its speed window, so each ball is fed at full speed. */
            GATED_PER_BALL
        }

        /** TUNE: flywheel speed for the manual launch button and the existing autonomous routines, in ticks per second. */
        public static final int MANUAL_TARGET_TPS = 1200;
        /** TUNE: the manual launch feeds only once the flywheel is faster than this, in ticks per second. */
        public static final int MANUAL_MIN_TPS = 950;

        /** FIXED (rule G407): a robot may control at most 4 POLLEN, so a volley is at most 4 shots. */
        public static final int MAX_SHOTS_PER_VOLLEY = 4;

        /**
         * ADD: flywheel speed in ticks per second for each horizontal distance from the BALL EXIT to
         * the cell aim point, in inches, sorted by distance. Each row is {distance_in, velocity_tps}.
         *
         * <p>ONE MEASURED ANCHOR (team, 2026-11-13): with the robot's FRONT pressed against the audience
         * wall and in line with the HIVE, the ball exit is 37.7 in from the aim point and the POLLEN
         * passes through the center to slightly ABOVE the center of the opening, at about the manual
         * launch speed. That is the row {37.7, 1200}. "Slightly high" means the speed that hits dead
         * center is probably a little lower. The other rows are PLACEHOLDERS that follow a typical
         * speed-vs-range slope; record the real speed that scores at 3-5 distances with the shooter
         * tuner and replace them.
         */
        public static final double[][] SHOT_TABLE = {
                {30.0, 1150.0},
                {37.7, 1200.0},
                {44.0, 1250.0}};

        /** TUNE: Shooter Tuner step sizes: flywheel speed in ticks per second (normal and with the left bumper held)... */
        public static final double TUNER_SPEED_STEP_TPS = 10.0;
        public static final double TUNER_SPEED_BIG_STEP_TPS = 50.0;
        /** TUNE: ... and the recorded range in inches. */
        public static final double TUNER_RANGE_STEP_IN = 0.5;
        public static final double TUNER_RANGE_BIG_STEP_IN = 2.0;
        /** TUNE: the range the Shooter Tuner starts at: the measured wall shot. */
        public static final double TUNER_START_RANGE_IN = 37.7;

        /** TUNE: slack on each end of the table when choosing a shooting range, so the planner aims inside it. */
        public static final double RANGE_MARGIN_IN = 1.5;

        /** VERIFY: apply the PIDF below at init. These are goBILDA's StarterBot values for a 6000 rpm motor (max about 2800 tps). */
        public static final boolean FLYWHEEL_APPLY_PIDF = true;
        public static final double FLYWHEEL_PIDF_P = 40.0;
        public static final double FLYWHEEL_PIDF_I = 0.0;
        public static final double FLYWHEEL_PIDF_D = 0.0;
        public static final double FLYWHEEL_PIDF_F = 12.5;
        /** VERIFY: true = brake when the flywheel is told to stop. False coasts, which is easier on the gearbox. */
        public static final boolean FLYWHEEL_BRAKE_ON_STOP = false;
        /** TUNE: speed to hold while idle between volleys, in ticks per second. 0 = stop completely. */
        public static final double FLYWHEEL_IDLE_TPS = 0.0;

        /** TUNE: the flywheel counts as "at speed" this far BELOW its target (fraction of target). */
        public static final double AT_SPEED_BELOW_FRAC = 0.06;
        /** TUNE: ... and this far ABOVE its target. */
        public static final double AT_SPEED_ABOVE_FRAC = 0.06;
        /** TUNE: absolute slack in ticks per second added to both ends (covers encoder quantization noise). */
        public static final double AT_SPEED_FLOOR_TPS = 20.0;
        /** TUNE: the speed must stay inside the window this long before the shooter is declared ready. */
        public static final double AT_SPEED_RISE_SEC = 0.10;
        /** TUNE: ... and must stay outside it this long before it is declared not ready (keep short). */
        public static final double AT_SPEED_FALL_SEC = 0.03;

        /** VERIFY: feeder servo power while pushing a ball (the team's code uses 0.3; the stock StarterBot uses 1.0). */
        public static final double FEEDER_FORWARD_POWER = 0.3;
        /** VERIFY: feeder power when clearing a jam (runs the other way). */
        public static final double FEEDER_REVERSE_POWER = 0.3;
        /** VERIFY: set true if "forward" feeds the ball the wrong way. */
        public static final boolean FEEDER_INVERTED = false;

        public static final ShotCountMode SHOT_COUNT_MODE = ShotCountMode.HYBRID;
        public static final FeedMode FEED_MODE = FeedMode.GATED_PER_BALL;

        /** TUNE: a ball counts as fired when the speed drops this far below target (fraction of target). Measure with the shooter tuner. */
        public static final double SHOT_DIP_FRAC = 0.05;
        /** TUNE: after a counted dip, no second shot is counted for this long. Keep below the time between balls. */
        public static final double SHOT_REFRACTORY_SEC = 0.15;
        /** TUNE: seconds the feeder may run per ball in TIMED/HYBRID mode. */
        public static final double FEED_SECONDS_PER_SHOT = 0.9;
        /** TUNE: absolute limit for one whole volley in seconds, whatever the count says. */
        public static final double VOLLEY_MAX_SEC = 5.0;
        /** TUNE: if no new ball has been seen for this long while feeding, the hopper is empty: finish the volley. */
        public static final double EMPTY_AFTER_LAST_SHOT_SEC = 1.0;
    }

    // =========================================================================================
    // AUTO-SHOOT: choosing where to stand (the "closest feasible scoring pose")
    // =========================================================================================
    public static final class AutoShoot {
        private AutoShoot() {
        }

        /** TUNE: extra clearance kept between the robot and the HIVE frame / FLOWERS, beyond the robot's own size. */
        public static final double KEEPOUT_MARGIN_IN = 1.5;
        /** TUNE: how close to a perimeter wall the edge of the robot may be placed (0 = touching). */
        public static final double WALL_CLEARANCE_IN = 0.25;

        /**
         * VERIFY: the shooter may stand anywhere inside a cone of this half-angle, measured from the
         * direction the cell opening faces. The opening is 20 in wide, so shots from the side see a
         * narrower window (17.2 in in line, 11.6 in at 25 degrees); 20 degrees is a conservative starting limit.
         */
        public static final double SHOOT_CONE_HALF_ANGLE_DEG = 20.0;
        /** TUNE: resolution of the search over the cone. */
        public static final double SHOOT_CONE_STEP_DEG = 2.5;
        /** TUNE: resolution of the search over range. */
        public static final double SHOOT_RANGE_STEP_IN = 1.0;
        /** TUNE: the range the solver prefers when several poses are equally close. 37.7 in is the one measured good shot: robot front against the wall, in line with the HIVE. */
        public static final double PREFERRED_RANGE_IN = 37.7;

        /** TUNE: planning cost of turning the robot, in inches of travel per degree. */
        public static final double PLAN_ROTATION_COST_IN_PER_DEG = 0.15;
        /** TUNE: planning cost of being away from the preferred range, in inches of travel per inch. */
        public static final double PLAN_RANGE_COST_IN_PER_IN = 0.40;
        /** TUNE: planning cost of standing off the opening's centre line, in inches of travel per degree. */
        public static final double PLAN_AZIMUTH_COST_IN_PER_DEG = 0.20;

        // ---- driving to the scoring pose -------------------------------------------------
        /** TUNE: the drive hands over to fine alignment when the robot is this close to the scoring pose (inches). */
        public static final double APPROACH_HANDOFF_IN = 6.0;
        /** TUNE: ... and its heading is within this of the scoring heading (degrees). */
        public static final double APPROACH_HANDOFF_DEG = 25.0;
        /** TUNE: ... and it is moving slower than this (inches per second). */
        public static final double APPROACH_HANDOFF_SPEED_IN_S = 14.0;
        /** TUNE: the approach gives up after distance / this speed plus a fixed allowance (inches per second). */
        public static final double APPROACH_TIMEOUT_SPEED_IN_S = 25.0;
        /** TUNE: fixed allowance added to the approach timeout (seconds). */
        public static final double APPROACH_TIMEOUT_EXTRA_SEC = 2.0;
        /** TUNE: slower than this for the stall time while still far from the pose means the robot is stuck (inches per second). */
        public static final double APPROACH_STALL_SPEED_IN_S = 1.6;
        /** TUNE: how long the robot must be stalled before the approach is abandoned (seconds). */
        public static final double APPROACH_STALL_SEC = 0.7;
        /** TUNE: stall detection is ignored for this long after the approach starts, while the robot accelerates (seconds). */
        public static final double APPROACH_STALL_GRACE_SEC = 0.8;
        /** TUNE: stall detection only applies while farther than this from the pose (inches). */
        public static final double APPROACH_STALL_MIN_DIST_IN = 4.0;

        // ---- fine alignment ---------------------------------------------------------------
        /** TUNE: alignment is settled when the position error is below this (inches). Mecanum drives rarely settle below about 1 in. */
        public static final double ALIGN_POS_TOL_IN = 1.5;
        /** TUNE: ... the heading error is below this (degrees). */
        public static final double ALIGN_HEADING_TOL_DEG = 2.0;
        /** TUNE: ... the speed is below this (inches per second). */
        public static final double ALIGN_SPEED_TOL_IN_S = 1.5;
        /** TUNE: ... the turn rate is below this (degrees per second). */
        public static final double ALIGN_OMEGA_TOL_DEG_S = 8.0;
        /** TUNE: ... and all of that has held for this long (seconds). */
        public static final double ALIGN_SETTLE_SEC = 0.20;
        /** TUNE: alignment gives up after this long (seconds); it still shoots if the relaxed shot tolerances are met. */
        public static final double ALIGN_TIMEOUT_SEC = 3.0;
        /** TUNE: if pushed this far from the scoring pose after alignment, drive back to it (inches). */
        public static final double REAPPROACH_IN = 8.0;

        /** TUNE: let the camera nudge the hold pose (heading toward the aim point, range toward the planned range). */
        public static final boolean VISION_TRIM_ENABLED = true;
        /** TUNE: fraction of the camera's heading error applied per update. */
        public static final double VISION_HEADING_GAIN = 0.6;
        /** TUNE: largest heading change per update (degrees). */
        public static final double VISION_HEADING_STEP_DEG = 3.0;
        /** TUNE: largest total heading trim (degrees). */
        public static final double VISION_HEADING_TRIM_MAX_DEG = 8.0;
        /** TUNE: fraction of the camera's range error applied per update. */
        public static final double VISION_RANGE_GAIN = 0.5;
        /** TUNE: largest range change per update (inches). */
        public static final double VISION_RANGE_STEP_IN = 1.5;
        /** TUNE: largest total range trim (inches). */
        public static final double VISION_RANGE_TRIM_MAX_IN = 4.0;
        /** TUNE: minimum seconds between two camera trims. */
        public static final double VISION_TRIM_PERIOD_SEC = 0.10;
        /** TUNE: the camera only trims once the robot is within this many alignment tolerances of the hold position. */
        public static final double VISION_TRIM_ENTER_POS_FACTOR = 2.0;
        /** TUNE: ... and within this heading error of the hold heading (degrees). */
        public static final double VISION_TRIM_ENTER_HEADING_DEG = 6.0;
        /** TUNE: the logic clock never advances more than this per loop, so one long loop cannot trip a timer (seconds). */
        public static final double MAX_LOOP_DT_SEC = 0.2;

        // ---- the shot gate -----------------------------------------------------------------
        /** TUNE: the robot must be slower than this to feed a ball (inches per second). */
        public static final double SHOT_MAX_SPEED_IN_S = 3.0;
        /** TUNE: ... and turning slower than this (degrees per second). */
        public static final double SHOT_MAX_OMEGA_DEG_S = 15.0;
        /** TUNE: ... and the shooter must point at the aim point within this (degrees). */
        public static final double SHOT_HEADING_TOL_DEG = 4.0;
        /** TUNE: if alignment times out, shooting is still allowed when the robot is within this of the scoring pose (inches). */
        public static final double SHOT_RELAXED_POS_TOL_IN = 3.0;
        /** TUNE: seconds the flywheel may take to reach speed once the robot is in place before the volley is abandoned. */
        public static final double SPINUP_TIMEOUT_SEC = 3.5;

        /** TUNE: a feeder paused for this long because the up cell's tags vanished (it may be tipping) (seconds). */
        public static final double VISION_LOSS_PAUSE_SEC = 0.15;
        /** VERIFY: refuse to feed until the camera (or the driver) has confirmed which cell is up, when a camera is working. */
        public static final boolean REQUIRE_HIVE_CONFIRMATION = true;
        /** TUNE: if standing at the scoring pose this long without ever seeing the HIVE's tags, assume the other cell is up (seconds). */
        public static final double HIVE_UNSEEN_FLIP_SEC = 1.0;
        /** TUNE: how many times one button press may flip to the other side because the tags were not seen. */
        public static final int MAX_UNSEEN_FLIPS = 1;

        // ---- driver ------------------------------------------------------------------------
        /** TUNE: moving the translation stick beyond this (0-1) cancels the automatic drive. */
        public static final double DRIVER_OVERRIDE_STICK = 0.6;
        /** TUNE: ... if held beyond it for this long (seconds). */
        public static final double DRIVER_OVERRIDE_SEC = 0.15;
    }

    // =========================================================================================
    // VISION (Limelight 3A, AprilTag pipeline, mounted on top facing the REAR with the shooter)
    // =========================================================================================
    public static final class Vision {
        private Vision() {
        }

        /** Which axes the Limelight's camera-space poses use. The Limelight docs contradict themselves; the diagnostic OpMode shows which is right. */
        public enum CameraSpaceConvention {
            /** x right, y DOWN, z forward out of the lens (the 3D coordinate-systems page, and OpenCV). */
            OPENCV_RIGHT_DOWN_FORWARD,
            /** x forward, y LEFT, z up in every space (the LimelightLib page). */
            NWU_FORWARD_LEFT_UP
        }

        /** VERIFY: how the Limelight reports camera-space positions. Flip if the diagnostic OpMode shows left/right or up/down swapped. */
        public static final CameraSpaceConvention CAMERA_SPACE_CONVENTION = CameraSpaceConvention.OPENCV_RIGHT_DOWN_FORWARD;
        /** VERIFY: true if a positive tx means the tag is to the RIGHT of the camera's crosshair (as the docs say). */
        public static final boolean TX_POSITIVE_IS_RIGHT = true;
        /** VERIFY: true if a positive ty means the tag is ABOVE the crosshair (the docs disagree with themselves). */
        public static final boolean TY_POSITIVE_IS_UP = true;

        /** ADD: where the lens is, measured from the robot's rotation center. It sits on top and behind center, so forward is negative. */
        public static final double CAMERA_FORWARD_IN = -6.0;
        /** ADD: sideways offset of the lens from the center line (positive = robot's left). */
        public static final double CAMERA_LEFT_IN = 0.0;
        /** ADD: height of the lens above the floor. The robot is 12.13 in tall; the camera sits on top. */
        public static final double CAMERA_HEIGHT_IN = 13.0;
        /** FIXED by the mounting: the camera looks out the back, 180 degrees from the robot's forward direction. */
        public static final double CAMERA_YAW_DEG = 180.0;
        /**
         * ADD: how far the camera is tilted UP from level. The tag rows are 35-50 in up and the verified
         * geometry says the camera needs roughly +30 to +40 degrees to keep them inside its 42 degree
         * vertical field of view.
         */
        public static final double CAMERA_PITCH_UP_DEG = 35.0;

        /** FIXED by the setup checklist (docs/LIMELIGHT_SETUP.md): the AprilTag pipeline is index 0. */
        public static final int PIPELINE_INDEX = 0;
        /** FIXED by the setup checklist: hardware-configuration name of the Limelight. */
        public static final String DEVICE_NAME = "limelight";
        /** TUNE: how often the Control Hub polls the Limelight (1-250 Hz; set before start). */
        public static final int POLL_RATE_HZ = 50;
        /** VERIFY: the tag family string the Limelight reports for 36h11 tags. */
        public static final String EXPECTED_TAG_FAMILY = "36H11C";

        /** TUNE: a result older than this is ignored, in milliseconds. */
        public static final double MAX_FRAME_AGE_MS = 250.0;
        /** TUNE: if the camera link is up but no NEW frame has arrived for this long, vision is treated as stalled, in milliseconds. */
        public static final double FRAME_STALL_MS = 400.0;
        /** TUNE: tags closer than this to the lens are ignored (inches). */
        public static final double TAG_MIN_RANGE_IN = 8.0;
        /** TUNE: tags farther than this are ignored; 3.25 in tags are hard to read beyond about 100 in (inches). */
        public static final double TAG_MAX_RANGE_IN = 110.0;
        /** TUNE: a tag whose measured height is this far from an UP or DOWN cell's tag row is rejected (inches). */
        public static final double TAG_HEIGHT_TOLERANCE_IN = 7.0;
        /** TUNE: UP/DOWN is judged from the left-to-right order of tag IDs only if the tags are at least this far apart in the image (degrees of tx). */
        public static final double ORDER_MIN_TX_SPAN_DEG = 1.0;

        /** TUNE: the HIVE state belief changes only after this many NEW frames in a row agree. */
        public static final int HIVE_CONFIRM_FRAMES = 3;
        /** TUNE: ... spread over at least this many seconds. */
        public static final double HIVE_CONFIRM_SEC = 0.15;
        /** TUNE: after seeing the up cell's tags steadily for this long, losing all of them means the HIVE may have tipped. */
        public static final double HIVE_STABLE_SEEN_SEC = 0.5;
        /** TUNE: tags missing this long (with the camera link healthy) after a steady sighting counts as a suspected tip. */
        public static final double HIVE_LOST_SEC = 0.6;

        // ---- using the camera to correct odometry drift ---------------------------------
        /**
         * VERIFY: let a good sighting of the HIVE's AprilTags nudge the odometry position back toward the
         * truth. Leave it false until the Limelight Diagnostic shows the implied position agreeing with the
         * real position within about an inch, because a wrong camera mount would otherwise pull the position
         * the wrong way. Only the position is corrected, never the heading.
         */
        public static final boolean POSE_CORRECTION_ENABLED = false;
        /** TUNE: fraction of the disagreement corrected each time (0-1). Small is smooth and safe. */
        public static final double POSE_CORRECTION_GAIN = 0.35;
        /** TUNE: largest single correction, in inches. */
        public static final double POSE_CORRECTION_MAX_STEP_IN = 3.0;
        /** TUNE: a sighting that disagrees with odometry by more than this is assumed to be wrong and ignored, in inches. */
        public static final double POSE_CORRECTION_GATE_IN = 10.0;
        /** TUNE: only correct while the robot is slower than this, in inches per second (the image is a little old). */
        public static final double POSE_CORRECTION_MAX_SPEED_IN_S = 3.0;
        /** TUNE: ... and turning slower than this, in degrees per second. */
        public static final double POSE_CORRECTION_MAX_OMEGA_DEG_S = 20.0;
        /** TUNE: ... and the image is no older than this, in seconds. */
        public static final double POSE_CORRECTION_MAX_FRAME_AGE_SEC = 0.12;
        /** TUNE: at least this many of the cluster's four tags must be seen. */
        public static final int POSE_CORRECTION_MIN_TAGS = 2;
        /** TUNE: at least this long between two corrections, in seconds (writing the position costs an I2C transaction). */
        public static final double POSE_CORRECTION_MIN_INTERVAL_SEC = 0.25;
    }

    // =========================================================================================
    // DRIVER CONTROLS (PlayStation-style gamepad 1) AND THE STARTING POSE
    // =========================================================================================
    public static final class Controls {
        private Controls() {
        }

        /** TUNE: hold this to drive to the scoring pose and shoot every stored POLLEN. */
        public static final Button SHOOT_BUTTON = Button.DPAD_UP;
        /** TUNE: press this if the HIVE tipped and the robot does not know: it flips which cell it believes is up. */
        public static final Button TOGGLE_HIVE_BUTTON = Button.DPAD_DOWN;
        /** TUNE: hold this to tell the robot it is standing exactly on the default start pose (resets odometry). */
        public static final Button RESET_POSE_BUTTON = Button.SHARE;
        /** TUNE: ... for this long, so it cannot be hit by accident (seconds). */
        public static final double RESET_POSE_HOLD_SEC = 0.6;
        /** TUNE: during INIT, press this to choose the RED alliance. */
        public static final Button ALLIANCE_RED_BUTTON = Button.DPAD_LEFT;
        /** TUNE: during INIT, press this to choose the BLUE alliance. */
        public static final Button ALLIANCE_BLUE_BUTTON = Button.DPAD_RIGHT;

        /** TUNE: a trigger counts as pressed beyond this (0-1). */
        public static final double TRIGGER_THRESHOLD = 0.3;
        /** TUNE: stick values smaller than this are treated as zero. */
        public static final double STICK_DEADBAND = 0.05;
        /** VERIFY: +1 or -1. If pushing the left stick forward drives the robot backward, flip this. */
        public static final double MANUAL_FORWARD_SIGN = 1.0;
        /** VERIFY: +1 or -1. If pushing the left stick right strafes left, flip this. */
        public static final double MANUAL_STRAFE_SIGN = 1.0;
        /** VERIFY: +1 or -1. If pushing the right stick right turns the robot counter-clockwise, flip this. */
        public static final double MANUAL_TURN_SIGN = 1.0;
        /** TUNE: manual drive power multipliers (0-1). */
        public static final double MANUAL_DRIVE_SCALE = 1.0;
        public static final double MANUAL_TURN_SCALE = 1.0;

        /**
         * VERIFY: where a RED robot stands at the start of TeleOp if the pose was not carried over from
         * autonomous (RED frame; BLUE is the half-turn of it): the CENTER of tile C1 (60 in from the red
         * wall, 0.75 in from the HIVE axis), robot FRONT pressed against the audience wall, back toward the
         * HIVE. This is also the one measured good shot position (37.7 in from the aim point). The robot must
         * really be placed here for odometry to be right.
         */
        public static final double DEFAULT_START_X_IN = 60.0;
        public static final double DEFAULT_START_Y_IN = 8.9;
        public static final double DEFAULT_START_HEADING_DEG = -90.0;
        /** TUNE: a pose saved by an earlier OpMode is only trusted if it is newer than this (seconds). */
        public static final double POSE_CARRYOVER_MAX_AGE_SEC = 300.0;

        /** TUNE: driver feedback, in milliseconds. */
        public static final int RUMBLE_STARTED_MS = 60;
        public static final int RUMBLE_ARRIVED_MS = 100;
        public static final int RUMBLE_DONE_BLIPS = 2;
        public static final int RUMBLE_FAULT_MS = 500;
        /** TUNE: gamepad light colors for the alliance (red, green, blue, each 0-1). */
        public static final double[] RED_LED = {1.0, 0.0, 0.0};
        public static final double[] BLUE_LED = {0.0, 0.0, 1.0};
    }

    // =========================================================================================
    // FIELD AND HIVE (manual TU09 sections 9.2-9.9; RED frame)
    // =========================================================================================
    public static final class Field {
        private Field() {
        }

        /** FIXED (manual 9.2): the field interior is 144 x 144 in. */
        public static final double SIZE_IN = 144.0;

        /** FIXED (manual 9.6.1): the HIVE frame is 49.46 in wide along x ... */
        public static final double HIVE_FRAME_WIDTH_X_IN = 49.46;
        /** FIXED (manual 9.6.1): ... and 38.95 in deep along y, centered on the field center. */
        public static final double HIVE_FRAME_DEPTH_Y_IN = 38.95;

        /** FIXED (manual Fig 9-10): the red HIVE axis is 12.75 in left of the field center (HIVEs are 25.5 in apart). */
        public static final double RED_HIVE_AXIS_X_IN = 59.25;
        /** FIXED (manual 9.6.2): a CELL opening is 20 in wide. */
        public static final double CELL_OPENING_WIDTH_IN = 20.0;

        /**
         * FIXED (derived from manual Fig 9-9/9-10 and the FTC SDK AprilTagGameDatabase): distance
         * from the HIVE pivot-axis plane (y = 72) out to the cluster ORIGIN of an UP cell. The cluster
         * origin is the center of the CELL opening, which is the point to aim at.
         */
        public static final double UP_CELL_ORIGIN_OUT_IN = 16.48;
        /** FIXED (same derivation): height of that aim point above the floor for an UP cell. */
        public static final double UP_CELL_ORIGIN_Z_IN = 58.31;
        /** FIXED (same derivation): distance out from the pivot plane to the middle of the AprilTag row of an UP cell. */
        public static final double UP_CELL_TAG_ROW_OUT_IN = 13.07;
        /** FIXED (same derivation): height of the AprilTag row of an UP cell. */
        public static final double UP_CELL_TAG_ROW_Z_IN = 49.85;
        /** FIXED (same derivation): height of the AprilTag row of a DOWN cell (used to tell UP from DOWN by tag height). */
        public static final double DOWN_CELL_TAG_ROW_Z_IN = 35.58;

        /** FIXED (manual 9.9 / SDK): tag centers along the sticker, from its centerline, in tag-ID order. */
        public static final double[] TAG_X_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};
        /** FIXED (manual 9.9): tags are 3.25 in square (82.55 mm). */
        public static final double TAG_SIZE_IN = 3.25;
        /** FIXED (manual 9.9): four tags per cell. */
        public static final int TAGS_PER_CLUSTER = 4;

        /** FIXED (manual 9.9): first tag ID of the red cell on the side opposite the audience ("RED SCORING"). */
        public static final int RED_REAR_BASE_TAG_ID = 30;
        /** FIXED (manual 9.9): first tag ID of the red cell on the audience side ("RED AUDIENCE"). */
        public static final int RED_AUDIENCE_BASE_TAG_ID = 34;
        /** FIXED (manual 9.9): first tag ID of the blue cell on the audience side ("BLUE AUDIENCE"). */
        public static final int BLUE_AUDIENCE_BASE_TAG_ID = 38;
        /** FIXED (manual 9.9): first tag ID of the blue cell opposite the audience ("BLUE SCORING"). */
        public static final int BLUE_REAR_BASE_TAG_ID = 42;

        /** FIXED (manual Fig 10-2): at MATCH start the red HIVE's UP cell is on this side. */
        public static final Side RED_UP_SIDE_AT_MATCH_START = Side.AUDIENCE;
        /** FIXED (manual Fig 10-2): at MATCH start the blue HIVE's UP cell is on this side. */
        public static final Side BLUE_UP_SIDE_AT_MATCH_START = Side.REAR;

        /**
         * FIXED (manual 9.7, Fig 9-17): the four FLOWERS, as {x, y} centers of their wall plates, in
         * the RED frame. They are obstacles for the path planner (a robot may touch one only to
         * retrieve POLLEN).
         */
        public static final double[][] FLOWER_CENTERS_IN = {
                {2.4, 48.0}, {48.0, 141.6}, {141.6, 96.0}, {96.0, 2.4}};
        /** FIXED (manual Fig 9-12): FLOWER plate extent measured along the wall; used as a planning keep-out. */
        public static final double FLOWER_KEEPOUT_ALONG_WALL_IN = 6.0;
        /** FIXED (manual Fig 9-12): FLOWER plate depth out from the wall; used as a planning keep-out. */
        public static final double FLOWER_KEEPOUT_DEPTH_IN = 5.0;
    }
}
