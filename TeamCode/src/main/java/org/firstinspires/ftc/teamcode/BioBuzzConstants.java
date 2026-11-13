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
        /** ADD: fixed launch angle above horizontal (CAD). The manual's geometry wants roughly 65-75 deg. */
        public static final double LAUNCH_ANGLE_DEG = 70.0;
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
         * PLACEHOLDER derived from the team's existing autos (about 1200 tps from 37-45 in); record
         * the real speed that scores at 3-5 distances with the shooter tuner and replace these rows.
         */
        public static final double[][] SHOT_TABLE = {
                {28.0, 1150.0},
                {34.0, 1200.0},
                {40.0, 1250.0},
                {46.0, 1300.0}};

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
        /** TUNE: the range the solver prefers when several poses are equally close (the middle of the measured table). */
        public static final double PREFERRED_RANGE_IN = 34.0;

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
         * autonomous (RED frame; BLUE is the half-turn of it): tile C1, against the audience wall, back
         * toward the HIVE. The robot must really be placed here for odometry to be right.
         */
        public static final double DEFAULT_START_X_IN = 58.0;
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
