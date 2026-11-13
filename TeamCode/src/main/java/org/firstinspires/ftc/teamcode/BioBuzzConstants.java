package org.firstinspires.ftc.teamcode;

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
