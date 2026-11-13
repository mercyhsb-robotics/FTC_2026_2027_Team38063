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

        /** FIXED (CAD drawing, tank version): overall length, front to back. ADD: re-measure the mecanum build. */
        public static final double LENGTH_IN = 17.80;
        /** FIXED (CAD drawing, tank version): overall width, side to side. ADD: re-measure the mecanum build. */
        public static final double WIDTH_IN = 16.77;

        /**
         * ADD: where the ball leaves the shooter, measured from the robot's rotation center. The
         * shooter fires out the BACK, so forward is NEGATIVE. Placeholder = the rear face.
         */
        public static final double LAUNCH_POINT_FORWARD_IN = -8.90;
        /** ADD: sideways offset of the exit from the center line (positive = robot's left). */
        public static final double LAUNCH_POINT_LEFT_IN = 0.0;

        /** ADD: height of the ball exit above the floor (used only for the diagnostic ballistics print-out). */
        public static final double LAUNCH_HEIGHT_IN = 11.0;
        /** ADD: fixed launch angle above horizontal (CAD). The manual's geometry wants roughly 65-75 deg. */
        public static final double LAUNCH_ANGLE_DEG = 70.0;
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
