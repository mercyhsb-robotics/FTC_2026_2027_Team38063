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
 * {@code _FRAC} a 0..1 fraction. Field coordinates are the Pedro frame: origin at the INNER corner where
 * the audience wall meets the red wall (the face of the walls, 141.348 in apart in the official CAD),
 * +x toward blue, +y away from the audience. Everything
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
            "Robot.INTAKE_REACH_IN",
            "Robot.INTAKE_WHEEL_OFFSET_IN",
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
            "Auto.FLOWER_PRESS_DEPTH_IN",
            "Auto.FLOWER_HEADING_TOLERANCE_DEG",
            "Auto.FLOWER_ALIGN_TOLERANCE_IN",
            "Auto.FLOWER_APPROACH_LENGTH_IN",
            "Auto.FLOWER_APPROACH_SPEED_FRACTION",
            "Auto.FLOWER_RETREAT_LENGTH_IN",
            "Auto.FLOWER_CHASSIS_CLEARANCE_IN",
            "Auto.FLOWER_WHEEL_START_OFFSET_IN",
            "Auto.FLOWER_WHEEL_PRESS_OFFSET_IN",
            "Auto.FLOWER_WHEEL_BALL_INBOARD",
            "Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC",
            "Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC",
            "Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN",
            "Auto.FLOWER_WIGGLE_TURN_SEC",
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
         * VERIFY (FIRST's own field documents: the Event Field Setup Guide section 12.3, the Field Acceptance Checklist
         * and the Field Mitigation Guide, NOT the game manual, which gives no number): every HIVE cell is calibrated so
         * that an EMPTY cell tips with the 8th POLLEN (7 do not) and a cell holding the 3 staged NECTAR tips with the
         * 3rd POLLEN (2 do not). To plan with those two facts the code counts every POLLEN as 1 and every NECTAR as
         * a little more than 1 POLLEN, and a cell tips when the total reaches {@link #TIP_POLLEN_EQUIVALENT_THRESHOLD}.
         * The weight is the fraction {@code NECTAR_POLLEN_EQUIVALENT_NUMERATOR / NECTAR_POLLEN_EQUIVALENT_DENOMINATOR}
         * = 5/3 = 1.667 POLLEN per NECTAR, kept as two whole numbers so the arithmetic is exact (a rounded decimal such as
         * 1.66 would make 3 NECTAR + 3 POLLEN come out as 7.98 and "not tip", the opposite of FIRST's table; see
         * TipModel). Any weight from 1.667 up to (not
         * including) 2.0 agrees with the two official facts; 5/3 also says "5 NECTAR tip, 4 NECTAR do not", which is the
         * team's rule of thumb (AndyMark's weights, 0.091 lb and 0.055 lb, give 1.65 the same way). Weigh your own balls.
         */
        public static final int NECTAR_POLLEN_EQUIVALENT_NUMERATOR = 5;
        /** VERIFY: the bottom of the fraction of {@link #NECTAR_POLLEN_EQUIVALENT_NUMERATOR}. */
        public static final int NECTAR_POLLEN_EQUIVALENT_DENOMINATOR = 3;
        /**
         * VERIFY: that fraction as a decimal (5/3 = 1.667 POLLEN per NECTAR), only for sentences and checks that want to
         * print it. The tip arithmetic never uses it: it uses the two whole numbers above.
         */
        public static final double NECTAR_POLLEN_EQUIVALENT =
                (double) NECTAR_POLLEN_EQUIVALENT_NUMERATOR / NECTAR_POLLEN_EQUIVALENT_DENOMINATOR;
        /**
         * VERIFY (FIRST's calibration, see {@link #NECTAR_POLLEN_EQUIVALENT_NUMERATOR}): a cell tips when its POLLEN plus
         * {@link #NECTAR_POLLEN_EQUIVALENT} per NECTAR add up to this many POLLEN: 8 POLLEN tip an empty cell, 7 do not.
         * With the 3 staged NECTAR (5 POLLEN-equivalents) the 3rd POLLEN tips the cell.
         */
        public static final int TIP_POLLEN_EQUIVALENT_THRESHOLD = 8;
        /**
         * TUNE: how many POLLEN more than the minimum a route should plan to put into a cell it wants to tip. A real field
         * drifts: FIRST's own Q&amp;A (QA46) tells of a HIVE that needed 9 POLLEN instead of 8 after 8 to 10 matches. A
         * route that puts in exactly the minimum has no spare and gets a WARN; with a spare of 1 a field that is one POLLEN
         * stiffer still tips. A volley that tips earlier stops there, and the spare stays in the robot (it is not launched at
         * a cell that is already tipping down).
         */
        public static final int TIP_PLANNING_MARGIN_POLLEN = 1;
        /**
         * VERIFY: for a route that assumes the alliance PARTNER shoots the first tip (FirstTip.PARTNER): the route
         * time, in seconds from the start of AUTO, at which that tip is assumed to be DONE, so the other HIVE cell is
         * UP from then on. 12 s is a first guess (a partner that shoots its 4 pre-loads from a nearby pose). The drive
         * team must agree the real time with the partner team before the match; until then a route that depends on
         * it is a plan, not a promise.
         */
        public static final double PARTNER_FIRST_TIP_DONE_SEC = 12.0;
        /**
         * TUNE: the least number of seconds of SLACK a route that waits for the partner's tip should have. Slack is the
         * time between the tip that the route ASSUMES (PARTNER_FIRST_TIP_DONE_SEC) and the latest partner tip for which
         * the route still finishes inside the AUTO limit minus TIME_SAFETY_MARGIN_SEC. Partner teams tip at different
         * times, so a route with little slack only works with a fast partner: the checker WARNs below this value and
         * the gallery shows the real number for every partner route. The robot never shoots because of a TIME: it shoots
         * after its camera has seen the cell UP, and gives the volley up when the wait runs out.
         */
        public static final double PARTNER_TIP_MIN_SLACK_SEC = 3.0;
        /**
         * VERIFY (rule G409): when a HIVE tips, the cell that was UP tips down and everything in it (the staged
         * NECTAR and the POLLEN launched into it) falls out of its opening onto the floor on that side of the HIVE.
         * G409 says a robot may not catch or deflect what a tipped HIVE releases, so for this many seconds after a
         * tip the robot footprint must stay out of that cell's spill zone (see TIP_SPILL_MARGIN_IN). 3 s is a first
         * guess for the balls to land and stop rolling: watch a real tip and replace it. The route checker counts it from
         * the planned tip time plus the settle time (Vision.HIVE_STABLE_SEEN_SEC); the robot itself counts it from the moment
         * its camera has confirmed the tip (TipConfirmation.spillClearAtSec).
         */
        public static final double TIP_SPILL_CLEAR_SEC = 3.0;
        /**
         * VERIFY (rule G409): the spill zone is the plan footprint of the cell that tips (a 21.06 in wide box; for the red
         * audience cell x [47.39, 68.45], y [50.61, 69.05], from the CAD) grown by this much on every side,
         * because the balls bounce and roll a little after they land. The other cell's side of the HIVE is not part of
         * it: a robot standing on the far side of the HIVE is clear. 6 in is a guess: POLLEN are 2.8 in balls.
         */
        public static final double TIP_SPILL_MARGIN_IN = 6.0;
        /** FIXED (manual Table 10-2): LEAVE is worth 3 points in AUTO. */
        public static final int LEAVE_POINTS = 3;
        /** FIXED (manual Table 10-2): PARK is worth 5 points in AUTO. */
        public static final int PARK_POINTS = 5;
        /** FIXED (manual Table 10-2): a HIVE TIP is worth 20 points in AUTO. */
        public static final int TIP_POINTS = 20;

        // ---- field zones and layout used by the validator and the diagrams (RED frame) --------

        /**
         * FIXED (checklist 3.2-3.4: 11 in +/- 0.125, measured from the wall to the far edge of the tape): the red LOADING ZONE
         * is 11.0 in deep, x [0, 11] ... The CAD models the tape 0.57 in off the wall (its outer edge is at 11.574), so the
         * physical zone is probably x 0 to 11.0; the checklist figure is the PLANNING value (a robot that overlaps the zone
         * by the planned amount is safe whichever is true). Both numbers are known; they differ by 0.57 in.
         */
        public static final double RED_LOADING_ZONE_X_MIN_IN = 0.0;
        public static final double RED_LOADING_ZONE_X_MAX_IN = 11.0;
        /**
         * FIXED (CAD, the outer edge of the tape; the zone is 22.69 in long and touches seam lines Y and Z within 0.41 in):
         * ... y [94.584, 117.274]. The blue one is its half-turn image (x [130.348, 141.348], y [24.074, 46.764]).
         */
        public static final double RED_LOADING_ZONE_Y_MIN_IN = 94.584;
        public static final double RED_LOADING_ZONE_Y_MAX_IN = 117.274;

        /**
         * FIXED (CAD, the outer edge of the two 1 in tape strips, which the CAD puts 0.574 in off both walls): the red GARDEN is
         * 22.69 x 2 in in the corner of tile A1 (the manual says about 23 x 2).
         */
        public static final double RED_GARDEN_X_MIN_IN = 0.574;
        public static final double RED_GARDEN_X_MAX_IN = 23.264;
        public static final double RED_GARDEN_Y_MIN_IN = 0.574;
        public static final double RED_GARDEN_Y_MAX_IN = 2.574;
        /**
         * FIXED (CAD, spacing 2.89 in; the first POLLEN touches the red wall): centers of the 4 red GARDEN POLLEN along the audience
         * wall, measured from the red wall.
         */
        public static final double[] RED_GARDEN_POLLEN_X_IN = {1.40, 4.29, 7.18, 10.07};
        /** FIXED (CAD): y of those centers: one POLLEN radius (1.4 in) off the audience wall. */
        public static final double RED_GARDEN_POLLEN_Y_IN = 1.40;

        /**
         * FIXED (CAD, the hull of the whole cell incl. cable ties, +/- 1 in on the real field): plan footprint of the UP cell, as the
         * distance OUT from the HIVE pivot line (y = Field.CENTER_IN) to its inner and outer ends. The red audience UP cell is
         * y [50.61, 69.05], i.e. 1.62 to 20.06 in out from the pivot line (the old report said 2.1 to 19.7).
         */
        public static final double UP_CELL_PLAN_OUT_NEAR_IN = 1.62;
        public static final double UP_CELL_PLAN_OUT_FAR_IN = 20.06;
        /**
         * FIXED (CAD): the same for the DOWN cell, which overhangs the frame (the red rear cell DOWN is y [73.39, 95.80]). The
         * part nearest the pivot is the cell's base tube; the box of the cell itself starts 7.4 in out. The overhang beyond the
         * frame is high up: at 35.5 in or more above the floor, so a 12.13 in tall robot passes under it.
         */
        public static final double DOWN_CELL_PLAN_OUT_NEAR_IN = 2.72;
        public static final double DOWN_CELL_PLAN_OUT_FAR_IN = 25.13;

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
        /**
         * ADD: default time the wheels stay pressed into a FLOWER's bottom POLLEN with the intake running (seconds): the
         * dwell between the slow approach and the retreat of a FLOWER press-in.
         */
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
        /**
         * FIXED: spacing of the COARSE samples that the validator checks along every leg (inches). Between two coarse
         * samples a corner of the robot can move up to {@code PATH_SAMPLE_STEP_IN + 13.22 in * TURN_SAMPLE_STEP_DEG in
         * radians} (2.15 in); the validator grows every obstacle by half of that and cuts a gap in two whenever it
         * cannot prove it is clear, so nothing can slip between two samples (see {@code routes/Sweep.java}).
         */
        public static final double PATH_SAMPLE_STEP_IN = 1.0;
        /**
         * FIXED: angular spacing of the coarse samples while the robot turns, whether in place or along a leg (degrees).
         * Together with {@link #PATH_SAMPLE_STEP_IN} it sets how big the first, cheap clearance test is.
         */
        public static final double TURN_SAMPLE_STEP_DEG = 5.0;
        /**
         * FIXED: how finely the validator measures a collision once it is close to one (inches). A gap between two
         * samples is cut in half until nothing deeper than this can hide in it, so every reported depth is right to
         * within this much. It is numeric precision, not a safety margin.
         */
        public static final double SWEEP_RESOLUTION_IN = 0.001;
        /**
         * TUNE (G417 / G409 design guard): extra clearance, in inches, kept on EVERY side of the HIVE frame footprint.
         * The validator's keep-out is the whole frame footprint (x 45.94 to 95.41, y 51.20 to 90.15) grown by this much.
         * (A route step that says {@code allowUnderHive} is checked against the exact solid instead, see {@link #HIVE_LANE_MARGIN_IN}.)
         *
         * <p><b>What the 12.13 in tall robot can actually hit</b> (the official CAD, measured and checked a second time):
         * <ul>
         *   <li>The only things of the HIVE below the robot's height are in the FRAME: two foot bars on the floor along y (1.98 in wide,
         *       2.15 in tall, x 45.94 to 47.92 and 93.42 to 95.41) and the four legs that lean inward from their ends. A leg is only 12.13
         *       in high where it has moved about 3.2 in in along x and 5.2 in along y, so below the robot's height the legs are two short
         *       stubs at each end of each bar, reaching in to x 50.61 (and 90.73). Nothing else of the HIVE is lower than 30.65 in.</li>
         *   <li>The crossbar is 40.95 in up and the lowest point of any CELL is 30.652 in up (an UP cell's is 43.96 in). The tallest part
         *       of the robot is 12.13 in (13.0 in with the camera), so no CELL, and no cell that is swinging up or down, can touch it.</li>
         *   <li>A DOWN cell hangs out past the frame footprint by up to 5.66 in (red rear cell: y up to 95.80, the frame ends at 90.15),
         *       but only at 35.5 in up or higher, so a robot passes UNDER that overhang. That is why the overhang is not part of the
         *       keep-out.</li>
         *   <li>Nothing else on the field is lower than the robot except the FLOWER plates (5.033 in from the wall, handled separately)
         *       and the loose POLLEN.</li>
         * </ul>
         *
         * <p><b>The lane.</b> Between the feet the plan is open: 45.5 in wide between the bars and 40.1 in wide where the leg stubs narrow
         * it (x 50.6 to 90.7), so a 17.80 in wide robot has about 11 in to spare on each side. A robot that is 12 to 13 in tall can
         * therefore drive north-south UNDER the HIVE. That saves a trip of about 20 in around it (see docs/AUTOS.md), but it is a design
         * choice, not a default: G409 says a robot may not catch what a tipped HIVE releases and G417 forbids launching into a cell that is
         * tipping, and a robot under a tipping HIVE is exactly where nobody wants to be. So the default keep-out below treats the WHOLE
         * footprint as solid, a route may use the lane only with its own flag ({@code allowUnderHive}), and the checker WARNs for a route that
         * is under the HIVE while the HIVE could still tip, i.e. before the camera has confirmed it at rest.
         *
         * <p><b>Why 3.0 in.</b> The frame measured in the CAD reaches at most 0.03 in beyond the foot bars at floor level (bolt heads, caps),
         * and a real field is only good to about +/- 1 in; 3.0 in covers both with about 2 in left for odometry drift and the robot's own
         * stopping error. The test HiveKeepOutTest checks that this is a superset of the footprint and of that measured envelope, edge by edge
         * and corner by corner.
         */
        public static final double HIVE_KEEPOUT_MARGIN_IN = 3.0;
        /**
         * TUNE: the height, in inches, below which a part of the HIVE counts as an obstacle for the robot: the tallest point of the robot, with
         * its camera (12.13 in tall, 13.0 in with the Limelight on top). The exact HIVE solid is everything of the HIVE lower than this: the foot
         * bars and the stubs of the legs, which are a little longer for a taller robot (the legs lean inward as they climb). Must stay below
         * {@code Field.HIVE_LOWEST_CELL_POINT_IN}: the solid has no cells in it.
         */
        public static final double HIVE_SOLID_HEIGHT_IN = 13.0;
        /**
         * TUNE (G409 / G417 design guard for a step that is allowed under the HIVE): the clearance, in inches, the robot keeps on BOTH sides to
         * the foot bars and the leg stubs while it drives along the lane between them. The lane is 40.1 in wide where the stubs narrow it, the
         * robot 17.80 in, so 3.0 in on each side leaves about 8 in of freedom to steer. It is the same size as the margin around the whole
         * footprint on purpose: a robot is never closer to the HIVE under it than beside it.
         */
        public static final double HIVE_LANE_MARGIN_IN = 3.0;
        /**
         * TUNE: extra clearance kept around a FLOWER plate (the plate itself is {@code Field.FLOWER_PLATE_*}) by every part
         * of the robot, except the one flat wheel that is pressing into that FLOWER's POLLEN.
         */
        public static final double FLOWER_KEEPOUT_MARGIN_IN = 0.5;

        // ---- FLOWER press-in (autoshoot.routes.FlowerPress) -------------------------------------------------------
        // To take the bottom POLLEN out of a FLOWER the flat intake wheels (they spin parallel to the floor at the front
        // corners) must PRESS INTO the POLLEN and roll it up and over the lip of the lower ring (0.35 in in the CAD, about 0.43 in
        // in the manual: the harder number is the safe one for a ball that must be lifted). The two wheels
        // are 12.2 in apart, too far to pinch one 2.8 in POLLEN, so ONE wheel does the pressing while the robot stands
        // to one side of the FLOWER's center line (Robot.INTAKE_WHEEL_OFFSET_IN). The robot faces the wall, drives
        // straight in until that wheel overlaps the POLLEN, runs the intake while it stays pressed, then backs straight
        // out. The numbers below describe that move. All distances are measured from the field wall the FLOWER stands on.

        /**
         * ADD: how far (inches) the pressing wheel pushes into the bottom POLLEN, past the POLLEN's field-side edge (which
         * is 2.634 + 1.4 = 4.034 in from the wall). The front tip of the wheel stops at 4.034 - this = 2.833 in from the wall and
         * never closer than the POLLEN's center line (2.634 in). 1.201 in is what the CAD gives when the chassis stands its
         * clearance off the plate and the wheels reach {@code Robot.INTAKE_REACH_IN}: 5.033 + 0.5 - 2.7 = 2.833 (the old 144 in
         * numbers gave 1.25). Measure how far the wheel must overlap the ball to roll it over the lip without jamming it
         * against the closed wall side.
         */
        public static final double FLOWER_PRESS_DEPTH_IN = 1.201;
        /**
         * ADD: how far (degrees) the robot's heading may be from "facing the wall squarely" when it lines up for a
         * FLOWER. A crooked robot presses the wheel in at an angle and lets the chassis corner swing toward the plate.
         */
        public static final double FLOWER_HEADING_TOLERANCE_DEG = 5.0;
        /**
         * ADD: how far (inches) along the wall the MIDDLE OF THE PRESSING WHEEL may be from the FLOWER's center line when
         * the robot lines up. A wheel that is off to the side presses the edge of the POLLEN instead of its middle.
         */
        public static final double FLOWER_ALIGN_TOLERANCE_IN = 1.0;
        /**
         * ADD: the straight final approach (inches) from the line-up pose to the press pose must be at least this long, so
         * the robot is already straight and slow when the wheel touches the POLLEN.
         */
        public static final double FLOWER_APPROACH_LENGTH_IN = 6.0;
        /** ADD: fraction (0-1) of the cruise speed used for the final approach and for the retreat. */
        public static final double FLOWER_APPROACH_SPEED_FRACTION = 0.3;
        /** ADD: how far (inches) the robot backs straight out after the dwell. Shorter than this is a WARN. */
        public static final double FLOWER_RETREAT_LENGTH_IN = 6.0;
        /**
         * ADD: the gap (inches) the CHASSIS (the robot without its two flat wheels) keeps from the FLOWER plate while the
         * wheel is pressed in. The plate's field-side edge is 5.033 in from the wall, so the chassis front stands at least
         * 5.033 + this from the wall and the wheel must reach the rest of the way: see {@code Robot.INTAKE_REACH_IN}. Measure the standoff
         * between the wall and the robot's chassis front when the wheel is properly pressed into the POLLEN.
         */
        public static final double FLOWER_CHASSIS_CLEARANCE_IN = 0.5;
        // ---- FLOWER wiggle numbers (autoshoot.routes.FlowerWiggle): how ONE flat wheel rolls the bottom POLLEN out of a FLOWER with a small turn
        // on the spot, toward the ball and back, once per POLLEN. The turn angle and the standoff are derived from these, never typed in.

        /**
         * ADD: how far (inches) the middle of the pressing wheel stands from the middle of the bottom POLLEN, measured along
         * the wall, when the straight approach ends and the wiggle begins. 3.8 in = the wheel's radius (1.4 in) plus the POLLEN's
         * radius (1.4 in) plus 1.0 in of air, so the wheel passes the ball without touching it. Measure it on the real robot:
         * close enough that a small turn reaches the ball, far enough that the wheel never drags it on the way in.
         */
        public static final double FLOWER_WHEEL_START_OFFSET_IN = 3.8;
        /**
         * ADD: how close (inches) the middle of the pressing wheel comes to the middle of the POLLEN at the end of the push.
         * 1.8 in = 1.4 + 1.4 - 1.0: the wheel overlaps the ball by 1.0 in. The turn angle is chosen so that the wheel just reaches this
         * distance, and never gets closer (a harder push would jam the ball against the closed side of the FLOWER). Measure how far the
         * wheel has to overlap the ball to roll it out over the lip.
         */
        public static final double FLOWER_WHEEL_PRESS_OFFSET_IN = 1.8;
        /**
         * ADD: true if the POLLEN is on the INBOARD side of the pressing wheel at the start, that is BETWEEN the wheel and the robot's
         * center line (the wheel is the outer one); false if the ball is on the OUTBOARD side of it (the wheel is the inner one).
         * Inboard is believed right: turning toward the ball then also swings the wheel forward, toward the wall, so it reaches the
         * ball. Confirm it on the real robot with BOTH wheels and write down any difference.
         */
        public static final boolean FLOWER_WHEEL_BALL_INBOARD = true;
        /**
         * TUNE: the biggest angle (degrees) the push may turn the robot. The turn is derived from the geometry; if the wheel would
         * have to turn farther than this to reach the ball, the pick-up does not work and the validator reports it.
         * A big turn swings the robot's corners toward the plate, so keep it small.
         */
        public static final double FLOWER_WIGGLE_MAX_TURN_DEG = 20.0;
        /**
         * ADD: how long (seconds) the robot stays turned in, with the wheel pressed on the ball and spinning, before it turns back.
         */
        public static final double FLOWER_WIGGLE_PUSH_HOLD_SEC = 0.25;
        /**
         * ADD: how long (seconds) the robot stays turned back, the wheel away from the ball, so the next POLLEN can drop down the chute
         * into the ring hole before the next push.
         */
        public static final double FLOWER_WIGGLE_RELEASE_HOLD_SEC = 0.4;
        /**
         * ADD: how many push-and-release cycles it takes to get ONE POLLEN out (1 is the guess). A FLOWER pick-up of 4 POLLEN makes
         * 4 times this many cycles.
         */
        public static final int FLOWER_WIGGLE_CYCLES_PER_POLLEN = 1;
        /** TUNE: never plan more than this many cycles for one pick-up, so a bad number cannot make a pick-up swallow the match. */
        public static final int FLOWER_WIGGLE_MAX_CYCLES = 6;
        /**
         * ADD: how long (seconds) ONE half of a cycle's turn takes: turning in to the ball, or turning back out. (The estimate never
         * uses less than the time the drive needs to turn that far at {@link #TURN_RATE_DEG_PER_SEC}.)
         */
        public static final double FLOWER_WIGGLE_TURN_SEC = 0.35;
        /**
         * TUNE: the robot counts a turn of the wiggle as finished when its heading is within this many degrees of the wanted heading
         * (or when the turn time below has passed, whichever comes first).
         */
        public static final double FLOWER_WIGGLE_HEADING_TOLERANCE_DEG = 2.0;
        /**
         * TUNE: the longest a single turn of the wiggle may take (seconds). If the heading has not got there by then, the robot gives up
         * this pick-up: it turns back, backs out and goes on with the rest of the route (fail closed: it never keeps pushing).
         */
        public static final double FLOWER_WIGGLE_TURN_TIMEOUT_SEC = 1.5;
        /** TUNE (G304.C): a start footprint within this distance of a perimeter wall counts as touching it. */
        public static final double START_WALL_CONTACT_TOLERANCE_IN = 0.75;
        /**
         * TUNE: how far (inches) any part of the robot may overlap a field wall before the validator reports it.
         * Touching a wall is legal; pushing into it is not. This tiny value only absorbs rounding in the numbers a
         * route is written with, so it is NOT permission to clip a wall. It is the same for every step: the start,
         * every DRIVE leg (marked wall-contact or not), every INTAKE, the PARK pose and the end pose.
         */
        public static final double FIELD_BOUNDS_TOLERANCE_IN = 0.1;
        /**
         * TUNE (FIRST's Q&amp;A answer QA21 and manual 10.5.4): LEAVE is judged at the END of AUTO and is lost by a robot that
         * touches ANY perimeter wall then, also the wall inside the LOADING ZONE: a robot that parks there touching the wall
         * keeps its PARK (5) but not its LEAVE (3). So the whole footprint must end at least this many inches off every wall.
         * Touching is not enough, and a field is only accurate to about 1 in. To claim both LEAVE and PARK end in the LOADING
         * ZONE standing this far off its wall (the zone is 11 in deep, a 17.8 in robot overlaps it by 9 in or more that way).
         * Only the robot counts: a POLLEN squeezed between the robot and the wall does not make the robot "touch" it (QA47).
         */
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
        /** TUNE: pixels per field inch. 141.348 in x 4.6 = 650 px leaves room for the title above and the footer below. */
        public static final double RENDER_FIELD_SCALE_PX_PER_IN = 4.6;
        /** TUNE: distance from the top of the SVG to the rear wall of the field. */
        public static final double RENDER_FIELD_TOP_PX = 64.0;
        /** TUNE: an ALLIANCE AREA is really 54 in deep outside the field wall (manual 9.5); only this much of it is drawn (faded out), to leave the field room in a square image. */
        public static final double RENDER_ALLIANCE_AREA_SHOWN_IN = 20.0;
        /** FIXED (CAD, the outer edges of the ALLIANCE AREA tape; the manual says about 97 in): the ALLIANCE AREA spans y 22.26 to 119.08 along the wall (96.82 in). */
        public static final double RENDER_ALLIANCE_AREA_Y_MIN_IN = 22.26;
        public static final double RENDER_ALLIANCE_AREA_Y_MAX_IN = 119.08;
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
        /**
         * TUNE: radius, in SVG pixels, of the small ringed dot drawn on every robot outline at the robot's center of rotation
         * (its position: the point the route's poses refer to). The dot is not in the middle of the outline, because the
         * robot is not centered on it (the front edge is 9.77 in from it, the rear edge 8.03 in).
         */
        public static final double RENDER_POSE_MARK_RADIUS_PX = 3.4;
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

        /**
         * FIXED (goBILDA mecanum StarterBot drawing 3200-2627-0004): overall length, front to back, 452 mm. This is the
         * OUTER PERIMETER: it INCLUDES the four mecanum wheels and the two flat intake wheels at the front (see
         * {@link #INTAKE_REACH_IN}). It is NOT centered on the robot's center of rotation (the point Pedro and the
         * odometry call the robot's position): see {@link #FRONT_EXTENT_IN} and {@link #REAR_EXTENT_IN} for where the
         * perimeter really lies around that point. Used for the 18 in start cube (rule R102).
         */
        public static final double LENGTH_IN = 17.80;
        /** FIXED (same drawing): overall width, side to side, 452 mm, wheels included. The robot is square, so it fits the 18 in start cube (rule R102) with 0.2 in to spare. */
        public static final double WIDTH_IN = 17.80;

        // ---- where the perimeter lies around the center of rotation -----------------------------------------
        // The robot's POSITION is the point it spins about (the "center of rotation": for a mecanum robot, about the
        // middle between the axles). It is NOT the middle of the 17.80 x 17.80 in perimeter, because the intake sticks
        // out in front of the front axle and the back of the robot ends nearer to the rear axle. These four numbers say
        // how far the perimeter is from that point in each direction. Facing the audience wall front-first, the robot's
        // position is FRONT_EXTENT_IN from the wall when the front touches it.

        /**
         * VERIFY: distance, in inches, from the center of rotation to the FRONT edge of the perimeter (the outer edge of
         * the two flat intake wheels). 9.77 in read off the goBILDA drawing, taking the center of rotation as the middle
         * between the two axles (the axles are 10.39 in apart; the drawing puts the middle 8.03 in from the rear edge and
         * 9.77 in from the front edge). Measure it on the real robot: mark the point the robot spins about, then measure
         * to the front edge. Must satisfy {@code FRONT_EXTENT_IN + REAR_EXTENT_IN = LENGTH_IN}; ConstantsCheck checks it.
         */
        public static final double FRONT_EXTENT_IN = 9.77;
        /**
         * VERIFY: distance, in inches, from the center of rotation to the REAR edge of the perimeter (the back lip, where
         * the ball leaves). 8.03 in from the drawing. See {@link #FRONT_EXTENT_IN}.
         */
        public static final double REAR_EXTENT_IN = 8.03;
        /**
         * VERIFY: distance, in inches, from the center of rotation to the LEFT edge of the perimeter (the outer face of
         * the left mecanum wheels, seen from above, with the robot facing forward). 8.90 in because the drive is symmetric
         * left to right. Must satisfy {@code LEFT_EXTENT_IN + RIGHT_EXTENT_IN = WIDTH_IN}.
         */
        public static final double LEFT_EXTENT_IN = 8.90;
        /** VERIFY: distance, in inches, from the center of rotation to the RIGHT edge of the perimeter. See {@link #LEFT_EXTENT_IN}. */
        public static final double RIGHT_EXTENT_IN = 8.90;
        /** FIXED (same drawing): height of the tallest part, 308 mm. */
        public static final double HEIGHT_IN = 12.13;
        /** FIXED (same drawing): mecanum wheel diameter, 104 mm. */
        public static final double WHEEL_DIAMETER_IN = 4.09;

        /**
         * ADD: where the ball leaves the shooter, measured from the robot's center of rotation. The
         * shooter fires out the BACK, so forward is NEGATIVE. The starting value, -8.03 in, is DERIVED FROM THE CAD
         * DRAWING, not measured: the drawing dimensions the launch height at the REAR lip of the robot, so the ball is
         * assumed to leave at the rear edge of the perimeter, {@link #REAR_EXTENT_IN} behind the center of rotation.
         * Measure the real exit (from the center of rotation straight back to where the ball leaves) and replace it.
         * With this value and the robot's front on the audience wall, the ball exit is 17.80 in from that wall, which is
         * the situation of the measured shot (37.7 in from the aim point).
         */
        public static final double LAUNCH_POINT_FORWARD_IN = -8.03;
        /** ADD: sideways offset of the exit from the center line (positive = robot's left). */
        public static final double LAUNCH_POINT_LEFT_IN = 0.0;

        // ---- the two flat intake wheels --------------------------------------------------------------------
        // The intake has two small wheels that lie FLAT (their discs are parallel to the floor, like coins on a table)
        // at the two front corners, one left and one right of the center line. In front of the chassis frame only these
        // two discs exist: there is no full-width strip. To take a FLOWER's bottom POLLEN, ONE of them presses into it
        // (see autoshoot.routes.FlowerPress). The numbers below were read off the goBILDA drawing and must be measured
        // on the real robot.

        /**
         * ADD: how far (inches) the flat intake wheels stick out in FRONT of the chassis frame. Measure from the front
         * edge of the frame (the part that must never touch a FLOWER plate) to the front tip of a wheel. 2.7 in = 68 mm
         * read off the drawing: the outer perimeter reaches 9.77 in ahead of the center of rotation and the frame 7.1 in.
         * {@link #LENGTH_IN} INCLUDES these wheels, so everything that plans with the footprint (the TeleOp auto-shoot,
         * the route validator) sees the same rectangle. The wheels must reach at least
         * {@code Field.FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN + Auto.FLOWER_CHASSIS_CLEARANCE_IN - the press stop line} = 2.7 in to press
         * a FLOWER's POLLEN with the chassis kept off its plate; the route validator checks it and says how many
         * inches are missing.
         */
        public static final double INTAKE_REACH_IN = 2.7;
        /**
         * Derived, not a setting: how far the chassis FRAME reaches in front of the center of rotation, in inches
         * ({@link #FRONT_EXTENT_IN} minus {@link #INTAKE_REACH_IN}: 7.07 in, the drawing shows about 7.1). It is the
         * perimeter without the stretch where only the two flat wheels stick out. Change the two numbers it is made of,
         * never this one.
         */
        public static final double CHASSIS_FRONT_EXTENT_IN = FRONT_EXTENT_IN - INTAKE_REACH_IN;
        /**
         * ADD: how far (inches) the middle of each flat wheel is from the robot's center line, to the left for the left
         * wheel and to the right for the right one. 6.1 in read off the drawing (the two wheels are 12.2 in apart, center to
         * center, with a 9.45 in gap between them). Too far apart to straddle a 2.8 in POLLEN, so the robot stands to
         * one side of a FLOWER and presses one wheel into the POLLEN: this number is how far to one side.
         */
        public static final double INTAKE_WHEEL_OFFSET_IN = 6.1;
        /**
         * VERIFY: diameter (inches) of a flat intake wheel, about 2.8 in from the drawing. It sets how big the square
         * around a wheel is that the validator keeps out of FLOWER plates.
         */
        public static final double INTAKE_WHEEL_DIAMETER_IN = 2.8;

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
         *
         * <p>THE ANCHOR ROW IS A MEASUREMENT, so it is NOT changed when the field numbers change. But the
         * model computes the range of that same physical shot from the field: with the official CAD the aim
         * point is 54.156 in from the audience wall (the old 144 in field said 55.52) and the ball exit is
         * 17.80 in from the wall, so the model says 36.36 in for the very place where the team measured
         * 37.7 in: 1.34 in less. Either the tape measured something slightly different, or the ball leaves
         * the robot farther forward than {@code Robot.LAUNCH_POINT_FORWARD_IN} says (the robot CAD suggests
         * 6.58 in behind the center of rotation, which would give 37.8 in). On the real robot,
         * re-measure the distance from the ball exit to the cell opening center with a tape (see
         * docs/DAY_ONE_CHECKLIST.md) before trusting the rows next to the anchor.
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
        /**
         * TUNE (default false: G409 / G417 caution): may the planner drive UNDER the HIVE, along the lane between its two foot bars? A robot
         * 12 to 13 in tall fits under it (the cells are 30.65 in up), and the way from the audience side to the rear side is about 20 in shorter
         * that way than around the frame (see docs/AUTOS.md). The price is being under a HIVE that can tip: the camera-confirmed rest of the HIVE
         * is not known to the planner, which is why this is off. With it on, the planner treats only the foot bars and the leg stubs as solid
         * (the exact shape, grown by {@link #KEEPOUT_MARGIN_IN} and the robot's reach) and falls back to the way around when the lane
         * cannot be driven safely. Route steps choose per step with {@code allowUnderHive()}; this flag is for the TeleOp planner.
         */
        public static final boolean ALLOW_UNDER_HIVE = false;
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
        /**
         * TUNE: when the drive turns a route into a Pedro path, a waypoint closer than this to the previous point is
         * dropped (inches), because Pedro cannot follow a zero-length line. The route checker drops the same points,
         * so it checks exactly the path that is driven.
         */
        public static final double PATH_MIN_LEG_IN = 0.5;

        // ---- keeping the whole robot (not just its center) off the walls while it drives ------
        // The route checker (core/RouteSweep) slides the robot's real 17.8 x 17.8 in outline (not centered on its position) along the route and tests it
        // against the walls, the HIVE frame and the FLOWER plates. It looks at least this often, and more often
        // wherever the robot is close to something.

        /** TUNE: the route checker looks at the robot at least every this many inches of driving (smaller = slower but finer). */
        public static final double PATH_SWEEP_STEP_IN = 1.0;
        /** TUNE: ... and at least every this many degrees of turning (a turning rectangle sweeps its corners through a wide arc). */
        public static final double PATH_SWEEP_STEP_DEG = 3.0;
        /**
         * VERIFY: the checker forgives the robot coming this much closer than the wall clearance or keep-out margin
         * (inches). It absorbs rounding and the tiny amount the robot can start inside a margin (it starts touching a
         * wall). Keep it well below {@code WALL_CLEARANCE_IN}, or the robot could be allowed to touch the wall.
         */
        public static final double PATH_SWEEP_TOLERANCE_IN = 0.05;
        /** TUNE: the finest spacing the checker refines to when the robot is very close to something (inches). */
        public static final double PATH_SWEEP_MIN_STEP_IN = 0.02;
        /**
         * TUNE: when the planner has to squeeze the robot's turn into part of a route (because the robot is against a
         * wall and cannot turn there) it never asks for more than this many degrees of turning per inch driven.
         * The robot cannot follow a heading that changes faster than it can spin: at a 30 in/s drive speed, 4 deg/in is
         * 120 degrees per second.
         */
        public static final double PATH_MAX_TURN_DEG_PER_IN = 4.0;
        /**
         * TUNE: when the planner squeezes a turn into the part of a route where the robot is far from every wall, it
         * finishes the turn this many inches before that part ends, so the real robot's heading can catch up with the
         * planned one while it is still far from the wall (inches).
         */
        public static final double PATH_TURN_SETTLE_IN = 3.0;
        /**
         * TUNE: when a robot standing against a wall has to turn a long way, the planner sends it out to open floor and
         * adds a bend to the trip so the turn has room to be gentle. It looks for that bend by trying points around a
         * ring, one every this many degrees (smaller = more places tried, slower).
         */
        public static final double PATH_DETOUR_ANGLE_STEP_DEG = 15.0;

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
        /**
         * TUNE: after seeing the up cell's tags steadily for this long, losing all of them means the HIVE may have tipped. It is
         * also the SETTLE TIME of the autonomous period: a HIVE counts as fully tipped only when the camera has seen the new up
         * cell's cluster, with a full geometry, without a break for this long (TipConfirmation), because a TIP is complete only
         * when the damper touches the frame and launching at a HIVE that is still rocking can spoil it (G417). The route plan
         * adds it to every WAIT_FOR_TIP, to the latest partner tip that still works and to the spill window.
         */
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
         * autonomous (RED frame; BLUE is the half-turn of it): the CENTER of tile C1 (58.924 in from the red
         * wall, 1.0 in from the HIVE axis), robot FRONT pressed against the audience wall, back toward the
         * HIVE. This is also the one measured good shot position (37.7 in from the aim point). The robot must
         * really be placed here for odometry to be right. The position is the robot's center of rotation, which is
         * {@link Robot#FRONT_EXTENT_IN} behind the front edge, so the Y value is that distance (the front edge is then
         * exactly on the wall; the start check in ConstantsCheck rejects a value that would put the robot through it).
         */
        public static final double DEFAULT_START_X_IN = Field.CENTER_IN - Field.TILE_PITCH_IN / 2.0;
        public static final double DEFAULT_START_Y_IN = Robot.FRONT_EXTENT_IN;
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
    // FIELD AND HIVE (the official field CAD, v26-27.2, read with the manual TU09 sections 9.2-9.9; RED frame)
    //
    // THE FRAME. Every coordinate in this group (and in Auto below) is in the WALL-CORNER frame: the origin is the
    // INNER corner where the red wall meets the audience wall, +x runs toward blue along the audience wall, +y runs away
    // from the audience along the red wall, heading 0 is +x and angles grow counter-clockwise. "The wall" always means the
    // inner face of the polycarbonate panel, the face a robot can touch. A robot pressed against a wall therefore has its
    // edge at 0 (or SIZE_IN), and its pose point is that edge's distance (Robot.*_EXTENT_IN) from the wall.
    // The BLUE alliance is the RED one turned half a turn about the field center (CENTER_IN, CENTER_IN):
    // (x, y, heading) becomes (SIZE_IN - x, SIZE_IN - y, heading + 180 degrees).
    // =========================================================================================
    public static final class Field {
        private Field() {
        }

        /**
         * FIXED (official field CAD, v26-27.2): the field interior, wall face to wall face, is 141.348 in square. The manual
         * (9.2) only says "approximately 144 in" (+/- 1 in), but its own CAD is 2.65 in smaller, an older FIRST field guide
         * says about 141 in for the same perimeter kit, and AndyMark's tiles (24.25 to 24.375 in across, laid edge to edge
         * with a 23.5 in pitch) agree. We trust the CAD. Two small facts: a thin rail runs along the top and the bottom of
         * every wall panel and sticks out 0.089 in past the glass (it is the first thing a robot with material at 10.5 to
         * 11.6 in or below 0.6 in touches), and real fields vary by about +/- 1 in (tiles can drift by 3/4 in across the
         * field), so 0.05 in digits are modelling precision, not field truth.
         */
        public static final double SIZE_IN = 141.348;
        /**
         * Derived from {@link #SIZE_IN}, not a setting: the field's middle on each axis, 70.674 in. The HIVE frame is centered on
         * it, the half turn between the alliances is about it, and y = this splits the audience half from the rear half.
         */
        public static final double CENTER_IN = SIZE_IN / 2.0;

        /**
         * FIXED (CAD + AndyMark): the floor tiles are laid with a 23.5 in pitch (centre to centre), not 24 in; six of them make
         * 141 in. The two outer tiles of each row are cut to fit the walls. So the seams (tile edges) are at
         * {@code CENTER_IN +/- 23.5, +/- 47.0}: x or y = 23.674, 47.174, 70.674, 94.174, 117.674. {@code core/TileGrid} turns
         * these numbers into seams and tile names.
         */
        public static final double TILE_PITCH_IN = 23.5;
        /** FIXED (manual 9.2): the floor is 6 x 6 tiles. Columns A to F run along x from the red wall, rows 1 to 6 along y from the audience wall. */
        public static final int TILES_PER_SIDE = 6;

        /**
         * FIXED (CAD, the two foot bars of the HIVE frame, which hold the frame to the floor; manual 9.6.1 says 49.46): the HIVE frame
         * is 49.466 in wide along x ... Two cells hang from the frame, and the red and the blue HIVE share it.
         */
        public static final double HIVE_FRAME_WIDTH_X_IN = 49.466;
        /** FIXED (CAD; manual 9.6.1 says 38.95): ... and 38.944 in deep along y, centered on the field center. */
        public static final double HIVE_FRAME_DEPTH_Y_IN = 38.944;

        // ---- the exact solid of the HIVE below a robot's height (official CAD) ----------------------------------------------------
        // Everything of the HIVE that a robot 12 to 13 in tall can touch is the FRAME: two foot bars on the floor and four legs that lean
        // inward from the ends of the bars up to the cells' axle. The cells themselves are 30.65 in up or higher and the crossbar 40.95 in,
        // so they pass over the robot. Between the two feet the floor is open: a lane the robot may drive along (north-south) if the HIVE
        // cannot tip while it is there. See core/HiveSolid for how these numbers make the shape, and docs/AUTOS.md for the lane.

        /**
         * FIXED (CAD, the two foot bars, sheet metal 1.983 in wide, 38.944 in long, 2.148 in tall): half the distance between the INNER faces
         * of the two bars, 22.75 in. The bars' outer faces are {@code HIVE_FRAME_WIDTH_X_IN / 2} = 24.733 in from the center line. The bars are
         * a 2.15 in high step: the chassis has 0.94 in of ground clearance, so a robot cannot drive across one, only along between them.
         */
        public static final double HIVE_FOOT_BAR_INNER_HALF_GAP_IN = 22.75;
        /** FIXED (CAD): the foot bars are 2.148 in tall. */
        public static final double HIVE_FOOT_BAR_HEIGHT_IN = 2.148;
        /**
         * FIXED (CAD, the tube that is the leg: 45.75 in long, 1.19 x 1.00 in, leaning 27.7 degrees from vertical): where the middle of a leg
         * stands on its foot bar, measured from the HIVE center: 23.811 in along x and 18.561 in along y (a leg at each of the four ends of the
         * two bars) ...
         */
        public static final double HIVE_LEG_FOOT_HALF_SPAN_X_IN = 23.811;
        /** FIXED (CAD): ... the y part of that, ... */
        public static final double HIVE_LEG_FOOT_HALF_SPAN_Y_IN = 18.561;
        /** FIXED (CAD): ... at this height above the floor (the leg starts on top of the foot cap), ... */
        public static final double HIVE_LEG_FOOT_Z_IN = 0.533;
        /**
         * FIXED (CAD; a second derivation gave 0.2777, the first report 0.273): ... and the middle of the leg moves this many inches TOWARD the
         * HIVE center along x for every inch it climbs ...
         */
        public static final double HIVE_LEG_LEAN_X_PER_Z = 0.2777;
        /** FIXED (CAD): ... and this many inches toward the center along y (so the legs of the two ends converge on the cell axles). */
        public static final double HIVE_LEG_LEAN_Y_PER_Z = 0.4452;
        /**
         * FIXED (CAD, fitted so that the plan "shadow" of everything below 12.13 in reproduces the measured one to 0.02 in: stub tips at
         * x 50.61 and y 57.82 / 83.53, in the wall-corner frame): half the width of a leg as seen from above, 0.55 in. The tube is 1.19 x 1.00 in,
         * and a leaning tube looks a little wider from above.
         */
        public static final double HIVE_LEG_HALF_WIDTH_IN = 0.55;
        /** FIXED (CAD): the lowest point of any cell, the rim of a cell that is DOWN, is 30.652 in above the floor. The solid model has no cells, so it is only valid for a robot that stays below this. */
        public static final double HIVE_LOWEST_CELL_POINT_IN = 30.652;

        /**
         * FIXED (manual Fig 9-10 and the CAD, which measures 25.500 in): the distance between the axes of the two HIVEs, 25.5 in.
         * The red HIVE axis is half of it left of the field center and the blue one half of it right. (The CAD axles are at
         * 57.931 and 83.431, i.e. 0.007 in right of this exact picture: its own half-turn mismatch, far below anything that matters.)
         */
        public static final double HIVE_AXIS_SPACING_IN = 25.5;
        /** FIXED (the line above): the red HIVE axis, the center line of its cell openings, 12.75 in left of the field center: 57.924 in. */
        public static final double RED_HIVE_AXIS_X_IN = CENTER_IN - HIVE_AXIS_SPACING_IN / 2.0;
        /** FIXED (CAD: 20.03 in; manual 9.6.2 says 20): a CELL opening is 20.03 in wide (and 14.03 in tall). */
        public static final double CELL_OPENING_WIDTH_IN = 20.03;
        /**
         * FIXED (CAD, the hull of a whole cell including its cable ties): a cell's PLAN outline is 21.06 in wide along x (the
         * opening plus the cell's side walls). The spill zone and the diagrams use it; the aim point and the cone use the opening.
         */
        public static final double CELL_PLAN_WIDTH_X_IN = 21.06;

        /**
         * FIXED (CAD: the sticker's cluster origin, the SDK AprilTagGameDatabase's point, lies at the middle of the opening on the
         * cell's front plane): distance from the HIVE pivot-axis plane (y = CENTER_IN) out to the cluster ORIGIN of an UP cell,
         * 16.518 in. The cluster origin is the center of the CELL opening, which is the point to aim at.
         */
        public static final double UP_CELL_ORIGIN_OUT_IN = 16.518;
        /** FIXED (CAD): height of that aim point above the floor for an UP cell, 58.254 in. */
        public static final double UP_CELL_ORIGIN_Z_IN = 58.254;
        /** FIXED (CAD, tag centres measured from the sticker art and the cell): distance out from the pivot plane to the middle of the AprilTag row of an UP cell, 13.104 in. */
        public static final double UP_CELL_TAG_ROW_OUT_IN = 13.104;
        /** FIXED (CAD): height of the AprilTag row of an UP cell, 49.791 in. */
        public static final double UP_CELL_TAG_ROW_Z_IN = 49.791;
        /** FIXED (CAD): height of the AprilTag row of a DOWN cell, 35.522 in (used to tell UP from DOWN by tag height). */
        public static final double DOWN_CELL_TAG_ROW_Z_IN = 35.522;
        /** FIXED (CAD): distance out from the pivot plane to the middle of the AprilTag row of a DOWN cell, 11.611 in (the cell swings in toward the pivot as it tips down). */
        public static final double DOWN_CELL_TAG_ROW_OUT_IN = 11.611;

        /** FIXED (CAD; the manual says 43.95): the HIVE pivot axle is 43.95 in above the floor, on the pivot plane y = {@link #CENTER_IN}. */
        public static final double HIVE_PIVOT_Z_IN = 43.95;
        /** FIXED (CAD, checked on the meshes to 0.001 in): a tip turns a whole cell by exactly 60 degrees about the pivot axle, so the DOWN rows above are the UP rows turned by this. */
        public static final double HIVE_TIP_ANGLE_DEG = 60.0;
        /**
         * FIXED (CAD, 30.00 degrees measured): the base of a cell, the axis of its opening and its AprilTag sticker are tilted 30 degrees
         * from horizontal. An UP cell opens 30 degrees above horizontal; its sticker is in the plane under the cell and every tag looks
         * 60 degrees below horizontal toward its own side (red audience UP cell: toward the audience wall and the floor). The camera has
         * to be on that side of the sticker plane to read it.
         */
        public static final double CELL_TILT_DEG = 30.0;
        /** FIXED (the FTC SDK's AprilTagGameDatabase; the CAD agrees to 0.001 in): the tag row is 7.1874 in behind the front of the cell, measured along the cell's tilted base ... */
        public static final double TAG_ROW_BEHIND_CELL_FRONT_IN = 7.1874;
        /** FIXED (the SDK; the CAD agrees to 0.14 in): ... and the cluster origin, the aim point, is 5.622 in inside the cell from the sticker plane, perpendicular to it. */
        public static final double CLUSTER_ORIGIN_ABOVE_STICKER_IN = 5.622;

        /** FIXED (manual 9.9 / SDK, and the production art; the CAD agrees exactly): tag centers along the sticker, from its centerline, in tag-ID order. */
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
         * FIXED (CAD): the four FLOWERS, as {x, y} centers of the POLLEN stack, in the RED frame, each 2.634 in out from its wall (see
         * {@link #FLOWER_POLLEN_CENTER_FROM_WALL_IN}). In order: the red (left) wall, the rear wall, the blue (right) wall, the
         * audience wall. They are obstacles for the path planner (a robot may touch one only to retrieve POLLEN). The CAD puts
         * every FLOWER on a tile seam to within 0.11 in, and the four are half-turn images of each other to 0.000 in.
         */
        public static final double[][] FLOWER_CENTERS_IN = {
                {2.634, 47.281}, {47.281, 138.714}, {138.714, 94.067}, {94.067, 2.634}};
        /**
         * FIXED (CAD, the bottom ring "Layer X": the plate the robot must not touch; the manual's Fig 9-12 says 4.75): the FLOWER base
         * plate is 4.871 in deep (measured out from its own back edge) ... The POLLEN stack stands in its middle. The rings above
         * it (the middle ring is 5.013 in deep and is at 3.9 to 5.25 in up, inside the robot's height) end at about the same
         * distance from the wall, so the plate is what a robot hits.
         */
        public static final double FLOWER_PLATE_DEPTH_IN = 4.871;
        /** FIXED (CAD): ... whose back edge stands 0.162 in off the wall face (a thin gap the robot cannot enter). */
        public static final double FLOWER_PLATE_GAP_TO_WALL_IN = 0.162;
        /**
         * Derived, not a setting: how far the FIELD-SIDE edge of the plate is from the wall, 5.033 in
         * ({@link #FLOWER_PLATE_GAP_TO_WALL_IN} + {@link #FLOWER_PLATE_DEPTH_IN}). Every check calls THIS "how deep the plate is",
         * because the robot cannot reach into the gap behind it: the plate is drawn and checked as a box from the wall out to here.
         */
        public static final double FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN = FLOWER_PLATE_GAP_TO_WALL_IN + FLOWER_PLATE_DEPTH_IN;
        /** FIXED (CAD; the manual says 5.87): ... and 5.95 in long along the wall, centered on the FLOWER's center point. */
        public static final double FLOWER_PLATE_ALONG_WALL_IN = 5.95;
        /**
         * FIXED (CAD; the manual's Fig 9-12 says 2.4): the center line of the POLLEN stack is this far (inches) out from the wall face.
         * The bottom POLLEN is 2.8 in across ({@code Auto.POLLEN_DIAMETER_IN}), so it spans 1.234 to 4.034 in from the wall.
         * (The FLOWER centers above are on this line.)
         */
        public static final double FLOWER_POLLEN_CENTER_FROM_WALL_IN = 2.634;
        /** Derived (the plate): FLOWER keep-out extent measured along the wall, 5.95 in; the TeleOp planner grows it by its own margin. */
        public static final double FLOWER_KEEPOUT_ALONG_WALL_IN = FLOWER_PLATE_ALONG_WALL_IN;
        /** Derived (the plate): FLOWER keep-out depth out from the wall, 5.033 in. */
        public static final double FLOWER_KEEPOUT_DEPTH_IN = FLOWER_PLATE_FAR_EDGE_FROM_WALL_IN;

        // ---- the rest of the FLOWER, from the CAD (what a 12.13 in tall robot can meet) ---------------------------------
        // The FLOWER stands on the wall as a stack: the bottom ring on the tile (z 0 to 0.354 in), two supports with the retrieval
        // opening between them (z 0.354 to 3.904), the middle ring (z 3.904 to 5.254), four green pipes (z 4.25 to 21.25) and the
        // top ring (z 20.254 to 21.404, hole 3.99 in), with the backstop 1 in above it. The robot meets only the part below its own
        // height: the plate, the supports, the middle ring, the pipes (their centers are 1.725 in either side of the POLLEN stack,
        // 2.40 in apart, closer than a POLLEN is wide) and the wall bracket.

        /** FIXED (CAD, the hole of the bottom ring; the CAD POLLEN, 2.80 in, sits on the tile inside it): 3.04 in. The manual disagrees, see below. */
        public static final double FLOWER_BOTTOM_HOLE_CAD_IN = 3.04;
        /** FIXED (manual 9.7, "Bottom Ring ID"): 2.79 in, 0.25 in smaller than the CAD (we could not reconcile them: the CAD researcher trusts the CAD, because a 2.80 in ball cannot sit in a 2.79 in hole). */
        public static final double FLOWER_BOTTOM_HOLE_MANUAL_IN = 2.79;
        /**
         * The hole diameter used for ANY clearance decision: the SMALLER of the two figures, 2.79 in. The two sources disagree by
         * 0.25 in and nobody has measured a real ring, so a decision that depends on how much room the bottom POLLEN has (how hard a
         * wheel must push to roll it out, whether a part of the robot can reach into the ring) must assume the tight one: if it works
         * with 2.79 in it works with 3.04 in too. (A POLLEN is 2.8 in, so with the manual's hole it is held, not loose.)
         */
        public static final double FLOWER_BOTTOM_HOLE_PLANNING_IN = FLOWER_BOTTOM_HOLE_CAD_IN < FLOWER_BOTTOM_HOLE_MANUAL_IN
                ? FLOWER_BOTTOM_HOLE_CAD_IN : FLOWER_BOTTOM_HOLE_MANUAL_IN;
        /** FIXED (CAD; the manual says about 0.43 in): the lip of the bottom ring stands 0.354 in above the tile; the POLLEN has to be rolled over it. Use 0.43 in when you size a lift. */
        public static final double FLOWER_BOTTOM_RING_LIP_CAD_IN = 0.354;
        /** FIXED (CAD, manual 9.7 agrees: 3.55): the retrieval opening, between the bottom ring and the middle ring, is 3.55 in tall ... */
        public static final double FLOWER_RETRIEVAL_OPENING_HEIGHT_IN = 3.55;
        /** FIXED (CAD; the manual says 3.57): ... and 3.58 in deep, measured from the supports' field-side face to the edge of the ring. The POLLEN leaves toward the field and along the wall. */
        public static final double FLOWER_RETRIEVAL_OPENING_DEPTH_IN = 3.58;
        /** FIXED (CAD, verified by a second derivation): the middle ring's hole is at least 3.76 in wide anywhere and about 3.86 to 3.91 in through most of its height (not 3.4 as a first reading said), so even a 3.6 in NECTAR passes it: what keeps NECTAR in is the bottom hole and the retrieval opening. */
        public static final double FLOWER_MIDDLE_RING_HOLE_MIN_IN = 3.76;

        /**
         * FIXED (CAD, the black "Field Bracket" that holds the FLOWER to the wall): the bracket sticks out 1.67 in from the wall face ...
         * It is only 0.4 in tall, at 11.437 to 11.837 in above the tile, which is INSIDE the 12.13 in the robot is tall: a part of
         * the robot that is taller than 11.44 in and within 1.67 in of the wall next to a FLOWER can touch it.
         */
        public static final double FLOWER_BRACKET_PROTRUSION_IN = 1.67;
        /** FIXED (CAD): ... and the bracket is 6.67 in long along the wall (the plate is 5.95), so it reaches 0.36 in past each end of the plate. */
        public static final double FLOWER_BRACKET_ALONG_WALL_IN = 6.67;
        /** FIXED (CAD): the bottom of the bracket, 11.437 in above the tile ... */
        public static final double FLOWER_BRACKET_BOTTOM_Z_IN = 11.437;
        /** FIXED (CAD): ... and its top, 11.837 in. */
        public static final double FLOWER_BRACKET_TOP_Z_IN = 11.837;
    }

    // =========================================================================================
    // ROBOT CHECK ("00 Robot Check": the guided test of every robot part, for people who have never run the robot)
    // The decisions live in autoshoot/core/RobotCheck*.java; the OpMode is diagnostics/RobotCheckOpMode.java.
    // =========================================================================================
    public static final class RobotCheck {
        private RobotCheck() {
        }

        // ---- gamepad 1 buttons (PlayStation names) ------------------------------------------------

        /** TUNE: "yes" and "continue": the answer to a question, and the button that starts a step or goes to the next one. */
        public static final Button YES_BUTTON = Button.CROSS;
        /** TUNE: "no" and "there is a problem". While a motor is running it also stops the motor at once. */
        public static final Button NO_BUTTON = Button.CIRCLE;
        /** TUNE: skip the step you are on (the safety step can never be skipped). */
        public static final Button SKIP_BUTTON = Button.TRIANGLE;
        /** TUNE: do the part you are on again (spin a wheel again, or repeat a whole step from its result screen). */
        public static final Button REPEAT_BUTTON = Button.SQUARE;
        /** TUNE: finish the gamepad step early. It must not be one of the buttons that step asks you to press. */
        public static final Button GAMEPAD_FINISH_BUTTON = Button.TOUCHPAD;
        /** TUNE: scroll the summary list up and down when it is longer than the screen. */
        public static final Button SCROLL_UP_BUTTON = Button.DPAD_UP;
        public static final Button SCROLL_DOWN_BUTTON = Button.DPAD_DOWN;
        /**
         * FIXED (the buttons BioBuzzRobot uses): the buttons the gamepad step asks you to press once. The two triggers
         * are always checked as well (they are not Buttons). The yes / no / skip / repeat buttons are in this list
         * too: in that step they are only tested, they do not act.
         */
        public static final Button[] GAMEPAD_BUTTONS_TO_CHECK = {
                Button.CROSS, Button.CIRCLE, Button.SQUARE, Button.TRIANGLE,
                Button.DPAD_UP, Button.DPAD_DOWN, Button.DPAD_LEFT, Button.DPAD_RIGHT,
                Button.LEFT_BUMPER, Button.RIGHT_BUMPER, Button.SHARE, Button.OPTIONS};
        /**
         * TUNE: a button press is ignored when it comes this soon (seconds) after the screen changed. It stops a
         * double click from racing through two screens, and so from starting a motor by accident.
         */
        public static final double BUTTON_LOCKOUT_SEC = 0.4;

        // ---- safety limits ---------------------------------------------------------------------------

        /** FIXED: no motor or servo power in this OpMode may be above this fraction of full power (0 to 1). The logic clamps every command to it. */
        public static final double POWER_LIMIT = 0.6;

        // ---- step 1: battery (volts, read from the Control Hub) --------------------------------------

        /** TUNE: this many volts or more is a PASS. A freshly charged 12 volt robot battery reads about 13 volts. */
        public static final double BATTERY_PASS_VOLTS = 12.4;
        /** TUNE: below this many volts is a FAIL. Between this and the PASS voltage is a WARN. */
        public static final double BATTERY_FAIL_VOLTS = 11.0;

        // ---- step 3: drive wheels ----------------------------------------------------------------------

        /** TUNE: power for each wheel test, as a fraction of full power (0 to 1). Gentle on purpose. */
        public static final double DRIVE_TEST_POWER = 0.25;
        /** TUNE: how long each wheel spins, in seconds. */
        public static final double DRIVE_TEST_SEC = 0.6;

        // ---- step 4: Pinpoint odometry (distances in inches, angles in degrees) ------------------------

        /** FIXED (a ruler is 12 inches, half a floor tile): how far the student pushes the robot forward, in inches. */
        public static final double PINPOINT_PUSH_DISTANCE_IN = 12.0;
        /** TUNE: the push counts as the right size when the measured forward distance is within this many inches of it. A hand push is not exact; a pod set up for the wrong wheel size is out by far more. */
        public static final double PINPOINT_PUSH_TOLERANCE_IN = 3.0;
        /** TUNE: more sideways drift than this many inches during the forward push is a WARN (a crooked push, or a strafe pod that reads too much). */
        public static final double PINPOINT_PUSH_SIDEWAYS_MAX_IN = 3.0;
        /** TUNE: the position changed by less than this many inches means "nothing moved" (the Pinpoint or its pods are not working). */
        public static final double PINPOINT_NO_MOVEMENT_IN = 1.0;
        /** FIXED: how far the student turns the robot counter-clockwise: a quarter turn, in degrees. */
        public static final double PINPOINT_TURN_DEG = 90.0;
        /** TUNE: the turn counts as a quarter turn when the heading changed by this many degrees either side of it. */
        public static final double PINPOINT_TURN_TOLERANCE_DEG = 15.0;
        /** TUNE: the heading changed by less than this many degrees means "the heading did not change". */
        public static final double PINPOINT_NO_TURN_DEG = 5.0;
        /** TUNE: during the turn the position may drift this many inches before a WARN (the robot slid, or the pod offsets in pedro/Constants.java are wrong). */
        public static final double PINPOINT_TURN_DRIFT_WARN_IN = 3.0;

        // ---- step 5: intake (power is a fraction of full power) ---------------------------------------

        /** TUNE: power of the front intake roller motor during its test (0 to 1). */
        public static final double INTAKE_MOTOR_POWER = 0.4;
        /** TUNE: power size of each corner servo during its test (0 to 1). */
        public static final double INTAKE_SERVO_POWER = 0.5;
        /** FIXED (BioBuzzRobot.manualControls runs the left corner servo at power -1): sign that pulls POLLEN inward. */
        public static final double INTAKE_LEFT_SERVO_SIGN = -1.0;
        /** FIXED (BioBuzzRobot.manualControls runs the right corner servo at power +1): sign that pulls POLLEN inward. */
        public static final double INTAKE_RIGHT_SERVO_SIGN = 1.0;
        /** TUNE: how long the roller motor, and then each corner servo, runs, in seconds. */
        public static final double INTAKE_RUN_SEC = 1.5;

        // ---- step 6: flywheel (speed in encoder ticks per second; the target is Shooter.MANUAL_TARGET_TPS) --

        /** FIXED: the TeleOp's own spin-up timeout (AutoShoot.SPINUP_TIMEOUT_SEC): the flywheel may take this many seconds to reach speed. */
        public static final double FLYWHEEL_TIMEOUT_SEC = AutoShoot.SPINUP_TIMEOUT_SEC;
        /** TUNE: a flywheel that never went faster than this many ticks per second (either way) counts as "not turning at all". */
        public static final double FLYWHEEL_NOT_TURNING_TPS = 50.0;

        // ---- step 7: feeder -------------------------------------------------------------------------------

        /** TUNE: power size of the feeder servo during its test (0 to 1). */
        public static final double FEEDER_TEST_POWER = 0.3;
        /** TUNE: the feeder runs forward (pushing toward the flywheel) this many seconds... */
        public static final double FEEDER_FORWARD_SEC = 1.0;
        /** TUNE: ... and then backward this many seconds. */
        public static final double FEEDER_REVERSE_SEC = 0.5;

        // ---- step 8: Limelight camera ---------------------------------------------------------------------

        /** TUNE: how long the camera is watched for new pictures, in seconds. */
        public static final double LIMELIGHT_CHECK_SEC = 1.0;
        /** TUNE: at least this many NEW camera pictures must arrive during the check. */
        public static final int LIMELIGHT_MIN_NEW_FRAMES = 3;
        /** TUNE: how long the student has to hold a HIVE tag in front of the camera, in seconds. */
        public static final double LIMELIGHT_TAG_TIMEOUT_SEC = 20.0;
        /** FIXED (manual 9.9, same as Field): the lowest HIVE AprilTag ID, 30. */
        public static final int HIVE_TAG_ID_MIN = Field.RED_REAR_BASE_TAG_ID;
        /** FIXED (manual 9.9, same as Field): the highest HIVE AprilTag ID, 45 (the last cell's base ID plus its 4 tags). */
        public static final int HIVE_TAG_ID_MAX = Field.BLUE_REAR_BASE_TAG_ID + Field.TAGS_PER_CLUSTER - 1;

        // ---- step 9: gamepad ---------------------------------------------------------------------------------

        /** TUNE: the gamepad step ends by itself after this many seconds, with whatever buttons it has seen. */
        public static final double GAMEPAD_TIMEOUT_SEC = 90.0;

        // ---- step 10: summary --------------------------------------------------------------------------------

        /** TUNE: lines of the summary shown at once; with more lines the scroll buttons move the list. */
        public static final int SUMMARY_LINES_PER_PAGE = 12;
    }
}
