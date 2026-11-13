package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
            // Catches a mix-up of units (for example millimeters typed as inches): the ball exit cannot be farther
            // from the robot's center than the robot is long or wide.
            if (Math.abs(BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN) > BioBuzzConstants.Robot.LENGTH_IN
                    || Math.abs(BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN) > BioBuzzConstants.Robot.WIDTH_IN) {
                r.warnings.add("Robot.LAUNCH_POINT_* is far outside the robot footprint.");
            }
        }
        // Forward is the intake's direction; the shooter fires out the back, so the ball exit is at negative forward.
        if (BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN >= 0) {
            r.warnings.add("Robot.LAUNCH_POINT_FORWARD_IN is not negative, but the shooter fires out the BACK.");
        }

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
        } catch (RuntimeException e) {
            r.errors.add("Field geometry is invalid: " + e.getMessage());
        }
        return r;
    }
}
