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
 */
public final class ConstantsCheck {

    /** The result of a check. */
    public static final class Report {
        public final List<String> errors = new ArrayList<String>();
        public final List<String> warnings = new ArrayList<String>();
        public final List<String> placeholders = new ArrayList<String>();

        public boolean ok() {
            return errors.isEmpty();
        }
    }

    private ConstantsCheck() {
    }

    public static Report run() {
        Report r = new Report();
        r.placeholders.addAll(Arrays.asList(BioBuzzConstants.PLACEHOLDERS));

        // ---- the shot table and the planner band
        ShotTable table = null;
        try {
            table = ShotTable.fromConstants();
        } catch (IllegalArgumentException e) {
            r.errors.add("Shooter.SHOT_TABLE: " + e.getMessage());
        }
        if (table != null) {
            double lo = table.minRange() + BioBuzzConstants.Shooter.RANGE_MARGIN_IN;
            double hi = table.maxRange() - BioBuzzConstants.Shooter.RANGE_MARGIN_IN;
            if (hi <= lo) {
                r.errors.add("Shooter.RANGE_MARGIN_IN is too large for the shot table: no range is left to shoot from.");
            }
            double preferred = BioBuzzConstants.AutoShoot.PREFERRED_RANGE_IN;
            if (preferred < lo || preferred > hi) {
                r.warnings.add("AutoShoot.PREFERRED_RANGE_IN (" + preferred + ") is outside the usable shot range ["
                        + lo + ", " + hi + "].");
            }
        }

        // ---- shooter timing and windows
        if (BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC <= 0 || BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC >= 0.5
                || BioBuzzConstants.Shooter.AT_SPEED_ABOVE_FRAC <= 0 || BioBuzzConstants.Shooter.AT_SPEED_ABOVE_FRAC >= 0.5) {
            r.errors.add("Shooter.AT_SPEED_BELOW_FRAC and AT_SPEED_ABOVE_FRAC must be between 0 and 0.5.");
        }
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
        if (BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY < 1 || BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY > 4) {
            r.errors.add("Shooter.MAX_SHOTS_PER_VOLLEY must be 1 to 4 (rule G407: a robot may hold at most 4 POLLEN).");
        }
        if (BioBuzzConstants.Shooter.FEED_SECONDS_PER_SHOT <= 0 || BioBuzzConstants.Shooter.VOLLEY_MAX_SEC <= 0) {
            r.errors.add("Shooter.FEED_SECONDS_PER_SHOT and VOLLEY_MAX_SEC must be positive.");
        } else if (BioBuzzConstants.Shooter.VOLLEY_MAX_SEC
                < BioBuzzConstants.Shooter.FEED_SECONDS_PER_SHOT * BioBuzzConstants.Shooter.MAX_SHOTS_PER_VOLLEY) {
            r.warnings.add("Shooter.VOLLEY_MAX_SEC is shorter than FEED_SECONDS_PER_SHOT x MAX_SHOTS_PER_VOLLEY: a full"
                    + " volley may be cut short.");
        }
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
            if (Math.abs(BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN) > BioBuzzConstants.Robot.LENGTH_IN
                    || Math.abs(BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN) > BioBuzzConstants.Robot.WIDTH_IN) {
                r.warnings.add("Robot.LAUNCH_POINT_* is far outside the robot footprint.");
            }
        }
        if (BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN >= 0) {
            r.warnings.add("Robot.LAUNCH_POINT_FORWARD_IN is not negative, but the shooter fires out the BACK.");
        }

        // ---- planner
        if (BioBuzzConstants.AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG < 0 || BioBuzzConstants.AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG > 60) {
            r.errors.add("AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG must be between 0 and 60.");
        }
        if (BioBuzzConstants.AutoShoot.SHOOT_CONE_STEP_DEG <= 0 || BioBuzzConstants.AutoShoot.SHOOT_RANGE_STEP_IN <= 0) {
            r.errors.add("AutoShoot.SHOOT_CONE_STEP_DEG and SHOOT_RANGE_STEP_IN must be positive.");
        }
        if (BioBuzzConstants.AutoShoot.ALIGN_POS_TOL_IN <= 0 || BioBuzzConstants.AutoShoot.ALIGN_HEADING_TOL_DEG <= 0) {
            r.errors.add("AutoShoot.ALIGN_*_TOL must be positive.");
        }
        if (BioBuzzConstants.AutoShoot.ALIGN_TIMEOUT_SEC <= BioBuzzConstants.AutoShoot.ALIGN_SETTLE_SEC) {
            r.errors.add("AutoShoot.ALIGN_TIMEOUT_SEC must be longer than ALIGN_SETTLE_SEC.");
        }
        if (BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_STICK <= 0 || BioBuzzConstants.AutoShoot.DRIVER_OVERRIDE_STICK > 1) {
            r.errors.add("AutoShoot.DRIVER_OVERRIDE_STICK must be between 0 and 1.");
        }

        // ---- vision
        if (BioBuzzConstants.Vision.TAG_MIN_RANGE_IN >= BioBuzzConstants.Vision.TAG_MAX_RANGE_IN) {
            r.errors.add("Vision.TAG_MIN_RANGE_IN must be smaller than TAG_MAX_RANGE_IN.");
        }
        if (BioBuzzConstants.Vision.CAMERA_PITCH_UP_DEG < -20 || BioBuzzConstants.Vision.CAMERA_PITCH_UP_DEG > 80) {
            r.errors.add("Vision.CAMERA_PITCH_UP_DEG is outside -20..80 degrees.");
        }
        if (BioBuzzConstants.Vision.CAMERA_HEIGHT_IN <= 0) {
            r.errors.add("Vision.CAMERA_HEIGHT_IN must be positive.");
        }
        if (BioBuzzConstants.Vision.POLL_RATE_HZ < 1 || BioBuzzConstants.Vision.POLL_RATE_HZ > 250) {
            r.errors.add("Vision.POLL_RATE_HZ must be between 1 and 250.");
        }
        if (BioBuzzConstants.Vision.HIVE_CONFIRM_FRAMES < 1) {
            r.errors.add("Vision.HIVE_CONFIRM_FRAMES must be at least 1.");
        }

        // ---- controls
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
        double[] signs = {BioBuzzConstants.Controls.MANUAL_FORWARD_SIGN, BioBuzzConstants.Controls.MANUAL_STRAFE_SIGN,
                BioBuzzConstants.Controls.MANUAL_TURN_SIGN};
        for (double sgn : signs) {
            if (Math.abs(sgn) != 1.0) {
                r.errors.add("Controls.MANUAL_*_SIGN must be +1 or -1.");
                break;
            }
        }

        // ---- the field model must hold together
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
