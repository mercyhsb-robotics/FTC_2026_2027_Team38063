package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Every number and button the Robot Check logic uses, copied from {@code BioBuzzConstants} (mostly the nested class
 * {@code RobotCheck}, plus the shooter's speed window) so the logic can be tested with other values.
 *
 * <p>Why a copy and not a direct read: a unit test can build a Config with, say, a tiny battery limit and check what the
 * logic does, without editing the real constants. On the robot {@link #fromConstants()} is always used, so the real
 * constants rule.
 *
 * <p>Words used here. A <b>tick</b> is one count of a motor's built-in encoder (a sensor that counts shaft turns);
 * speeds are in ticks per second. A <b>fraction</b> is a number from 0 to 1 (0.25 means a quarter of full power).
 */
public final class RobotCheckConfig {
    // ---- buttons
    /** Yes / continue. */
    public Button yes;
    /** No / there is a problem (also stops a running motor at once). */
    public Button no;
    /** Skip the step (never the safety step). */
    public Button skip;
    /** Do this part again. */
    public Button repeat;
    /** Finishes the gamepad step early. */
    public Button gamepadFinish;
    /** Summary scroll buttons. */
    public Button scrollUp;
    public Button scrollDown;
    /** The buttons the gamepad step asks the student to press once (the two triggers are always checked too). */
    public Button[] gamepadButtons;
    /** Presses closer than this to a screen change are ignored, in seconds. */
    public double lockoutSec;

    // ---- safety
    /** No power in any command is allowed above this fraction (0 to 1). */
    public double powerLimit;

    // ---- battery (volts)
    public double batteryPassVolts;
    public double batteryFailVolts;

    // ---- drive wheels
    public double drivePower;
    public double driveSec;

    // ---- Pinpoint (inches, degrees)
    public double pushDistanceIn;
    public double pushToleranceIn;
    public double pushSidewaysMaxIn;
    public double noMovementIn;
    public double turnDeg;
    public double turnToleranceDeg;
    public double noTurnDeg;
    public double turnDriftWarnIn;

    // ---- intake
    public double intakeMotorPower;
    public double intakeServoPower;
    public double leftServoSign;
    public double rightServoSign;
    public double intakeSec;

    // ---- flywheel (ticks per second; fractions of the target)
    public double flywheelTargetTps;
    public double flywheelTimeoutSec;
    public double flywheelNotTurningTps;
    public double atSpeedBelowFrac;
    public double atSpeedAboveFrac;
    public double atSpeedFloorTps;
    public double atSpeedRiseSec;
    public double atSpeedFallSec;

    // ---- feeder
    public double feederPower;
    public double feederForwardSec;
    public double feederReverseSec;
    /** The current value of Shooter.FEEDER_INVERTED, so the advice can say what to change it to. */
    public boolean feederInverted;

    // ---- Limelight
    public double limelightCheckSec;
    public int limelightMinNewFrames;
    public double limelightTagTimeoutSec;
    public int pipelineIndex;
    public int tagIdMin;
    public int tagIdMax;

    // ---- gamepad and summary
    public double gamepadTimeoutSec;
    public int summaryLinesPerPage;

    /** A Config with the real values from BioBuzzConstants. */
    public static RobotCheckConfig fromConstants() {
        RobotCheckConfig c = new RobotCheckConfig();
        c.yes = BioBuzzConstants.RobotCheck.YES_BUTTON;
        c.no = BioBuzzConstants.RobotCheck.NO_BUTTON;
        c.skip = BioBuzzConstants.RobotCheck.SKIP_BUTTON;
        c.repeat = BioBuzzConstants.RobotCheck.REPEAT_BUTTON;
        c.gamepadFinish = BioBuzzConstants.RobotCheck.GAMEPAD_FINISH_BUTTON;
        c.scrollUp = BioBuzzConstants.RobotCheck.SCROLL_UP_BUTTON;
        c.scrollDown = BioBuzzConstants.RobotCheck.SCROLL_DOWN_BUTTON;
        c.gamepadButtons = BioBuzzConstants.RobotCheck.GAMEPAD_BUTTONS_TO_CHECK.clone();
        c.lockoutSec = BioBuzzConstants.RobotCheck.BUTTON_LOCKOUT_SEC;
        c.powerLimit = BioBuzzConstants.RobotCheck.POWER_LIMIT;
        c.batteryPassVolts = BioBuzzConstants.RobotCheck.BATTERY_PASS_VOLTS;
        c.batteryFailVolts = BioBuzzConstants.RobotCheck.BATTERY_FAIL_VOLTS;
        c.drivePower = BioBuzzConstants.RobotCheck.DRIVE_TEST_POWER;
        c.driveSec = BioBuzzConstants.RobotCheck.DRIVE_TEST_SEC;
        c.pushDistanceIn = BioBuzzConstants.RobotCheck.PINPOINT_PUSH_DISTANCE_IN;
        c.pushToleranceIn = BioBuzzConstants.RobotCheck.PINPOINT_PUSH_TOLERANCE_IN;
        c.pushSidewaysMaxIn = BioBuzzConstants.RobotCheck.PINPOINT_PUSH_SIDEWAYS_MAX_IN;
        c.noMovementIn = BioBuzzConstants.RobotCheck.PINPOINT_NO_MOVEMENT_IN;
        c.turnDeg = BioBuzzConstants.RobotCheck.PINPOINT_TURN_DEG;
        c.turnToleranceDeg = BioBuzzConstants.RobotCheck.PINPOINT_TURN_TOLERANCE_DEG;
        c.noTurnDeg = BioBuzzConstants.RobotCheck.PINPOINT_NO_TURN_DEG;
        c.turnDriftWarnIn = BioBuzzConstants.RobotCheck.PINPOINT_TURN_DRIFT_WARN_IN;
        c.intakeMotorPower = BioBuzzConstants.RobotCheck.INTAKE_MOTOR_POWER;
        c.intakeServoPower = BioBuzzConstants.RobotCheck.INTAKE_SERVO_POWER;
        c.leftServoSign = BioBuzzConstants.RobotCheck.INTAKE_LEFT_SERVO_SIGN;
        c.rightServoSign = BioBuzzConstants.RobotCheck.INTAKE_RIGHT_SERVO_SIGN;
        c.intakeSec = BioBuzzConstants.RobotCheck.INTAKE_RUN_SEC;
        c.flywheelTargetTps = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
        c.flywheelTimeoutSec = BioBuzzConstants.RobotCheck.FLYWHEEL_TIMEOUT_SEC;
        c.flywheelNotTurningTps = BioBuzzConstants.RobotCheck.FLYWHEEL_NOT_TURNING_TPS;
        c.atSpeedBelowFrac = BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC;
        c.atSpeedAboveFrac = BioBuzzConstants.Shooter.AT_SPEED_ABOVE_FRAC;
        c.atSpeedFloorTps = BioBuzzConstants.Shooter.AT_SPEED_FLOOR_TPS;
        c.atSpeedRiseSec = BioBuzzConstants.Shooter.AT_SPEED_RISE_SEC;
        c.atSpeedFallSec = BioBuzzConstants.Shooter.AT_SPEED_FALL_SEC;
        c.feederPower = BioBuzzConstants.RobotCheck.FEEDER_TEST_POWER;
        c.feederForwardSec = BioBuzzConstants.RobotCheck.FEEDER_FORWARD_SEC;
        c.feederReverseSec = BioBuzzConstants.RobotCheck.FEEDER_REVERSE_SEC;
        c.feederInverted = BioBuzzConstants.Shooter.FEEDER_INVERTED;
        c.limelightCheckSec = BioBuzzConstants.RobotCheck.LIMELIGHT_CHECK_SEC;
        c.limelightMinNewFrames = BioBuzzConstants.RobotCheck.LIMELIGHT_MIN_NEW_FRAMES;
        c.limelightTagTimeoutSec = BioBuzzConstants.RobotCheck.LIMELIGHT_TAG_TIMEOUT_SEC;
        c.pipelineIndex = BioBuzzConstants.Vision.PIPELINE_INDEX;
        c.tagIdMin = BioBuzzConstants.RobotCheck.HIVE_TAG_ID_MIN;
        c.tagIdMax = BioBuzzConstants.RobotCheck.HIVE_TAG_ID_MAX;
        c.gamepadTimeoutSec = BioBuzzConstants.RobotCheck.GAMEPAD_TIMEOUT_SEC;
        c.summaryLinesPerPage = BioBuzzConstants.RobotCheck.SUMMARY_LINES_PER_PAGE;
        return c;
    }

    /**
     * Looks for values that cannot work and returns a plain-language message for each (empty list = all fine). The
     * OpMode shows these on the Driver Station and refuses to start the check, so a typo in the constants is found
     * before any motor turns.
     */
    public List<String> problems() {
        List<String> p = new ArrayList<String>();
        if (yes == null || no == null || skip == null || repeat == null || gamepadFinish == null
                || scrollUp == null || scrollDown == null || gamepadButtons == null) {
            p.add("RobotCheck: a button is missing in BioBuzzConstants.RobotCheck.");
            return p;
        }
        // The four answer buttons must be different, or one press would mean two things.
        Set<Button> answers = EnumSet.noneOf(Button.class);
        answers.add(yes);
        answers.add(no);
        answers.add(skip);
        answers.add(repeat);
        if (answers.size() != 4) {
            p.add("RobotCheck: YES_BUTTON, NO_BUTTON, SKIP_BUTTON and REPEAT_BUTTON must be four different buttons.");
        }
        if (answers.contains(gamepadFinish)) {
            p.add("RobotCheck: GAMEPAD_FINISH_BUTTON must not be the same as the yes / no / skip / repeat buttons.");
        }
        if (scrollUp == scrollDown) {
            p.add("RobotCheck: SCROLL_UP_BUTTON and SCROLL_DOWN_BUTTON must be different buttons.");
        }
        if (gamepadButtons.length == 0) {
            p.add("RobotCheck: GAMEPAD_BUTTONS_TO_CHECK is empty.");
        }
        for (Button b : gamepadButtons) {
            if (b == gamepadFinish) {
                p.add("RobotCheck: GAMEPAD_FINISH_BUTTON is also in GAMEPAD_BUTTONS_TO_CHECK, so the step could never ask for it.");
            }
        }
        if (lockoutSec < 0) {
            p.add("RobotCheck: BUTTON_LOCKOUT_SEC cannot be negative.");
        }
        if (powerLimit <= 0 || powerLimit > 1.0) {
            p.add("RobotCheck: POWER_LIMIT must be above 0 and at most 1.");
        }
        double[] powers = {drivePower, intakeMotorPower, intakeServoPower, feederPower};
        for (double power : powers) {
            if (power <= 0 || power > powerLimit) {
                p.add("RobotCheck: every test power (DRIVE_TEST_POWER, INTAKE_MOTOR_POWER, INTAKE_SERVO_POWER, FEEDER_TEST_POWER)"
                        + " must be above 0 and not above POWER_LIMIT.");
                break;
            }
        }
        if (Math.abs(leftServoSign) != 1.0 || Math.abs(rightServoSign) != 1.0) {
            p.add("RobotCheck: INTAKE_LEFT_SERVO_SIGN and INTAKE_RIGHT_SERVO_SIGN must each be 1 or -1.");
        }
        if (driveSec <= 0 || intakeSec <= 0 || feederForwardSec <= 0 || feederReverseSec <= 0
                || limelightCheckSec <= 0 || limelightTagTimeoutSec <= 0 || flywheelTimeoutSec <= 0
                || gamepadTimeoutSec <= 0) {
            p.add("RobotCheck: every time (seconds) must be above 0.");
        }
        if (batteryPassVolts <= 0 || batteryFailVolts <= 0 || batteryFailVolts >= batteryPassVolts) {
            p.add("RobotCheck: BATTERY_FAIL_VOLTS must be above 0 and below BATTERY_PASS_VOLTS.");
        }
        if (pushDistanceIn <= 0 || pushToleranceIn <= 0 || pushToleranceIn >= pushDistanceIn
                || pushSidewaysMaxIn <= 0 || noMovementIn <= 0 || noMovementIn >= pushDistanceIn - pushToleranceIn) {
            p.add("RobotCheck: the PINPOINT_PUSH_* and PINPOINT_NO_MOVEMENT_IN distances must be positive, the tolerance"
                    + " smaller than the push, and the 'nothing moved' distance smaller than the smallest good push.");
        }
        if (turnDeg <= 0 || turnToleranceDeg <= 0 || turnToleranceDeg >= turnDeg
                || noTurnDeg <= 0 || noTurnDeg >= turnDeg - turnToleranceDeg || turnDriftWarnIn <= 0) {
            p.add("RobotCheck: the PINPOINT_TURN_* angles and distances must be positive, the tolerance smaller than the"
                    + " turn, and the 'no turn' angle smaller than the smallest good turn.");
        }
        if (flywheelTargetTps <= 0 || flywheelNotTurningTps <= 0 || flywheelNotTurningTps >= flywheelTargetTps) {
            p.add("RobotCheck: the flywheel target must be above 0 and above FLYWHEEL_NOT_TURNING_TPS.");
        }
        if (limelightMinNewFrames < 1) {
            p.add("RobotCheck: LIMELIGHT_MIN_NEW_FRAMES must be at least 1.");
        }
        if (tagIdMin > tagIdMax) {
            p.add("RobotCheck: HIVE_TAG_ID_MIN must not be above HIVE_TAG_ID_MAX.");
        }
        if (summaryLinesPerPage < 4) {
            p.add("RobotCheck: SUMMARY_LINES_PER_PAGE must be at least 4.");
        }
        return p;
    }
}
