package org.firstinspires.ftc.teamcode.autoshoot.run;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Button;
import org.firstinspires.ftc.teamcode.autoshoot.core.ConstantsCheck;

/**
 * Checks the numbers of {@code BioBuzzConstants.Run} (the INIT menu buttons and the tolerances and time-outs of the route runner) for
 * values that cannot work, the same way {@link ConstantsCheck} checks the rest: errors stop the Auto Route Picker at INIT with a message that
 * names the constant, warnings are shown, nothing is ever changed. It is a separate class so the runner's checks live next to the runner.
 * The result has the same shape as {@link ConstantsCheck.Report} so the OpMode can show both in one list.
 */
public final class RunSettingsCheck {
    private RunSettingsCheck() {
    }

    /** Checks the values in {@link BioBuzzConstants.Run} (and the few other constants the runner leans on). */
    public static ConstantsCheck.Report run() {
        ConstantsCheck.Report r = new ConstantsCheck.Report();
        checkMenuButtons(r, BioBuzzConstants.Run.MENU_PREVIOUS_ROUTE_BUTTON, BioBuzzConstants.Run.MENU_NEXT_ROUTE_BUTTON,
                BioBuzzConstants.Run.MENU_RED_BUTTON, BioBuzzConstants.Run.MENU_BLUE_BUTTON);
        if (BioBuzzConstants.Run.MENU_MAX_ISSUE_LINES < 1 || BioBuzzConstants.Run.MENU_MAX_PLACEHOLDER_NAMES < 1) {
            r.errors.add("Run.MENU_MAX_ISSUE_LINES and Run.MENU_MAX_PLACEHOLDER_NAMES must be at least 1, or the INIT screen would show nothing.");
        }
        checkClock(r, BioBuzzConstants.Run.CLOCK_STOP_SEC, BioBuzzConstants.Auto.AUTO_DURATION_SEC);
        checkMoves(r, BioBuzzConstants.Run.ARRIVE_POS_TOL_IN, BioBuzzConstants.Run.ARRIVE_LOOSE_IN, BioBuzzConstants.Run.ARRIVE_SPEED_IN_S,
                BioBuzzConstants.Run.ARRIVE_LOOSE_STILL_SEC, BioBuzzConstants.Run.TURN_DONE_DEG, BioBuzzConstants.Run.PRE_TURN_MIN_DEG,
                BioBuzzConstants.Run.TURN_TIMEOUT_SEC, BioBuzzConstants.Run.MOVE_TIMEOUT_SPEED_IN_S);
        checkIntake(r, BioBuzzConstants.Run.INTAKE_LEFT_SERVO_POWER, BioBuzzConstants.Run.INTAKE_RIGHT_SERVO_POWER,
                BioBuzzConstants.Run.FLOWER_LINEUP_TOL_IN, BioBuzzConstants.Run.FLOWER_START_TOL_IN);
        checkShootStep(r, BioBuzzConstants.Run.SHOOT_STEP_TIMEOUT_SEC, BioBuzzConstants.Shooter.VOLLEY_MAX_SEC, BioBuzzConstants.AutoShoot.SPINUP_TIMEOUT_SEC);
        return r;
    }

    /** The four menu buttons must be four different buttons: one press may only mean one thing. */
    static void checkMenuButtons(ConstantsCheck.Report r, Button previous, Button next, Button red, Button blue) {
        Button[] all = {previous, next, red, blue};
        String[] names = {"MENU_PREVIOUS_ROUTE_BUTTON", "MENU_NEXT_ROUTE_BUTTON", "MENU_RED_BUTTON", "MENU_BLUE_BUTTON"};
        for (int i = 0; i < all.length; i++) {
            for (int j = i + 1; j < all.length; j++) {
                if (all[i] == all[j]) {
                    r.errors.add("Run." + names[i] + " and Run." + names[j] + " are both " + all[i] + ": one button cannot do two things in the INIT menu.");
                }
            }
        }
    }

    /** The route clock may stop the run earlier than the AUTO period, never later. */
    static void checkClock(ConstantsCheck.Report r, double clockStopSec, double autoSec) {
        if (!(clockStopSec > 0.0) || clockStopSec > autoSec) {
            r.errors.add("Run.CLOCK_STOP_SEC (" + clockStopSec + " s) must be above 0 and no more than the AUTO period (" + autoSec + " s).");
        }
    }

    /** Tolerances and time-outs of a move: positive, in a sensible order. */
    static void checkMoves(ConstantsCheck.Report r, double posTolIn, double looseIn, double speedInS, double looseStillSec, double turnDoneDeg,
                           double preTurnMinDeg, double turnTimeoutSec, double timeoutSpeedInS) {
        if (!(posTolIn > 0.0) || !(speedInS > 0.0) || !(turnDoneDeg > 0.0) || !(looseStillSec > 0.0)) {
            r.errors.add("Run.ARRIVE_POS_TOL_IN, ARRIVE_SPEED_IN_S, ARRIVE_LOOSE_STILL_SEC and TURN_DONE_DEG must all be above 0, or a move could never count as arrived.");
        }
        if (looseIn < posTolIn) {
            r.errors.add("Run.ARRIVE_LOOSE_IN (" + looseIn + " in) must be at least Run.ARRIVE_POS_TOL_IN (" + posTolIn + " in): it is the looser of the two.");
        }
        // A turn before a leg that ends within TURN_DONE_DEG must not be one that PRE_TURN_MIN_DEG would start again, or the robot would turn forever.
        if (preTurnMinDeg <= turnDoneDeg) {
            r.errors.add("Run.PRE_TURN_MIN_DEG (" + preTurnMinDeg + ") must be more than Run.TURN_DONE_DEG (" + turnDoneDeg
                    + "): a turn that is finished must not be big enough to start another one.");
        }
        if (!(turnTimeoutSec > 0.0) || !(timeoutSpeedInS > 0.0)) {
            r.errors.add("Run.TURN_TIMEOUT_SEC and Run.MOVE_TIMEOUT_SPEED_IN_S must be above 0: a move that is stuck must give up some day.");
        }
    }

    /** The corner servos run between -1 and 1 and must actually turn; the FLOWER tolerances must be positive. */
    static void checkIntake(ConstantsCheck.Report r, double leftPower, double rightPower, double lineUpTolIn, double startTolIn) {
        if (Math.abs(leftPower) > 1.0 || Math.abs(rightPower) > 1.0 || leftPower == 0.0 || rightPower == 0.0) {
            r.errors.add("Run.INTAKE_LEFT_SERVO_POWER and INTAKE_RIGHT_SERVO_POWER must be between -1 and 1 and not 0 (they are the powers the TeleOp triggers give).");
        }
        if (!(lineUpTolIn > 0.0) || !(startTolIn > 0.0)) {
            r.errors.add("Run.FLOWER_LINEUP_TOL_IN and Run.FLOWER_START_TOL_IN must be above 0.");
        }
    }

    /** A shoot step must be allowed to last for the longest volley plus the spin-up wait, or a good volley is cut short. */
    static void checkShootStep(ConstantsCheck.Report r, double stepTimeoutSec, double volleyMaxSec, double spinupTimeoutSec) {
        if (!(stepTimeoutSec > 0.0)) {
            r.errors.add("Run.SHOOT_STEP_TIMEOUT_SEC must be above 0.");
        } else if (stepTimeoutSec < volleyMaxSec + spinupTimeoutSec) {
            r.warnings.add("Run.SHOOT_STEP_TIMEOUT_SEC (" + stepTimeoutSec + " s) is shorter than Shooter.VOLLEY_MAX_SEC + AutoShoot.SPINUP_TIMEOUT_SEC ("
                    + (volleyMaxSec + spinupTimeoutSec) + " s): a slow volley could be cut off by the step time-out before the shooting sequence's own limits.");
        }
    }
}
