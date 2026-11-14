package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Button;
import org.firstinspires.ftc.teamcode.autoshoot.core.ManualRules;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.GamepadButtons;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;

/**
 * The shooter and the intake under the driver's hands: the one place that decides what the flywheel, the feeder,
 * the intake roller and the two corner servos do for each button. Both TeleOps ("Manual TeleOp" and "BioBuzzRobot")
 * call {@link #update} once per loop, so the buttons mean the same thing in both.
 *
 * <p><b>The rules (the same every loop).</b>
 * <ul>
 *   <li><b>Shoot = hold the RIGHT TRIGGER.</b> The flywheel spins up to the shooting speed and the feeder pushes
 *       POLLEN into it, but only once the flywheel is fast enough ({@code Shooter.MANUAL_MIN_TPS}), so the first
 *       ball is never fed too early. <b>Let go and the flywheel brakes to a stop at once and the feeder stops</b>
 *       (a coasting flywheel once threw out a POLLEN nobody meant to shoot).</li>
 *   <li><b>Intake = hold the LEFT TRIGGER.</b> The front roller and both corner servos pull POLLEN in. Let go and
 *       they stop.</li>
 *   <li><b>Reverse, every mechanism.</b> Hold a button and that mechanism runs BACKWARD (settings in
 *       {@code BioBuzzConstants.Controls}): the <b>indexer</b> (feeder) and the <b>shooter flywheel</b> run slowly backward to clear a
 *       jam; the <b>intake roller</b> and the two corner servos (the <b>feelers</b>) run backward to push a POLLEN out. Any combination works.</li>
 *   <li><b>Priority:</b> clearing a jam beats shooting, and shooting beats pulling in. The robot never takes POLLEN
 *       in while it is shooting or clearing a jam (it may hold at most 4 POLLEN). Pushing POLLEN out works at any time.</li>
 * </ul>
 * A second gamepad (the "operator") can press the same buttons: either gamepad can shoot, intake or clear a jam.
 * The sticks are read by the OpMode, not here.
 *
 * <p>Words: a <b>trigger</b> is the analog button under each index finger (it counts as pressed past
 * {@code Controls.TRIGGER_THRESHOLD}); the <b>feeder</b> is the servo that pushes the next POLLEN into the flywheel;
 * <b>ticks per second</b> (tps) is how fast the flywheel's encoder counts.
 */
final class ManualMechanisms {
    private final Launcher launcher;
    private final Intake intake;
    // The two corner-pulling intake servos (continuous-rotation servos: setPower gives speed and direction).
    private final CRServo servoLeft;
    private final CRServo servoRight;

    // What the last update() decided, for the Driver Station screen.
    private boolean shooting;
    private boolean intaking;
    private boolean reversingRoller;
    private boolean reversingFeelers;
    private boolean unjammingIndexer;
    private boolean unjammingShooter;

    ManualMechanisms(Launcher launcher, Intake intake, CRServo servoLeft, CRServo servoRight) {
        this.launcher = launcher;
        this.intake = intake;
        this.servoLeft = servoLeft;
        this.servoRight = servoRight;
    }

    /**
     * Reads the buttons and drives the mechanisms. Call once per loop.
     *
     * @param driver gamepad 1
     * @param operator gamepad 2 (may be unused: a gamepad that is not connected simply presses nothing)
     * @param shootTps the flywheel speed to shoot at, in ticks per second
     */
    void update(Gamepad driver, Gamepad operator, double shootTps) {
        double threshold = BioBuzzConstants.Controls.TRIGGER_THRESHOLD;
        boolean shootAsked = driver.right_trigger > threshold || (operator != null && operator.right_trigger > threshold);
        boolean intakeAsked = driver.left_trigger > threshold || (operator != null && operator.left_trigger > threshold);
        unjammingIndexer = held(driver, operator, BioBuzzConstants.Controls.UNJAM_INDEXER_BUTTON);
        unjammingShooter = held(driver, operator, BioBuzzConstants.Controls.UNJAM_SHOOTER_BUTTON);
        // The priority rules (jam beats shoot beats intake) are plain functions in ManualRules, so they are unit-tested.
        boolean rollerReverseAsked = held(driver, operator, BioBuzzConstants.Controls.REVERSE_INTAKE_BUTTON);
        boolean feelerReverseAsked = held(driver, operator, BioBuzzConstants.Controls.REVERSE_FEELERS_BUTTON);
        ManualRules.Decision d = ManualRules.decide(shootAsked, intakeAsked, unjammingIndexer, unjammingShooter,
                rollerReverseAsked, feelerReverseAsked);
        shooting = d.shoot;
        intaking = d.pullingIn();
        reversingRoller = d.rollerDirection < 0;
        reversingFeelers = d.feelerDirection < 0;

        if (d.reverseShooter) {
            // The flywheel runs slowly backward (the launcher forgets any forward speed first).
            launcher.setTargetVelocity(0.0);
            launcher.reverseFlywheel(true);
            if (d.reverseIndexer) {
                launcher.feederReverse();
            } else {
                launcher.feederStop();
            }
        } else {
            launcher.reverseFlywheel(false);
            if (d.reverseIndexer) {
                launcher.setTargetVelocity(0.0);
                launcher.feederReverse();
            } else if (d.shoot) {
                // Spin up and feed together: the feeder only runs while the flywheel is fast enough.
                launcher.setTargetVelocity(shootTps);
                launcher.setFeederOn(launcher.velocity() > BioBuzzConstants.Shooter.MANUAL_MIN_TPS);
            } else {
                // Nothing asked for: the flywheel brakes to a stop (see Launcher.setTargetVelocity) and the feeder stops.
                launcher.setTargetVelocity(0.0);
                launcher.setFeederOn(false);
            }
        }

        // The intake roller: pull in, push out, or off.
        if (d.rollerDirection > 0) {
            intake.startIntake();
        } else if (d.rollerDirection < 0) {
            intake.reverseIntake(BioBuzzConstants.Controls.INTAKE_REVERSE_POWER);
        } else {
            intake.stopIntake();
        }
        // The two corner servos (feelers). They are mirror images, so "pull in" is a different sign on each side; pushing out is
        // the opposite of that sign at the reverse power.
        double feelerPower = d.feelerDirection > 0 ? 1.0 : (d.feelerDirection < 0 ? -BioBuzzConstants.Controls.FEELER_REVERSE_POWER : 0.0);
        servoLeft.setPower(feelerPower * BioBuzzConstants.Run.INTAKE_LEFT_SERVO_POWER);
        servoRight.setPower(feelerPower * BioBuzzConstants.Run.INTAKE_RIGHT_SERVO_POWER);
    }

    /** Stops everything (the flywheel brakes). For the end of the OpMode. */
    void stop() {
        launcher.stop();
        intake.stopIntake();
        servoLeft.setPower(0.0);
        servoRight.setPower(0.0);
        shooting = false;
        intaking = false;
        reversingRoller = false;
        reversingFeelers = false;
        unjammingIndexer = false;
        unjammingShooter = false;
    }

    /** True while the shoot trigger is held and nothing with higher priority is active. */
    boolean shooting() {
        return shooting;
    }

    /** True while the intake is running. */
    boolean intaking() {
        return intaking;
    }

    /** True while the intake roller is being run backward (pushing POLLEN out). */
    boolean reversingRoller() {
        return reversingRoller;
    }

    /** True while the two corner servos (the feelers) are being run backward (pushing POLLEN out). */
    boolean reversingFeelers() {
        return reversingFeelers;
    }

    /** True while the indexer (feeder) is being run slowly backward by hand. */
    boolean unjammingIndexer() {
        return unjammingIndexer;
    }

    /** True while the shooter flywheel is being run slowly backward by hand. */
    boolean unjammingShooter() {
        return unjammingShooter;
    }

    // True if the button is held on the driver's gamepad or on the operator's.
    private static boolean held(Gamepad driver, Gamepad operator, Button b) {
        return GamepadButtons.pressed(driver, b) || (operator != null && GamepadButtons.pressed(operator, b));
    }
}
