package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;

/**
 * Launcher test: lets you run the flywheel and the feeder by hand, with no driving and no automatic shooting, to
 * check the shooter's motors and directions. The flywheel is the fast-spinning wheel that throws a POLLEN (a ball) out
 * the back of the robot; the feeder is the servo that pushes a POLLEN into it.
 *
 * <p>When to run it: after wiring or changing the flywheel or the feeder servo, or when the shooter does nothing in
 * the TeleOp. For measuring flywheel speed against distance use "Shooter Tuner" instead. An "OpMode" is one program
 * the Driver Station can run; this one appears in the TeleOp list as "LauncherTest". Press INIT, then PLAY. Nothing
 * is shown in telemetry: watch and listen to the robot (keep hands and loose objects clear of the flywheel).
 *
 * <p>Controls (gamepad 1). There is no "release to stop": each command stays until another one replaces it, so
 * press X or D-pad down to stop everything.
 * <ul>
 *   <li>Y (hold): flywheel to the manual speed ({@code BioBuzzConstants.Shooter.MANUAL_TARGET_TPS}) and the
 *       feeder runs once the flywheel is faster than {@code MANUAL_MIN_TPS}.</li>
 *   <li>D-pad up: flywheel to the manual speed (ticks per second), feeder untouched.</li>
 *   <li>D-pad left: feeder backward (clears a jam). D-pad right: feeder forward (pushes a POLLEN into the
 *       flywheel).</li>
 *   <li>X or D-pad down: stop the flywheel and the feeder.</li>
 * </ul>
 * What you should see: the flywheel spins up (the exit side is the BACK of the robot) and a POLLEN pushed by the
 * feeder is thrown out the back. If the feeder pushes the wrong way, the fix is {@code Shooter.FEEDER_INVERTED}.
 */
@TeleOp
public class LauncherTest extends OpMode {
    // The shooter under test (flywheel motor and feeder servo).
    Launcher launcher;

    // Runs once when INIT is pressed: find and set up the flywheel and the feeder.
    @Override
    public void init() {
        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);
    }

    // Runs over and over after PLAY: each button simply sends one command to the launcher.
    @Override
    public void loop() {
        if (gamepad1.y) {
            launcher.launch(true);
        }

        if (gamepad1.dpad_up) {
            // (increaseRpm takes ticks per second, despite its name)
            launcher.increaseRpm(BioBuzzConstants.Shooter.MANUAL_TARGET_TPS);
        }

        if (gamepad1.dpad_down) {
            launcher.stop();
        }

        if (gamepad1.dpad_left) {
            launcher.feederReverse();
        }

        if (gamepad1.dpad_right) {
            launcher.feederForward();
        }

        if (gamepad1.x) {
            launcher.stop();
        }
    }
}

