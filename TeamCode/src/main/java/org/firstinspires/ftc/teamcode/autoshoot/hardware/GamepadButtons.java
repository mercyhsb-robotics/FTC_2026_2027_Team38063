package org.firstinspires.ftc.teamcode.autoshoot.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.autoshoot.core.Button;

/**
 * Reads a {@link Button} from the SDK's Gamepad, so the button assignments can live in constants.
 *
 * <p>This is the only place that maps the project's own {@link Button} enum to the FTC SDK's
 * {@code Gamepad} fields (the SDK calls them {@code dpad_up}, {@code a} and so on). Everything else asks
 * "is the shoot button held?" using {@code BioBuzzConstants.Controls}, and this class answers. Callers:
 * {@code BioBuzzRobot} and the Limelight Diagnostic OpMode.
 */
public final class GamepadButtons {
    private GamepadButtons() {
    }

    /**
     * True for as long as the button is held down. It is a level, not an "edge": it does NOT tell you the
     * button was just pressed this loop. A caller that wants "pressed once" remembers the previous loop's
     * value and compares (the TeleOp does this with {@code prevToggle}).
     *
     * @param g the gamepad to read (for example {@code gamepad1})
     * @param b which button to look at
     */
    public static boolean pressed(Gamepad g, Button b) {
        switch (b) {
            case DPAD_UP:
                return g.dpad_up;
            case DPAD_DOWN:
                return g.dpad_down;
            case DPAD_LEFT:
                return g.dpad_left;
            case DPAD_RIGHT:
                return g.dpad_right;
            case A:
                return g.a;
            case B:
                return g.b;
            case X:
                return g.x;
            case Y:
                return g.y;
            case LEFT_BUMPER:
                return g.left_bumper;
            case RIGHT_BUMPER:
                return g.right_bumper;
            case LEFT_STICK_BUTTON:
                return g.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return g.right_stick_button;
            case BACK:
                return g.back;
            case START:
                return g.start;
            case GUIDE:
                return g.guide;
            default:
                // Not reachable for the current Button values; "not pressed" is the safe answer for a new one.
                return false;
        }
    }
}
