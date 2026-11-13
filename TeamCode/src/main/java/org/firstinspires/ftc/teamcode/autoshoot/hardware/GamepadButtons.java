package org.firstinspires.ftc.teamcode.autoshoot.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.autoshoot.core.Button;

/** Reads a {@link Button} from the SDK's Gamepad, so the button assignments can live in constants. */
public final class GamepadButtons {
    private GamepadButtons() {
    }

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
            case CROSS:
                return g.cross;
            case CIRCLE:
                return g.circle;
            case SQUARE:
                return g.square;
            case TRIANGLE:
                return g.triangle;
            case LEFT_BUMPER:
                return g.left_bumper;
            case RIGHT_BUMPER:
                return g.right_bumper;
            case LEFT_STICK_BUTTON:
                return g.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return g.right_stick_button;
            case SHARE:
                return g.share;
            case OPTIONS:
                return g.options;
            case PS:
                return g.ps;
            case TOUCHPAD:
                return g.touchpad;
            default:
                return false;
        }
    }
}
