package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * The gamepad buttons the driver controls can be assigned to. The assignments live in
 * {@code BioBuzzConstants.Controls}; the hardware adapter turns a {@code Button} into the matching
 * field of the SDK's Gamepad. Names follow the PlayStation labels.
 *
 * <p>Why an enum instead of reading {@code gamepad1.dpad_up} directly? So this package stays free of the
 * FTC SDK (it can run in PC unit tests) and so a driver can change which button does what by editing one
 * constant instead of the logic. {@code GamepadButtons.pressed} is the only place that maps a Button to the SDK.
 */
public enum Button {
    // The four arrows of the direction pad.
    DPAD_UP,
    DPAD_DOWN,
    DPAD_LEFT,
    DPAD_RIGHT,
    // The four face buttons (right-hand cluster).
    CROSS,
    CIRCLE,
    SQUARE,
    TRIANGLE,
    // The two shoulder buttons above the triggers (the triggers themselves are analog and are not Buttons).
    LEFT_BUMPER,
    RIGHT_BUMPER,
    // Pressing a stick straight down like a button.
    LEFT_STICK_BUTTON,
    RIGHT_STICK_BUTTON,
    // The small buttons in the middle of the pad; the touchpad also works as one big button.
    SHARE,
    OPTIONS,
    PS,
    TOUCHPAD
}
