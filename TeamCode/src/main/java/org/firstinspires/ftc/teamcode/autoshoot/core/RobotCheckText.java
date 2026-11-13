package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Small helpers that turn numbers and button names into the plain words shown on the Driver Station by the Robot
 * Check. Plain Java, no FTC SDK.
 *
 * <p>Why a separate class: every screen needs the same few things (a number with one decimal, "cross (X)" instead of
 * CROSS, the name of a wheel), and keeping them in one place keeps the wording the same everywhere.
 */
public final class RobotCheckText {
    private RobotCheckText() {
    }

    /** A number with no decimals, for example 1190 (ticks per second, whole degrees). */
    public static String f0(double value) {
        return String.format(Locale.US, "%.0f", value);
    }

    /** A number with one decimal, for example 11.8 (inches, volts, seconds). */
    public static String f1(double value) {
        return String.format(Locale.US, "%.1f", value);
    }

    /** A button the way a PlayStation controller labels it, with the plain name first: "cross (X)", "left bumper (L1)". */
    public static String button(Button b) {
        switch (b) {
            case CROSS:
                return "cross (X)";
            case CIRCLE:
                return "circle (O)";
            case SQUARE:
                return "square";
            case TRIANGLE:
                return "triangle";
            case DPAD_UP:
                return "D-pad up";
            case DPAD_DOWN:
                return "D-pad down";
            case DPAD_LEFT:
                return "D-pad left";
            case DPAD_RIGHT:
                return "D-pad right";
            case LEFT_BUMPER:
                return "left bumper (L1)";
            case RIGHT_BUMPER:
                return "right bumper (R1)";
            case LEFT_STICK_BUTTON:
                return "left stick button (L3)";
            case RIGHT_STICK_BUTTON:
                return "right stick button (R3)";
            case SHARE:
                return "share";
            case OPTIONS:
                return "options";
            case PS:
                return "PS button";
            case TOUCHPAD:
                return "touchpad (the big pad in the middle)";
            default:
                return b.name();
        }
    }

    /** The name of wheel number {@code index} (0 front-left, 1 back-left, 2 front-right, 3 back-right), as a lower-case phrase. */
    public static String wheel(int index) {
        switch (index) {
            case 0:
                return "front-left";
            case 1:
                return "back-left";
            case 2:
                return "front-right";
            default:
                return "back-right";
        }
    }

    /** The Pedro setting that holds the direction of wheel number {@code index} (in pedro/Constants.java, drivetrainConfig). */
    public static String wheelPedroSetting(int index) {
        switch (index) {
            case 0:
                return "c.frontLeftDirection";
            case 1:
                return "c.backLeftDirection";
            case 2:
                return "c.frontRightDirection";
            default:
                return "c.backRightDirection";
        }
    }

    /** The variable that holds the direction of wheel number {@code index} in the backup drive (MecanumDrive.init()). */
    public static String wheelLegacySetting(int index) {
        switch (index) {
            case 0:
                return "frontLeftMotor";
            case 1:
                return "backLeftMotor";
            case 2:
                return "frontRightMotor";
            default:
                return "backRightMotor";
        }
    }

    /** Joins the parts with the separator between them (Java 8 has no String.join for lists of our own, so this is the small version). */
    public static String join(List<String> parts, String separator) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < parts.size(); i++) {
            if (i > 0) {
                sb.append(separator);
            }
            sb.append(parts.get(i));
        }
        return sb.toString();
    }

    /** Splits a text with line breaks into its lines (an empty text gives an empty list). */
    public static List<String> lines(String text) {
        List<String> out = new ArrayList<String>();
        if (text == null || text.isEmpty()) {
            return out;
        }
        for (String line : text.split("\n")) {
            out.add(line);
        }
        return out;
    }
}
