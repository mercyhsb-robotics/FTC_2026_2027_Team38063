package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.mecanumdrive.MecanumDrive;

/**
 * Mecanum drive test: drives the robot with the sticks using only the legacy {@code MecanumDrive} class (no Pedro,
 * no Pinpoint, no camera). It checks that all four wheels turn the right way.
 *
 * <p>When to run it: after wiring or changing the drive motors, or if the robot drives wrongly and you want to know
 * whether the problem is the basic motor setup. "Mecanum" wheels have angled rollers, so the robot can strafe
 * (move sideways) as well as drive forward and turn. An "OpMode" is one program the Driver Station can run; this one
 * appears in the TeleOp list as "MecanumDriveTest". Put the robot on blocks or in open space, press INIT, then PLAY.
 *
 * <p>Controls (gamepad 1, relative to the robot):
 * <ul>
 *   <li>Left stick up / down: drive forward / backward.</li>
 *   <li>Left stick right / left: strafe right / left.</li>
 *   <li>Right stick right / left: turn clockwise / counter-clockwise.</li>
 * </ul>
 * What you should see: the robot moves the way the sticks point. Telemetry shows the three raw stick values; note
 * that "forward" is NEGATIVE when the left stick is pushed up (that is how the SDK reports it, and
 * {@code MecanumDrive} expects it). If one wheel spins the wrong way the robot will twist or crab when you push
 * forward; fix the motor directions in {@code MecanumDrive.init()}.
 *
 * <p>Where it fits: it is a diagnostic. The robot's normal drive is Pedro (see {@code pedro/Constants.java}); the
 * {@code MecanumDrive} class is only its fallback.
 */
@TeleOp
public class MecanumDriveTest extends OpMode {
    // The legacy drive under test.
    MecanumDrive mecanumDrive = new MecanumDrive();
    // The latest raw stick values, each -1..1 (not changed in any way):
    // forward = left stick Y (negative when pushed up), strafe = left stick X (positive = right),
    // rotate = right stick X (positive = right, which turns clockwise).
    double forward;
    double strafe;
    double rotate;

    // Runs once when INIT is pressed: find the motors and set them up.
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
    }

    // Runs over and over after PLAY: read the sticks, show them, and drive.
    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);

        mecanumDrive.drive(forward, strafe, rotate);
    }
}
