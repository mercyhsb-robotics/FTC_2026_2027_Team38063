package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Button;
import org.firstinspires.ftc.teamcode.autoshoot.core.ConstantsCheck;
import org.firstinspires.ftc.teamcode.autoshoot.core.ManualRules;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckText;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.GamepadButtons;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.mecanumdrive.MecanumDrive;

/**
 * "Manual TeleOp": the simple driver-controlled OpMode for competition. The driver aims and shoots by hand. Nothing
 * automatic runs: no odometry, no camera, no auto-shoot. It needs only the drive motors, the shooter, the intake and
 * the Control Hub, so there is very little that can go wrong.
 *
 * <p><b>Controls (Xbox-style gamepad 1; gamepad 2 can press the shooter, intake, reverse and speed buttons too).</b>
 * <ul>
 *   <li><b>Left stick:</b> drive forward / back and slide left / right. <b>Right stick (left / right):</b> turn.</li>
 *   <li><b>Right trigger (hold): SHOOT.</b> The flywheel spins up and the feeder pushes POLLEN in once it is fast
 *       enough, both from this one button. <b>Let go and the flywheel brakes to a stop at once.</b></li>
 *   <li><b>Left trigger (hold): INTAKE.</b> The roller and both corner servos pull POLLEN in.</li>
 *   <li><b>Left bumper (hold): SLOW MODE.</b> Driving and turning at about a third of the speed, for careful aiming.</li>
 *   <li><b>D-pad up / down:</b> shooting speed a little faster / slower (shown on the screen). <b>D-pad left:</b> back to
 *       the normal speed.</li>
 *   <li><b>Reverse buttons (hold), one for each mechanism:</b> <b>right bumper</b> = indexer (feeder) slowly backward,
 *       <b>B</b> = shooter flywheel slowly backward (both clear a jam), <b>A</b> = intake roller backward,
 *       <b>X</b> = the two corner servos (the 'feelers') backward (these push a POLLEN out). Any combination works.</li>
 * </ul>
 * Priority: clearing a jam beats shooting, and shooting beats intaking.
 *
 * <p>Pick "Manual TeleOp" (group "0 Competition") in the Driver Station TeleOp list, press INIT, then PLAY. The
 * buttons and numbers are in {@code BioBuzzConstants} (groups {@code Controls}, {@code Manual} and {@code Shooter}).
 * The robot drives with the simple "legacy" mecanum drive ({@link MecanumDrive}), which needs no position tracking.
 * The OpMode "BioBuzzRobot" is the one with automatic shooting and parking.
 *
 * <p>At INIT it refuses to start only if a setting THIS OpMode uses is wrong (the manual controls: see
 * {@code ConstantsCheck.runManual}). Problems in settings it does not use (the camera, the field, the routes) are shown on
 * the screen, but do not stop it.
 *
 * <p>This class is wiring only: the shooter and intake rules are in {@link ManualMechanisms}.
 */
@TeleOp(name = "Manual TeleOp", group = "0 Competition")
public class ManualTeleOp extends OpMode {
    private MecanumDrive drive;
    private Launcher launcher;
    private Intake intake;
    private CRServo servoLeft;
    private CRServo servoRight;
    private ManualMechanisms mechanisms;
    // The whole constants check, only for showing warnings, placeholders and other problems at INIT.
    private ConstantsCheck.Report fullReport;

    // The flywheel speed used for shooting, in ticks per second. Starts at the normal speed; D-pad up / down change it.
    private double shootTps = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
    // One of these per button and per gamepad, so a press on either gamepad counts once, even if the other gamepad holds the same button.
    private final Press fasterOn1 = new Press();
    private final Press fasterOn2 = new Press();
    private final Press slowerOn1 = new Press();
    private final Press slowerOn2 = new Press();
    private final Press normalOn1 = new Press();
    private final Press normalOn2 = new Press();

    /** Remembers whether a button was down on the previous loop, so a press is acted on once and not on every loop it is held. */
    private static final class Press {
        private boolean previous;

        /** True only on the loop where the button goes from up to down. */
        boolean pressed(boolean down) {
            boolean edge = down && !previous;
            previous = down;
            return edge;
        }
    }

    /** Runs once at INIT: checks the constants and finds the hardware. */
    @Override
    public void init() {
        // Refuse a wrong manual-control setting here, with a message that names it, instead of failing in the middle of a match.
        ConstantsCheck.Report manual = ConstantsCheck.runManual();
        if (!manual.ok()) {
            throw new IllegalStateException("The manual-control settings in BioBuzzConstants have errors: " + manual.errors);
        }
        // The rest of the constants are checked for the screen only (they belong to the camera, field and route features).
        fullReport = ConstantsCheck.run();
        // Bulk caching: the hub sends all its readings in one batch instead of one slow request per reading.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);
        intake = new Intake();
        intake.init(hardwareMap);
        servoLeft = hardwareMap.get(CRServo.class, BioBuzzConstants.Hardware.LEFT_INTAKE_SERVO);
        servoRight = hardwareMap.get(CRServo.class, BioBuzzConstants.Hardware.RIGHT_INTAKE_SERVO);
        mechanisms = new ManualMechanisms(launcher, intake, servoLeft, servoRight);
        // The drive is kept in a local variable until init() has worked, so stop() never meets a half-built drive.
        MecanumDrive legacyDrive = new MecanumDrive();
        legacyDrive.init(hardwareMap);
        drive = legacyDrive;
    }

    /** Runs while waiting for PLAY: shows the checks and the buttons so nobody has to remember them. */
    @Override
    public void init_loop() {
        telemetry.addLine("Manual TeleOp ready. Press PLAY.");
        if (!fullReport.warnings.isEmpty()) {
            telemetry.addData("Constants WARNINGS", fullReport.warnings);
        }
        if (!fullReport.errors.isEmpty()) {
            telemetry.addData("Other constants problems (not used by this OpMode)", fullReport.errors);
        }
        telemetry.addData("Placeholders still to measure", fullReport.placeholders.size() + ": " + fullReport.placeholders);
        showButtons();
    }

    /** Runs over and over after PLAY: read the sticks and buttons, drive, shoot. */
    @Override
    public void loop() {
        // Slow mode: hold the slow button and every stick request is scaled down for careful aiming.
        boolean slow = pressedAny(BioBuzzConstants.Manual.SLOW_MODE_BUTTON);
        double scale = slow ? BioBuzzConstants.Manual.SLOW_MODE_SCALE : 1.0;
        // MecanumDrive wants the raw stick values (see MecanumDrive.drive for the signs).
        drive.drive(gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale, gamepad1.right_stick_x * scale);

        // Shooting speed: each press on either gamepad counts once.
        boolean faster = fasterOn1.pressed(GamepadButtons.pressed(gamepad1, BioBuzzConstants.Manual.SPEED_UP_BUTTON))
                | fasterOn2.pressed(GamepadButtons.pressed(gamepad2, BioBuzzConstants.Manual.SPEED_UP_BUTTON));
        boolean slower = slowerOn1.pressed(GamepadButtons.pressed(gamepad1, BioBuzzConstants.Manual.SPEED_DOWN_BUTTON))
                | slowerOn2.pressed(GamepadButtons.pressed(gamepad2, BioBuzzConstants.Manual.SPEED_DOWN_BUTTON));
        boolean normal = normalOn1.pressed(GamepadButtons.pressed(gamepad1, BioBuzzConstants.Manual.SPEED_RESET_BUTTON))
                | normalOn2.pressed(GamepadButtons.pressed(gamepad2, BioBuzzConstants.Manual.SPEED_RESET_BUTTON));
        if (faster) {
            shootTps = ManualRules.trimShootSpeed(shootTps, 1, BioBuzzConstants.Manual.SPEED_STEP_TPS,
                    BioBuzzConstants.Manual.SPEED_MIN_TPS, BioBuzzConstants.Manual.SPEED_MAX_TPS);
        }
        if (slower) {
            shootTps = ManualRules.trimShootSpeed(shootTps, -1, BioBuzzConstants.Manual.SPEED_STEP_TPS,
                    BioBuzzConstants.Manual.SPEED_MIN_TPS, BioBuzzConstants.Manual.SPEED_MAX_TPS);
        }
        if (normal) {
            shootTps = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
        }

        // The shooter and the intake (see ManualMechanisms for the rules).
        mechanisms.update(gamepad1, gamepad2, shootTps);

        report(slow);
    }

    /** Runs once at STOP: everything off, the flywheel brakes. */
    @Override
    public void stop() {
        if (mechanisms != null) {
            mechanisms.stop();
        }
        if (drive != null) {
            drive.drive(0.0, 0.0, 0.0);
        }
    }

    // True if the button is held on either gamepad.
    private boolean pressedAny(Button b) {
        return GamepadButtons.pressed(gamepad1, b) || GamepadButtons.pressed(gamepad2, b);
    }

    /** The Driver Station screen while driving: what the shooter is doing and how fast it will shoot. */
    private void report(boolean slow) {
        String doing;
        if (mechanisms.unjammingIndexer() || mechanisms.unjammingShooter()) {
            doing = "CLEARING A JAM: " + (mechanisms.unjammingIndexer() ? "indexer BACKWARD (slow)  " : "")
                    + (mechanisms.unjammingShooter() ? "shooter wheel BACKWARD (slow)" : "");
        } else if (mechanisms.shooting()) {
            doing = mechanisms.feeding() ? "SHOOTING (feeding)" : "SHOOTING: wheel spinning up";
        } else if (mechanisms.intaking()) {
            doing = "INTAKE";
        } else {
            doing = "idle";
        }
        if (mechanisms.reversingRoller() || mechanisms.reversingFeelers()) {
            doing += "   REVERSING: " + (mechanisms.reversingRoller() ? "intake roller  " : "") + (mechanisms.reversingFeelers() ? "feelers" : "");
        }
        telemetry.addData("Now", doing);
        telemetry.addData("Shoot speed", String.format("%.0f ticks per second  (flywheel now %.0f, feeds above %.0f)", shootTps,
                mechanisms.flywheelTps(), ManualMechanisms.feedAboveTps(shootTps)));
        telemetry.addData("Drive", slow ? "SLOW MODE" : "normal");
        showButtons();
    }

    // The button names come from the settings, so this text is right even if a button is changed in BioBuzzConstants.
    private void showButtons() {
        telemetry.addLine("RIGHT TRIGGER (hold) = SHOOT   LEFT TRIGGER (hold) = INTAKE   "
                + RobotCheckText.button(BioBuzzConstants.Manual.SLOW_MODE_BUTTON) + " (hold) = slow drive");
        telemetry.addLine(RobotCheckText.button(BioBuzzConstants.Manual.SPEED_UP_BUTTON) + " / "
                + RobotCheckText.button(BioBuzzConstants.Manual.SPEED_DOWN_BUTTON) + " = shoot speed faster / slower   "
                + RobotCheckText.button(BioBuzzConstants.Manual.SPEED_RESET_BUTTON) + " = normal speed");
        telemetry.addLine("Reverse (hold): " + RobotCheckText.button(BioBuzzConstants.Controls.UNJAM_INDEXER_BUTTON)
                + " = indexer   " + RobotCheckText.button(BioBuzzConstants.Controls.UNJAM_SHOOTER_BUTTON)
                + " = shooter wheel   " + RobotCheckText.button(BioBuzzConstants.Controls.REVERSE_INTAKE_BUTTON)
                + " = intake roller   " + RobotCheckText.button(BioBuzzConstants.Controls.REVERSE_FEELERS_BUTTON) + " = feelers");
    }
}
