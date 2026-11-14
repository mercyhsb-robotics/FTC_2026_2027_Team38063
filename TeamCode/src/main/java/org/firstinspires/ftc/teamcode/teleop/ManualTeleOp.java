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
 * <p><b>Controls (Xbox-style gamepad 1; gamepad 2 can press the shooter and intake buttons too).</b>
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

    // The flywheel speed used for shooting, in ticks per second. Starts at the normal speed; D-pad up / down change it.
    private double shootTps = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
    // Were the three speed buttons down on the previous loop? Used to act once per press, not on every loop.
    private boolean prevFaster;
    private boolean prevSlower;
    private boolean prevNormal;

    /** Runs once at INIT: checks the constants and finds the hardware. */
    @Override
    public void init() {
        // Refuse a nonsense setting here, with a message that names it, instead of failing in the middle of a match.
        ConstantsCheck.Report report = ConstantsCheck.run();
        if (!report.ok()) {
            throw new IllegalStateException("BioBuzzConstants has errors: " + report.errors);
        }
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
        drive = new MecanumDrive();
        drive.init(hardwareMap);
        telemetry.addLine("Manual TeleOp ready. Press PLAY.");
    }

    /** Runs while waiting for PLAY: shows the buttons so nobody has to remember them. */
    @Override
    public void init_loop() {
        showButtons();
    }

    /** Runs over and over after PLAY: read the sticks and buttons, drive, shoot. */
    @Override
    public void loop() {
        // Slow mode: hold the slow button and every stick request is scaled down for careful aiming.
        boolean slow = GamepadButtons.pressed(gamepad1, BioBuzzConstants.Manual.SLOW_MODE_BUTTON);
        double scale = slow ? BioBuzzConstants.Manual.SLOW_MODE_SCALE : 1.0;
        // MecanumDrive wants the raw stick values (see MecanumDrive.drive for the signs).
        drive.drive(gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale, gamepad1.right_stick_x * scale);

        // Shooting speed: act once per press.
        boolean faster = pressedAny(BioBuzzConstants.Manual.SPEED_UP_BUTTON);
        boolean slower = pressedAny(BioBuzzConstants.Manual.SPEED_DOWN_BUTTON);
        boolean normal = pressedAny(BioBuzzConstants.Manual.SPEED_RESET_BUTTON);
        if (faster && !prevFaster) {
            shootTps = ManualRules.trimShootSpeed(shootTps, 1, BioBuzzConstants.Manual.SPEED_STEP_TPS,
                    BioBuzzConstants.Manual.SPEED_MIN_TPS, BioBuzzConstants.Manual.SPEED_MAX_TPS);
        }
        if (slower && !prevSlower) {
            shootTps = ManualRules.trimShootSpeed(shootTps, -1, BioBuzzConstants.Manual.SPEED_STEP_TPS,
                    BioBuzzConstants.Manual.SPEED_MIN_TPS, BioBuzzConstants.Manual.SPEED_MAX_TPS);
        }
        if (normal && !prevNormal) {
            shootTps = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
        }
        prevFaster = faster;
        prevSlower = slower;
        prevNormal = normal;

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
            doing = launcher.velocity() > BioBuzzConstants.Shooter.MANUAL_MIN_TPS ? "SHOOTING (feeding)" : "SHOOTING: wheel spinning up";
        } else if (mechanisms.intaking()) {
            doing = "INTAKE";
        } else {
            doing = "idle";
        }
        if (mechanisms.reversingRoller() || mechanisms.reversingFeelers()) {
            doing += "   REVERSING: " + (mechanisms.reversingRoller() ? "intake roller  " : "") + (mechanisms.reversingFeelers() ? "feelers" : "");
        }
        telemetry.addData("Now", doing);
        telemetry.addData("Shoot speed", String.format("%.0f ticks per second  (flywheel now %.0f)", shootTps, launcher.velocity()));
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
