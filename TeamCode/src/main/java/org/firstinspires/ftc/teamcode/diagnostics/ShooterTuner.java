package org.firstinspires.ftc.teamcode.diagnostics;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.FlywheelReady;
import org.firstinspires.ftc.teamcode.autoshoot.core.ShotCounter;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;

/**
 * Builds the shot table. Park the robot at a known distance from the HIVE, dial in a flywheel speed, fire a
 * ball, note where it went, and record the row when the speed scores.
 *
 * <p>Measuring the range: use a tape measure from the BALL EXIT (back face of the robot, unless
 * LAUNCH_POINT_FORWARD_IN says otherwise) to the foot line of the HIVE frame at the end you are shooting
 * into, then ADD 3.0 in (the aim point is 3.0 in inside the foot line). Example: the robot front against the
 * audience wall in line with the HIVE is 37.7 in, the team's one measured good shot.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *   <li>D-pad up / down: flywheel target +/- 10 ticks per second (hold the left bumper for +/- 50);</li>
 *   <li>D-pad right / left: recorded range +/- 0.5 in (hold the left bumper for +/- 2 in);</li>
 *   <li>cross: start / stop the flywheel;</li>
 *   <li>circle (hold): run the feeder; the dip each ball causes is measured;</li>
 *   <li>triangle: record a table row with the current range and target (also written to the robot log);</li>
 *   <li>square: forget the recorded rows.</li>
 * </ul>
 * The dip numbers tell you how to set SHOT_DIP_FRAC and AT_SPEED_*: SHOT_DIP_FRAC should be about half the
 * smallest dip you see, and the feeder should wait until the speed has recovered.
 *
 * <p><b>Why a shot table.</b> The shooter has one fixed launch angle, so the only way to reach the HIVE from a
 * different distance is to change how fast the <b>flywheel</b> (the fast-spinning wheel that throws the POLLEN)
 * turns. The shot table lists, for several distances, the flywheel speed that scores. Speeds are in encoder
 * <b>ticks per second</b> ("tps"; a tick is one count of the motor's encoder). The <b>feeder</b> is the servo that
 * pushes a POLLEN into the flywheel. This tool gets the numbers you paste into
 * {@code BioBuzzConstants.Shooter.SHOT_TABLE} (see docs/TUNING.md, section 2).
 *
 * <p><b>When to run it and what to do.</b> Use it when the shot table is new or no longer fits (a new flywheel, a new
 * PIDF, a changed launch point). It is in the Driver Station TeleOp list as "Shooter Tuner" (group "AutoShoot").
 * Put the robot at a measured range, load a POLLEN, press INIT then PLAY, start the flywheel with cross, wait for
 * "at speed: true", and hold circle to feed one ball. Keep hands and loose objects clear of the flywheel and the
 * back of the robot, where the POLLEN is thrown. The robot does not drive.
 *
 * <p><b>What the screen shows.</b> The flywheel state, target and measured speed (tps) and whether it is "at
 * speed"; the speed window that counts as "at speed" (lowest to highest tps); the range that will be recorded
 * (inches, ball exit to HIVE foot line, plus 3.0); how many shots the speed dips counted; the dip threshold (a
 * ball is counted when the speed falls below it); the lowest speed during the last feed and how deep the dip
 * was (percent of the target); and the rows recorded so far, as {@code {range_in, speed_tps},} text that can be
 * pasted straight into the table.
 */
@TeleOp(name = "Shooter Tuner", group = "AutoShoot")
public class ShooterTuner extends OpMode {
    // The shooter under test (flywheel motor and feeder servo).
    private Launcher launcher;
    // Decides when the flywheel counts as "at speed": its speed is inside a window around the target for long enough.
    private FlywheelReady ready;
    // Counts the balls fired by watching the flywheel speed dip each time a ball is pulled through.
    private ShotCounter counter;

    // The flywheel speed being tried, in ticks per second. Starts at the manual launch speed.
    private double targetTps = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
    // The range, in inches, that a recorded row will be labelled with. Starts at the measured wall shot.
    private double rangeIn = BioBuzzConstants.Shooter.TUNER_START_RANGE_IN;
    // True while the flywheel is switched on (cross toggles it).
    private boolean running;
    // System.nanoTime() (nanoseconds) at PLAY; the logic clock counts from here.
    private long startNanos;

    // Each button's state on the previous loop. A button acts only on the loop where it goes down (compare with
    // the current state), so holding it does not repeat the action every loop.
    private boolean prevUp;
    private boolean prevDown;
    private boolean prevLeft;
    private boolean prevRight;
    private boolean prevCross;
    private boolean prevTriangle;
    private boolean prevSquare;

    // The lowest flywheel speed (tps) seen since the feeder last started; NaN until the first feed.
    private double minSinceFeed = Double.NaN;
    // How deep the dip of the last feed was, as a fraction of the target speed (0.05 = 5 %); NaN until a feed ends.
    private double lastDipDepthFrac = Double.NaN;
    // Was the feeder running on the previous loop? Used to notice when a feed starts and stops.
    private boolean prevFeeding;
    // The rows recorded with triangle, as text ready to paste into the shot table.
    private final List<String> rows = new ArrayList<String>();

    // Runs once at INIT: sets up the shooter and the two helpers, using the same settings the real robot uses.
    @Override
    public void init() {
        launcher = new Launcher();
        launcher.init(hardwareMap);
        ready = new FlywheelReady(BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC, BioBuzzConstants.Shooter.AT_SPEED_ABOVE_FRAC,
                BioBuzzConstants.Shooter.AT_SPEED_FLOOR_TPS, BioBuzzConstants.Shooter.AT_SPEED_RISE_SEC,
                BioBuzzConstants.Shooter.AT_SPEED_FALL_SEC);
        counter = new ShotCounter(BioBuzzConstants.Shooter.SHOT_DIP_FRAC, BioBuzzConstants.Shooter.AT_SPEED_FLOOR_TPS,
                BioBuzzConstants.Shooter.SHOT_REFRACTORY_SEC);
    }

    // Runs once at PLAY: start the clock and the shot counter.
    @Override
    public void start() {
        startNanos = System.nanoTime();
        counter.start(0.0);
    }

    // Runs over and over after PLAY.
    @Override
    public void loop() {
        // Seconds since PLAY.
        double now = (System.nanoTime() - startNanos) / 1.0e9;
        // Step sizes for this loop: speed in tps and range in inches; the left bumper selects the big step.
        double stepV = gamepad1.left_bumper ? BioBuzzConstants.Shooter.TUNER_SPEED_BIG_STEP_TPS : BioBuzzConstants.Shooter.TUNER_SPEED_STEP_TPS;
        double stepR = gamepad1.left_bumper ? BioBuzzConstants.Shooter.TUNER_RANGE_BIG_STEP_IN : BioBuzzConstants.Shooter.TUNER_RANGE_STEP_IN;

        // Adjust the target speed and the range, and toggle the flywheel, once per button press.
        if (gamepad1.dpad_up && !prevUp) {
            targetTps += stepV;
        }
        if (gamepad1.dpad_down && !prevDown) {
            targetTps = Math.max(0.0, targetTps - stepV);
        }
        if (gamepad1.dpad_right && !prevRight) {
            rangeIn += stepR;
        }
        if (gamepad1.dpad_left && !prevLeft) {
            rangeIn = Math.max(0.0, rangeIn - stepR);
        }
        if (gamepad1.cross && !prevCross) {
            running = !running;
            counter.start(now);
        }
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevRight = gamepad1.dpad_right;
        prevLeft = gamepad1.dpad_left;
        prevCross = gamepad1.cross;

        // Command the flywheel (0 = off) and read its measured speed.
        launcher.setTargetVelocity(running ? targetTps : 0.0);
        double v = launcher.velocity();
        // "At speed" = the speed has been inside the window around the target for long enough (debounced).
        boolean atSpeed = ready.update(now, running, targetTps, v);
        // The feeder runs only while the flywheel is on AND circle is held.
        boolean feeding = running && gamepad1.circle;
        launcher.setFeederOn(feeding);
        counter.update(now, feeding, atSpeed, targetTps, v);

        // Remember the lowest speed during a feed; when the feed ends, work out how deep the dip was as a fraction
        // of the target (this is the number to compare with SHOT_DIP_FRAC).
        if (feeding && !prevFeeding) {
            minSinceFeed = v;
        }
        if (feeding) {
            minSinceFeed = Math.min(minSinceFeed, v);
        }
        if (!feeding && prevFeeding && !Double.isNaN(minSinceFeed) && targetTps > 0) {
            lastDipDepthFrac = 1.0 - minSinceFeed / targetTps;
        }
        prevFeeding = feeding;

        // Triangle records a row "{range, speed}," (also logged); square forgets all recorded rows.
        if (gamepad1.triangle && !prevTriangle) {
            String row = String.format(Locale.US, "{%.1f, %.0f},", rangeIn, targetTps);
            rows.add(row);
            RobotLog.ii("ShooterTuner", "SHOT_TABLE row: " + row);
        }
        if (gamepad1.square && !prevSquare) {
            rows.clear();
        }
        prevTriangle = gamepad1.triangle;
        prevSquare = gamepad1.square;

        telemetry.addData("Flywheel", String.format(Locale.US, "%s  target %.0f  measured %.0f  at speed: %s",
                running ? "RUNNING" : "stopped", targetTps, v, atSpeed));
        // The lowest and highest speed (tps) that count as "at speed" for the current target.
        telemetry.addData("Window", String.format(Locale.US, "%.0f .. %.0f tps", ready.lowerBound(targetTps), ready.upperBound(targetTps)));
        telemetry.addData("Range to record", String.format(Locale.US, "%.1f in  (ball exit to HIVE foot line, plus 3.0)", rangeIn));
        telemetry.addData("Shots counted by dip", counter.count());
        // A ball is counted when the speed falls below this while the feeder runs.
        telemetry.addData("Dip threshold", String.format(Locale.US, "%.0f tps (SHOT_DIP_FRAC %.2f)", counter.dipThreshold(targetTps),
                BioBuzzConstants.Shooter.SHOT_DIP_FRAC));
        telemetry.addData("Last feed: lowest speed", Double.isNaN(minSinceFeed) ? "-" : String.format(Locale.US, "%.0f tps  (dip %.1f%%)",
                minSinceFeed, Double.isNaN(lastDipDepthFrac) ? 0.0 : lastDipDepthFrac * 100.0));
        telemetry.addLine("Recorded rows (paste into BioBuzzConstants.Shooter.SHOT_TABLE, sorted by range):");
        for (String r : rows) {
            telemetry.addLine("  " + r);
        }
    }

    // Runs once at STOP: switch the flywheel and feeder off.
    @Override
    public void stop() {
        launcher.stop();
    }
}
