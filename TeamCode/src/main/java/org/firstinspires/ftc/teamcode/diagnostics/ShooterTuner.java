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
 */
@TeleOp(name = "Shooter Tuner", group = "AutoShoot")
public class ShooterTuner extends OpMode {
    private Launcher launcher;
    private FlywheelReady ready;
    private ShotCounter counter;

    private double targetTps = BioBuzzConstants.Shooter.MANUAL_TARGET_TPS;
    private double rangeIn = BioBuzzConstants.Shooter.TUNER_START_RANGE_IN;
    private boolean running;
    private long startNanos;

    private boolean prevUp;
    private boolean prevDown;
    private boolean prevLeft;
    private boolean prevRight;
    private boolean prevCross;
    private boolean prevTriangle;
    private boolean prevSquare;

    private double minSinceFeed = Double.NaN;
    private double lastDipDepthFrac = Double.NaN;
    private boolean prevFeeding;
    private final List<String> rows = new ArrayList<String>();

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

    @Override
    public void start() {
        startNanos = System.nanoTime();
        counter.start(0.0);
    }

    @Override
    public void loop() {
        double now = (System.nanoTime() - startNanos) / 1.0e9;
        double stepV = gamepad1.left_bumper ? BioBuzzConstants.Shooter.TUNER_SPEED_BIG_STEP_TPS : BioBuzzConstants.Shooter.TUNER_SPEED_STEP_TPS;
        double stepR = gamepad1.left_bumper ? BioBuzzConstants.Shooter.TUNER_RANGE_BIG_STEP_IN : BioBuzzConstants.Shooter.TUNER_RANGE_STEP_IN;

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

        launcher.setTargetVelocity(running ? targetTps : 0.0);
        double v = launcher.velocity();
        boolean atSpeed = ready.update(now, running, targetTps, v);
        boolean feeding = running && gamepad1.circle;
        launcher.setFeederOn(feeding);
        counter.update(now, feeding, atSpeed, targetTps, v);

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
        telemetry.addData("Window", String.format(Locale.US, "%.0f .. %.0f tps", ready.lowerBound(targetTps), ready.upperBound(targetTps)));
        telemetry.addData("Range to record", String.format(Locale.US, "%.1f in  (ball exit to HIVE foot line, plus 3.0)", rangeIn));
        telemetry.addData("Shots counted by dip", counter.count());
        telemetry.addData("Dip threshold", String.format(Locale.US, "%.0f tps (SHOT_DIP_FRAC %.2f)", counter.dipThreshold(targetTps),
                BioBuzzConstants.Shooter.SHOT_DIP_FRAC));
        telemetry.addData("Last feed: lowest speed", Double.isNaN(minSinceFeed) ? "-" : String.format(Locale.US, "%.0f tps  (dip %.1f%%)",
                minSinceFeed, Double.isNaN(lastDipDepthFrac) ? 0.0 : lastDipDepthFrac * 100.0));
        telemetry.addLine("Recorded rows (paste into BioBuzzConstants.Shooter.SHOT_TABLE, sorted by range):");
        for (String r : rows) {
            telemetry.addLine("  " + r);
        }
    }

    @Override
    public void stop() {
        launcher.stop();
    }
}
