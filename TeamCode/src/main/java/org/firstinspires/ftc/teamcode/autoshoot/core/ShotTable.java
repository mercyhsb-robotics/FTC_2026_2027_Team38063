package org.firstinspires.ftc.teamcode.autoshoot.core;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Flywheel speed as a function of range: the horizontal distance from the ball exit to the cell aim
 * point, in inches, mapped to flywheel ticks per second by linear interpolation between measured
 * rows. Outside the table the nearest row is used, and {@link #inRange} reports that the shot is
 * outside what has been measured (the shooter refuses to fire out of range).
 *
 * <p>Why a table: the shooter has one fixed launch angle, so the only way to reach the cell from a different
 * distance is to change how fast the flywheel spins (a farther shot needs more speed). The team measures
 * which speed scores at a few distances and writes them in {@code BioBuzzConstants.Shooter.SHOT_TABLE}. A "tick"
 * is one count of the motor's encoder, so "ticks per second" (tps) is the flywheel's speed.
 *
 * <p>Example: with rows {30 in, 1150 tps} and {39 in, 1200 tps}, a 34.5 in shot is halfway between them, so
 * {@code velocityAt(34.5)} is about 1175 tps.
 *
 * <p>Who uses it: {@link ScoringPlanner} (its minimum and maximum range define where the robot may stand),
 * {@link AutoShootController} (speed to command, and the "is the range inside the table" gate), and
 * {@link ConstantsCheck} (checks the rows at INIT).
 */
public final class ShotTable {
    // Parallel arrays: range[i] (inches) belongs with velocity[i] (ticks per second); range is strictly increasing.
    private final double[] range;
    private final double[] velocity;

    /**
     * @param rows each row is {range_in, velocity_tps}, with strictly increasing range
     * @throws IllegalArgumentException if the table is empty or a row is malformed (caught at INIT by
     *         {@link ConstantsCheck} so a typo shows as a message instead of a crash)
     */
    public ShotTable(double[][] rows) {
        if (rows == null || rows.length == 0) {
            throw new IllegalArgumentException("The shot table needs at least one row");
        }
        range = new double[rows.length];
        velocity = new double[rows.length];
        for (int i = 0; i < rows.length; i++) {
            if (rows[i].length != 2) {
                throw new IllegalArgumentException("Shot table row " + i + " must be {range_in, velocity_tps}");
            }
            range[i] = rows[i][0];
            velocity[i] = rows[i][1];
            if (!(range[i] > 0.0) || !(velocity[i] > 0.0)) {
                throw new IllegalArgumentException("Shot table row " + i + " must have a positive range and velocity");
            }
            if (i > 0 && range[i] <= range[i - 1]) {
                throw new IllegalArgumentException("Shot table ranges must be strictly increasing (row " + i + ")");
            }
        }
    }

    /** The table the robot runs with: {@code BioBuzzConstants.Shooter.SHOT_TABLE}. */
    public static ShotTable fromConstants() {
        return new ShotTable(BioBuzzConstants.Shooter.SHOT_TABLE);
    }

    /** Number of rows (measured distances). */
    public int rows() {
        return range.length;
    }

    /** The shortest measured range, in inches. */
    public double minRange() {
        return range[0];
    }

    /** The longest measured range, in inches. */
    public double maxRange() {
        return range[range.length - 1];
    }

    /**
     * True if the range lies between the first and last measured rows, shrunk by {@code margin} at each end.
     * (The planner uses a margin so it aims safely inside the table; the shot gate uses none and only refuses
     * ranges that are truly outside the measured band.)
     */
    public boolean inRange(double rangeIn, double margin) {
        return rangeIn >= range[0] + margin && rangeIn <= range[range.length - 1] - margin;
    }

    /** {@link #inRange(double, double)} with no margin: between the first and last measured rows, inclusive. */
    public boolean inRange(double rangeIn) {
        return inRange(rangeIn, 0.0);
    }

    /** Interpolated flywheel speed for a range, in ticks per second; clamps to the first/last row outside the table. */
    public double velocityAt(double rangeIn) {
        if (rangeIn <= range[0]) {
            return velocity[0];
        }
        int last = range.length - 1;
        if (rangeIn >= range[last]) {
            return velocity[last];
        }
        // Find the first row that is at or beyond this range; the answer lies between it and the row before it.
        int i = 1;
        while (range[i] < rangeIn) {
            i++;
        }
        // t is how far (0..1) the range is from the lower row toward the upper row: a straight line between them.
        double t = (rangeIn - range[i - 1]) / (range[i] - range[i - 1]);
        return velocity[i - 1] + t * (velocity[i] - velocity[i - 1]);
    }
}
