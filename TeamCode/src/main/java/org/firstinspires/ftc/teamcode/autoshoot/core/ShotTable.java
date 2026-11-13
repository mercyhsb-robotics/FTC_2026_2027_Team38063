package org.firstinspires.ftc.teamcode.autoshoot.core;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Flywheel speed as a function of range: the horizontal distance from the ball exit to the cell aim
 * point, in inches, mapped to flywheel ticks per second by linear interpolation between measured
 * rows. Outside the table the nearest row is used, and {@link #inRange} reports that the shot is
 * outside what has been measured (the shooter refuses to fire out of range).
 */
public final class ShotTable {
    private final double[] range;
    private final double[] velocity;

    /** @param rows each row is {range_in, velocity_tps}, with strictly increasing range */
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

    public static ShotTable fromConstants() {
        return new ShotTable(BioBuzzConstants.Shooter.SHOT_TABLE);
    }

    public int rows() {
        return range.length;
    }

    public double minRange() {
        return range[0];
    }

    public double maxRange() {
        return range[range.length - 1];
    }

    /** True if the range lies between the first and last measured rows, shrunk by {@code margin} at each end. */
    public boolean inRange(double rangeIn, double margin) {
        return rangeIn >= range[0] + margin && rangeIn <= range[range.length - 1] - margin;
    }

    public boolean inRange(double rangeIn) {
        return inRange(rangeIn, 0.0);
    }

    /** Interpolated flywheel speed for a range; clamps to the first/last row outside the table. */
    public double velocityAt(double rangeIn) {
        if (rangeIn <= range[0]) {
            return velocity[0];
        }
        int last = range.length - 1;
        if (rangeIn >= range[last]) {
            return velocity[last];
        }
        int i = 1;
        while (range[i] < rangeIn) {
            i++;
        }
        double t = (rangeIn - range[i - 1]) / (range[i] - range[i - 1]);
        return velocity[i - 1] + t * (velocity[i] - velocity[i - 1]);
    }
}
