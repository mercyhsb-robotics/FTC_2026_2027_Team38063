package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.autoshoot.core.Side;

/**
 * Which cell of the own HIVE a SHOOT step fires into: the cell that is UP when the shot is taken
 * (the HIVE flips after every TIP, so this is the normal choice), or a fixed side. Only an UP cell
 * may be launched into (rule G417), so a fixed side is a promise that this side is UP at that moment.
 */
public final class ShootTarget {
    private static final ShootTarget UP = new ShootTarget(null);

    private final Side fixedSide;

    private ShootTarget(Side fixedSide) {
        this.fixedSide = fixedSide;
    }

    /** The cell of the own HIVE that is UP when the shot is taken. */
    public static ShootTarget upCell() {
        return UP;
    }

    /** The cell on the given side of the own HIVE, whichever state it is in. */
    public static ShootTarget fixedSide(Side side) {
        if (side == null) {
            throw new IllegalArgumentException("side must not be null");
        }
        return new ShootTarget(side);
    }

    public boolean isUpCell() {
        return fixedSide == null;
    }

    /** The fixed side, or null when this target is "whichever cell is UP". */
    public Side fixedSideOrNull() {
        return fixedSide;
    }

    /**
     * The half-turn image of this target. The blue cell on side S is the half-turn image of the red
     * cell on the OPPOSITE side (the red audience cell maps to the blue REAR cell), so a fixed side
     * flips; "the UP cell" stays "the UP cell".
     */
    public ShootTarget halfTurn() {
        return fixedSide == null ? this : new ShootTarget(fixedSide.opposite());
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof ShootTarget && ((ShootTarget) o).fixedSide == fixedSide;
    }

    @Override
    public int hashCode() {
        return fixedSide == null ? 0 : fixedSide.hashCode() + 1;
    }

    @Override
    public String toString() {
        return fixedSide == null ? "UP cell" : fixedSide + " cell";
    }
}
