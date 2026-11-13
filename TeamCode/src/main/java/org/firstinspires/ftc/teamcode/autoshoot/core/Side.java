package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Which end of a HIVE a CELL is on. Each HIVE has one CELL toward the audience wall (y small) and one
 * toward the rear wall (y large). Exactly one of the two is tipped UP (scorable) at any time, and
 * every TIP flips which one it is.
 *
 * <p>A Side is a real place on the field, the same for both alliances: AUDIENCE is always the small-y end and
 * REAR is always the large-y end (it is not flipped for BLUE). The HIVE tracker's belief "which cell is up" is
 * a Side, and so is "which end of the HIVE is the robot standing on".
 */
public enum Side {
    /** The cell nearer the audience wall (small y). Its UP opening faces -y. */
    AUDIENCE,
    /** The cell nearer the rear wall (large y). Its UP opening faces +y. */
    REAR;

    /** The other end of the HIVE (what a tip changes the up side to). */
    public Side opposite() {
        return this == AUDIENCE ? REAR : AUDIENCE;
    }

    /**
     * Direction, in radians, the opening of an UP cell on this side faces in the floor plan:
     * the audience-side opening faces -y (toward the audience), the rear-side opening faces +y.
     * (Field heading convention: 0 = +x, counter-clockwise positive, so -PI/2 points along -y.)
     */
    public double openingFacingHeading() {
        return this == AUDIENCE ? -Math.PI / 2.0 : Math.PI / 2.0;
    }

    /** +1 if this side is at larger y than the HIVE pivot, -1 if at smaller y. */
    public int ySign() {
        return this == REAR ? 1 : -1;
    }
}
