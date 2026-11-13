package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Which end of a HIVE a CELL is on. Each HIVE has one CELL toward the audience wall (y small) and one
 * toward the rear wall (y large). Exactly one of the two is tipped UP (scorable) at any time, and
 * every TIP flips which one it is.
 */
public enum Side {
    AUDIENCE,
    REAR;

    public Side opposite() {
        return this == AUDIENCE ? REAR : AUDIENCE;
    }

    /**
     * Direction, in radians, the opening of an UP cell on this side faces in the floor plan:
     * the audience-side opening faces -y (toward the audience), the rear-side opening faces +y.
     */
    public double openingFacingHeading() {
        return this == AUDIENCE ? -Math.PI / 2.0 : Math.PI / 2.0;
    }

    /** +1 if this side is at larger y than the HIVE pivot, -1 if at smaller y. */
    public int ySign() {
        return this == REAR ? 1 : -1;
    }
}
