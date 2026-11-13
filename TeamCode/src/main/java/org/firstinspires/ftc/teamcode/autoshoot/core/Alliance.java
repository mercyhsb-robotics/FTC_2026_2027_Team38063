package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * The two alliances. The BIOBUZZ field is 180-degree ROTATIONALLY symmetric about its center (it is
 * NOT mirror symmetric): rotating any red feature half a turn about (72, 72) lands exactly on the
 * matching blue feature (red GARDEN corner (0,0) -> blue GARDEN corner (144,144), red HIVE axis
 * x=59.25 -> blue x=84.75, red start tile B1 -> blue tile E6, ...).
 *
 * <p>All field data is therefore written ONCE, for RED, and converted for BLUE here:
 * {@code blue = (size - x, size - y, heading + 180 deg)}.
 */
public enum Alliance {
    RED,
    BLUE;

    public Alliance opposite() {
        return this == RED ? BLUE : RED;
    }

    /** Converts a RED-frame x to this alliance's equivalent x. */
    public double fromRedX(double redX, double fieldSize) {
        return this == RED ? redX : fieldSize - redX;
    }

    /** Converts a RED-frame y to this alliance's equivalent y. */
    public double fromRedY(double redY, double fieldSize) {
        return this == RED ? redY : fieldSize - redY;
    }

    /** Converts a RED-frame heading (radians) to this alliance's equivalent heading. */
    public double fromRedHeading(double redHeading) {
        return this == RED ? Angles.normalize(redHeading) : Angles.opposite(redHeading);
    }

    /** Converts a RED-frame pose to this alliance's equivalent pose. */
    public Pose2 fromRed(Pose2 red, double fieldSize) {
        if (this == RED) {
            return red;
        }
        return new Pose2(fieldSize - red.x, fieldSize - red.y, Angles.opposite(red.heading));
    }

    /** The inverse of {@link #fromRed}; a half turn is its own inverse, so it is the same operation. */
    public Pose2 toRed(Pose2 own, double fieldSize) {
        return fromRed(own, fieldSize);
    }
}
