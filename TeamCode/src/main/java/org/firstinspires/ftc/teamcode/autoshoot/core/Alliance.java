package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * The two alliances. The BIOBUZZ field is 180-degree ROTATIONALLY symmetric about its center (it is
 * NOT mirror symmetric): rotating any red feature half a turn about the field center (70.674, 70.674) lands exactly on
 * the matching blue feature (red GARDEN corner (0,0) -> blue GARDEN corner (141.348, 141.348), red HIVE axis
 * x = 57.924 -> blue x = 83.424, red start tile B1 -> blue tile E6, ...). The official CAD confirms the symmetry to 0.000 in
 * for the walls, tiles, FLOWERS and POLLEN and to 0.017 in for the HIVE parts.
 *
 * <p>All field data is therefore written ONCE, for RED, and converted for BLUE here:
 * {@code blue = (size - x, size - y, heading + 180 deg)}.
 *
 * <p>Why not just mirror left/right? Because the field is not a mirror image: the blue HIVE is the red one
 * turned half a turn, so the blue cell that is on the audience side is the half-turn image of the red cell on
 * the REAR side. A mirror would put the blue features in the wrong places.
 *
 * <p>Used by {@code FieldModel} (to build the blue cells) and the TeleOp and Limelight Diagnostic OpModes (to
 * turn the red-frame start pose into the real one for whichever alliance the driver picked). Any number that
 * {@code BioBuzzConstants} labels "RED frame" must go through {@link #fromRed} before it is used for BLUE.
 */
public enum Alliance {
    RED,
    BLUE;

    /** The other alliance. */
    public Alliance opposite() {
        return this == RED ? BLUE : RED;
    }

    /** Converts a RED-frame x to this alliance's equivalent x. {@code fieldSize} is the field width in inches ({@code Field.SIZE_IN}, 141.348). */
    public double fromRedX(double redX, double fieldSize) {
        return this == RED ? redX : fieldSize - redX;
    }

    /** Converts a RED-frame y to this alliance's equivalent y. {@code fieldSize} is the field depth in inches ({@code Field.SIZE_IN}, 141.348). */
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
