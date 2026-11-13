package org.firstinspires.ftc.teamcode.autoshoot.routes;

/** Where an INTAKE step takes POLLEN from. */
public enum IntakeSource {
    /** Loose POLLEN lying on the tiles (at match start there is none; only missed or spilled shots). */
    FLOOR,
    /** A FLOWER's bottom retrieval opening (manual 9.7); the robot has its front at the wall. */
    FLOWER,
    /** The line of POLLEN in a GARDEN along the wall (manual 9.3). */
    GARDEN;

    /** True for the sources the robot reaches by touching the field wall (the footprint may graze the wall). */
    public boolean isWallContact() {
        return this != FLOOR;
    }
}
