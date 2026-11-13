package org.firstinspires.ftc.teamcode.autoshoot.routes;

/** The kinds of step an autonomous route is made of. */
public enum StepKind {
    /** Where the robot is placed before the match (rule G304). Exactly one, always first. */
    START,
    /** Follow a path to a target pose. */
    DRIVE,
    /** Take POLLEN from the floor, a FLOWER or the GARDEN. */
    INTAKE,
    /** Launch POLLEN at a HIVE cell. */
    SHOOT,
    /** Do nothing for a while. */
    WAIT,
    /** Finish partly inside the own LOADING ZONE (AUTO PARK, manual 10.5.4). */
    PARK
}
