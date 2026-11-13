package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * One of the two flat intake wheels at the front of the robot. They spin parallel to the floor (the disc lies flat, like a
 * coin on a table) and sit at the two front corners of the chassis, one on each side of the robot's center line.
 *
 * <p>"Left" and "right" are the robot's own left and right when it faces forward (the intake end), NOT field directions.
 * They are the same for the red and the blue alliance: a half turn of the field turns the whole robot with it, so the
 * wheel that is on the robot's left stays on the robot's left.
 *
 * <p>Why the enum exists: the two wheels are 12.2 in apart, far wider than a 2.8 in POLLEN, so the robot cannot straddle a
 * FLOWER's bottom POLLEN with both wheels. It presses ONE wheel into the POLLEN while it stands to one side, and the route
 * data has to say which wheel that is (see {@code FlowerPress}).
 */
public enum IntakeWheel {
    /** The wheel on the robot's left (positive "left" in the robot frame of {@link Pose2}). */
    LEFT(1.0),
    /** The wheel on the robot's right. */
    RIGHT(-1.0);

    private final double sign;

    IntakeWheel(double sign) {
        this.sign = sign;
    }

    /** +1 for the left wheel and -1 for the right one: the direction of the wheel from the robot's center line. */
    public double sign() {
        return sign;
    }

    /** The other wheel. */
    public IntakeWheel other() {
        return this == LEFT ? RIGHT : LEFT;
    }
}
