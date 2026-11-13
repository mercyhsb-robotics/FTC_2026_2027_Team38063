package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * Which of the two flat intake wheels does a job. The wheels spin parallel to the floor (the disc lies flat, like a coin on a
 * table) and sit at the two front corners of the chassis, one on each side of the robot's center line.
 *
 * <p>"Left" and "right" are the robot's own left and right when it faces forward (the intake end), NOT field directions.
 * They are the same for the red and the blue alliance: a half turn of the field turns the whole robot with it, so the
 * wheel that is on the robot's left stays on the robot's left.
 *
 * <p>Why the enum exists: the two wheels are 12.2 in apart, far wider than a 2.8 in POLLEN, so the robot cannot straddle a
 * FLOWER's bottom POLLEN with both wheels. It wiggles ONE wheel into the POLLEN (see {@code FlowerWiggle}) and either wheel
 * can do it. A route does not have to say which: {@link #AUTO}, the default, lets the program choose the cheaper one
 * (see {@code WheelChoice}); {@link #LEFT} and {@link #RIGHT} force it.
 *
 * <p>{@link #AUTO} is a request, not a wheel: it has no side, so {@link #sign()} refuses it. Use {@link #physical()} when you
 * want to go through the two real wheels.
 */
public enum IntakeWheel {
    /** "Pick the better wheel for me": the shared choice picks LEFT or RIGHT by cost. Not a wheel of the robot. */
    AUTO(0.0),
    /** The wheel on the robot's left (positive "left" in the robot frame of {@link Pose2}). */
    LEFT(1.0),
    /** The wheel on the robot's right. */
    RIGHT(-1.0);

    private static final IntakeWheel[] REAL = {LEFT, RIGHT};

    private final double sign;

    IntakeWheel(double sign) {
        this.sign = sign;
    }

    /** True for {@link #AUTO}, the request that is not a wheel. */
    public boolean isAuto() {
        return this == AUTO;
    }

    /**
     * +1 for the left wheel and -1 for the right one: the direction of the wheel from the robot's center line.
     *
     * @throws IllegalStateException for {@link #AUTO}, which is not a wheel and so has no side
     */
    public double sign() {
        if (this == AUTO) {
            throw new IllegalStateException("AUTO is not a wheel: it has no side. Choose LEFT or RIGHT first (WheelChoice does).");
        }
        return sign;
    }

    /**
     * The other wheel.
     *
     * @throws IllegalStateException for {@link #AUTO}
     */
    public IntakeWheel other() {
        if (this == AUTO) {
            throw new IllegalStateException("AUTO is not a wheel, so it has no other wheel.");
        }
        return this == LEFT ? RIGHT : LEFT;
    }

    /** The two real wheels, LEFT then RIGHT, in a fresh array (so a caller cannot change the enum's own list). */
    public static IntakeWheel[] physical() {
        return REAL.clone();
    }
}
