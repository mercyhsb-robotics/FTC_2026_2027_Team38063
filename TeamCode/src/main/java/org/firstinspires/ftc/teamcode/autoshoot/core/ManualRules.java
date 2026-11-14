package org.firstinspires.ftc.teamcode.autoshoot.core;

/**
 * The decisions of the manual controls, as plain functions: which of "shoot", "intake" and "clear a jam" really happens
 * when the driver holds several buttons at once, and how the shooting speed changes when the speed buttons are pressed.
 * Plain Java with no FTC SDK, so the unit tests can check it on a PC. {@code ManualMechanisms} (the TeleOp side) asks
 * this class and then moves the motors.
 *
 * <p>Priority (highest first): <b>clear a jam</b>, then <b>shoot</b>, then <b>pull in</b>. The robot never takes POLLEN in
 * while it is shooting or clearing a jam (it may hold at most 4 POLLEN), and a jam button always works, even when the
 * shoot trigger is also held. The two intake reverse buttons (the roller, and the two corner servos called the "feelers")
 * push POLLEN out: each one beats pulling in for its own part and works at any time. When nothing is asked for,
 * everything is off: in particular the flywheel is told to stop, which brakes it (see {@code Launcher.setTargetVelocity}).
 */
public final class ManualRules {
    private ManualRules() {
    }

    /** What the mechanisms should do this loop. Directions: 1 = pull POLLEN in, -1 = push it out, 0 = off. */
    public static final class Decision {
        /** Spin the flywheel up to the shooting speed and feed (the feeder still waits for the flywheel to be fast enough). */
        public final boolean shoot;
        /** The intake roller at the front. */
        public final int rollerDirection;
        /** The two corner servos (the feelers). */
        public final int feelerDirection;
        /** Run the feeder (indexer) slowly backward. */
        public final boolean reverseIndexer;
        /** Run the flywheel slowly backward. */
        public final boolean reverseShooter;

        Decision(boolean shoot, int rollerDirection, int feelerDirection, boolean reverseIndexer, boolean reverseShooter) {
            this.shoot = shoot;
            this.rollerDirection = rollerDirection;
            this.feelerDirection = feelerDirection;
            this.reverseIndexer = reverseIndexer;
            this.reverseShooter = reverseShooter;
        }

        /** True if any part of the intake is pulling POLLEN in. */
        public boolean pullingIn() {
            return rollerDirection > 0 || feelerDirection > 0;
        }

        /** True if any part of the intake is pushing POLLEN out. */
        public boolean pushingOut() {
            return rollerDirection < 0 || feelerDirection < 0;
        }
    }

    /**
     * @param shootAsked the shoot trigger is held
     * @param intakeAsked the intake trigger is held
     * @param indexerJam the "clear a jam in the indexer" button is held
     * @param shooterJam the "clear a jam in the shooter" button is held
     * @param rollerReverseAsked the "run the intake roller backward" button is held
     * @param feelerReverseAsked the "run the corner servos backward" button is held
     */
    public static Decision decide(boolean shootAsked, boolean intakeAsked, boolean indexerJam, boolean shooterJam,
                                  boolean rollerReverseAsked, boolean feelerReverseAsked) {
        boolean clearing = indexerJam || shooterJam;
        boolean shoot = shootAsked && !clearing;
        boolean pullIn = intakeAsked && !shoot && !clearing;
        int roller = rollerReverseAsked ? -1 : (pullIn ? 1 : 0);
        int feelers = feelerReverseAsked ? -1 : (pullIn ? 1 : 0);
        return new Decision(shoot, roller, feelers, indexerJam, shooterJam);
    }

    /**
     * The shooting speed after one press of a speed button.
     *
     * @param currentTps the speed now, in ticks per second
     * @param direction +1 to go faster, -1 to go slower
     * @param stepTps how much one press changes the speed
     * @param minTps the slowest allowed speed
     * @param maxTps the fastest allowed speed
     * @return the new speed, never outside {@code minTps..maxTps}
     */
    public static double trimShootSpeed(double currentTps, int direction, double stepTps, double minTps, double maxTps) {
        double wanted = currentTps + direction * stepTps;
        return Math.max(minTps, Math.min(maxTps, wanted));
    }
}
