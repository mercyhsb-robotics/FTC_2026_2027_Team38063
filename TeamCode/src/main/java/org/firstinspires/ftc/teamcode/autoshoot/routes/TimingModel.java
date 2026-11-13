package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * A pure estimate of how long each step of a route takes, from the placeholder numbers in
 * {@link BioBuzzConstants.Auto}. The robot has not been measured yet, so the result is a PLAN, good
 * for comparing routes and catching ones that cannot fit in 30 s, never a promise.
 *
 * <p>The model: a drive leg accelerates to the cruise speed, cruises and brakes (a trapezoid, or a
 * triangle on short legs); turning happens while driving, so a leg costs the longer of the two; every
 * leg that moved ends with a settle time. An INTAKE costs its dwell, a SHOOT costs the aim and
 * spin-up plus one feed time per POLLEN (or its fixed duration), a WAIT costs its seconds, and START
 * and PARK are free (the drive before them already paid the settle).
 */
public final class TimingModel {

    /** All inputs of the model. Defaults come from {@link BioBuzzConstants.Auto}. */
    public static final class Params {
        public double cruiseInPerSec = BioBuzzConstants.Auto.CRUISE_SPEED_IN_PER_SEC;
        public double accelInPerSec2 = BioBuzzConstants.Auto.ACCEL_IN_PER_SEC2;
        public double turnRateDegPerSec = BioBuzzConstants.Auto.TURN_RATE_DEG_PER_SEC;
        public double settleSec = BioBuzzConstants.Auto.SETTLE_SEC;
        public double aimSpinUpSec = BioBuzzConstants.Auto.AIM_SPINUP_SEC;
        public double timePerShotSec = BioBuzzConstants.Auto.TIME_PER_SHOT_SEC;
    }

    private static final double TINY = 1e-9;

    private final Params p;

    public TimingModel() {
        this(new Params());
    }

    public TimingModel(Params params) {
        this.p = params;
    }

    /** Time to travel {@code distanceIn} inches from rest to rest at the given fraction of the cruise speed. */
    public double translationSeconds(double distanceIn, double speedFraction) {
        if (distanceIn <= TINY) {
            return 0.0;
        }
        double v = p.cruiseInPerSec * speedFraction;
        double a = p.accelInPerSec2;
        double rampDistance = v * v / a;          // distance used to reach v and to brake again
        if (distanceIn >= rampDistance) {
            return distanceIn / v + v / a;
        }
        return 2.0 * Math.sqrt(distanceIn / a);   // never reaches cruise speed
    }

    /** Time to turn through {@code radians}. */
    public double turnSeconds(double radians) {
        return Math.abs(radians) / Math.toRadians(p.turnRateDegPerSec);
    }

    /** Time the robot spends moving along a DRIVE leg: the longer of driving and turning (no settle time). */
    public double legTravelSeconds(DrivePath path, double speedFraction) {
        return Math.max(translationSeconds(path.length(), speedFraction), turnSeconds(path.totalTurnRadians()));
    }

    /** Duration of a DRIVE leg: the longer of driving and turning, plus the settle if the robot moved at all. */
    public double legSeconds(DrivePath path, double speedFraction) {
        double travel = legTravelSeconds(path, speedFraction);
        if (travel <= TINY) {
            return 0.0;
        }
        return travel + p.settleSec;
    }

    /**
     * How far along a path of {@code distanceIn} inches the robot is {@code seconds} after it starts moving, with the
     * same speed profile as {@link #translationSeconds}: it speeds up at the acceleration, cruises, and brakes so it
     * arrives at rest (or it speeds up and brakes straight away on a short path). 0 at the start, the whole distance
     * once {@code seconds} reaches the travel time. It lets a check ask "where is the robot at second 14?".
     */
    public double distanceAfter(double distanceIn, double speedFraction, double seconds) {
        if (seconds <= 0.0 || distanceIn <= TINY) {
            return 0.0;
        }
        double total = translationSeconds(distanceIn, speedFraction);
        if (seconds >= total) {
            return distanceIn;
        }
        double v = p.cruiseInPerSec * speedFraction;
        double a = p.accelInPerSec2;
        boolean reachesCruise = distanceIn >= v * v / a;
        double rampSec = reachesCruise ? v / a : total / 2.0;
        if (seconds <= rampSec) {
            return 0.5 * a * seconds * seconds;                  // speeding up
        }
        double left = total - seconds;
        if (!reachesCruise || left <= rampSec) {
            return distanceIn - 0.5 * a * left * left;           // braking: mirror image of the speed-up
        }
        return 0.5 * a * rampSec * rampSec + v * (seconds - rampSec);     // cruising
    }

    /** Duration of the implicit straight move to an INTAKE, SHOOT or PARK pose (0 if the robot is already there). */
    public double moveSeconds(Pose2 from, Pose2 to) {
        double drive = translationSeconds(from.distanceTo(to), 1.0);
        double turn = turnSeconds(Angles.error(from.heading, to.heading));
        if (drive <= TINY && turn <= TINY) {
            return 0.0;
        }
        return Math.max(drive, turn) + p.settleSec;
    }

    /** Time spent at the intake pose with the intake running. */
    public double intakeSeconds(Step.Intake step) {
        return step.dwellSec();
    }

    /** Aim and spin-up plus one feed time per POLLEN, or the step's fixed duration. */
    public double shootSeconds(Step.Shoot step) {
        if (step.hasFixedSeconds()) {
            return step.fixedSeconds();
        }
        return p.aimSpinUpSec + step.count() * p.timePerShotSec;
    }

    public double waitSeconds(Step.Wait step) {
        return step.seconds();
    }

    /**
     * How long a WAIT_FOR_TIP lasts when it starts at {@code nowSec} and the tip it waits for happens at
     * {@code tipSec}: until the tip (0 if it has already happened), but never longer than the timeout. If there is no
     * tip to wait for ({@code tipSec} is NaN) the cell is already UP and the robot does not wait at all.
     */
    public double waitForTipSeconds(double nowSec, double tipSec, double timeoutSec) {
        if (Double.isNaN(tipSec)) {
            return 0.0;
        }
        return Math.min(timeoutSec, Math.max(0.0, tipSec - nowSec));
    }
}
