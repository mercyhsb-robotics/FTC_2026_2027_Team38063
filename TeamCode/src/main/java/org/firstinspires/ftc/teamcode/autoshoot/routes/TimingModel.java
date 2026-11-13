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
 * leg that moved ends with a settle time. An INTAKE costs its dwell (a FLOWER press-in also costs its slow
 * straight approach and its slow straight retreat), a SHOOT costs the aim and
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
        /** Fraction of the cruise speed for the final approach to a FLOWER and for the retreat from it. */
        public double flowerSpeedFraction = BioBuzzConstants.Auto.FLOWER_APPROACH_SPEED_FRACTION;
        /**
         * How long the camera must have seen the tipped cell's tags steadily before the robot believes the HIVE has FULLY
         * tipped and goes on ({@code Vision.HIVE_STABLE_SEEN_SEC}). A tip is complete only when the damper touches the frame,
         * and launching at a HIVE that is still rocking can spoil it (G417), so a WAIT_FOR_TIP never ends at the instant of
         * the tip but this long after it.
         */
        public double tipSettleSec = BioBuzzConstants.Vision.HIVE_STABLE_SEEN_SEC;
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

    /**
     * Time of a FLOWER press-in after the robot is lined up: the slow straight approach, the dwell with the intake on and
     * the slow straight retreat (each leg that moves ends with the settle time).
     */
    public double flowerSeconds(Step.Intake step, FlowerPress press) {
        return legSeconds(press.approach, p.flowerSpeedFraction) + intakeSeconds(step)
                + legSeconds(press.retreatPath, p.flowerSpeedFraction);
    }

    /** Aim and spin-up plus one feed time per POLLEN, or the step's fixed duration. */
    public double shootSeconds(Step.Shoot step) {
        return shootSeconds(step, step.count());
    }

    /**
     * How long a volley takes when it fires only {@code shots} of the POLLEN it plans (a volley that tips the HIVE stops
     * at the tip and keeps the rest): the aim and spin-up plus one feed time per POLLEN actually fired. A volley with a
     * fixed duration (an old OpMode that feeds for a set time) is spread evenly over its POLLEN, so the n-th one leaves
     * {@code n / count} of the way through it.
     */
    public double shootSeconds(Step.Shoot step, int shots) {
        if (step.hasFixedSeconds()) {
            return step.fixedSeconds() * shots / step.count();
        }
        return p.aimSpinUpSec + shots * p.timePerShotSec;
    }

    public double waitSeconds(Step.Wait step) {
        return step.seconds();
    }

    /**
     * How long a WAIT_FOR_TIP lasts when it starts at {@code nowSec}, the tip it waits for is complete at {@code tipSec}
     * and the robot gives up at {@code giveUpSec} (all three on the route clock). The robot goes on only when its camera
     * has SEEN the tipped cell steadily for {@code tipSettleSec} (a HIVE that is still rocking must not be shot at, G417),
     * so the wait lasts until {@code tipSec + tipSettleSec} (0 if that is already past), but never past the give-up time.
     * If there is no tip to wait for ({@code tipSec} is NaN) the cell is UP already and the robot does not wait at all.
     */
    public double waitForTipSeconds(double nowSec, double tipSec, double giveUpSec) {
        if (Double.isNaN(tipSec)) {
            return 0.0;
        }
        double endSec = Math.min(giveUpSec, Math.max(nowSec, confirmedAtSec(tipSec)));
        return Math.max(0.0, endSec - nowSec);
    }

    /** The route time at which the camera has confirmed a tip that is complete at {@code tipSec}: the tip plus the settle time. */
    public double confirmedAtSec(double tipSec) {
        return tipSec + p.tipSettleSec;
    }

    /** The settle time the camera needs before it calls a tipped HIVE fully tipped, in seconds. */
    public double tipSettleSec() {
        return p.tipSettleSec;
    }
}
