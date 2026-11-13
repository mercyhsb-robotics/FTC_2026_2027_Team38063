package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.List;

/**
 * The robot's side of a WAIT_FOR_TIP step: stand still until the camera confirms the HIVE has fully tipped
 * ({@link TipConfirmation}), but give up at a DEADLINE on the route clock.
 *
 * <p><b>The deadline is not a guess of when the partner tips.</b> It is the latest moment at which the robot must stop waiting so
 * that the rest of the route still fits in the AUTO period. The route analysis computes it
 * ({@code RouteAnalysis.effectiveGiveUpSec}: the step's own give-up time, or the latest time the rest of the route allows, whichever
 * is earlier) and hands it in. When it passes without a confirmed tip the outcome is {@link Outcome#GAVE_UP}: the robot must NOT
 * shoot any volley that depends on the tip, skips it and carries on with the safe steps (LEAVE, PARK). The same happens when there
 * is no camera: nothing can be confirmed, so the wait runs out at the deadline ("no camera, no shot").
 *
 * <p><b>A time never lets the robot shoot.</b> The only way to {@link Outcome#CONFIRMED} is the camera. The deadline can only END
 * the wait without a shot. A tip that is confirmed after the deadline does not count: the rest of the route no longer fits.
 *
 * <p>Every loop, call {@link HiveTracker#update} first, then {@link #update} with the same frame and views. Time is a parameter, so
 * tests can replay any sequence. The outcome is latched: once it is not WAITING it never changes.
 */
public final class TipWait {

    /** How the wait stands. */
    public enum Outcome {
        /** Still waiting: the camera has not confirmed the tip and the deadline has not passed. */
        WAITING,
        /** The camera has confirmed the tip: the volley that follows may start (its own shot gate still applies). */
        CONFIRMED,
        /** The deadline passed without a confirmed tip: skip the tip-dependent volleys and go on with the safe steps. */
        GAVE_UP
    }

    private final TipConfirmation confirmation;
    private final double giveUpSec;
    private Outcome outcome = Outcome.WAITING;
    private double endedAtSec = Double.NaN;

    /**
     * @param confirmation the camera-only confirmation of the cell the volley aims at
     * @param giveUpSec    the route-clock time at which the robot stops waiting (see the class comment)
     */
    public TipWait(TipConfirmation confirmation, double giveUpSec) {
        this.confirmation = confirmation;
        this.giveUpSec = giveUpSec;
    }

    /** The deadline on the route clock. */
    public double giveUpSec() {
        return giveUpSec;
    }

    /** The current outcome. */
    public Outcome outcome() {
        return outcome;
    }

    /** True only when the camera has confirmed the tip in time: the one condition under which the volley may start. */
    public boolean volleyMayStart() {
        return outcome == Outcome.CONFIRMED;
    }

    /** True if the robot gave up: it must leave the tip-dependent volleys out. */
    public boolean gaveUp() {
        return outcome == Outcome.GAVE_UP;
    }

    /** The route time at which the wait ended (confirmed or gave up); NaN while it is still waiting. */
    public double endedAtSec() {
        return endedAtSec;
    }

    /** The underlying confirmation, for telemetry and for the spill rule ({@link TipConfirmation#spillClearAtSec()}). */
    public TipConfirmation confirmation() {
        return confirmation;
    }

    /**
     * Call once per loop, after {@link HiveTracker#update} with the same frame.
     *
     * @param nowSec the route clock (seconds since the start of AUTO)
     */
    public Outcome update(double nowSec, VisionFrame frame, List<ClusterView> views) {
        if (outcome != Outcome.WAITING) {
            return outcome;
        }
        TipConfirmation.State state = confirmation.update(nowSec, frame, views);
        if (state == TipConfirmation.State.CONFIRMED && nowSec <= giveUpSec) {
            outcome = Outcome.CONFIRMED;
            endedAtSec = nowSec;
        } else if (nowSec >= giveUpSec) {
            outcome = Outcome.GAVE_UP;
            endedAtSec = nowSec;
        }
        return outcome;
    }

    /** Why the volley is not (yet) allowed, in plain words, or "" once it is. */
    public String why(double nowSec) {
        switch (outcome) {
            case CONFIRMED:
                return "";
            case GAVE_UP:
                return "gave up at " + String.format(java.util.Locale.ROOT, "%.1f", endedAtSec) + " s without a confirmed tip: " + confirmation.why(nowSec);
            default:
                return confirmation.why(nowSec);
        }
    }
}
