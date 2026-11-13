package org.firstinspires.ftc.teamcode.mechanisms.launcher;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The launcher: one flywheel motor run in closed-loop velocity mode and one continuous-rotation
 * feeder servo that pushes POLLEN into it. All speeds, powers and the PIDF come from
 * {@link BioBuzzConstants.Shooter}.
 *
 * <p>The class only commands and measures; it never writes telemetry and never decides when to feed.
 * The auto-shoot controller (and the manual TeleOp buttons) make those decisions.
 *
 * <p><b>How the shooter works.</b> The shooter is at the BACK of the robot and fires out the back. The
 * <b>flywheel</b> is a wheel spun very fast; a POLLEN pushed against it is squeezed and thrown. The <b>feeder</b>
 * (also called the "windmill") is a continuous-rotation servo, a servo that spins like a motor
 * (power -1..1 sets speed and direction) instead of turning to a position. Running it pushes the next POLLEN into the
 * flywheel. The launch angle is fixed, so distance is controlled only by flywheel speed (see the shot table,
 * {@code BioBuzzConstants.Shooter.SHOT_TABLE}).
 *
 * <p><b>Units.</b> Flywheel speed is in encoder TICKS PER SECOND ("tps"). A tick is one count of the motor's
 * built-in encoder (a sensor that counts shaft rotation). With our 6000 rpm motor the top speed is about 2800 tps.
 * Positive speed means the motor's FORWARD direction; the code never reverses the flywheel.
 *
 * <p>Who uses it: {@code BioBuzzRobot} (TeleOp), the {@code RedAlliance_*} autonomous routines, the
 * {@code LauncherTest} and {@code ShooterTuner} diagnostics.
 */
public class Launcher {

    // The flywheel motor. DcMotorEx is the SDK motor type that also offers velocity (speed) control.
    private DcMotorEx flywheel;
    // The feeder servo. Positive power pushes a POLLEN toward the flywheel (BioBuzzConstants.Shooter.FEEDER_INVERTED
    // flips this if the servo is mounted the other way).
    private CRServo feeder;

    // The speed last commanded, in ticks per second (0 = flywheel off). Not the measured speed: see velocity().
    private double targetTps;
    // The power last given to the feeder: positive = forward (toward the flywheel), negative = backward, 0 = stopped.
    // Remembering the power (not just on/off) is what lets setFeederOn(true) turn a backward feeder around.
    private double feederPower;
    // System.nanoTime() (nanoseconds) when targetTps last changed; used by secondsSinceTargetChange().
    private long targetSetAtNanos;
    // True while the flywheel is being run BACKWARD by hand to clear a jam (see reverseFlywheel).
    private boolean flywheelReversing;

    /** @param telemetry unused; kept so existing callers keep compiling. */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap);
    }

    /**
     * Finds the flywheel and feeder in the robot configuration (names in BioBuzzConstants.Hardware) and sets them up.
     * Call once, from the OpMode's init(). Leaves both stopped.
     */
    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.Hardware.FLYWHEEL);
        // RUN_USING_ENCODER: the motor controller uses the encoder to hold the speed we ask for (closed loop),
        // which is what setVelocity() needs.
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // What the motor does when told to stop: BRAKE resists turning, FLOAT lets the flywheel coast down.
        flywheel.setZeroPowerBehavior(BioBuzzConstants.Shooter.FLYWHEEL_BRAKE_ON_STOP
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        // PIDF is the speed controller inside the motor controller: P pushes harder the farther the speed is from the
        // target, I corrects a steady error, D damps changes, and F is the base power that a given speed needs
        // before the error terms add anything. (In the current constants I and D are 0.)
        if (BioBuzzConstants.Shooter.FLYWHEEL_APPLY_PIDF) {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    BioBuzzConstants.Shooter.FLYWHEEL_PIDF_P, BioBuzzConstants.Shooter.FLYWHEEL_PIDF_I,
                    BioBuzzConstants.Shooter.FLYWHEEL_PIDF_D, BioBuzzConstants.Shooter.FLYWHEEL_PIDF_F));
        }

        feeder = hardwareMap.get(CRServo.class, BioBuzzConstants.Hardware.FEEDER);
        // The direction is set ONCE; reversing is done with the sign of the power, so a command is never
        // sent with the old direction.
        feeder.setDirection(BioBuzzConstants.Shooter.FEEDER_INVERTED
                ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        feeder.setPower(0.0);
        targetTps = 0.0;
        feederPower = 0.0;
        flywheelReversing = false;
        targetSetAtNanos = System.nanoTime();
    }

    // ------------------------------------------------------------------------------------ flywheel

    /**
     * Sets the flywheel speed in ticks per second; zero or less stops it (it never spins backward). Only talks to the
     * hub when the target changes, so it is safe to call every loop.
     */
    public void setTargetVelocity(double tps) {
        double wanted = Math.max(0.0, tps);
        if (wanted == targetTps) {
            return;
        }
        targetTps = wanted;
        targetSetAtNanos = System.nanoTime();
        if (wanted > 0.0) {
            // Closed-loop speed command: the motor controller works to hold this many ticks per second.
            flywheel.setVelocity(wanted);
        } else {
            // Cut the power (coast or brake, as chosen in init()).
            flywheel.setPower(0.0);
        }
    }

    /**
     * Runs the flywheel slowly BACKWARD (or stops that) to free a jammed POLLEN. While {@code on} is true the motor gets
     * a small steady reverse power (BioBuzzConstants.Shooter.FLYWHEEL_UNJAM_POWER) with no speed control; the moment it
     * is false the motor stops and goes back to speed-control mode, ready for the next shot. Safe to call every loop.
     * Call it only when the flywheel is not being used to shoot.
     */
    public void reverseFlywheel(boolean on) {
        if (on) {
            if (!flywheelReversing) {
                // Forget any forward speed target, then leave speed-control mode: a speed controller would fight a
                // plain reverse power.
                targetTps = 0.0;
                targetSetAtNanos = System.nanoTime();
                flywheel.setPower(0.0);
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheelReversing = true;
            }
            flywheel.setPower(-BioBuzzConstants.Shooter.FLYWHEEL_UNJAM_POWER);
        } else if (flywheelReversing) {
            flywheel.setPower(0.0);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelReversing = false;
        }
    }

    /** True while the flywheel is being run backward by hand (see reverseFlywheel). */
    public boolean flywheelReversing() {
        return flywheelReversing;
    }

    /** The last commanded flywheel speed in ticks per second (0 = stopped). For the measured speed use velocity(). */
    public double targetVelocity() {
        return targetTps;
    }

    /** Measured flywheel speed in ticks per second (a hub read: call once per loop). */
    public double velocity() {
        return flywheel.getVelocity();
    }

    /** Seconds since the target last changed. */
    public double secondsSinceTargetChange() {
        return (System.nanoTime() - targetSetAtNanos) / 1.0e9;
    }

    // -------------------------------------------------------------------------------------- feeder

    /**
     * Runs the feeder forward at the configured power, or stops it. Does nothing if the feeder is already doing that
     * (so it is safe to call every loop). A feeder that is running backward (feederReverse) is turned around to
     * forward, or stopped, as asked.
     */
    public void setFeederOn(boolean on) {
        double wanted = on ? BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER : 0.0;
        if (wanted == feederPower) {
            return;
        }
        feederPower = wanted;
        feeder.setPower(wanted);
    }

    /** Runs the feeder forward (pushing a POLLEN into the flywheel) at FEEDER_FORWARD_POWER, whatever it was doing. */
    public void feederForward() {
        feederPower = BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER;
        feeder.setPower(feederPower);
    }

    /** Runs the feeder backwards to clear a jam. */
    public void feederReverse() {
        feederPower = -BioBuzzConstants.Shooter.FEEDER_REVERSE_POWER;
        feeder.setPower(feederPower);
    }

    /** Stops the feeder. */
    public void feederStop() {
        feederPower = 0.0;
        feeder.setPower(0.0);
    }

    /** True if the feeder has been told to run (in either direction). */
    public boolean feederRunning() {
        return feederPower != 0.0;
    }

    // ------------------------------------------------------------------- simple (manual / autonomous) use

    /**
     * Spins the flywheel up to the given speed. Despite the name, the unit is TICKS PER SECOND, not rpm; it is the
     * same as {@link #setTargetVelocity}. The old name is kept so existing callers (the autonomous routines and
     * LauncherTest) keep compiling.
     */
    public void increaseRpm(long rpm) {
        setTargetVelocity(rpm);
    }

    /**
     * Manual launch: with {@code true} the flywheel runs at the manual speed
     * ({@code BioBuzzConstants.Shooter.MANUAL_TARGET_TPS}) and the feeder runs once it
     * is fast enough (faster than {@code MANUAL_MIN_TPS}); with {@code false} both stop. Safe to call every loop.
     */
    public void launch(boolean isLaunch) {
        if (isLaunch) {
            setTargetVelocity(BioBuzzConstants.Shooter.MANUAL_TARGET_TPS);
            setFeederOn(velocity() > BioBuzzConstants.Shooter.MANUAL_MIN_TPS);
        } else {
            setTargetVelocity(0.0);
            setFeederOn(false);
        }
    }

    /**
     * True once the flywheel has reached the LOWER edge of its speed window for the current target (the target
     * minus AT_SPEED_BELOW_FRAC, minus AT_SPEED_FLOOR_TPS); the upper edge is not checked, so a flywheel that is
     * running fast also counts. Also true if it has been trying for longer than the spin-up timeout (so a routine
     * waiting on this can never hang), and always true when the target is zero. The name says "rpm", but the unit is
     * ticks per second.
     */
    public boolean isAtTargetRpm() {
        if (targetTps <= 0.0) {
            return true;
        }
        double lower = targetTps * (1.0 - BioBuzzConstants.Shooter.AT_SPEED_BELOW_FRAC)
                - BioBuzzConstants.Shooter.AT_SPEED_FLOOR_TPS;
        return velocity() >= lower
                || secondsSinceTargetChange() > BioBuzzConstants.AutoShoot.SPINUP_TIMEOUT_SEC;
    }

    /** Stops the flywheel and the feeder. */
    public void stop() {
        reverseFlywheel(false);
        setTargetVelocity(0.0);
        feederStop();
    }
}
