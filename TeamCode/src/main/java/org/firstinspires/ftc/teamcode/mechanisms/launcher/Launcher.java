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
 */
public class Launcher {

    private DcMotorEx flywheel;
    private CRServo feeder;

    private double targetTps;
    private boolean feederOn;
    private long targetSetAtNanos;

    /** @param telemetry unused; kept so existing callers keep compiling. */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, BioBuzzConstants.Hardware.FLYWHEEL);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(BioBuzzConstants.Shooter.FLYWHEEL_BRAKE_ON_STOP
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
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
        feederOn = false;
        targetSetAtNanos = System.nanoTime();
    }

    // ------------------------------------------------------------------------------------ flywheel

    /** Sets the flywheel speed in ticks per second; zero or less stops it. Only talks to the hub when the target changes. */
    public void setTargetVelocity(double tps) {
        double wanted = Math.max(0.0, tps);
        if (wanted == targetTps) {
            return;
        }
        targetTps = wanted;
        targetSetAtNanos = System.nanoTime();
        if (wanted > 0.0) {
            flywheel.setVelocity(wanted);
        } else {
            flywheel.setPower(0.0);
        }
    }

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

    /** Runs the feeder forward at the configured power, or stops it. */
    public void setFeederOn(boolean on) {
        if (on == feederOn) {
            return;
        }
        feederOn = on;
        feeder.setPower(on ? BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER : 0.0);
    }

    public void feederForward() {
        feederOn = true;
        feeder.setPower(BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER);
    }

    /** Runs the feeder backwards to clear a jam. */
    public void feederReverse() {
        feederOn = true;
        feeder.setPower(-BioBuzzConstants.Shooter.FEEDER_REVERSE_POWER);
    }

    public void feederStop() {
        feederOn = false;
        feeder.setPower(0.0);
    }

    public boolean feederRunning() {
        return feederOn;
    }

    // ------------------------------------------------------------------- simple (manual / autonomous) use

    /** Spins the flywheel up to the given speed in ticks per second. */
    public void increaseRpm(long rpm) {
        setTargetVelocity(rpm);
    }

    /**
     * Manual launch: with {@code true} the flywheel runs at the manual speed and the feeder runs once it
     * is fast enough; with {@code false} both stop. Safe to call every loop.
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
     * True once the flywheel is inside its speed window for the current target, or if it has been
     * trying for longer than the spin-up timeout (so a routine waiting on this can never hang).
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

    public void stop() {
        setTargetVelocity(0.0);
        feederStop();
    }
}
