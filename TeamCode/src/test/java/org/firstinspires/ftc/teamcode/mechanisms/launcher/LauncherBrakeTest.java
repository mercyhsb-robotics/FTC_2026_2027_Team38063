package org.firstinspires.ftc.teamcode.mechanisms.launcher;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.junit.Before;
import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Checks that the flywheel BRAKES to a stop when it is told to stop, instead of coasting. The motors are pretend ones
 * that only write down every command they receive, so the test runs on a PC. Run with
 * {@code ./gradlew :TeamCode:testDebugUnitTest}.
 *
 * <p>Why this matters: a flywheel that only has its power cut keeps spinning for seconds, and a loose POLLEN that
 * touches it was thrown out of the field. The fix is to stop with zero power in plain power mode while the motor's
 * zero-power behavior is BRAKE (the motor is shorted and stops fast). These tests pin that exact command sequence.
 */
public class LauncherBrakeTest {
    private final List<String> flywheelLog = new ArrayList<String>();
    private final List<String> feederLog = new ArrayList<String>();
    private Launcher launcher;

    @Before
    public void setUp() {
        launcher = new Launcher();
        launcher.configure(pretendFlywheel(), pretendFeeder());
    }

    /**
     * Situation: the launcher has just been set up.
     * Expected: the flywheel's zero-power behavior was set to BRAKE (the shipped setting {@code Shooter.FLYWHEEL_BRAKE_ON_STOP}
     * is true) and the feeder was told to stand still.
     * Why: BRAKE is what makes zero power stop the wheel instead of letting it coast.
     */
    @Test
    public void theShippedSettingBrakesTheFlywheel() {
        assertTrue("Shooter.FLYWHEEL_BRAKE_ON_STOP must be true: a coasting flywheel threw out a POLLEN that was not meant to be shot",
                BioBuzzConstants.Shooter.FLYWHEEL_BRAKE_ON_STOP);
        assertTrue("the flywheel must be set to BRAKE: " + flywheelLog, flywheelLog.contains("zeroPower BRAKE"));
        assertEquals(Collections.singletonList("power 0.0"), feederLog);
    }

    /**
     * Situation: the flywheel is shooting at 1200 ticks per second and the driver lets go (the speed is set to zero).
     * Expected: the commands are exactly "power 0, switch to plain power mode, power 0" and no speed command follows.
     * Why: that sequence drops any speed target first and then applies zero power to a braking motor.
     */
    @Test
    public void letGoOfShootBrakesInPlainPowerMode() {
        launcher.setTargetVelocity(1200.0);
        assertEquals(Collections.singletonList("velocity 1200.0"), flywheelLogSinceSetUp());
        flywheelLog.clear();

        launcher.setTargetVelocity(0.0);

        assertEquals(Arrays.asList("power 0.0", "mode RUN_WITHOUT_ENCODER", "power 0.0"), flywheelLog);
    }

    /**
     * Situation: the flywheel was stopped (braked) and the driver shoots again.
     * Expected: speed control is switched back on BEFORE the speed command.
     * Why: a speed command in plain power mode would do nothing, so the second shot would never spin up.
     */
    @Test
    public void shootingAgainSwitchesSpeedControlBackOn() {
        launcher.setTargetVelocity(1200.0);
        launcher.setTargetVelocity(0.0);
        flywheelLog.clear();

        launcher.setTargetVelocity(1250.0);

        assertEquals(Arrays.asList("mode RUN_USING_ENCODER", "velocity 1250.0"), flywheelLog);
    }

    /**
     * Situation: the stop command is repeated every loop (as the TeleOp does while no button is held).
     * Expected: only the first one reaches the motor.
     * Why: a command every 20 ms would flood the hub for no reason.
     */
    @Test
    public void repeatedStopsAreNotResent() {
        launcher.setTargetVelocity(1200.0);
        launcher.setTargetVelocity(0.0);
        flywheelLog.clear();

        launcher.setTargetVelocity(0.0);
        launcher.setTargetVelocity(0.0);

        assertTrue(flywheelLog.toString(), flywheelLog.isEmpty());
    }

    /**
     * Situation: the shooter jam button is held (the flywheel runs slowly backward) and then let go.
     * Expected: while held the reverse power is applied; on release the power goes to zero (braking) and speed control is NOT
     * switched back on until a forward speed is asked for.
     * Why: after clearing a jam the wheel must stop at once, not coast, and a forward speed asked for during the reverse is ignored.
     */
    @Test
    public void clearingAJamEndsWithABrake() {
        launcher.setTargetVelocity(1200.0);
        flywheelLog.clear();

        launcher.reverseFlywheel(true);
        assertEquals("power " + (-BioBuzzConstants.Shooter.FLYWHEEL_UNJAM_POWER), flywheelLog.get(flywheelLog.size() - 1));
        assertTrue(launcher.flywheelReversing());

        flywheelLog.clear();
        launcher.setTargetVelocity(1200.0);
        assertTrue("a forward speed must be ignored while reversing: " + flywheelLog, flywheelLog.isEmpty());

        launcher.reverseFlywheel(false);
        assertEquals(Collections.singletonList("power 0.0"), flywheelLog);
        assertFalse(launcher.flywheelReversing());
        assertFalse("no speed control until a speed is asked for", flywheelLog.contains("mode RUN_USING_ENCODER"));

        flywheelLog.clear();
        launcher.setTargetVelocity(1200.0);
        assertEquals(Arrays.asList("mode RUN_USING_ENCODER", "velocity 1200.0"), flywheelLog);
    }

    /**
     * Situation: {@code stop()} while shooting.
     * Expected: the flywheel brakes and the feeder power is zero.
     * Why: STOP at the end of the match must leave nothing spinning and nothing feeding.
     */
    @Test
    public void stopBrakesTheFlywheelAndStopsTheFeeder() {
        launcher.setTargetVelocity(1200.0);
        launcher.setFeederOn(true);
        flywheelLog.clear();
        feederLog.clear();

        launcher.stop();

        assertEquals(Arrays.asList("power 0.0", "mode RUN_WITHOUT_ENCODER", "power 0.0"), flywheelLog);
        assertEquals(Collections.singletonList("power 0.0"), feederLog);
        assertEquals(0.0, launcher.targetVelocity(), 0.0);
    }

    // The log of the commands the flywheel received after set-up (set-up itself is checked in its own test).
    private List<String> flywheelLogSinceSetUp() {
        List<String> afterSetUp = new ArrayList<String>(flywheelLog);
        afterSetUp.removeAll(Arrays.asList("mode RUN_USING_ENCODER", "zeroPower BRAKE", "zeroPower FLOAT", "power 0.0"));
        return afterSetUp;
    }

    // ---- pretend devices: java.lang.reflect.Proxy answers every method call; the ones that matter are written down ----

    private DcMotorEx pretendFlywheel() {
        return (DcMotorEx) Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
            String name = method.getName();
            if (name.equals("setMode")) {
                flywheelLog.add("mode " + args[0]);
            } else if (name.equals("setPower")) {
                flywheelLog.add("power " + args[0]);
            } else if (name.equals("setVelocity") && args.length == 1) {
                flywheelLog.add("velocity " + args[0]);
            } else if (name.equals("setZeroPowerBehavior")) {
                flywheelLog.add("zeroPower " + args[0]);
            }
            return defaultFor(method.getReturnType());
        });
    }

    private CRServo pretendFeeder() {
        return (CRServo) Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[]{CRServo.class}, (proxy, method, args) -> {
            if (method.getName().equals("setPower")) {
                feederLog.add("power " + args[0]);
            }
            return defaultFor(method.getReturnType());
        });
    }

    private static Object defaultFor(Class<?> type) {
        if (type == boolean.class) {
            return false;
        } else if (type == double.class) {
            return 0.0;
        } else if (type == int.class) {
            return 0;
        } else if (type == long.class) {
            return 0L;
        } else if (type == float.class) {
            return 0.0f;
        }
        return null;
    }
}
