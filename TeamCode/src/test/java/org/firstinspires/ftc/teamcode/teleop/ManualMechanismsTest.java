package org.firstinspires.ftc.teamcode.teleop;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.junit.Before;
import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.ManualRules;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;

/**
 * Checks what {@link ManualMechanisms} tells the motors and servos for each decision of the manual controls, using pretend
 * devices that only write down the commands they get (so the test runs on a PC). Run with {@code ./gradlew :TeamCode:testDebugUnitTest}.
 *
 * <p>{@code ManualRulesTest} checks the decisions; {@code LauncherBrakeTest} checks the flywheel stop. This test checks the wiring
 * between them: that "nothing held" really ends in a braked flywheel and a stopped feeder, that the feeder waits for the flywheel to
 * be fast enough, and that each reverse button turns the right mechanism the right way.
 */
public class ManualMechanismsTest {
    private final List<String> flywheelLog = new ArrayList<String>();
    private final List<String> feederLog = new ArrayList<String>();
    private final List<String> rollerLog = new ArrayList<String>();
    private final List<String> leftLog = new ArrayList<String>();
    private final List<String> rightLog = new ArrayList<String>();
    private double pretendFlywheelSpeed;
    private ManualMechanisms mechanisms;

    @Before
    public void setUp() {
        Launcher launcher = new Launcher();
        launcher.configure(pretendFlywheel(), pretendServo(feederLog));
        Intake intake = new Intake();
        intake.configure(pretendRoller());
        mechanisms = new ManualMechanisms(launcher, intake, pretendServo(leftLog), pretendServo(rightLog));
        clearLogs();
    }

    /**
     * Situation: the driver was shooting and lets go of everything.
     * Expected: the flywheel gets the braking commands (zero power in plain power mode) and the feeder, roller and corner servos all get zero.
     * Why: this is the user's requirement: a flywheel that is no longer being asked to shoot must stop, not coast.
     */
    @Test
    public void releasingTheShootTriggerBrakesTheFlywheelAndStopsTheFeeder() {
        pretendFlywheelSpeed = 1500.0;
        mechanisms.apply(decide(true, false, false, false, false, false), 1200.0);
        clearLogs();

        mechanisms.apply(decide(false, false, false, false, false, false), 1200.0);

        assertEquals(Arrays.asList("power 0.0", "mode RUN_WITHOUT_ENCODER", "power 0.0"), flywheelLog);
        assertEquals("power 0.0", last(feederLog));
        assertEquals("power 0.0", last(rollerLog));
        assertEquals("power 0.0", last(leftLog));
        assertEquals("power 0.0", last(rightLog));
        assertFalse(mechanisms.shooting());
    }

    /**
     * Situation: the shoot trigger is held while the flywheel is slow, then fast, at the normal speed and at a trimmed speed.
     * Expected: the feeder stays off until the flywheel is above the gate for that speed (950 at the normal 1200, higher at 1600).
     * Why: the first ball must not be fed before the wheel is fast enough, and the gate must follow the trimmed speed.
     */
    @Test
    public void theFeederWaitsForTheFlywheelToBeFastEnough() {
        double feed = BioBuzzConstants.Shooter.FEEDER_FORWARD_POWER;

        pretendFlywheelSpeed = 500.0;
        mechanisms.apply(decide(true, false, false, false, false, false), 1200.0);
        assertFalse("too slow to feed", mechanisms.feeding());
        assertFalse(feederLog.contains("power " + feed));
        assertEquals("velocity 1200.0", last(flywheelLog));

        pretendFlywheelSpeed = 1000.0;
        mechanisms.apply(decide(true, false, false, false, false, false), 1200.0);
        assertTrue("1000 is above the gate of 950", mechanisms.feeding());
        assertEquals("power " + feed, last(feederLog));

        clearLogs();
        pretendFlywheelSpeed = 1000.0;
        mechanisms.apply(decide(true, false, false, false, false, false), 1600.0);
        assertFalse("1000 is far too slow for a shot at 1600", mechanisms.feeding());
        assertEquals("power 0.0", last(feederLog));

        pretendFlywheelSpeed = 1400.0;
        mechanisms.apply(decide(true, false, false, false, false, false), 1600.0);
        assertTrue("1400 is above the gate for 1600", mechanisms.feeding());
    }

    /**
     * Situation: the two jam buttons (shooter wheel and indexer) are held, alone and together.
     * Expected: the flywheel runs backward at the unjam power and the feeder at the reverse power, each only when its own button is held.
     * Why: B must reverse the flywheel and the right bumper the feeder, and neither may start shooting.
     */
    @Test
    public void theJamButtonsReverseTheFlywheelAndTheFeeder() {
        mechanisms.apply(decide(true, false, false, true, false, false), 1200.0);
        assertEquals("power " + (-BioBuzzConstants.Shooter.FLYWHEEL_UNJAM_POWER), last(flywheelLog));
        assertEquals("power 0.0", last(feederLog));
        assertTrue(mechanisms.unjammingShooter());
        assertFalse("a jam button beats shooting", mechanisms.shooting());

        clearLogs();
        mechanisms.apply(decide(false, false, true, false, false, false), 1200.0);
        assertEquals("power " + (-BioBuzzConstants.Shooter.FEEDER_REVERSE_POWER), last(feederLog));
        assertTrue(mechanisms.unjammingIndexer());
    }

    /**
     * Situation: the intake trigger, then each intake reverse button.
     * Expected: pulling in gives the roller +1 and the corner servos their pull-in signs (left -1, right +1); reversing gives the roller the
     * negative reverse power and each corner servo the OPPOSITE of its pull-in sign at the feeler reverse power.
     * Why: a servo that reversed the wrong way would pull POLLEN in when the driver wants one pushed out.
     */
    @Test
    public void theIntakeAndItsReverseTurnTheRightWay() {
        mechanisms.apply(decide(false, true, false, false, false, false), 1200.0);
        assertEquals("power 1.0", last(rollerLog));
        assertEquals("power " + BioBuzzConstants.Run.INTAKE_LEFT_SERVO_POWER, last(leftLog));
        assertEquals("power " + BioBuzzConstants.Run.INTAKE_RIGHT_SERVO_POWER, last(rightLog));
        assertTrue(mechanisms.intaking());

        clearLogs();
        mechanisms.apply(decide(false, false, false, false, true, true), 1200.0);
        assertEquals("power " + (-BioBuzzConstants.Controls.INTAKE_REVERSE_POWER), last(rollerLog));
        double leftExpected = -BioBuzzConstants.Controls.FEELER_REVERSE_POWER * BioBuzzConstants.Run.INTAKE_LEFT_SERVO_POWER;
        double rightExpected = -BioBuzzConstants.Controls.FEELER_REVERSE_POWER * BioBuzzConstants.Run.INTAKE_RIGHT_SERVO_POWER;
        assertEquals("power " + leftExpected, last(leftLog));
        assertEquals("power " + rightExpected, last(rightLog));
        assertTrue("the left servo reverses opposite to its pull-in sign", leftExpected * BioBuzzConstants.Run.INTAKE_LEFT_SERVO_POWER < 0.0);
        assertTrue("the right servo reverses opposite to its pull-in sign", rightExpected * BioBuzzConstants.Run.INTAKE_RIGHT_SERVO_POWER < 0.0);
        assertTrue(mechanisms.reversingRoller());
        assertTrue(mechanisms.reversingFeelers());
        assertFalse(mechanisms.intaking());
    }

    /**
     * Situation: {@code clearStatus()} after a loop that was reversing.
     * Expected: the screen flags are cleared and no command is sent to any device.
     * Why: BioBuzzRobot calls it while the automatic shooting owns the mechanisms, so the screen shows no old state.
     */
    @Test
    public void clearStatusTouchesNoHardware() {
        mechanisms.apply(decide(false, false, false, false, true, true), 1200.0);
        clearLogs();

        mechanisms.clearStatus();

        assertFalse(mechanisms.reversingRoller());
        assertFalse(mechanisms.reversingFeelers());
        assertTrue(flywheelLog.isEmpty() && feederLog.isEmpty() && rollerLog.isEmpty() && leftLog.isEmpty() && rightLog.isEmpty());
    }

    private static ManualRules.Decision decide(boolean shoot, boolean intake, boolean indexerJam, boolean shooterJam,
                                               boolean rollerReverse, boolean feelerReverse) {
        return ManualRules.decide(shoot, intake, indexerJam, shooterJam, rollerReverse, feelerReverse);
    }

    private void clearLogs() {
        flywheelLog.clear();
        feederLog.clear();
        rollerLog.clear();
        leftLog.clear();
        rightLog.clear();
    }

    private static String last(List<String> log) {
        return log.isEmpty() ? "(nothing)" : log.get(log.size() - 1);
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
            } else if (name.equals("getVelocity") && (args == null || args.length == 0)) {
                return pretendFlywheelSpeed;
            }
            return defaultFor(method.getReturnType());
        });
    }

    private DcMotor pretendRoller() {
        return (DcMotor) Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[]{DcMotor.class}, (proxy, method, args) -> {
            if (method.getName().equals("setPower")) {
                rollerLog.add("power " + args[0]);
            }
            return defaultFor(method.getReturnType());
        });
    }

    private CRServo pretendServo(List<String> log) {
        return (CRServo) Proxy.newProxyInstance(getClass().getClassLoader(), new Class<?>[]{CRServo.class}, (proxy, method, args) -> {
            if (method.getName().equals("setPower")) {
                log.add("power " + args[0]);
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
