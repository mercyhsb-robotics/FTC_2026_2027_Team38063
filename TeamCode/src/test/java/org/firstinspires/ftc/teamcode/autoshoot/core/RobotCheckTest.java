package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheck.DriveKind;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheck.StepId;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheck.Verdict;

/**
 * Tests the guided robot check ({@link RobotCheck}, {@link RobotCheckJudge}, {@link RobotCheckConfig}) with no robot:
 * a small pretend operator ({@code Run}) presses buttons, moves the clock and sets the readings, and the tests read
 * back the text and the motor commands.
 *
 * <p>What is covered: every step's PASS, WARN, FAIL and SKIPPED paths, every piece of advice text (each test checks the
 * key words of the advice: the file, the setting or the wire the student is sent to), the answer 'no', a missing
 * Limelight or missing devices, the button rules (edges, lockout, the safety step cannot be skipped), the fail-safe
 * rules (no motor before a press, motors off when a part ends, powers clamped), the summary and its 'All clear' rule,
 * and the validation of the constants.
 *
 * <p>Words used in this file. <b>Edge:</b> the moment a button goes from up to down. <b>Lockout:</b> the short pause
 * after a screen change during which presses are ignored. <b>Run:</b> the pretend operator below. A <b>tick</b> is
 * one pass of the robot loop: the pretend clock moves by 20 milliseconds each time.
 *
 * <p>Run it with {@code ./gradlew :TeamCode:testDebugUnitTest} (add {@code --tests "*RobotCheckTest"} to run only this
 * class).
 */
public class RobotCheckTest {
    /** One tick of the pretend clock, in seconds (the real loop runs about this fast). */
    private static final double DT = 0.02;

    // =============================================================================== the pretend operator

    /** Presses buttons, advances time, sets readings, and keeps a record of everything the check showed and commanded. */
    private static final class Run {
        final RobotCheckConfig cfg;
        final RobotCheck check;
        final RobotCheck.Inputs in = new RobotCheck.Inputs();
        double t;
        RobotCheck.Outputs last;
        /** Every line ever shown, for the "no jargon" guard. */
        final List<String> everyLine = new ArrayList<String>();
        /** When the yes button was last pressed: a motor may only be running soon after a press. */
        double lastYesPress = -1000.0;
        /** If true, the camera frame counter grows by one on every tick (a camera that is sending pictures). */
        boolean autoFrames;
        /** The largest motor power ever commanded. */
        double maxPower;

        Run() {
            this(RobotCheckConfig.fromConstants());
        }

        Run(RobotCheckConfig cfg) {
            this.cfg = cfg;
            this.check = new RobotCheck(cfg);
            this.in.poseAvailable = true;
            this.in.batteryVolts = 12.8;
            this.in.visionConnected = true;
            tick();
        }

        RobotCheck.Outputs tick() {
            t += DT;
            in.nowSec = t;
            if (autoFrames) {
                in.visionFrames++;
            }
            last = check.update(in);
            everyLine.addAll(last.lines);
            RobotCheck.Commands c = last.commands;
            double biggest = Math.max(Math.max(Math.abs(c.wheelPower), Math.abs(c.intakeMotorPower)),
                    Math.max(Math.max(Math.abs(c.leftServoPower), Math.abs(c.rightServoPower)), Math.abs(c.feederPower)));
            maxPower = Math.max(maxPower, biggest);
            assertTrue("a power above the limit: " + biggest, biggest <= cfg.powerLimit + 1e-9);
            if (c.anyOn()) {
                // Fail safe: a motor may only run shortly after the student pressed the yes button (the longest run is a few seconds).
                assertTrue("a motor ran without a press, at t=" + t, t - lastYesPress < 6.0);
            }
            return last;
        }

        void wait(double seconds) {
            int n = (int) Math.ceil(seconds / DT) + 1;
            for (int i = 0; i < n; i++) {
                tick();
            }
        }

        /** Holds a button for two ticks, then lets go (a real press: one edge). */
        void press(Button b) {
            if (b == cfg.yes) {
                lastYesPress = t + DT;
            }
            in.buttons.add(b);
            tick();
            tick();
            in.buttons.remove(b);
            tick();
        }

        /** Waits out the lockout, then presses. */
        void go(Button b) {
            wait(cfg.lockoutSec + 0.05);
            press(b);
        }

        void yes() {
            go(cfg.yes);
        }

        void no() {
            go(cfg.no);
        }

        void skip() {
            go(cfg.skip);
        }

        void repeat() {
            go(cfg.repeat);
        }

        StepId step() {
            return check.currentStep();
        }

        /** Confirms safety, then skips steps until the wanted step is showing. */
        void toStep(StepId target) {
            yes();
            int guard = 0;
            while (step() != target && guard++ < 20) {
                if (shows("Result: ")) {
                    yes();          // a step that already finished (for example a missing device): go on
                } else {
                    skip();
                }
            }
            assertEquals(target, step());
        }

        String text() {
            StringBuilder sb = new StringBuilder();
            for (String s : last.lines) {
                sb.append(s).append("\n");
            }
            return sb.toString();
        }

        boolean shows(String fragment) {
            return text().contains(fragment);
        }

        Verdict verdict(StepId id) {
            return check.verdict(id);
        }
    }

    private static void assertShows(Run r, String fragment) {
        assertTrue("expected the screen to show '" + fragment + "' but it shows:\n" + r.text(), r.shows(fragment));
    }

    private static void assertNotShows(Run r, String fragment) {
        assertFalse("the screen must not show '" + fragment + "' but it shows:\n" + r.text(), r.shows(fragment));
    }

    private static void assertContains(String what, String text, String fragment) {
        assertTrue(what + " should contain '" + fragment + "' but is: " + text, text.contains(fragment));
    }

    // ===================================================================================== helpers for steps

    private static TagObservation tag(int id) {
        return TagObservation.withPosition(id, 5.0, -2.0, 1.0, 0.0, 30.0, 0.0, 50.0);
    }

    private static VisionFrame frame(VisionFrame.Status status, double now, TagObservation... tags) {
        return new VisionFrame(status, true, 0.02, now, Arrays.asList(tags));
    }

    /** Runs one trial of a trial step: start, run for its time, then answer. */
    private static void trial(Run r, double seconds, boolean answerYes) {
        r.yes();
        r.wait(seconds);
        if (answerYes) {
            r.yes();
        } else {
            r.no();
        }
    }

    private static void passWheels(Run r) {
        for (int i = 0; i < 4; i++) {
            trial(r, r.cfg.driveSec, true);
        }
    }

    private static void passIntake(Run r) {
        for (int i = 0; i < 3; i++) {
            trial(r, r.cfg.intakeSec, true);
        }
    }

    private static void passFlywheel(Run r) {
        r.yes();
        r.in.flywheelTps = r.cfg.flywheelTargetTps;
        r.wait(0.5);
    }

    private static void passLimelight(Run r) {
        r.autoFrames = true;
        r.in.frame = frame(VisionFrame.Status.OK, r.t);
        r.yes();
        r.wait(r.cfg.limelightCheckSec + 0.2);
        r.in.frame = frame(VisionFrame.Status.OK, r.t, tag(34));
        r.tick();
    }

    private static void passGamepad(Run r) {
        r.yes();
        for (Button b : r.cfg.gamepadButtons) {
            r.wait(0.1);
            r.press(b);
        }
        r.wait(0.1);
        r.in.leftTrigger = true;
        r.tick();
        r.in.leftTrigger = false;
        r.tick();
        r.in.rightTrigger = true;
        r.tick();
        r.in.rightTrigger = false;
        r.tick();
    }

    /** Pushes the robot, turns it, and answers: a clean pass of the Pinpoint step. */
    private static void passPinpoint(Run r) {
        r.in.poseXIn = 0.0;
        r.in.poseYIn = 0.0;
        r.in.poseHeadingRad = 0.0;
        r.yes();
        r.in.poseXIn = r.cfg.pushDistanceIn;
        r.yes();
        r.in.poseHeadingRad = Math.toRadians(r.cfg.turnDeg);
        r.yes();
    }

    /** A complete, clean run of every step. Leaves the check at the summary. */
    private static Run passingRun() {
        Run r = new Run();
        r.yes();                                  // safety -> battery
        r.yes();                                  // read the battery
        r.yes();                                  // next
        r.yes();                                  // check the names
        r.yes();                                  // next -> wheels
        passWheels(r);
        r.yes();                                  // next -> Pinpoint
        passPinpoint(r);
        r.yes();                                  // next -> intake
        passIntake(r);
        r.yes();                                  // next -> flywheel
        passFlywheel(r);
        r.yes();                                  // next -> feeder
        trial(r, r.cfg.feederForwardSec + r.cfg.feederReverseSec, true);
        r.yes();                                  // next -> Limelight
        passLimelight(r);
        r.yes();                                  // next -> gamepad
        passGamepad(r);
        r.yes();                                  // next -> summary
        return r;
    }

    // ============================================================================ the safety screen and buttons

    /**
     * Situation: the check has just started.
     * Expected: the safety screen (step 0 of 10) shows the four safety rules and "Press A to start the check.",
     *     and no motor is commanded.
     * Why: nothing may move before the team has read the safety rules and pressed the start button.
     */
    @Test
    public void startsOnTheSafetyScreenWithEverythingOff() {
        Run r = new Run();
        assertEquals(StepId.SAFETY, r.step());
        assertEquals("Step 0 of 10: Safety first", r.last.title);
        assertShows(r, "Put the robot on blocks");
        assertShows(r, "POLLEN");
        assertShows(r, "STOP");
        assertShows(r, "Press A to start the check.");
        assertFalse(r.last.commands.anyOn());
    }

    /**
     * Situation: on the safety screen the skip button, then the no button, are pressed.
     * Expected: the safety step stays (skipping is refused with a message; 'no' says take your time), and only the yes
     *     button moves on, recording a PASS for the safety step.
     * Why: the safety check is the one step that must never be skipped.
     */
    @Test
    public void theSafetyStepCannotBeSkippedAndNoJustWaits() {
        Run r = new Run();
        r.skip();
        assertEquals(StepId.SAFETY, r.step());
        assertShows(r, "The safety step cannot be skipped.");
        r.no();
        assertEquals(StepId.SAFETY, r.step());
        assertShows(r, "take your time");
        assertEquals(Verdict.NOT_RUN, r.verdict(StepId.SAFETY));
        r.yes();
        assertEquals(StepId.BATTERY, r.step());
        assertEquals(Verdict.PASS, r.verdict(StepId.SAFETY));
    }

    /**
     * Situation: the yes button is held down for a whole second on the safety screen.
     * Expected: the check goes to the battery screen once and stops there; the held button does not also start the battery
     *     step.
     * Why: only the moment a button goes down counts, so a held button never repeats and never skips a screen.
     */
    @Test
    public void aHeldButtonActsOnlyOnce() {
        Run r = new Run();
        r.wait(r.cfg.lockoutSec + 0.05);
        r.in.buttons.add(r.cfg.yes);
        r.wait(1.0);
        assertEquals(StepId.BATTERY, r.step());
        assertEquals(Verdict.NOT_RUN, r.verdict(StepId.BATTERY));
    }

    /**
     * Situation: right after the safety screen the yes button is pressed again at once (a double click), then again after
     *     the lockout.
     * Expected: the first press is ignored (the battery step has no result), the second one reads the battery.
     * Why: a double click must not race through two screens, and above all must not start a motor by accident.
     */
    @Test
    public void aPressRightAfterAScreenChangeIsIgnored() {
        Run r = new Run();
        r.yes();
        r.press(r.cfg.yes);
        assertEquals(Verdict.NOT_RUN, r.verdict(StepId.BATTERY));
        r.yes();
        assertEquals(Verdict.PASS, r.verdict(StepId.BATTERY));
    }

    // ===================================================================================== 1. battery

    /**
     * Situation: the battery reads 12.8, exactly the pass voltage, then 12.0, exactly the fail voltage, then 10.5 volts
     *     (and the pass / fail limits are the ones in BioBuzzConstants.RobotCheck).
     * Expected: PASS at or above the pass voltage, WARN from the fail voltage up to it, FAIL below the fail voltage.
     * Why: PASS at or above a threshold, WARN lower, FAIL much lower.
     */
    @Test
    public void batteryPassWarnAndFail() {
        RobotCheckJudge j = new RobotCheckJudge(RobotCheckConfig.fromConstants());
        RobotCheckConfig c = RobotCheckConfig.fromConstants();
        assertEquals(Verdict.PASS, j.battery(13.0).verdict);
        assertEquals(Verdict.PASS, j.battery(c.batteryPassVolts).verdict);
        assertEquals(Verdict.WARN, j.battery(c.batteryPassVolts - 0.01).verdict);
        assertEquals(Verdict.WARN, j.battery(c.batteryFailVolts).verdict);
        assertEquals(Verdict.FAIL, j.battery(c.batteryFailVolts - 0.01).verdict);
        assertContains("pass text", j.battery(13.0).detail, "13.0 volts");
        assertEquals("", j.battery(13.0).advice);
        assertContains("warn advice", j.battery(12.0).advice, "fully charged battery");
        assertContains("fail advice", j.battery(10.5).advice, "fully charged battery now");
    }

    /**
     * Situation: the Control Hub reports no battery voltage (NaN, then 0).
     * Expected: a WARN that says the voltage could not be read, and says what to do.
     * Why: a missing reading is not a pass, but it must not stop the check either.
     */
    @Test
    public void aBatteryThatCannotBeReadIsAWarning() {
        RobotCheckJudge j = new RobotCheckJudge(RobotCheckConfig.fromConstants());
        RobotCheckJudge.Judgement nan = j.battery(Double.NaN);
        assertEquals(Verdict.WARN, nan.verdict);
        assertContains("detail", nan.detail, "did not report a battery voltage");
        assertContains("advice", nan.advice, "plugged in and charged");
        assertEquals(Verdict.WARN, j.battery(0.0).verdict);
    }

    /**
     * Situation: the battery step is run through the screens with 12.8 volts, then repeated with a weak battery.
     * Expected: the ready screen explains the step and does not move the robot; the result screen says PASS and shows
     *     the voltage; after the repeat button and a new reading the step shows FAIL with its advice.
     * Why: checks the whole ready, result, repeat flow of an instant step.
     */
    @Test
    public void theBatteryStepFlowsFromReadyToResultAndCanBeRepeated() {
        Run r = new Run();
        r.yes();
        assertEquals("Step 1 of 10: Battery", r.last.title);
        assertShows(r, "The robot will not move.");
        assertShows(r, "Press A to read it, or Y to skip this step.");
        r.yes();
        assertShows(r, "Result: PASS.");
        assertShows(r, "12.8 volts");
        assertShows(r, "Press A for the next step, or X to do this step again.");
        r.in.batteryVolts = 10.0;
        r.repeat();
        assertShows(r, "Press A to read it");
        r.yes();
        assertShows(r, "Result: FAIL.");
        assertShows(r, "What to do: Put in a fully charged battery now");
        assertEquals(Verdict.FAIL, r.verdict(StepId.BATTERY));
    }

    /**
     * Situation: the battery step is skipped.
     * Expected: SKIPPED with advice to run it again, and the check moves on to the hardware names.
     * Why: skipping works on every step except safety.
     */
    @Test
    public void skippingTheBatteryStepRecordsSkipped() {
        Run r = new Run();
        r.yes();
        r.skip();
        assertEquals(Verdict.SKIPPED, r.verdict(StepId.BATTERY));
        assertContains("advice", r.check.advice(StepId.BATTERY), "Run this check again");
        assertEquals(StepId.HARDWARE_NAMES, r.step());
    }

    // ============================================================================ 2. hardware names

    /**
     * Situation: every device is in the robot configuration.
     * Expected: PASS saying all 12 names were found.
     * Why: the normal, healthy case.
     */
    @Test
    public void allNamesFoundIsAPass() {
        Run r = new Run();
        r.toStep(StepId.HARDWARE_NAMES);
        r.yes();
        assertShows(r, "Result: PASS.");
        assertShows(r, "All 12 names from BioBuzzConstants.Hardware");
    }

    /**
     * Situation: the feeder servo is not in the robot configuration.
     * Expected: FAIL; the screen says the robot configuration has no device called 'feeder' and says to open the Driver
     *     Station > Configure Robot; the advice names it.
     * Why: this is the example message in the brief: plain words, and where to fix it.
     */
    @Test
    public void aMissingFeederIsAFailWithPlainAdvice() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.FEEDER);
        r.toStep(StepId.HARDWARE_NAMES);
        r.yes();
        assertEquals(Verdict.FAIL, r.verdict(StepId.HARDWARE_NAMES));
        assertShows(r, "The robot configuration has no device called 'feeder'");
        assertShows(r, "the feeder servo");
        assertShows(r, "continuous-rotation servo");
        assertShows(r, "Driver Station > Configure Robot");
        assertContains("advice", r.check.advice(StepId.HARDWARE_NAMES), "'feeder'");
        assertContains("advice", r.check.advice(StepId.HARDWARE_NAMES), "same spelling and capital letters");
    }

    /**
     * Situation: only the IMU, only the Limelight, and then two required devices are missing.
     * Expected: WARN, WARN, FAIL; the FAIL lists both names.
     * Why: the IMU (backup drive only) and the camera are optional; a missing wheel or the Pinpoint is not.
     */
    @Test
    public void missingOptionalDevicesWarnAndMissingRequiredOnesFail() {
        RobotCheckJudge j = new RobotCheckJudge(RobotCheckConfig.fromConstants());
        assertEquals(Verdict.WARN, j.names(12, Arrays.asList(RobotCheckDevice.IMU)).verdict);
        assertEquals(Verdict.WARN, j.names(12, Arrays.asList(RobotCheckDevice.LIMELIGHT)).verdict);
        RobotCheckJudge.Judgement two = j.names(12, Arrays.asList(RobotCheckDevice.FLYWHEEL, RobotCheckDevice.LIMELIGHT));
        assertEquals(Verdict.FAIL, two.verdict);
        assertContains("detail", two.detail, "'flywheel'");
        assertContains("detail", two.detail, "'limelight'");
        assertContains("advice", two.advice, "'flywheel', 'limelight'");
    }

    // ========================================================================================= 3. wheels

    /**
     * Situation: a normal Pedro run of the wheel step, answering yes for each wheel.
     * Expected: nothing moves on the ready screens; each press of A spins exactly one wheel, in the order front-left,
     *     back-left, front-right, back-right, at the gentle power for the set time; the wheel stops by itself and the
     *     question appears; four yes answers give PASS.
     * Why: the brief: spin each wheel one at a time, gently, 0.6 seconds, forward.
     */
    @Test
    public void eachWheelSpinsAloneInOrderAndStopsByItself() {
        Run r = new Run();
        r.toStep(StepId.DRIVE_WHEELS);
        assertEquals("Step 3 of 10: Drive wheels", r.last.title);
        assertShows(r, "Drive in use: Pedro with the Pinpoint");
        assertShows(r, "Next: the front-left wheel (the wheel at the FRONT on the robot's LEFT side)");
        r.wait(2.0);
        assertFalse("no motor before a press", r.last.commands.anyOn());
        int[] order = {0, 1, 2, 3};
        for (int wheel : order) {
            assertShows(r, "Next: the " + RobotCheckText.wheel(wheel) + " wheel");
            r.wait(r.cfg.lockoutSec + 0.05);
            r.press(r.cfg.yes);
            assertEquals(wheel, r.last.commands.wheel);
            assertEquals(r.cfg.drivePower, r.last.commands.wheelPower, 1e-9);
            assertShows(r, "Spinning the " + RobotCheckText.wheel(wheel) + " wheel now");
            r.wait(r.cfg.driveSec + 0.1);
            assertEquals(RobotCheck.NO_WHEEL, r.last.commands.wheel);
            assertFalse(r.last.commands.anyOn());
            assertShows(r, "Did the " + RobotCheckText.wheel(wheel) + " wheel turn FORWARD");
            r.yes();
        }
        assertShows(r, "Result: PASS.");
        assertShows(r, "All 4 wheels turned forward.");
        assertFalse(r.last.commands.anyOn());
    }

    /**
     * Situation: the TeleOp is on the backup drive (Pedro could not start) and the front-left wheel is tested.
     * Expected: the screen says the backup drive is in use (with the reason) and the wheel gets NEGATIVE power, because the
     *     backup drive drives forward with negative power.
     * Why: "use the same direction setup the TeleOp really uses".
     */
    @Test
    public void theBackupDriveIsTestedWithNegativePower() {
        Run r = new Run();
        r.in.driveKind = DriveKind.LEGACY;
        r.in.driveNote = "no Pinpoint";
        r.toStep(StepId.DRIVE_WHEELS);
        assertShows(r, "BACKUP drive (MecanumDrive), because Pedro could not start (no Pinpoint).");
        r.yes();
        assertEquals(0, r.last.commands.wheel);
        assertEquals(-r.cfg.drivePower, r.last.commands.wheelPower, 1e-9);
    }

    /**
     * Situation: the backup drive is in use and no reason was given.
     * Expected: the screen still says the backup drive is in use, without an empty pair of brackets.
     * Why: the text must read well whether or not the OpMode knows why Pedro could not start.
     */
    @Test
    public void theBackupDriveNoteWorksWithoutAReason() {
        Run r = new Run();
        r.in.driveKind = DriveKind.LEGACY;
        r.toStep(StepId.DRIVE_WHEELS);
        assertShows(r, "BACKUP drive (MecanumDrive), because Pedro could not start.");
    }

    /**
     * Situation: the front-left wheel is answered no, with Pedro in use; then with the backup drive; then two wheels fail.
     * Expected: FAIL; the Pedro advice names pedro/Constants.java, drivetrainConfig and c.frontLeftDirection; the backup
     *     advice names MecanumDrive.java and frontLeftMotor.setDirection; two failures give both pieces of advice.
     * Why: on 'no' the student must be told which direction setting to flip and in which file.
     */
    @Test
    public void aWheelAnsweredNoTellsWhichDirectionToFlipAndWhere() {
        Run pedro = new Run();
        pedro.toStep(StepId.DRIVE_WHEELS);
        trial(pedro, pedro.cfg.driveSec, false);
        for (int i = 0; i < 3; i++) {
            trial(pedro, pedro.cfg.driveSec, true);
        }
        assertEquals(Verdict.FAIL, pedro.verdict(StepId.DRIVE_WHEELS));
        assertShows(pedro, "These wheels did not turn forward: front-left.");
        String advice = pedro.check.advice(StepId.DRIVE_WHEELS);
        assertContains("advice", advice, "pedro/Constants.java");
        assertContains("advice", advice, "drivetrainConfig");
        assertContains("advice", advice, "c.frontLeftDirection");
        assertContains("advice", advice, "FORWARD becomes REVERSE");
        assertContains("advice", advice, "DIFFERENT wheel");

        Run legacy = new Run();
        legacy.in.driveKind = DriveKind.LEGACY;
        legacy.toStep(StepId.DRIVE_WHEELS);
        for (int i = 0; i < 3; i++) {
            trial(legacy, legacy.cfg.driveSec, true);
        }
        trial(legacy, legacy.cfg.driveSec, false);
        String legacyAdvice = legacy.check.advice(StepId.DRIVE_WHEELS);
        assertContains("advice", legacyAdvice, "mechanisms/mecanumdrive/MecanumDrive.java");
        assertContains("advice", legacyAdvice, "backRightMotor.setDirection");

        Run two = new Run();
        two.toStep(StepId.DRIVE_WHEELS);
        trial(two, two.cfg.driveSec, false);
        trial(two, two.cfg.driveSec, true);
        trial(two, two.cfg.driveSec, true);
        trial(two, two.cfg.driveSec, false);
        assertShows(two, "front-left, back-right");
        assertContains("advice", two.check.advice(StepId.DRIVE_WHEELS), "c.frontLeftDirection");
        assertContains("advice", two.check.advice(StepId.DRIVE_WHEELS), "c.backRightDirection");
    }

    /**
     * Situation: after a wheel has spun, the repeat button is pressed on the question.
     * Expected: the same wheel's ready screen comes back, no motor runs until A is pressed again, and the answer is
     *     asked for that same wheel.
     * Why: a 0.6 second spin is easy to miss; the student must be able to see it again, with a fresh press.
     */
    @Test
    public void aWheelCanBeSpunAgain() {
        Run r = new Run();
        r.toStep(StepId.DRIVE_WHEELS);
        r.yes();
        r.wait(r.cfg.driveSec + 0.1);
        r.repeat();
        assertShows(r, "Next: the front-left wheel");
        r.wait(1.0);
        assertFalse(r.last.commands.anyOn());
        r.yes();
        assertEquals(0, r.last.commands.wheel);
    }

    /**
     * Situation: the no button is pressed while a wheel is spinning.
     * Expected: the wheel stops at once and the question is shown.
     * Why: a student must be able to stop a motor without reaching for the Driver Station.
     */
    @Test
    public void noStopsARunningWheelAtOnce() {
        Run r = new Run();
        r.toStep(StepId.DRIVE_WHEELS);
        r.yes();
        r.wait(0.1);
        assertTrue(r.last.commands.anyOn());
        r.press(r.cfg.no);
        assertFalse(r.last.commands.anyOn());
        assertShows(r, "Did the front-left wheel turn FORWARD");
    }

    /**
     * Situation: the wheel step is skipped from its ready screen, and also from a question.
     * Expected: SKIPPED each time, and no motor runs in the first case.
     * Why: skip works at any point of a step that has not finished.
     */
    @Test
    public void theWheelStepCanBeSkipped() {
        Run r = new Run();
        r.toStep(StepId.DRIVE_WHEELS);
        r.skip();
        assertEquals(Verdict.SKIPPED, r.verdict(StepId.DRIVE_WHEELS));
        assertEquals(StepId.PINPOINT, r.step());

        Run q = new Run();
        q.toStep(StepId.DRIVE_WHEELS);
        q.yes();
        q.wait(q.cfg.driveSec + 0.1);
        q.skip();
        assertEquals(Verdict.SKIPPED, q.verdict(StepId.DRIVE_WHEELS));
        assertFalse(q.last.commands.anyOn());
    }

    /**
     * Situation: a wheel motor is missing from the robot configuration.
     * Expected: the wheel step fails at once, without running any motor, and says to fix the missing name first.
     * Why: the other wheels would be tested with unreliable directions, and there is nothing to test on the missing one.
     */
    @Test
    public void aMissingWheelMotorFailsTheWheelStepWithoutRunningAnything() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.FRONT_RIGHT_MOTOR);
        r.toStep(StepId.DRIVE_WHEELS);
        assertEquals(Verdict.FAIL, r.verdict(StepId.DRIVE_WHEELS));
        assertShows(r, "'frontRight'");
        assertShows(r, "Configure Robot");
        r.wait(1.0);
        assertFalse(r.last.commands.anyOn());
    }

    /**
     * Situation: the test power in the settings is set far above the limit.
     * Expected: the commanded wheel power never exceeds the limit.
     * Why: powers must stay low even if a number is mistyped; the logic clamps every command.
     */
    @Test
    public void commandsAreClampedToThePowerLimit() {
        RobotCheckConfig c = RobotCheckConfig.fromConstants();
        c.drivePower = 5.0;
        Run r = new Run(c);
        r.toStep(StepId.DRIVE_WHEELS);
        r.yes();
        assertEquals(c.powerLimit, r.last.commands.wheelPower, 1e-9);
        assertTrue(r.maxPower <= c.powerLimit + 1e-9);
    }

    // ======================================================================================= 4. Pinpoint

    private static Run pinpointRun() {
        Run r = new Run();
        r.toStep(StepId.PINPOINT);
        return r;
    }

    /**
     * Situation: the robot is pushed forward 12 inches and turned a quarter turn counter-clockwise.
     * Expected: PASS; the screens show the live pose, the live forward and sideways distances and the turn so far; the
     *     result text says what was seen; no motor is ever commanded.
     * Why: the happy path of the odometry check.
     */
    @Test
    public void pinpointPassesAForwardPushAndAQuarterTurn() {
        Run r = pinpointRun();
        assertShows(r, "OFF the blocks");
        r.in.poseXIn = 5.0;
        r.in.poseYIn = 1.0;
        r.yes();
        r.in.poseXIn = 11.0;
        r.in.poseYIn = 1.0;
        r.tick();
        assertShows(r, "Pose now: x 11.0 inches, y 1.0 inches, heading 0 degrees.");
        assertShows(r, "So far: forward 6.0 inches, sideways 0.0 inches");
        r.in.poseXIn = 17.0;
        r.yes();
        assertShows(r, "Part 2 of 2");
        r.in.poseHeadingRad = Math.toRadians(40.0);
        r.tick();
        assertShows(r, "turned 40 degrees");
        r.in.poseHeadingRad = Math.toRadians(88.0);
        r.yes();
        assertEquals(Verdict.PASS, r.verdict(StepId.PINPOINT));
        assertShows(r, "Push: the Pinpoint says the robot moved 12.0 inches forward");
        assertShows(r, "Turn: the heading changed by 88 degrees");
        assertFalse(r.last.commands.anyOn());
        assertEquals(0.0, r.maxPower, 1e-9);
    }

    /**
     * Situation: the robot starts facing +y (heading 90 degrees) and is pushed forward, so the pose's y grows; and a second
     *     run starts at heading 170 degrees and ends at -100 degrees (a quarter turn across the +/-180 degree seam).
     * Expected: both PASS.
     * Why: the brief says not to hard-code the axis, and a heading that wraps round must not look like a huge turn.
     */
    @Test
    public void pinpointDoesNotCareWhichAxisIsForwardOrWhereTheHeadingWraps() {
        Run r = pinpointRun();
        r.in.poseHeadingRad = Math.toRadians(90.0);
        r.yes();
        r.in.poseYIn = 12.0;
        r.yes();
        r.in.poseHeadingRad = Math.toRadians(180.0);
        r.yes();
        assertEquals(Verdict.PASS, r.verdict(StepId.PINPOINT));

        Run w = pinpointRun();
        w.in.poseHeadingRad = Math.toRadians(170.0);
        w.yes();
        w.in.poseXIn = 12.0 * Math.cos(Math.toRadians(170.0));
        w.in.poseYIn = 12.0 * Math.sin(Math.toRadians(170.0));
        w.yes();
        w.in.poseHeadingRad = Math.toRadians(-100.0);
        w.yes();
        assertEquals(Verdict.PASS, w.verdict(StepId.PINPOINT));
    }

    /**
     * Situation: the forward push is judged for the sizes and directions the pods can get wrong.
     * Expected: 9 inches (the lower edge) and 15 inches PASS; the pose going backward fails with the xPodDirection advice;
     *     sideways movement fails with the X port / Y port advice; 24 inches fails with the podType advice; 0.2 inches
     *     fails with the 'nothing changed' advice; a crooked push is only a WARN.
     * Why: every outcome must tell the student what to look at (pod direction, offsets, scale).
     */
    @Test
    public void pinpointPushOutcomesAndAdvice() {
        RobotCheckJudge j = new RobotCheckJudge(RobotCheckConfig.fromConstants());
        assertEquals(Verdict.PASS, j.pinpointPush(9.0, 0.0).verdict);
        assertEquals(Verdict.PASS, j.pinpointPush(15.0, 1.0).verdict);
        assertEquals(Verdict.FAIL, j.pinpointPush(8.9, 0.0).verdict);
        RobotCheckJudge.Judgement back = j.pinpointPush(-12.0, 0.5);
        assertEquals(Verdict.FAIL, back.verdict);
        assertContains("advice", back.advice, "c.xPodDirection");
        assertContains("advice", back.advice, "pedro/Constants.java");
        assertContains("advice", back.advice, "localizerConfig");
        RobotCheckJudge.Judgement side = j.pinpointPush(0.5, 12.0);
        assertEquals(Verdict.FAIL, side.verdict);
        assertContains("advice", side.advice, "X port");
        assertContains("advice", side.advice, "Y port");
        RobotCheckJudge.Judgement right = j.pinpointPush(0.5, -12.0);
        assertContains("advice", right.advice, "SIDEWAYS");
        RobotCheckJudge.Judgement far = j.pinpointPush(24.0, 0.0);
        assertEquals(Verdict.FAIL, far.verdict);
        assertContains("advice", far.advice, "podType");
        assertContains("advice", far.advice, "ruler");
        RobotCheckJudge.Judgement still = j.pinpointPush(0.2, 0.2);
        assertEquals(Verdict.FAIL, still.verdict);
        assertContains("advice", still.advice, "Nothing changed");
        assertContains("advice", still.advice, "OFF the blocks");
        RobotCheckJudge.Judgement crooked = j.pinpointPush(12.0, 5.0);
        assertEquals(Verdict.WARN, crooked.verdict);
        assertContains("advice", crooked.advice, "strafe (Y) pod");
        assertContains("detail", crooked.detail, "12.0 inches forward");
    }

    /**
     * Situation: the quarter turn is judged for the sizes and directions the heading can get wrong.
     * Expected: 90 degrees (and the 75 / 105 degree edges) PASS; -90 degrees fails with the 'wrong way' advice; 2 degrees
     *     fails with the 'did not change' advice; 45 degrees fails with the 'quarter turn' advice; a quarter turn with
     *     5 inches of drift is a WARN that names xPodOffset and yPodOffset.
     * Why: the brief: heading must increase by about 90 degrees, tolerance from the constants, with advice for offsets.
     */
    @Test
    public void pinpointTurnOutcomesAndAdvice() {
        RobotCheckJudge j = new RobotCheckJudge(RobotCheckConfig.fromConstants());
        assertEquals(Verdict.PASS, j.pinpointTurn(90.0, 0.0).verdict);
        assertEquals(Verdict.PASS, j.pinpointTurn(75.0, 0.0).verdict);
        assertEquals(Verdict.PASS, j.pinpointTurn(105.0, 0.0).verdict);
        assertEquals(Verdict.FAIL, j.pinpointTurn(74.0, 0.0).verdict);
        RobotCheckJudge.Judgement wrong = j.pinpointTurn(-90.0, 0.0);
        assertEquals(Verdict.FAIL, wrong.verdict);
        assertContains("advice", wrong.advice, "wrong way");
        assertContains("advice", wrong.advice, "counter-clockwise");
        RobotCheckJudge.Judgement none = j.pinpointTurn(2.0, 0.0);
        assertEquals(Verdict.FAIL, none.verdict);
        assertContains("advice", none.advice, "did not change");
        RobotCheckJudge.Judgement amount = j.pinpointTurn(45.0, 0.0);
        assertEquals(Verdict.FAIL, amount.verdict);
        assertContains("advice", amount.advice, "quarter turn");
        RobotCheckJudge.Judgement drift = j.pinpointTurn(90.0, 5.0);
        assertEquals(Verdict.WARN, drift.verdict);
        assertContains("advice", drift.advice, "xPodOffset");
        assertContains("advice", drift.advice, "yPodOffset");
    }

    /**
     * Situation: a full run where the pods count backward on the push and the turn is fine.
     * Expected: the step ends FAIL, with the push advice kept in the summary line's advice.
     * Why: the worst verdict of the two parts wins and its advice is kept.
     */
    @Test
    public void pinpointWorstPartWins() {
        Run r = pinpointRun();
        r.yes();
        r.in.poseXIn = -12.0;
        r.yes();
        r.in.poseHeadingRad = Math.toRadians(90.0);
        r.yes();
        assertEquals(Verdict.FAIL, r.verdict(StepId.PINPOINT));
        assertContains("advice", r.check.advice(StepId.PINPOINT), "c.xPodDirection");
        assertShows(r, "Turn: the heading changed by 90 degrees");
    }

    /**
     * Situation: the problem button is pressed during part 1, and in another run during part 2 after a good push.
     * Expected: FAIL both times; the part 2 case keeps the push result (a PASS line) and adds the heading advice.
     * Why: the student can say "nothing changes" without finishing the motion.
     */
    @Test
    public void reportingAProblemFailsThePinpointStep() {
        Run r = pinpointRun();
        r.yes();
        r.no();
        assertEquals(Verdict.FAIL, r.verdict(StepId.PINPOINT));
        assertShows(r, "You reported that the pose did not follow the push.");

        Run s = pinpointRun();
        s.yes();
        s.in.poseXIn = 12.0;
        s.yes();
        s.no();
        assertEquals(Verdict.FAIL, s.verdict(StepId.PINPOINT));
        assertShows(s, "Push: the Pinpoint says the robot moved 12.0 inches forward");
        assertShows(s, "You reported that the heading did not follow the turn.");
    }

    /**
     * Situation: the Pinpoint reports each kind of fault when the step is started.
     * Expected: FAIL at once (no waiting for a push) with advice that names the right pod or port; READY is not a fault.
     * Why: a faulty Pinpoint can be diagnosed without any pushing.
     */
    @Test
    public void pinpointFaultsFailImmediatelyWithTheRightAdvice() {
        String[][] cases = {
                {"FAULT_X_POD_NOT_DETECTED", "X port"},
                {"FAULT_Y_POD_NOT_DETECTED", "Y port"},
                {"FAULT_NO_PODS_DETECTED", "both odometry pod cables"},
                {"FAULT_IMU_RUNAWAY", "keep it still"},
                {"FAULT_BAD_READ", "Pinpoint's cable"}};
        for (String[] c : cases) {
            Run r = pinpointRun();
            r.in.pinpointStatus = c[0];
            r.yes();
            assertEquals(c[0], Verdict.FAIL, r.verdict(StepId.PINPOINT));
            assertShows(r, "it says " + c[0]);
            assertContains(c[0], r.check.advice(StepId.PINPOINT), c[1]);
        }
        Run ok = pinpointRun();
        ok.in.pinpointStatus = "READY";
        ok.yes();
        assertNotShows(ok, "Result:");
        assertShows(ok, "Pinpoint status: READY.");
    }

    /**
     * Situation: Pedro could not start the Pinpoint ("no device"); and separately the 'pinpoint' name is missing from the
     *     configuration.
     * Expected: FAIL at once; the first says Pedro could not start the Pinpoint with the reason, the second names 'pinpoint'.
     * Why: a robot without odometry must be reported in plain words, not crash.
     */
    @Test
    public void pinpointThatCannotStartOrIsMissingFails() {
        Run r = pinpointRun();
        r.in.poseAvailable = false;
        r.in.poseProblem = "no device";
        r.yes();
        assertEquals(Verdict.FAIL, r.verdict(StepId.PINPOINT));
        assertShows(r, "Pedro could not start the Pinpoint (no device)");
        assertContains("advice", r.check.advice(StepId.PINPOINT), "named 'pinpoint'");

        Run m = new Run();
        m.in.present.remove(RobotCheckDevice.PINPOINT);
        m.toStep(StepId.PINPOINT);
        m.yes();
        assertEquals(Verdict.FAIL, m.verdict(StepId.PINPOINT));
        assertShows(m, "'pinpoint'");
    }

    /**
     * Situation: the pose disappears (the Pinpoint stops answering) while the student is pushing the robot.
     * Expected: pressing yes then fails the step with the "could not start the Pinpoint" message instead of measuring
     *     numbers that are not there; the waiting screen says there is no pose.
     * Why: a half-working Pinpoint must end in a clear FAIL, not a crash or a made-up PASS.
     */
    @Test
    public void aPoseThatDisappearsMidStepFails() {
        Run r = pinpointRun();
        r.yes();
        r.in.poseAvailable = false;
        r.in.poseProblem = "";
        r.tick();
        assertShows(r, "No pose: the Pinpoint did not start.");
        r.yes();
        assertEquals(Verdict.FAIL, r.verdict(StepId.PINPOINT));
        assertShows(r, "Pedro could not start the Pinpoint, so the robot does not know where it is.");
    }

    /**
     * Situation: the Pinpoint step is skipped from its start screen and from the waiting screen.
     * Expected: SKIPPED both times.
     * Why: skip works anywhere in a step.
     */
    @Test
    public void pinpointCanBeSkipped() {
        Run r = pinpointRun();
        r.skip();
        assertEquals(Verdict.SKIPPED, r.verdict(StepId.PINPOINT));
        Run s = pinpointRun();
        s.yes();
        s.skip();
        assertEquals(Verdict.SKIPPED, s.verdict(StepId.PINPOINT));
    }

    // ========================================================================================== 5. intake

    /**
     * Situation: the three intake parts are run and answered yes.
     * Expected: the roller motor, then the left corner servo, then the right corner servo run, each alone at the gentle
     *     power for the set time (the left servo with a negative sign, the right with a positive one, like the TeleOp);
     *     all yes gives PASS.
     * Why: the brief: run the intake motor and the two corner servos for 1.5 seconds each.
     */
    @Test
    public void intakePartsRunOneAtATimeWithTheTeleOpSigns() {
        Run r = new Run();
        r.toStep(StepId.INTAKE);
        assertEquals("Step 5 of 10: Intake", r.last.title);
        assertShows(r, "front roller motor");
        r.wait(1.0);
        assertFalse(r.last.commands.anyOn());

        r.yes();
        assertEquals(r.cfg.intakeMotorPower, r.last.commands.intakeMotorPower, 1e-9);
        assertEquals(0.0, r.last.commands.leftServoPower, 1e-9);
        assertEquals(0.0, r.last.commands.rightServoPower, 1e-9);
        r.wait(r.cfg.intakeSec + 0.1);
        assertFalse(r.last.commands.anyOn());
        assertShows(r, "Did the front roller motor");
        r.yes();

        assertShows(r, "Next: the left corner servo");
        r.yes();
        assertEquals(-r.cfg.intakeServoPower, r.last.commands.leftServoPower, 1e-9);
        assertEquals(0.0, r.last.commands.intakeMotorPower, 1e-9);
        r.wait(r.cfg.intakeSec + 0.1);
        r.yes();

        assertShows(r, "Next: the right corner servo");
        r.yes();
        assertEquals(r.cfg.intakeServoPower, r.last.commands.rightServoPower, 1e-9);
        r.wait(r.cfg.intakeSec + 0.1);
        r.yes();
        assertEquals(Verdict.PASS, r.verdict(StepId.INTAKE));
        assertShows(r, "pulled inward");
    }

    /**
     * Situation: each intake part in turn is answered no.
     * Expected: FAIL naming that part; the roller advice names Intake.java and setDirection; the servo advice names
     *     BioBuzzRobot.manualControls and the matching sign constant.
     * Why: on 'no' the student must be told what to change and where.
     */
    @Test
    public void anIntakePartAnsweredNoGivesItsOwnAdvice() {
        boolean[][] answers = {{false, true, true}, {true, false, true}, {true, true, false}};
        String[] expected = {"Intake.java", "INTAKE_LEFT_SERVO_SIGN", "INTAKE_RIGHT_SERVO_SIGN"};
        String[] names = {"intakeMotor", "servoLeft", "servoRight"};
        for (int i = 0; i < 3; i++) {
            Run r = new Run();
            r.toStep(StepId.INTAKE);
            for (int part = 0; part < 3; part++) {
                trial(r, r.cfg.intakeSec, answers[i][part]);
            }
            assertEquals(Verdict.FAIL, r.verdict(StepId.INTAKE));
            assertContains("advice", r.check.advice(StepId.INTAKE), expected[i]);
            assertContains("advice", r.check.advice(StepId.INTAKE), names[i]);
            assertContains("advice", r.check.advice(StepId.INTAKE), "did not pull inward");
        }
        Run roller = new Run();
        roller.toStep(StepId.INTAKE);
        trial(roller, roller.cfg.intakeSec, false);
        trial(roller, roller.cfg.intakeSec, true);
        trial(roller, roller.cfg.intakeSec, true);
        assertContains("advice", roller.check.advice(StepId.INTAKE), "intakeMotor.setDirection(DcMotor.Direction.REVERSE)");
    }

    /**
     * Situation: the left corner servo is missing from the configuration; then all three devices are missing.
     * Expected: the missing part is skipped automatically (the roller and the right servo are asked as usual) and the
     *     result is FAIL saying it was not tested; with all three missing the step fails at once with no run at all.
     * Why: a missing device must not crash or hang the check.
     */
    @Test
    public void missingIntakeDevicesAreReportedNotRun() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.LEFT_INTAKE_SERVO);
        r.toStep(StepId.INTAKE);
        trial(r, r.cfg.intakeSec, true);
        assertShows(r, "Next: the right corner servo");
        trial(r, r.cfg.intakeSec, true);
        assertEquals(Verdict.FAIL, r.verdict(StepId.INTAKE));
        assertShows(r, "not tested: 'servoLeft' is missing");
        assertContains("advice", r.check.advice(StepId.INTAKE), "'servoLeft'");

        Run all = new Run();
        all.in.present.remove(RobotCheckDevice.INTAKE_MOTOR);
        all.in.present.remove(RobotCheckDevice.LEFT_INTAKE_SERVO);
        all.in.present.remove(RobotCheckDevice.RIGHT_INTAKE_SERVO);
        all.toStep(StepId.INTAKE);
        assertEquals(Verdict.FAIL, all.verdict(StepId.INTAKE));
        all.wait(1.0);
        assertFalse(all.last.commands.anyOn());
    }

    /**
     * Situation: the intake step is skipped from the first screen.
     * Expected: SKIPPED and nothing runs.
     * Why: skip works.
     */
    @Test
    public void intakeCanBeSkipped() {
        Run r = new Run();
        r.toStep(StepId.INTAKE);
        r.skip();
        assertEquals(Verdict.SKIPPED, r.verdict(StepId.INTAKE));
        assertFalse(r.last.commands.anyOn());
    }

    // ======================================================================================== 6. flywheel

    /**
     * Situation: the flywheel is started and its measured speed climbs to the target.
     * Expected: nothing runs before the press; the target speed (Shooter.MANUAL_TARGET_TPS) is commanded after it; once
     *     the speed has been inside the at-speed window long enough the step PASSes with the number shown and the flywheel
     *     command goes back to zero.
     * Why: PASS if the measured speed gets within the at-speed window; always stop the flywheel afterwards.
     */
    @Test
    public void flywheelPassesWhenItReachesTheWindowAndThenStops() {
        Run r = new Run();
        r.toStep(StepId.FLYWHEEL);
        assertEquals("Step 6 of 10: Flywheel", r.last.title);
        assertShows(r, "1200 ticks per second");
        assertShows(r, "1108 to 1292 ticks per second");
        r.wait(1.0);
        assertEquals(0.0, r.last.commands.flywheelTps, 1e-9);
        r.yes();
        assertEquals(1200.0, r.last.commands.flywheelTps, 1e-9);
        r.in.flywheelTps = 600.0;
        r.wait(0.3);
        assertShows(r, "Speed now: 600 ticks per second.");
        assertEquals(1200.0, r.last.commands.flywheelTps, 1e-9);
        r.in.flywheelTps = 1190.0;
        r.wait(0.3);
        assertEquals(Verdict.PASS, r.verdict(StepId.FLYWHEEL));
        assertEquals(0.0, r.last.commands.flywheelTps, 1e-9);
        assertShows(r, "The flywheel reached 1190 ticks per second");
    }

    /**
     * Situation: the speed touches the window for one tick and falls out again.
     * Expected: that does not count as at speed (no PASS).
     * Why: it uses the same debounce as the TeleOp: a speed that only passes through the window is not "at speed".
     */
    @Test
    public void flywheelSpeedOnlyPassingThroughTheWindowDoesNotPass() {
        Run r = new Run();
        r.toStep(StepId.FLYWHEEL);
        r.yes();
        r.in.flywheelTps = 1200.0;
        r.tick();
        r.in.flywheelTps = 500.0;
        r.wait(0.5);
        assertEquals(Verdict.NOT_RUN, r.verdict(StepId.FLYWHEEL));
    }

    /**
     * Situation: the flywheel stays too slow, does not turn at all, or turns backward, until the spin-up time is over.
     * Expected: FAIL each time, with the matching advice: PIDF and battery; motor cable and name; direction in Launcher.java.
     *     The flywheel command is zero afterwards in all cases.
     * Why: "FAIL tells them to check the motor cable / direction / PIDF", and the flywheel must always stop.
     */
    @Test
    public void flywheelFailuresGiveTheRightAdviceAndAlwaysStop() {
        Run slow = new Run();
        slow.toStep(StepId.FLYWHEEL);
        slow.yes();
        slow.in.flywheelTps = 800.0;
        slow.wait(slow.cfg.flywheelTimeoutSec + 0.2);
        assertEquals(Verdict.FAIL, slow.verdict(StepId.FLYWHEEL));
        assertShows(slow, "too slow");
        assertContains("advice", slow.check.advice(StepId.FLYWHEEL), "FLYWHEEL_PIDF_P");
        assertContains("advice", slow.check.advice(StepId.FLYWHEEL), "battery");
        assertEquals(0.0, slow.last.commands.flywheelTps, 1e-9);

        Run still = new Run();
        still.toStep(StepId.FLYWHEEL);
        still.yes();
        still.in.flywheelTps = 0.0;
        still.wait(still.cfg.flywheelTimeoutSec + 0.2);
        assertEquals(Verdict.FAIL, still.verdict(StepId.FLYWHEEL));
        assertShows(still, "did not turn");
        assertContains("advice", still.check.advice(StepId.FLYWHEEL), "motor's cable");
        assertContains("advice", still.check.advice(StepId.FLYWHEEL), "'flywheel'");
        assertEquals(0.0, still.last.commands.flywheelTps, 1e-9);

        Run back = new Run();
        back.toStep(StepId.FLYWHEEL);
        back.yes();
        back.in.flywheelTps = -900.0;
        back.wait(back.cfg.flywheelTimeoutSec + 0.2);
        assertEquals(Verdict.FAIL, back.verdict(StepId.FLYWHEEL));
        assertShows(back, "WRONG WAY");
        assertContains("advice", back.check.advice(StepId.FLYWHEEL), "Launcher.java");
        assertContains("advice", back.check.advice(StepId.FLYWHEEL), "setDirection");
        assertEquals(0.0, back.last.commands.flywheelTps, 1e-9);
    }

    /**
     * Situation: the no button is pressed while the flywheel is still climbing.
     * Expected: the flywheel stops at once and the step fails with "you stopped the flywheel before it reached speed".
     * Why: the student can always stop the flywheel from the gamepad.
     */
    @Test
    public void flywheelCanBeStoppedEarly() {
        Run r = new Run();
        r.toStep(StepId.FLYWHEEL);
        r.yes();
        r.in.flywheelTps = 400.0;
        r.wait(0.5);
        r.press(r.cfg.no);
        assertEquals(0.0, r.last.commands.flywheelTps, 1e-9);
        assertEquals(Verdict.FAIL, r.verdict(StepId.FLYWHEEL));
        assertShows(r, "You stopped the flywheel before it reached speed");
    }

    /**
     * Situation: the flywheel or the feeder is missing, and in another run the step is skipped.
     * Expected: FAIL without spinning anything (the shooter code needs both names); SKIPPED for the skip.
     * Why: a missing device must not crash the check, and the flywheel must not start if the shooter code cannot.
     */
    @Test
    public void flywheelStepNeedsBothShooterNamesAndCanBeSkipped() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.FEEDER);
        r.toStep(StepId.FLYWHEEL);
        r.yes();
        assertEquals(Verdict.FAIL, r.verdict(StepId.FLYWHEEL));
        assertShows(r, "'feeder'");
        assertEquals(0.0, r.last.commands.flywheelTps, 1e-9);

        Run s = new Run();
        s.toStep(StepId.FLYWHEEL);
        s.skip();
        assertEquals(Verdict.SKIPPED, s.verdict(StepId.FLYWHEEL));
    }

    // ========================================================================================= 7. feeder

    /**
     * Situation: the feeder step is run.
     * Expected: the feeder runs forward (positive power) for the forward time, then backward (negative power) for the
     *     reverse time, then stops by itself and asks the question.
     * Why: the brief: run the feeder forward 1 second then reverse 0.5 seconds.
     */
    @Test
    public void feederRunsForwardThenBackwardThenStops() {
        Run r = new Run();
        r.toStep(StepId.FEEDER);
        assertEquals("Step 7 of 10: Feeder", r.last.title);
        r.wait(1.0);
        assertFalse(r.last.commands.anyOn());
        r.yes();
        r.wait(0.5);
        assertEquals(r.cfg.feederPower, r.last.commands.feederPower, 1e-9);
        assertShows(r, "turning FORWARD");
        r.wait(r.cfg.feederForwardSec - 0.4);
        assertEquals(-r.cfg.feederPower, r.last.commands.feederPower, 1e-9);
        assertShows(r, "turning BACKWARD");
        r.wait(r.cfg.feederReverseSec + 0.1);
        assertFalse(r.last.commands.anyOn());
        assertShows(r, "Did the feeder turn forward");
        r.yes();
        assertEquals(Verdict.PASS, r.verdict(StepId.FEEDER));
    }

    /**
     * Situation: the feeder answer is no, with FEEDER_INVERTED false and then true.
     * Expected: FAIL; the advice says to change Shooter.FEEDER_INVERTED from the current value to the other one, and to
     *     check the cable and servo type if nothing moved.
     * Why: the fix for a feeder that turns the wrong way is one constant, so the advice names it and its value.
     */
    @Test
    public void feederNoSaysWhichConstantToFlip() {
        for (boolean inverted : new boolean[]{false, true}) {
            RobotCheckConfig c = RobotCheckConfig.fromConstants();
            c.feederInverted = inverted;
            Run r = new Run(c);
            r.toStep(StepId.FEEDER);
            trial(r, c.feederForwardSec + c.feederReverseSec, false);
            assertEquals(Verdict.FAIL, r.verdict(StepId.FEEDER));
            assertContains("advice", r.check.advice(StepId.FEEDER), "FEEDER_INVERTED from " + inverted + " to " + !inverted);
            assertContains("advice", r.check.advice(StepId.FEEDER), "continuous-rotation servo");
        }
    }

    /**
     * Situation: the feeder is missing from the configuration, and in another run the step is skipped.
     * Expected: FAIL without running; SKIPPED.
     * Why: no crash, no motion.
     */
    @Test
    public void feederMissingFailsAndFeederCanBeSkipped() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.FEEDER);
        r.toStep(StepId.FEEDER);
        assertEquals(Verdict.FAIL, r.verdict(StepId.FEEDER));
        r.wait(1.0);
        assertFalse(r.last.commands.anyOn());

        Run s = new Run();
        s.toStep(StepId.FEEDER);
        s.skip();
        assertEquals(Verdict.SKIPPED, s.verdict(StepId.FEEDER));
    }

    // ======================================================================================= 8. Limelight

    private static Run limelightRun() {
        Run r = new Run();
        r.toStep(StepId.LIMELIGHT);
        r.autoFrames = true;
        r.in.frame = frame(VisionFrame.Status.OK, r.t);
        return r;
    }

    /**
     * Situation: the Limelight is connected, sending pictures on the right pipeline, and a HIVE tag (id 34) is held in
     *     front of it.
     * Expected: after the camera check the screen asks for a tag; when tag 34 appears the step PASSes and shows the id,
     *     tx and ty with their units; no motor runs.
     * Why: the brief: connected, frames arriving, pipeline right, then a HIVE tag.
     */
    @Test
    public void limelightPassesWhenAHiveTagIsSeen() {
        Run r = limelightRun();
        assertEquals("Step 8 of 10: Limelight camera", r.last.title);
        r.yes();
        assertShows(r, "Checking the camera");
        r.wait(r.cfg.limelightCheckSec + 0.2);
        assertShows(r, "Now hold a HIVE tag (number 30 to 45)");
        assertShows(r, "The camera sees no tag yet.");
        r.in.frame = frame(VisionFrame.Status.OK, r.t, tag(34));
        r.tick();
        assertEquals(Verdict.PASS, r.verdict(StepId.LIMELIGHT));
        assertShows(r, "Saw HIVE tag number 34");
        assertShows(r, "tx = 5.0 degrees");
        assertShows(r, "ty = -2.0 degrees");
        assertFalse(r.last.commands.anyOn());
    }

    /**
     * Situation: tag ids 30 and 45 (the edges), then 29 and 46 (just outside) are held in front of the camera.
     * Expected: 30 and 45 PASS; 29 and 46 do not pass, and the failure names the wrong tag and the range 30 to 45.
     * Why: "a tag id in that range (30 to 45) is seen".
     */
    @Test
    public void onlyHiveTagIdsCount() {
        for (int id : new int[]{30, 45}) {
            Run r = limelightRun();
            r.yes();
            r.wait(r.cfg.limelightCheckSec + 0.2);
            r.in.frame = frame(VisionFrame.Status.OK, r.t, tag(id));
            r.tick();
            assertEquals("id " + id, Verdict.PASS, r.verdict(StepId.LIMELIGHT));
        }
        for (int id : new int[]{29, 46}) {
            Run r = limelightRun();
            r.yes();
            r.wait(r.cfg.limelightCheckSec + 0.2);
            r.in.frame = frame(VisionFrame.Status.OK, r.t, tag(id));
            r.wait(r.cfg.limelightTagTimeoutSec + 0.5);
            assertEquals("id " + id, Verdict.FAIL, r.verdict(StepId.LIMELIGHT));
            assertShows(r, "tag number " + id + ", which is not a HIVE tag (HIVE tags are 30 to 45)");
            assertContains("advice", r.check.advice(StepId.LIMELIGHT), "ID filter");
        }
    }

    /**
     * Situation: the tag has no 3D distance ("Full 3D" is off in the pipeline).
     * Expected: a WARN that says there is no distance and to turn on Full 3D.
     * Why: the robot needs the distance to the tags; the docs say Full 3D must be on.
     */
    @Test
    public void aTagWithoutDistanceIsAWarning() {
        Run r = limelightRun();
        r.yes();
        r.wait(r.cfg.limelightCheckSec + 0.2);
        r.in.frame = frame(VisionFrame.Status.OK, r.t, TagObservation.bearingOnly(34, 1.0, 2.0, 1.0, 0.0));
        r.tick();
        assertEquals(Verdict.WARN, r.verdict(StepId.LIMELIGHT));
        assertContains("advice", r.check.advice(StepId.LIMELIGHT), "Full 3D");
    }

    /**
     * Situation: no tag is shown for the whole time, and in another run the problem button is pressed.
     * Expected: FAIL both times; the first says no tag was seen in 20 seconds, the second that the student pressed the
     *     problem button; the advice lists marker size, ID filter and exposure.
     * Why: a camera that never sees a tag needs concrete things to check.
     */
    @Test
    public void noTagFailsWithCameraAdvice() {
        Run r = limelightRun();
        r.yes();
        r.wait(r.cfg.limelightCheckSec + 0.2);
        r.wait(r.cfg.limelightTagTimeoutSec + 0.5);
        assertEquals(Verdict.FAIL, r.verdict(StepId.LIMELIGHT));
        assertShows(r, "did not see any tag in 20 seconds.");
        assertContains("advice", r.check.advice(StepId.LIMELIGHT), "marker size 82.55 millimeters");
        assertContains("advice", r.check.advice(StepId.LIMELIGHT), "exposure");

        Run g = limelightRun();
        g.yes();
        g.wait(g.cfg.limelightCheckSec + 0.2);
        g.no();
        assertEquals(Verdict.FAIL, g.verdict(StepId.LIMELIGHT));
        assertShows(g, "you pressed the problem button");
    }

    /**
     * Situation: the Limelight is not in the robot configuration; in another run it is configured but the hub cannot reach it.
     * Expected: WARN both times (no crash): the first says the robot configuration has no Limelight called 'limelight' and the
     *     robot still drives and shoots without it; the second says the hub cannot reach it and points at the USB cable.
     * Why: "If the Limelight is missing say so in plain words and continue (WARN, not a crash)".
     */
    @Test
    public void aMissingLimelightIsAWarningNotACrash() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.LIMELIGHT);
        r.in.visionConnected = false;
        r.toStep(StepId.LIMELIGHT);
        r.yes();
        assertEquals(Verdict.WARN, r.verdict(StepId.LIMELIGHT));
        assertShows(r, "The robot configuration has no Limelight called 'limelight'");
        assertShows(r, "still drives and shoots without it");
        assertContains("advice", r.check.advice(StepId.LIMELIGHT), "Configure Robot");
        r.yes();
        assertEquals(StepId.GAMEPAD, r.step());

        Run c = new Run();
        c.in.visionConnected = false;
        c.toStep(StepId.LIMELIGHT);
        c.yes();
        c.wait(c.cfg.limelightCheckSec + 0.2);
        assertEquals(Verdict.WARN, c.verdict(StepId.LIMELIGHT));
        assertShows(c, "Control Hub cannot reach it");
        assertContains("advice", c.check.advice(StepId.LIMELIGHT), "USB-C cable");
    }

    /**
     * Situation: the camera is connected, but no new pictures arrive (status OK, then status NO_RESULT).
     * Expected: FAIL saying how many new pictures arrived; the NO_RESULT advice says to wait 20 seconds after switching on.
     * Why: "frames arriving (count increasing)?"
     */
    @Test
    public void aCameraWithNoNewPicturesFails() {
        Run r = new Run();
        r.toStep(StepId.LIMELIGHT);
        r.in.frame = frame(VisionFrame.Status.OK, r.t);
        r.yes();
        r.wait(r.cfg.limelightCheckSec + 0.2);
        assertEquals(Verdict.FAIL, r.verdict(StepId.LIMELIGHT));
        assertShows(r, "only 0 new pictures arrived in 1.0 seconds (at least 3 are needed)");
        assertContains("advice", r.check.advice(StepId.LIMELIGHT), "USB cable");

        Run n = new Run();
        n.toStep(StepId.LIMELIGHT);
        n.in.frame = frame(VisionFrame.Status.NO_RESULT, n.t);
        n.yes();
        n.wait(n.cfg.limelightCheckSec + 0.2);
        assertContains("advice", n.check.advice(StepId.LIMELIGHT), "20 seconds");
    }

    /**
     * Situation: pictures arrive, but the camera runs the wrong pipeline; and in another run the pictures are stale.
     * Expected: FAIL; the first says pipeline 0 is not running and points at the Limelight web page; the second says
     *     the pictures are too old.
     * Why: "pipeline index correct?"
     */
    @Test
    public void aWrongPipelineOrStalePicturesFail() {
        Run r = limelightRun();
        r.in.frame = frame(VisionFrame.Status.WRONG_PIPELINE, r.t);
        r.yes();
        r.wait(r.cfg.limelightCheckSec + 0.2);
        assertEquals(Verdict.FAIL, r.verdict(StepId.LIMELIGHT));
        assertShows(r, "not running pipeline 0");
        assertContains("advice", r.check.advice(StepId.LIMELIGHT), "http://limelight.local:5801");

        Run s = limelightRun();
        s.in.frame = frame(VisionFrame.Status.STALE, s.t);
        s.yes();
        s.wait(s.cfg.limelightCheckSec + 0.2);
        assertEquals(Verdict.FAIL, s.verdict(StepId.LIMELIGHT));
        assertShows(s, "too old or frozen (status STALE)");

        Run f = limelightRun();
        f.in.frame = frame(VisionFrame.Status.STALLED, f.t);
        f.yes();
        f.wait(f.cfg.limelightCheckSec + 0.2);
        assertShows(f, "too old or frozen (status STALLED)");
    }

    /**
     * Situation: the Limelight step is skipped from the start and from the waiting screen.
     * Expected: SKIPPED both times.
     * Why: skip works anywhere.
     */
    @Test
    public void limelightCanBeSkipped() {
        Run r = limelightRun();
        r.skip();
        assertEquals(Verdict.SKIPPED, r.verdict(StepId.LIMELIGHT));
        Run s = limelightRun();
        s.yes();
        s.skip();
        assertEquals(Verdict.SKIPPED, s.verdict(StepId.LIMELIGHT));
    }

    // ======================================================================================== 9. gamepad

    /**
     * Situation: every button and both triggers are pressed once.
     * Expected: the screen shows what is pressed right now and how many were seen; when the last one is seen the step
     *     PASSes by itself with "All 14 controls".
     * Why: the brief: show which buttons the gamepad is pressing so the Xbox mapping can be confirmed.
     */
    @Test
    public void gamepadPassesWhenEveryControlWasSeen() {
        Run r = new Run();
        r.toStep(StepId.GAMEPAD);
        assertEquals("Step 9 of 10: Gamepad", r.last.title);
        r.in.gamepadType = "SONY_PS4";
        r.yes();
        assertShows(r, "SEEN 0 of 14.");
        r.in.buttons.add(Button.X);
        r.in.leftTrigger = true;
        r.tick();
        assertShows(r, "Pressing now: X, left trigger (LT).");
        assertShows(r, "SEEN 2 of 14.");
        assertShows(r, "this controller is: SONY_PS4");
        r.in.buttons.clear();
        r.in.leftTrigger = false;
        r.tick();
        passGamepadRest(r);
        assertEquals(Verdict.PASS, r.verdict(StepId.GAMEPAD));
        assertShows(r, "All 14 controls");
    }

    /** Presses everything except the X button and the left trigger (already seen). */
    private static void passGamepadRest(Run r) {
        for (Button b : r.cfg.gamepadButtons) {
            if (b != Button.X) {
                r.wait(0.1);
                r.press(b);
            }
        }
        r.wait(0.1);
        r.in.rightTrigger = true;
        r.tick();
        r.in.rightTrigger = false;
        r.tick();
    }

    /**
     * Situation: while the gamepad step waits, the skip, no, yes and repeat buttons are pressed.
     * Expected: they do nothing but get marked as seen: the step stays on the waiting screen.
     * Why: in this step those buttons are being tested, so they must not act.
     */
    @Test
    public void inTheGamepadStepTheAnswerButtonsOnlyGetTested() {
        Run r = new Run();
        r.toStep(StepId.GAMEPAD);
        r.yes();
        r.go(r.cfg.skip);
        r.go(r.cfg.no);
        r.go(r.cfg.repeat);
        assertEquals(StepId.GAMEPAD, r.step());
        assertEquals(Verdict.NOT_RUN, r.verdict(StepId.GAMEPAD));
        assertShows(r, "SEEN 3 of 14.");
    }

    /**
     * Situation: the button that started the step is still held, and the student finishes early with only some buttons seen.
     * Expected: the starting press is not counted as a seen button (A is listed as not seen, D-pad up is not); finishing
     *     early gives FAIL listing the controls that were not seen (in words), and advice about the gamepad mode.
     * Why: only fresh presses count (so the press that started the step is not one), and a missing button must be named.
     */
    @Test
    public void finishingEarlyListsTheControlsThatWereNotSeen() {
        Run r = new Run();
        r.toStep(StepId.GAMEPAD);
        r.wait(r.cfg.lockoutSec + 0.05);
        r.in.buttons.add(r.cfg.yes);
        r.wait(0.5);
        r.in.buttons.remove(r.cfg.yes);
        r.tick();
        r.press(Button.DPAD_UP);
        r.go(r.cfg.gamepadFinish);
        assertEquals(Verdict.FAIL, r.verdict(StepId.GAMEPAD));
        String detail = r.check.detail(StepId.GAMEPAD);
        assertContains("detail", detail, "These controls were not seen:");
        assertContains("detail", detail, "B");
        assertContains("detail", detail, "left bumper (LB)");
        assertContains("detail", detail, "right trigger (RT)");
        assertContains("detail", detail, "A");
        assertFalse(detail.contains("D-pad up"));
        assertContains("advice", r.check.advice(StepId.GAMEPAD), "Start and A");
    }

    /**
     * Situation: nothing is pressed for the whole gamepad time.
     * Expected: the step ends by itself with FAIL.
     * Why: the step must never get stuck, even with a controller whose finish button is broken.
     */
    @Test
    public void theGamepadStepEndsByItselfAfterItsTime() {
        Run r = new Run();
        r.toStep(StepId.GAMEPAD);
        r.yes();
        r.wait(r.cfg.gamepadTimeoutSec + 0.5);
        assertEquals(Verdict.FAIL, r.verdict(StepId.GAMEPAD));
    }

    /**
     * Situation: the gamepad step is skipped from its start screen.
     * Expected: SKIPPED.
     * Why: skip works.
     */
    @Test
    public void gamepadCanBeSkipped() {
        Run r = new Run();
        r.toStep(StepId.GAMEPAD);
        r.skip();
        assertEquals(Verdict.SKIPPED, r.verdict(StepId.GAMEPAD));
        assertEquals(StepId.SUMMARY, r.step());
    }

    // ======================================================================================== 10. summary

    /**
     * Situation: every step passes.
     * Expected: the summary lists all ten steps as PASS, counts them, and shows "All clear: you can run the TeleOp" and
     *     the line that it stays until STOP; allClear() is true and finished() is true.
     * Why: the closing screen of a good run.
     */
    @Test
    public void aCleanRunEndsWithAllClear() {
        Run r = passingRun();
        assertEquals(StepId.SUMMARY, r.step());
        assertEquals("Step 10 of 10: Summary", r.last.title);
        assertTrue(r.last.finished);
        assertShows(r, "RESULT: 10 PASS, 0 WARN, 0 FAIL, 0 SKIPPED.");
        assertShows(r, "PASS  Safety first");
        assertShows(r, "PASS  Gamepad");
        assertShows(r, "All clear: you can run the TeleOp");
        assertShows(r, "stays until you press STOP");
        assertNotShows(r, "To do:");
        assertTrue(r.check.allClear());
        assertEquals(Verdict.PASS, r.verdict(StepId.GAMEPAD));
        assertFalse(r.last.commands.anyOn());
    }

    /**
     * Situation: the front-left wheel fails in an otherwise clean run.
     * Expected: the summary says 1 FAIL, says NOT READY, lists the failing step first with a "To do:" line holding its
     *     advice, and does NOT show the all-clear line; allClear() is false.
     * Why: "the line 'All clear' only if nothing failed".
     */
    @Test
    public void aFailureMeansNoAllClearAndShowsWhatToFix() {
        Run r = new Run();
        r.yes();
        r.yes();
        r.yes();
        r.yes();
        r.yes();
        trial(r, r.cfg.driveSec, false);
        for (int i = 0; i < 3; i++) {
            trial(r, r.cfg.driveSec, true);
        }
        r.yes();
        while (r.step() != StepId.SUMMARY) {
            r.skip();
        }
        assertShows(r, "1 FAIL");
        assertShows(r, "FAIL  Drive wheels");
        assertShows(r, "To do: The front-left wheel did not turn forward");
        assertShows(r, "NOT READY: fix every FAIL below");
        assertNotShows(r, "All clear");
        assertFalse(r.check.allClear());
    }

    /**
     * Situation: everything passes except a Limelight that is missing from the configuration (a WARN in the names step and a
     *     WARN in the Limelight step).
     * Expected: the all-clear line still shows (a warning is not a failure) with a note to read the warnings, and each
     *     warning is listed before the passes with its "To do:" line.
     * Why: "All clear only if nothing failed"; the missing Limelight is a WARN by design.
     */
    @Test
    public void aWarningDoesNotStopAllClearButIsShown() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.LIMELIGHT);
        r.in.visionConnected = false;
        r.yes();
        r.yes();
        r.yes();
        r.yes();
        r.yes();
        passWheels(r);
        r.yes();
        passPinpoint(r);
        r.yes();
        passIntake(r);
        r.yes();
        passFlywheel(r);
        r.yes();
        trial(r, r.cfg.feederForwardSec + r.cfg.feederReverseSec, true);
        r.yes();
        r.yes();
        r.yes();
        passGamepad(r);
        r.yes();
        assertEquals(StepId.SUMMARY, r.step());
        assertShows(r, "8 PASS, 2 WARN, 0 FAIL, 0 SKIPPED");
        assertShows(r, "WARN  Hardware names");
        assertShows(r, "WARN  Limelight camera");
        assertShows(r, "To do: To use the camera");
        assertShows(r, "All clear: you can run the TeleOp");
        assertShows(r, "Warnings are not failures");
        assertTrue(r.check.allClear());
    }

    /**
     * Situation: the whole check is skipped step by step after the safety screen.
     * Expected: nine SKIPPED, no all-clear; the summary says "NOT FULLY CHECKED" and how many steps were skipped, and each
     *     skipped step has its "To do:" line.
     * Why: a team that skips everything must not be told it is all clear.
     */
    @Test
    public void skippedStepsAreNotAllClear() {
        Run r = new Run();
        r.toStep(StepId.SUMMARY);
        assertShows(r, "9 SKIPPED");
        assertShows(r, "NOT FULLY CHECKED: you skipped 9 step(s)");
        assertNotShows(r, "All clear");
        assertFalse(r.check.allClear());
    }

    /**
     * Situation: at the summary all the buttons are pressed and a long time passes.
     * Expected: the summary does not change and no motor is commanded.
     * Why: "the summary stays on screen until STOP".
     */
    @Test
    public void theSummaryStaysUntilStop() {
        Run r = passingRun();
        List<String> before = new ArrayList<String>(r.last.lines);
        for (Button b : Button.values()) {
            if (b != r.cfg.scrollUp && b != r.cfg.scrollDown) {
                r.go(b);
            }
        }
        r.wait(30.0);
        assertEquals(StepId.SUMMARY, r.step());
        assertEquals(before, r.last.lines);
        assertFalse(r.last.commands.anyOn());
    }

    /**
     * Situation: a summary with many failures is longer than one screen page.
     * Expected: only one page is shown, ending with a "Lines a to b of n" hint; the scroll-down button moves the list, it
     *     stops at the end; the scroll-up button moves back.
     * Why: the Driver Station screen is small; nothing may be hidden without a way to see it.
     */
    @Test
    public void aLongSummaryCanBeScrolled() {
        Run r = new Run();
        r.in.present.remove(RobotCheckDevice.FEEDER);
        r.in.present.remove(RobotCheckDevice.FLYWHEEL);
        r.in.present.remove(RobotCheckDevice.INTAKE_MOTOR);
        r.in.present.remove(RobotCheckDevice.PINPOINT);
        r.in.present.remove(RobotCheckDevice.FRONT_LEFT_MOTOR);
        r.toStep(StepId.SUMMARY);
        int perPage = r.cfg.summaryLinesPerPage;
        assertEquals(perPage, r.last.lines.size());
        assertTrue(r.last.lines.get(perPage - 1).startsWith("Lines 1 to " + (perPage - 1) + " of "));
        String firstBefore = r.last.lines.get(0);
        r.go(r.cfg.scrollDown);
        assertNotEquals(firstBefore, r.last.lines.get(0));
        for (int i = 0; i < 40; i++) {
            r.go(r.cfg.scrollDown);
        }
        assertShows(r, "Run this check again and do this step before a match.");
        assertNotShows(r, "RESULT:");
        for (int i = 0; i < 60; i++) {
            r.go(r.cfg.scrollUp);
        }
        assertEquals(firstBefore, r.last.lines.get(0));
    }

    // ====================================================================================== text and safety

    /**
     * Situation: every line shown during a clean run and during a run full of failures is collected.
     * Expected: no line uses the bare abbreviation "tps" or the word "rpm"; every line is plain text without a leading
     *     or trailing blank.
     * Why: no abbreviations like tps without saying ticks per second; the first-year students read these screens.
     */
    @Test
    public void screensNeverUseTheBareAbbreviationTps() {
        Pattern jargon = Pattern.compile("\\b(tps|rpm)\\b", Pattern.CASE_INSENSITIVE);
        List<Run> runs = new ArrayList<Run>();
        runs.add(passingRun());
        Run bad = new Run();
        bad.in.present.remove(RobotCheckDevice.LIMELIGHT);
        bad.in.visionConnected = false;
        bad.toStep(StepId.FLYWHEEL);
        bad.yes();
        bad.in.flywheelTps = 300.0;
        bad.wait(bad.cfg.flywheelTimeoutSec + 0.5);
        bad.yes();
        trial(bad, bad.cfg.feederForwardSec + bad.cfg.feederReverseSec, false);
        runs.add(bad);
        for (Run r : runs) {
            assertTrue(r.everyLine.size() > 100);
            for (String line : r.everyLine) {
                assertFalse("jargon in: " + line, jargon.matcher(line).find());
                assertEquals("untrimmed line: '" + line + "'", line.trim(), line);
                assertFalse(line.isEmpty());
            }
        }
    }

    /**
     * Situation: a whole clean run, watched at every tick.
     * Expected: the run harness (which fails the test on any motor command that is not soon after a press, or above the
     *     power limit) never complains, and the highest power seen is the highest of the test powers.
     * Why: fail safe: no motor without a press, powers stay low.
     */
    @Test
    public void aWholeRunNeverMovesAMotorWithoutAPressOrAboveTheLimit() {
        Run r = passingRun();
        double top = Math.max(Math.max(r.cfg.drivePower, r.cfg.intakeMotorPower), Math.max(r.cfg.intakeServoPower, r.cfg.feederPower));
        assertEquals(top, r.maxPower, 1e-9);
        assertTrue(r.maxPower <= r.cfg.powerLimit);
    }

    /**
     * Situation: all commands in the ready, asking and result phases of every motor step.
     * Expected: no motor is commanded on any ready screen, question screen or result screen.
     * Why: a motor may only run in the running phase, which only a press can start.
     */
    @Test
    public void motorsAreOnlyCommandedWhileARunIsInProgress() {
        Run r = new Run();
        r.toStep(StepId.DRIVE_WHEELS);
        r.wait(1.0);
        assertFalse(r.last.commands.anyOn());
        r.yes();
        assertTrue(r.last.commands.anyOn());
        r.wait(r.cfg.driveSec + 0.1);
        assertFalse(r.last.commands.anyOn());
        r.wait(5.0);
        assertFalse(r.last.commands.anyOn());
    }

    // ================================================================================ the constants

    /**
     * Situation: the real values in BioBuzzConstants.RobotCheck are checked.
     * Expected: no problems.
     * Why: the OpMode refuses to start on a problem, so the shipped values must be clean.
     */
    @Test
    public void theShippedConstantsAreValid() {
        assertEquals(new ArrayList<String>(), RobotCheckConfig.fromConstants().problems());
    }

    /**
     * Situation: a copy of the settings is made worse in one way at a time.
     * Expected: each way is reported: two answer buttons the same, the finish button also a tested button, a test power above
     *     the limit, a battery fail voltage above the pass voltage, and a tolerance larger than the push.
     * Why: a typo in the constants must be found before any motor turns.
     */
    @Test
    public void badConstantsAreReported() {
        RobotCheckConfig c = RobotCheckConfig.fromConstants();
        c.no = c.yes;
        assertContains("problems", c.problems().toString(), "four different buttons");

        c = RobotCheckConfig.fromConstants();
        c.gamepadFinish = Button.A;
        assertContains("problems", c.problems().toString(), "GAMEPAD_FINISH_BUTTON");

        c = RobotCheckConfig.fromConstants();
        c.gamepadButtons = new Button[]{Button.A, Button.RIGHT_STICK_BUTTON};
        assertContains("problems", c.problems().toString(), "also in GAMEPAD_BUTTONS_TO_CHECK");

        c = RobotCheckConfig.fromConstants();
        c.drivePower = c.powerLimit + 0.1;
        assertContains("problems", c.problems().toString(), "POWER_LIMIT");

        c = RobotCheckConfig.fromConstants();
        c.batteryFailVolts = c.batteryPassVolts + 1.0;
        assertContains("problems", c.problems().toString(), "BATTERY_FAIL_VOLTS");

        c = RobotCheckConfig.fromConstants();
        c.pushToleranceIn = c.pushDistanceIn + 1.0;
        assertContains("problems", c.problems().toString(), "PINPOINT_PUSH_");

        c = RobotCheckConfig.fromConstants();
        c.turnToleranceDeg = c.turnDeg;
        assertContains("problems", c.problems().toString(), "PINPOINT_TURN_");

        c = RobotCheckConfig.fromConstants();
        c.leftServoSign = 0.5;
        assertContains("problems", c.problems().toString(), "SERVO_SIGN");

        c = RobotCheckConfig.fromConstants();
        c.tagIdMin = 50;
        assertContains("problems", c.problems().toString(), "HIVE_TAG_ID_MIN");

        c = RobotCheckConfig.fromConstants();
        c.summaryLinesPerPage = 1;
        assertContains("problems", c.problems().toString(), "SUMMARY_LINES_PER_PAGE");

        c = RobotCheckConfig.fromConstants();
        c.scrollUp = null;
        assertContains("problems", c.problems().toString(), "a button is missing");

        c = RobotCheckConfig.fromConstants();
        c.scrollDown = c.scrollUp;
        assertContains("problems", c.problems().toString(), "SCROLL_UP_BUTTON and SCROLL_DOWN_BUTTON");

        c = RobotCheckConfig.fromConstants();
        c.gamepadButtons = new Button[0];
        assertContains("problems", c.problems().toString(), "GAMEPAD_BUTTONS_TO_CHECK is empty");

        c = RobotCheckConfig.fromConstants();
        c.lockoutSec = -1.0;
        assertContains("problems", c.problems().toString(), "BUTTON_LOCKOUT_SEC");

        c = RobotCheckConfig.fromConstants();
        c.powerLimit = 0.0;
        assertContains("problems", c.problems().toString(), "POWER_LIMIT must be above 0");

        c = RobotCheckConfig.fromConstants();
        c.driveSec = 0.0;
        assertContains("problems", c.problems().toString(), "every time (seconds)");

        c = RobotCheckConfig.fromConstants();
        c.flywheelTargetTps = c.flywheelNotTurningTps;
        assertContains("problems", c.problems().toString(), "flywheel target");

        c = RobotCheckConfig.fromConstants();
        c.limelightMinNewFrames = 0;
        assertContains("problems", c.problems().toString(), "LIMELIGHT_MIN_NEW_FRAMES");
    }

    /**
     * Situation: the real constants are read.
     * Expected: the button choices are A = yes, B = no, Y = skip, and the HIVE tag range is 30 to 45.
     * Why: pins the conventions the student guide describes.
     */
    @Test
    public void theButtonAndTagConventionsAreTheDocumentedOnes() {
        RobotCheckConfig c = RobotCheckConfig.fromConstants();
        assertEquals(Button.A, c.yes);
        assertEquals(Button.B, c.no);
        assertEquals(Button.Y, c.skip);
        assertEquals(30, c.tagIdMin);
        assertEquals(45, c.tagIdMax);
        Set<Button> checked = EnumSet.noneOf(Button.class);
        checked.addAll(Arrays.asList(c.gamepadButtons));
        assertTrue(checked.containsAll(EnumSet.of(Button.A, Button.B, Button.X, Button.Y, Button.DPAD_UP,
                Button.DPAD_DOWN, Button.DPAD_LEFT, Button.DPAD_RIGHT, Button.BACK, Button.START, Button.LEFT_BUMPER,
                Button.RIGHT_BUMPER)));
    }
}
