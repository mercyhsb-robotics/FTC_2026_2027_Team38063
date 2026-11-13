package org.firstinspires.ftc.teamcode.autoshoot.core;

import static org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckText.f0;
import static org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckText.f1;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheck.DriveKind;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheck.Verdict;

/**
 * Every PASS / WARN / FAIL decision of the Robot Check, and the plain-language advice that goes with it. Each method
 * takes a measurement or the student's answer and returns a {@link Judgement}: a verdict, a short description of what
 * was seen, and the ONE thing to do about it (empty for a PASS).
 *
 * <p>Why it is separate from {@link RobotCheck}: the state machine decides WHEN things happen (which screen, which
 * motor, how long); this class decides WHAT the numbers mean. Keeping the two apart means every threshold and every
 * piece of advice can be tested by calling one method with one number. All thresholds come from
 * {@link RobotCheckConfig} (and so from {@code BioBuzzConstants.RobotCheck}); nothing here is a hidden number.
 *
 * <p>Plain Java, no FTC SDK. Numbers shown to the student always carry their unit written out (inches, degrees,
 * volts, seconds, "ticks per second").
 *
 * <p>Words used here. <b>Odometry</b> means working out where the robot is from how far its wheels (or small
 * tracking wheels called pods) have turned. The <b>Pinpoint</b> is the goBILDA computer that does this for us. A
 * <b>pose</b> is a position plus a heading (the direction the robot faces). <b>Pedro</b> is the library that drives
 * the robot with that pose; its settings are in pedro/Constants.java.
 */
public final class RobotCheckJudge {

    /** The outcome of one step: the verdict, what was seen, and the one thing to do (advice is empty for a PASS). */
    public static final class Judgement {
        public final Verdict verdict;
        /** What was seen, in plain words. May hold several lines separated by a line break. */
        public final String detail;
        /** The one thing to do about it. Empty for a PASS. */
        public final String advice;

        public Judgement(Verdict verdict, String detail, String advice) {
            this.verdict = verdict;
            this.detail = detail;
            this.advice = advice;
        }

        @Override
        public String toString() {
            return verdict + ": " + detail + (advice.isEmpty() ? "" : "  -> " + advice);
        }
    }

    private final RobotCheckConfig cfg;
    // Only used to ask "what are the lowest and highest speeds that count as at speed?" (the same window the TeleOp uses).
    private final FlywheelReady window;

    public RobotCheckJudge(RobotCheckConfig cfg) {
        this.cfg = cfg;
        this.window = new FlywheelReady(cfg.atSpeedBelowFrac, cfg.atSpeedAboveFrac, cfg.atSpeedFloorTps,
                cfg.atSpeedRiseSec, cfg.atSpeedFallSec);
    }

    private static Judgement pass(String detail) {
        return new Judgement(Verdict.PASS, detail, "");
    }

    private static Judgement warn(String detail, String advice) {
        return new Judgement(Verdict.WARN, detail, advice);
    }

    private static Judgement fail(String detail, String advice) {
        return new Judgement(Verdict.FAIL, detail, advice);
    }

    /** The result of a step the student skipped. */
    public Judgement skipped() {
        return new Judgement(Verdict.SKIPPED, "You skipped this step.",
                "Run this check again and do this step before a match.");
    }

    /** The safety step has no measurement: pressing the yes button is the confirmation. */
    public Judgement safetyConfirmed() {
        return pass("You confirmed the robot was on blocks and everyone was clear.");
    }

    // ------------------------------------------------------------------------------------ 1. battery

    /** Step 1: PASS at or above the pass voltage, FAIL below the fail voltage, WARN in between; WARN if no voltage was read. */
    public Judgement battery(double volts) {
        if (Double.isNaN(volts) || volts <= 0) {
            return warn("The Control Hub did not report a battery voltage.",
                    "Check that the battery is plugged in and charged, then press " + RobotCheckText.button(cfg.repeat)
                            + " to read it again.");
        }
        String reading = "Battery: " + f1(volts) + " volts";
        if (volts >= cfg.batteryPassVolts) {
            return pass(reading + " (good: at least " + f1(cfg.batteryPassVolts) + " volts).");
        }
        if (volts >= cfg.batteryFailVolts) {
            return warn(reading + " (a little low: good is at least " + f1(cfg.batteryPassVolts) + " volts).",
                    "Swap in a fully charged battery before you test the shooter: a weak battery makes the flywheel slow.");
        }
        return fail(reading + " (too low: it must be at least " + f1(cfg.batteryFailVolts) + " volts).",
                "Put in a fully charged battery now, then run this check again. A very low battery makes the motors weak"
                        + " and the robot may restart.");
    }

    // ------------------------------------------------------------------------------- 2. hardware names

    /**
     * Step 2: one line per missing device, in plain words. FAIL if a device the robot needs is missing, WARN if only
     * optional ones (the IMU, the Limelight) are, PASS if none is.
     *
     * @param total   how many names were checked
     * @param missing the devices that were not found in the robot configuration
     */
    public Judgement names(int total, List<RobotCheckDevice> missing) {
        if (missing.isEmpty()) {
            return pass("All " + total + " names from BioBuzzConstants.Hardware are in the robot configuration.");
        }
        List<String> lines = new ArrayList<String>();
        List<String> names = new ArrayList<String>();
        boolean anyRequired = false;
        for (RobotCheckDevice d : missing) {
            lines.add("The robot configuration has no device called '" + d.configName + "': " + d.description
                    + ". Device type needed: " + d.kind.words + ".");
            names.add("'" + d.configName + "'");
            anyRequired |= d.required;
        }
        String advice = "Open the Driver Station > Configure Robot and add or rename: " + RobotCheckText.join(names, ", ")
                + ". The name must have exactly the same spelling and capital letters. Then Save and Activate."
                + " (Or change the name in BioBuzzConstants.Hardware.)";
        String detail = RobotCheckText.join(lines, "\n");
        return anyRequired ? fail(detail, advice) : warn(detail, advice);
    }

    /** A step that cannot run because a device it needs is not in the robot configuration (the FAIL says which, and what to do first). */
    public Judgement deviceMissing(List<RobotCheckDevice> missing) {
        List<String> names = new ArrayList<String>();
        for (RobotCheckDevice d : missing) {
            names.add("'" + d.configName + "' (" + d.description + ")");
        }
        return fail("This step cannot run: the robot configuration has no device called " + RobotCheckText.join(names, " and ") + ".",
                "Fix the missing name first (step 2: Driver Station > Configure Robot), then run this step again.");
    }

    // -------------------------------------------------------------------------------- 3. drive wheels

    /**
     * Step 3: the student's four answers.
     *
     * @param turnedForward for each wheel (front-left, back-left, front-right, back-right) whether it turned forward
     * @param kind          which drive the TeleOp really uses: Pedro, or the backup MecanumDrive
     */
    public Judgement wheels(boolean[] turnedForward, DriveKind kind) {
        List<String> bad = new ArrayList<String>();
        List<String> advice = new ArrayList<String>();
        for (int i = 0; i < turnedForward.length; i++) {
            if (!turnedForward[i]) {
                bad.add(RobotCheckText.wheel(i));
                advice.add(wheelAdvice(i, kind));
            }
        }
        if (bad.isEmpty()) {
            return pass("All " + turnedForward.length + " wheels turned forward.");
        }
        return fail("These wheels did not turn forward: " + RobotCheckText.join(bad, ", ") + ".",
                RobotCheckText.join(advice, " "));
    }

    private String wheelAdvice(int wheel, DriveKind kind) {
        String name = RobotCheckText.wheel(wheel);
        String other = " If a DIFFERENT wheel moved, two motor names or two cables are swapped: check the " + name
                + " motor's name (BioBuzzConstants.Hardware) and its port.";
        if (kind == DriveKind.PEDRO) {
            return "The " + name + " wheel did not turn forward. Open pedro/Constants.java, find drivetrainConfig and change "
                    + RobotCheckText.wheelPedroSetting(wheel) + " to the other value (FORWARD becomes REVERSE, or REVERSE becomes FORWARD)."
                    + other;
        }
        return "The " + name + " wheel did not turn forward. Open mechanisms/mecanumdrive/MecanumDrive.java, find init() and change "
                + RobotCheckText.wheelLegacySetting(wheel) + ".setDirection(...) to the other value (FORWARD becomes REVERSE, or REVERSE"
                + " becomes FORWARD)." + other;
    }

    // ------------------------------------------------------------------------------ 4. Pinpoint odometry

    /** The Pinpoint (or Pedro) could not be started at all: the plain-language FAIL. */
    public Judgement pinpointUnavailable(String reason) {
        String why = reason == null || reason.isEmpty() ? "" : " (" + reason + ")";
        return fail("Pedro could not start the Pinpoint" + why + ", so the robot does not know where it is.",
                "Check the Pinpoint's cable (and that the Control Hub I2C port matches the robot configuration) and that the"
                        + " Pinpoint is named 'pinpoint' there, then run the check again.");
    }

    /** If the Pinpoint reports a fault, the FAIL that explains it; null if the status is not a fault. */
    public Judgement pinpointFault(String status) {
        if (status == null || !status.startsWith("FAULT")) {
            return null;
        }
        String what;
        String advice;
        if (status.contains("NO_PODS")) {
            what = "cannot see either tracking pod";
            advice = "Plug in both odometry pod cables (the forward pod into the X port, the strafe pod into the Y port).";
        } else if (status.contains("X_POD")) {
            what = "cannot see the forward (X) pod";
            advice = "Check the forward pod's cable in the Pinpoint's X port.";
        } else if (status.contains("Y_POD")) {
            what = "cannot see the strafe (Y) pod";
            advice = "Check the strafe pod's cable in the Pinpoint's Y port.";
        } else if (status.contains("IMU")) {
            what = "has a gyro fault";
            advice = "Switch the robot off and on again and keep it still for a few seconds while it starts.";
        } else {
            what = "reported a bad read";
            advice = "Check the Pinpoint's cable and its connector, then switch the robot off and on again.";
        }
        return fail("The Pinpoint " + what + " (it says " + status + ").", advice);
    }

    /**
     * Part 1 of the Pinpoint step: the robot was pushed forward by hand. The numbers are measured along the robot's
     * own forward and left directions (so it does not matter which way the robot faced on the field).
     *
     * @param forwardIn  how far the pose moved along the robot's forward direction, in inches (negative = backward)
     * @param sidewaysIn how far it moved to the robot's left, in inches (negative = to the right)
     */
    public Judgement pinpointPush(double forwardIn, double sidewaysIn) {
        double want = cfg.pushDistanceIn;
        double tol = cfg.pushToleranceIn;
        String seen = "Push: the Pinpoint says the robot moved " + f1(forwardIn) + " inches forward (negative means backward) and "
                + f1(sidewaysIn) + " inches to the left (negative means right); you pushed about " + f1(want) + " inches forward.";
        if (Math.hypot(forwardIn, sidewaysIn) < cfg.noMovementIn) {
            return fail(seen, "Nothing changed. Check that the Pinpoint has power (its light is on) and its cable is plugged in,"
                    + " that both pods touch the floor (the robot must be OFF the blocks), and the name 'pinpoint' in Configure Robot.");
        }
        if (Math.abs(forwardIn - want) <= tol) {
            if (Math.abs(sidewaysIn) <= cfg.pushSidewaysMaxIn) {
                return pass(seen);
            }
            return warn(seen, "The robot also moved sideways. Push it straight forward again (press " + RobotCheckText.button(cfg.repeat)
                    + "). If it keeps happening, check the strafe (Y) pod: its direction and offset in pedro/Constants.java, localizerConfig.");
        }
        if (Math.abs(forwardIn + want) <= tol) {
            return fail(seen, "The Pinpoint thinks the robot went BACKWARD. Open pedro/Constants.java, find localizerConfig and change"
                    + " c.xPodDirection to the other value (FORWARD becomes REVERSED, or REVERSED becomes FORWARD).");
        }
        if (Math.abs(sidewaysIn) > Math.abs(forwardIn) && Math.abs(Math.abs(sidewaysIn) - want) <= tol) {
            return fail(seen, "The Pinpoint saw SIDEWAYS movement for a forward push. Check that the forward pod is in the Pinpoint's X"
                    + " port and the strafe pod in its Y port, and that the Pinpoint's X arrow points to the front of the robot"
                    + " (pedro/Constants.java, localizerConfig: xPodDirection, yPodDirection).");
        }
        return fail(seen, "The distance is wrong. If you really pushed about " + f1(want) + " inches, check podType (the kind of pod fitted)"
                + " and the offsets in pedro/Constants.java, localizerConfig, or run the Pinpoint Tuner. Not sure how far you pushed?"
                + " Press " + RobotCheckText.button(cfg.repeat) + " and push along a ruler.");
    }

    /**
     * Part 2 of the Pinpoint step: the robot was turned by hand a quarter turn counter-clockwise on the spot.
     *
     * @param headingChangeDeg how the heading changed, in degrees (counter-clockwise is positive)
     * @param driftIn          how far the position moved during the turn, in inches (it should stay almost still)
     */
    public Judgement pinpointTurn(double headingChangeDeg, double driftIn) {
        double want = cfg.turnDeg;
        double tol = cfg.turnToleranceDeg;
        String seen = "Turn: the heading changed by " + f0(headingChangeDeg) + " degrees (counter-clockwise is positive); you turned about "
                + f0(want) + " degrees. The position moved " + f1(driftIn) + " inches while turning.";
        if (Math.abs(headingChangeDeg) < cfg.noTurnDeg) {
            return fail(seen, "The heading did not change. Check the Pinpoint's cable and keep the robot still for a second after INIT"
                    + " so its gyro can start, then repeat (press " + RobotCheckText.button(cfg.repeat) + ").");
        }
        if (Math.abs(headingChangeDeg - want) <= tol) {
            if (driftIn > cfg.turnDriftWarnIn) {
                return warn(seen, "The position slid while you turned the robot. Turn it again on the spot (press "
                        + RobotCheckText.button(cfg.repeat) + "). If it still slides, the pod offsets are wrong: in"
                        + " pedro/Constants.java, localizerConfig, check xPodOffset and yPodOffset or run the Pinpoint Tuner.");
            }
            return pass(seen);
        }
        if (Math.abs(headingChangeDeg + want) <= tol) {
            return fail(seen, "The heading went the wrong way. Make sure you turned the robot counter-clockwise (the front swings to the"
                    + " LEFT). If you did, check that the Pinpoint is mounted flat with its label facing up.");
        }
        return fail(seen, "The heading does not match a quarter turn. Turn the robot again to exactly a quarter turn (press "
                + RobotCheckText.button(cfg.repeat) + "). If it still disagrees, ask a mentor to check the Pinpoint's gyro.");
    }

    /** The student pressed the 'problem' button during one part of the Pinpoint step. */
    public Judgement pinpointProblemReported(int part) {
        if (part == 0) {
            return fail("You reported that the pose did not follow the push.",
                    "Check that the Pinpoint has power (its light is on) and its cable is plugged in, that both pods touch the floor"
                            + " (the robot must be OFF the blocks), and the name 'pinpoint' in Configure Robot.");
        }
        return fail("You reported that the heading did not follow the turn.",
                "Check the Pinpoint's cable and keep the robot still for a second after INIT so its gyro can start, then repeat.");
    }

    /** Puts the push and the turn together: the worst verdict wins and both pieces of advice are kept. */
    public Judgement pinpointBoth(Judgement push, Judgement turn) {
        Verdict worst = worse(push.verdict, turn.verdict);
        List<String> advice = new ArrayList<String>();
        if (!push.advice.isEmpty()) {
            advice.add(push.advice);
        }
        if (!turn.advice.isEmpty()) {
            advice.add(turn.advice);
        }
        return new Judgement(worst, push.detail + "\n" + turn.detail, RobotCheckText.join(advice, " "));
    }

    private static Verdict worse(Verdict a, Verdict b) {
        return rank(a) >= rank(b) ? a : b;
    }

    private static int rank(Verdict v) {
        switch (v) {
            case FAIL:
                return 3;
            case WARN:
                return 2;
            case PASS:
                return 1;
            default:
                return 0;
        }
    }

    // ------------------------------------------------------------------------------------ 5. intake

    /** Plain name of intake part number {@code part} (0 roller, 1 left corner servo, 2 right corner servo). */
    public static String intakePartName(int part) {
        switch (part) {
            case 0:
                return "front roller motor (the big roller across the front)";
            case 1:
                return "left corner servo (the corner wheel on the robot's left)";
            default:
                return "right corner servo (the corner wheel on the robot's right)";
        }
    }

    /** The device that intake part number {@code part} uses. */
    public static RobotCheckDevice intakePartDevice(int part) {
        switch (part) {
            case 0:
                return RobotCheckDevice.INTAKE_MOTOR;
            case 1:
                return RobotCheckDevice.LEFT_INTAKE_SERVO;
            default:
                return RobotCheckDevice.RIGHT_INTAKE_SERVO;
        }
    }

    private String intakeAdvice(int part) {
        RobotCheckDevice d = intakePartDevice(part);
        String missing = "Did it not move at all? Check its cable and that '" + d.configName + "' is in Configure Robot.";
        if (part == 0) {
            return missing + " Did it push POLLEN OUT instead of in? Open mechanisms/intake/Intake.java and add"
                    + " intakeMotor.setDirection(DcMotor.Direction.REVERSE); at the end of init().";
        }
        boolean left = part == 1;
        return missing + " Did it turn the wrong way? In teleop/BioBuzzRobot.java, manualControls(), change the sign of the "
                + (left ? "servoLeft" : "servoRight") + " power, and set BioBuzzConstants.RobotCheck."
                + (left ? "INTAKE_LEFT_SERVO_SIGN" : "INTAKE_RIGHT_SERVO_SIGN") + " to match.";
    }

    /**
     * Step 5: the answers for the three parts.
     *
     * @param pulledInward for each part, whether it pulled inward
     * @param tested       for each part, false if it could not be run because its device is missing (that counts as a FAIL)
     */
    public Judgement intake(boolean[] pulledInward, boolean[] tested) {
        List<String> bad = new ArrayList<String>();
        List<String> advice = new ArrayList<String>();
        for (int i = 0; i < pulledInward.length; i++) {
            if (!tested[i]) {
                bad.add("the " + intakePartName(i) + " (not tested: '" + intakePartDevice(i).configName + "' is missing)");
                advice.add("Fix the missing name '" + intakePartDevice(i).configName + "' first (step 2: Configure Robot).");
            } else if (!pulledInward[i]) {
                bad.add("the " + intakePartName(i));
                advice.add("The " + intakePartName(i) + " did not pull inward. " + intakeAdvice(i));
            }
        }
        if (bad.isEmpty()) {
            return pass("The front roller and both corner servos pulled inward.");
        }
        return fail("These did not pull inward: " + RobotCheckText.join(bad, "; ") + ".", RobotCheckText.join(advice, " "));
    }

    // ----------------------------------------------------------------------------------- 6. flywheel

    /** The slowest speed that counts as "at speed" for the check's target, in ticks per second. */
    public double flywheelLowerTps() {
        return window.lowerBound(cfg.flywheelTargetTps);
    }

    /** The fastest speed that counts as "at speed" for the check's target, in ticks per second. */
    public double flywheelUpperTps() {
        return window.upperBound(cfg.flywheelTargetTps);
    }

    /**
     * Step 6: did the flywheel get within the at-speed window?
     *
     * @param reached      true if the measured speed got inside the window (and stayed there long enough)
     * @param lastTps      the last measured speed, in ticks per second
     * @param peakTps      the highest measured speed, in ticks per second
     * @param lowestTps    the lowest measured speed (negative means it turned backward), in ticks per second
     * @param stoppedEarly true if the student stopped the run before it finished
     */
    public Judgement flywheel(boolean reached, double lastTps, double peakTps, double lowestTps, boolean stoppedEarly) {
        String goal = "goal: between " + f0(flywheelLowerTps()) + " and " + f0(flywheelUpperTps()) + " ticks per second";
        if (reached) {
            return pass("The flywheel reached " + f0(lastTps) + " ticks per second (" + goal + ").");
        }
        if (stoppedEarly) {
            return fail("You stopped the flywheel before it reached speed (it was at " + f0(lastTps) + " ticks per second; " + goal + ").",
                    "Run this step again (press " + RobotCheckText.button(cfg.repeat) + ") and let it run until it reaches speed.");
        }
        double notTurning = cfg.flywheelNotTurningTps;
        if (peakTps < notTurning && -lowestTps < notTurning) {
            return fail("The flywheel did not turn (its fastest speed was " + f0(peakTps) + " ticks per second; " + goal + ").",
                    "Check the flywheel motor's cable and port, that it is named '" + RobotCheckDevice.FLYWHEEL.configName
                            + "' in Configure Robot, and that nothing blocks the wheel.");
        }
        if (lowestTps <= -notTurning && -lowestTps > peakTps) {
            return fail("The flywheel turned the WRONG WAY (it reached " + f0(lowestTps) + " ticks per second; " + goal + ").",
                    "Reverse the flywheel motor: open mechanisms/launcher/Launcher.java and add"
                            + " flywheel.setDirection(DcMotor.Direction.REVERSE); in init().");
        }
        return fail("The flywheel was too slow: it reached only " + f0(peakTps) + " ticks per second in "
                        + f1(cfg.flywheelTimeoutSec) + " seconds (" + goal + ").",
                "Check the battery voltage (step 1), the motor cable and direction, and the speed controller settings"
                        + " BioBuzzConstants.Shooter.FLYWHEEL_PIDF_P and FLYWHEEL_PIDF_F.");
    }

    // ------------------------------------------------------------------------------------ 7. feeder

    /** Step 7: the student's answer to "did the feeder turn forward and then back?". */
    public Judgement feeder(boolean turnedRightWay) {
        if (turnedRightWay) {
            return pass("The feeder turned forward (toward the flywheel) and then back.");
        }
        return fail("The feeder did not turn the right way.",
                "Did it not move at all? Check its cable and that '" + RobotCheckDevice.FEEDER.configName + "' is a continuous-rotation"
                        + " servo in Configure Robot. Did it turn the wrong way? In BioBuzzConstants.Shooter change FEEDER_INVERTED from "
                        + cfg.feederInverted + " to " + !cfg.feederInverted + ".");
    }

    // ----------------------------------------------------------------------------------- 8. Limelight

    /** The Limelight is not in the robot configuration, or the hub cannot reach it. A WARN: the robot still works without it. */
    public Judgement limelightMissing(boolean inConfiguration) {
        if (!inConfiguration) {
            return warn("The robot configuration has no Limelight called '" + RobotCheckDevice.LIMELIGHT.configName
                            + "'. The robot still drives and shoots without it, but it cannot see the HIVE tags.",
                    "To use the camera: Driver Station > Configure Robot, add the Limelight 3A as a USB device named exactly '"
                            + RobotCheckDevice.LIMELIGHT.configName + "' (BioBuzzConstants.Hardware.LIMELIGHT).");
        }
        return warn("The Limelight is in the robot configuration but the Control Hub cannot reach it. The robot still drives and"
                        + " shoots without it, but it cannot see the HIVE tags.",
                "Check the USB-C cable from the Limelight to the Control Hub's blue USB 3 port, and wait about 20 seconds after"
                        + " switching on, then repeat (press " + RobotCheckText.button(cfg.repeat) + ").");
    }

    /**
     * The first half of the Limelight step: is the camera sending pictures, and is it running the right pipeline?
     * Returns null if all is well, otherwise the verdict to show.
     *
     * @param connected whether the Control Hub can reach the camera
     * @param newFrames how many NEW pictures arrived during the check
     * @param status    the status of the latest result
     */
    public Judgement limelightLink(boolean connected, int newFrames, VisionFrame.Status status) {
        if (!connected || status == VisionFrame.Status.NOT_CONNECTED) {
            return limelightMissing(true);
        }
        if (newFrames < cfg.limelightMinNewFrames) {
            String advice = status == VisionFrame.Status.NO_RESULT
                    ? "The camera has not sent any result yet. Wait about 20 seconds after switching on and press "
                    + RobotCheckText.button(cfg.repeat) + "; if it never does, open the Limelight web page and check that pipeline "
                    + cfg.pipelineIndex + " exists."
                    : "Check the USB cable, switch the Limelight off and on again, and open its web page to see if it is running.";
            return fail("The camera is connected but only " + newFrames + " new pictures arrived in " + f1(cfg.limelightCheckSec)
                    + " seconds (at least " + cfg.limelightMinNewFrames + " are needed).", advice);
        }
        if (status == VisionFrame.Status.WRONG_PIPELINE) {
            return fail("The Limelight is not running pipeline " + cfg.pipelineIndex + " (the AprilTag pipeline).",
                    "Open the Limelight web page (http://limelight.local:5801) and make sure pipeline " + cfg.pipelineIndex
                            + " is the AprilTag pipeline (BioBuzzConstants.Vision.PIPELINE_INDEX).");
        }
        if (status == VisionFrame.Status.STALE || status == VisionFrame.Status.STALLED) {
            return fail("The camera pictures are too old or frozen (status " + status + ").",
                    "Check the USB cable, switch the Limelight off and on again, and open its web page to see if it is running.");
        }
        return null;
    }

    /** A HIVE tag was seen: PASS (WARN if the tag has no distance because 'Full 3D' is off). */
    public Judgement limelightTag(TagObservation tag) {
        String seen = "Saw HIVE tag number " + tag.id + ": tx = " + f1(tag.txDeg) + " degrees (positive means right of the picture's middle), ty = "
                + f1(tag.tyDeg) + " degrees (positive means above the middle).";
        if (!tag.hasPosition) {
            return warn(seen + " But there is no distance for it.",
                    "In the Limelight web page turn on 'Full 3D' (Advanced tab) so the robot can measure how far away the tags are.");
        }
        return pass(seen);
    }

    /**
     * No HIVE tag was found.
     *
     * @param otherTagId the id of a tag that was seen but is not a HIVE tag, or -1 if no tag at all was seen
     * @param gaveUp     true if the student pressed the problem button, false if the time ran out
     */
    public Judgement limelightNoTag(int otherTagId, boolean gaveUp) {
        String range = cfg.tagIdMin + " to " + cfg.tagIdMax;
        if (otherTagId >= 0) {
            return fail("The camera saw tag number " + otherTagId + ", which is not a HIVE tag (HIVE tags are " + range + ").",
                    "Hold a HIVE tag (" + range + "), or fix the ID filter on the Limelight web page so it allows " + range + ".");
        }
        String when = gaveUp ? " (you pressed the problem button)." : " in " + f0(cfg.limelightTagTimeoutSec) + " seconds.";
        return fail("The camera did not see any tag" + when,
                "Hold the tag flat, facing the lens (the camera looks out the BACK of the robot), about an arm's length away, in good light."
                        + " If it still sees nothing, check the pipeline on the Limelight web page: marker size 82.55 millimeters, ID filter "
                        + range + ", family 36h11, and the exposure.");
    }

    // ------------------------------------------------------------------------------------ 9. gamepad

    /** Step 9: PASS if every control was seen; otherwise FAIL naming the ones that were not. */
    public Judgement gamepad(List<String> notSeen, int total) {
        if (notSeen.isEmpty()) {
            return pass("All " + total + " controls (buttons and triggers) were seen.");
        }
        return fail("These controls were not seen: " + RobotCheckText.join(notSeen, ", ") + ".",
                "Press each of those once (press " + RobotCheckText.button(cfg.repeat) + " to try again). If one never shows up, the gamepad may be in the"
                        + " wrong mode or the wrong gamepad is paired: on the Driver Station press Start and A on the PlayStation gamepad again.");
    }
}
