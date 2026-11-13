package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro Pathing tuning procedures. Every {@code @Tuner} method below shows up as a procedure in
 * Pedro's tuning menu, by the name its Procedure gives itself.
 *
 * <p>Run them in this order. Each one needs the one before it to be right:
 * <ol>
 *   <li><b>Mecanum Tuner</b> - spins each wheel so you can confirm the motor names and directions
 *       in {@link Constants#drivetrainConfig}.</li>
 *   <li><b>Pinpoint Tuner</b> - checks the pod direction and scalar, and gives the offsets for
 *       {@link Constants#localizerConfig}. Re-run it whenever the pods are moved.</li>
 *   <li><b>Foresight Tuner</b> - measures how the robot accelerates and brakes. It prints a
 *       complete {@code foresightConfig} Java block: paste it over
 *       {@link Constants#foresightConfig}.</li>
 *   <li><b>Tests</b> - runs a hold test and a line test with the tuned values so you can see
 *       whether the robot stops where it should.</li>
 * </ol>
 *
 * <p>Foresight Tuner starting values (all editable in the tuner's input screens, nothing needs to
 * change in code): 48 in drive for the top-speed runs, 30 in/s for the deceleration runs and 36 in
 * for the braking runs (the tuner enforces a 15 in minimum there). Clear about 6 ft of floor in a
 * straight line forward and to the side of the robot, with the battery freshly charged, because
 * every run ends with the robot coasting past its target. The values already in
 * {@link Constants#foresightConfig} are the example values from Pedro's documentation and are
 * only a working starting point until this tuner has been run on the real robot.
 *
 * <p><b>What this class is.</b> It is only a list: it tells Pedro's tuning menu which procedures exist. The procedures
 * themselves are in the {@code pedro/procedures} folder. Pedro finds the methods below by their {@code @Tuner}
 * annotation (all four are {@code public static} and return a {@code Procedure}). The tuning menu is a web page, not an
 * item in the Driver Station list: open http://192.168.43.1:10158 in a browser on a laptop that is on the Robot Controller's Wi-Fi
 * (keep the Driver Station connected); there you run a procedure, answer its questions, and it shows
 * the numbers and the Java code to paste into {@code Constants.java}. See docs/START_HERE.md for the full guide.
 *
 * <p><b>Words used here.</b> <b>Pedro Pathing</b> is the library that drives the robot along paths using its
 * measured position. The <b>Pinpoint</b> is the goBILDA odometry computer that measures that position.
 * <b>Mecanum</b> wheels have angled rollers so the robot can move sideways. <b>Foresight</b> is Pedro's
 * path-following algorithm; it needs to know how this robot accelerates and brakes. A "localizer" is Pedro's word for
 * the part that works out where the robot is.
 */
public class Tuning {
    // Tuners go here
    // Step 1: finds the wheel motor directions (result goes to Constants.drivetrainConfig).
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    // Step 2: checks the Pinpoint pods and measures their offsets (result goes to Constants.localizerConfig).
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // Step 3: measures speed, braking and gains (result goes to Constants.foresightConfig). It needs to build the
    // localizer and the drivetrain itself. The two arrows below are small functions that do that when the
    // procedure runs: given the robot's hardwareMap (the list of connected devices), they build a Pinpoint localizer
    // and a mecanum drivetrain from the settings currently in Constants.
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    // Step 4: hold test, line test and so on, using the tuned values. It builds its drivetrain, localizer and
    // Foresight algorithm from Constants in the same way (the last function takes no arguments because Foresight needs
    // no hardware).
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
