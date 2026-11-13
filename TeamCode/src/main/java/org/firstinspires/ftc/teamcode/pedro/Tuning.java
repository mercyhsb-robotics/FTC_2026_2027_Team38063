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
 */
public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
