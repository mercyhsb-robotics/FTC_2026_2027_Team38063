package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Pedro Pathing settings for our robot, and {@link #create}, which builds Pedro's {@code Follower} from them.
 *
 * <p><b>What Pedro is.</b> Pedro Pathing is the library that drives the robot to a point, along a path, or holds a
 * pose (a position plus a heading), using the robot's measured position. Its main object is the <b>Follower</b>: every
 * loop you call {@code follower.update()} and it reads the odometry and sends power to the wheels. A Follower is
 * built from three parts, which are the three settings blocks below:
 * <ul>
 *   <li><b>drivetrainConfig</b>: how to move the four mecanum wheel motors (names and directions);</li>
 *   <li><b>localizerConfig</b>: how the robot knows where it is. The <b>Pinpoint</b> is a goBILDA odometry computer
 *       with two small tracking wheels ("pods") that roll on the floor (one measures forward motion, one measures
 *       sideways motion) plus a gyro for heading. Pedro calls this part the "localizer";</li>
 *   <li><b>foresightConfig</b>: the numbers for the <b>Foresight</b> algorithm, the part of Pedro that turns "be at
 *       this point" into motor powers. It needs to know how this robot speeds up, coasts and brakes.</li>
 * </ul>
 *
 * <p><b>Where the numbers come from.</b> This is the one file that Pedro's own tuners write; that is why these
 * numbers are here and not in {@link BioBuzzConstants} (only the motor and Pinpoint NAMES come from there). The tuners
 * are in Pedro's tuning menu (see {@code Tuning.java} and docs/START_HERE.md). Run them in this order, each one needs the
 * one before it to be right:
 * <ol>
 *   <li><b>Mecanum Tuner</b> produces {@link #drivetrainConfig};</li>
 *   <li><b>Pinpoint Tuner</b> produces {@link #localizerConfig};</li>
 *   <li><b>Foresight Tuner</b> produces {@link #foresightConfig} (it prints a complete block to paste over the old
 *       one);</li>
 *   <li><b>Tests</b> writes nothing: it checks that the robot now holds and follows lines accurately.</li>
 * </ol>
 *
 * <p>Who uses it: {@code PedroDrive} and {@code BioBuzzRobot} (TeleOp), every autonomous routine and path test, and
 * the tuning procedures.
 */
public class Constants {
    // The drivetrain: which four motors drive the wheels (names must match the Driver Station robot configuration; they
    // are shared with BioBuzzConstants.Hardware) and which way each one must turn. Produced by the Mecanum Tuner: it
    // spins each motor at positive power and you say whether its wheel went forward or backward. Pedro drives forward
    // with POSITIVE power, so a motor whose wheel then turns backward needs REVERSE. (The older MecanumDrive class sets
    // the opposite directions on purpose, because it is fed the raw gamepad sticks where forward is negative; both
    // work for this robot.)
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(BioBuzzConstants.Hardware.FRONT_LEFT_MOTOR);
        c.frontRightName.set(BioBuzzConstants.Hardware.FRONT_RIGHT_MOTOR);
        c.backLeftName.set(BioBuzzConstants.Hardware.BACK_LEFT_MOTOR);
        c.backRightName.set(BioBuzzConstants.Hardware.BACK_RIGHT_MOTOR);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Pinpoint pod offsets (inches) come from the Pedro PinpointTuner. Re-run the tuner after any
    // change to where the pods are mounted and paste the new numbers here.
    //
    // The whole block below is produced by the Pinpoint Tuner. In its words, the "x pod" is the FORWARD pod (it
    // measures motion forward and back) and the "y pod" is the STRAFE pod (it measures sideways motion).
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        // The Pinpoint's name in the robot configuration (BioBuzzConstants.Hardware.PINPOINT).
        c.name.set(BioBuzzConstants.Hardware.PINPOINT);
        // Which kind of goBILDA odometry pod is fitted (chosen on the tuner's first screen). It tells the Pinpoint how
        // many encoder ticks make one inch (the "scalar"). A home-made pod would use ticksPerUnit instead, which the
        // tuner measures by pushing the robot a known distance.
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Where each pod sits relative to the robot's center of rotation, in inches (see offsetUnits below). Measured
        // by the tuner's "Offsets Identification" step: spin the robot in place 180 degrees counter-clockwise. Both
        // values here are tiny (under 0.2 inch).
        c.xPodOffset.set(-0.16720460155817468);
        c.yPodOffset.set(0.03722864342486765);
        // Whether a pod's counting direction has to be flipped: FORWARD = leave it, REVERSED = flip it. Found by the
        // tuner's "Forward Direction" step (push the robot forward) for the x pod and its "Strafe Direction" step
        // (push the robot to the left) for the y pod: a pod whose reading goes negative is REVERSED.
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Units: positions Pedro reports are in inches, and the two offsets above are in inches.
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // PROVISIONAL: these are the official documentation's example Foresight values, NOT values
    // measured on this robot. They are complete and work as a starting point (every required
    // field is set), so paths run today, just less accurately.
    //
    // TO REPLACE THEM: run "Foresight Tuner" from the Pedro tuning menu (registered in
    // Tuning.java; see the order and floor-space notes there). It ends by printing a full
    // foresightConfig block; paste its contents over the body of the lambda below, then delete
    // this PROVISIONAL note.
    //
    // What the settings mean (the Foresight Tuner measures every one of them; the names it prints are in quotes).
    // Distances are in inches, speeds in inches per second, turn angles in radians.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Position correction: a "controller" turns an error (how far the robot is from where it should be) into a
        // motor power. "Proportional" means the output is the error times a constant (kP). The lines below build one
        // controller for forward / backward and one for sideways: kP 0.1 ("Secondary kP") while the error is under 2.5
        // inches, and kP 0.3 ("Primary kP") from 2.5 inches up. Produced by the tuner's "Forward Translational" and
        // "Strafe Translational" steps.
        Controller forwardPrimary = Controller.proportional(0.3);
        Controller forwardSecondary = Controller.proportional(0.1);
        Controller strafePrimary = Controller.proportional(0.3);
        Controller strafeSecondary = Controller.proportional(0.1);
        c.forwardTranslational.set(Controller.piecewise(forwardSecondary).put(2.5, forwardPrimary));
        c.strafeTranslational.set(Controller.piecewise(strafeSecondary).put(2.5, strafePrimary));
        // Feedforward, the power a given speed needs, so the robot does not wait for an error to build up (kV = power
        // per unit of speed). "coast" is the setting for cruising along a path and "brake" the one for slowing down;
        // the tuner prints them as "coast kV" and "brake kV" (it makes the brake value from the coast one).
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        // How hard the robot turns to correct a heading error (a proportional kP). From the tuner's "Heading Tuner"
        // step (it spins the robot in place for a couple of seconds); printed as "heading kP".
        c.headingFeedback.set(Controller.proportional(5.258721785960744));
        // How far the robot keeps turning after power is cut: turn distance (radians) = first number x turn speed
        // + second number x turn speed squared (turn speed in radians per second). From the tuner's "Heading
        // Braking" step (the robot turns back and forth at several speeds).
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
        // How far the robot keeps sliding after power is cut: braking distance (inches) = linear number x speed
        // + quadratic number x speed squared. In each pair the first value is FORWARD and the second is STRAFE
        // (sideways). From the tuner's "Forward Braking" and "Strafe Braking" steps (it drives and brakes at several
        // speeds, so it needs room: see the notes in Tuning.java).
        c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
        // The top speed the robot can really reach, in inches per second (forward, then sideways). From the tuner's
        // "Max Forward Velocity" and "Max Strafe Velocity" steps.
        c.maxAchievableForwardVelocity.set(72.72923108818539);
        c.maxAchievableStrafeVelocity.set(52.34323936525474);
        // How fast the robot slows down by itself when the motors are switched off, in inches per second per second
        // (forward, then sideways). From the tuner's "Forward Deceleration" and "Strafe Deceleration" steps.
        c.naturalForwardDeceleration.set(85.01144677379789);
        c.naturalStrafeDeceleration.set(104.49787535782846);
    });

    /**
     * Builds the Follower, Pedro's main object, from the three settings above: the Pinpoint localizer (where the
     * robot is), the mecanum drivetrain (how to move the wheels) and the Foresight algorithm (how to turn a target
     * into motor powers). Call it once per OpMode, in init. If a device (for example the Pinpoint) is missing from the
     * robot configuration it throws, and {@code BioBuzzRobot} catches that and falls back to the legacy drive.
     */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
            new PinpointLocalizer(hardwareMap, localizerConfig),
            new Mecanum(hardwareMap, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
}
