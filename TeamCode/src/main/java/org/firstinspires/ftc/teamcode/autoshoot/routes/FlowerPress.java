package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * A FLOWER pick-up planned for ONE flat wheel at ONE FLOWER: the poses the robot passes through and the paths between them. Route authors write only
 * {@code intakeFlower(which FLOWER, how many POLLEN)}; everything here is derived from the FLOWER, the robot and the constants
 * ({@link FlowerWiggle} does the maths, {@link WheelChoice} picks the wheel).
 *
 * <p><b>Why a wiggle.</b> The bottom POLLEN of a FLOWER sits in the hole of a ring about 0.4 in tall, so it has to be rolled up AND over the ring's
 * edge (the "lip"). Our two flat wheels lie flat at the front corners and are 12.2 in apart, too far to pinch the 2.8 in ball. So the robot
 * works ONE wheel with a small oscillation: it drives in OFF CENTER so that the wheel is beside the ball, turns on the spot toward the ball (the wheel
 * sweeps sideways into it; the spinning wheel and the push of the whole robot roll it out over the lip), turns back so the next ball can drop into
 * the hole, and repeats once per POLLEN wanted. Then it backs straight out.
 *
 * <p>Top view of the working wheel (the wall at the bottom, the robot facing it; distances from the wall; wheels that stick out 4 in so that it works).
 * {@code (L)} is the middle of the working wheel, {@code ( o )} the bottom POLLEN. Not to scale:
 * <pre>
 *  5.4 in                                  (L)    start: the wheel is 3.8 in to the side of the ball's middle
 *                                           :
 *                                           :     the robot turns about its pose point, 17 degrees toward the ball
 *  4.0 in                  (L)  . . . . . .'      end of the push: 1.8 in from the ball's middle, 1.0 in into it
 *                        ( o )
 *  2.6 in
 *  ======================== wall ===========================================
 * </pre>
 * The chassis (the robot body) stays outside the plate the whole time, also while it turns; only the wheel may be over the plate.
 *
 * <p>The poses (the position is the pose point, the point the robot spins about):
 * <ol>
 *   <li>{@link #lineUp}: where the straight, slow final approach begins ({@code Auto.FLOWER_APPROACH_LENGTH_IN} before {@link #start}); the robot
 *       faces the wall squarely. The step's move to it is a planned, checked straight line from wherever the robot was.</li>
 *   <li>{@link #start}: the approach ends and the wiggle begins. The wheel's middle is {@code Auto.FLOWER_WHEEL_START_OFFSET_IN} from the ball's middle along
 *       the wall. How far from the wall it stands is DERIVED so that the chassis keeps its clearance off the plate at every pose of the turn.</li>
 *   <li>{@link #pushed}: the same position turned by {@link #turnRad} toward the ball, so the wheel is {@code Auto.FLOWER_WHEEL_PRESS_OFFSET_IN} from the ball's middle.</li>
 *   <li>{@link #retreat}: straight back from {@link #start} by the step's retreat length. The step ends here.</li>
 * </ol>
 *
 * <p><b>The rules the validator enforces</b> (walls, plates and the HIVE are checked by {@link RouteValidator} with the real footprint at every pose of
 * every leg, the turn included):
 * <ul>
 *   <li><b>I1</b> nothing of the robot ever goes into or past a wall;</li>
 *   <li><b>I2</b> the CHASSIS (the footprint without the front {@code Robot.INTAKE_REACH_IN}, where only the wheels stick out) never comes within
 *       {@code Auto.FLOWER_CHASSIS_CLEARANCE_IN} of the plate, at ANY pose of the approach, the swing or the retreat;</li>
 *   <li><b>I3</b> only the ONE pressing wheel disc may be over the plate, and it never goes nearer the wall than the POLLEN's center line; the other wheel stays out;</li>
 *   <li><b>I4</b> the approach is perpendicular to the wall, straight and slow, and the retreat is straight: they are derived, so they hold by construction;</li>
 *   <li><b>I5</b> the wheel can actually reach the ball ({@link #reaches()}); if not, the finding says by how many inches the wheels fall short.</li>
 * </ul>
 */
public final class FlowerPress {
    /** Index of the FLOWER (position in {@code Field.FLOWER_CENTERS_IN}). */
    public final int flower;
    /** The flat intake wheel that does the work: LEFT or RIGHT, the robot's own left or right (never AUTO). */
    public final IntakeWheel wheel;
    /** What the route asked for: AUTO (the program chose {@link #wheel}) or the wheel it was told to use. */
    public final IntakeWheel requested;
    /** The numbers (wheel reach, clearance, offsets) the pick-up was derived from. */
    public final FlowerWiggle.Spec spec;
    /** The derived wiggle for this wheel (in the wall frame): standoff, turn, whether the wheel reaches. */
    public final FlowerWiggle.Solution solution;
    /** The pose the straight final approach starts from. */
    public final Pose2 lineUp;
    /** The pose where the approach ends and the wiggle begins (the robot faces the wall, the wheel beside the ball). */
    public final Pose2 start;
    /** The pose at the end of the push: {@link #start} turned toward the ball. */
    public final Pose2 pushed;
    /** The pose after backing straight out. The INTAKE step ends here. */
    public final Pose2 retreat;
    /** How many POLLEN the pick-up is for. */
    public final int count;
    /**
     * How many push-and-release cycles it makes (capped by {@code Auto.FLOWER_WIGGLE_MAX_CYCLES}, so with several cycles per POLLEN it can be fewer than
     * the {@link #count} asked for needs, see {@link #pollenDelivered}).
     */
    public final int cycles;
    /** How far the straight final approach is, in inches. */
    public final double approachLengthIn;
    /** How far the robot backs out, in inches. */
    public final double retreatLengthIn;
    /** The straight, slow final approach {@code lineUp -> start}. */
    public final DrivePath approach;
    /** The turn on the spot {@code start -> pushed} (the way back, {@code pushed -> start}, sweeps the same poses). */
    public final DrivePath swing;
    /** The straight, slow retreat {@code start -> retreat}. */
    public final DrivePath retreatPath;

    /**
     * How many POLLEN this pick-up really delivers: the POLLEN its {@link #cycles} take out of the FLOWER, but never more than {@link #count}, the number
     * asked for. It is less than {@code count} when the cycles were capped ({@code Auto.FLOWER_WIGGLE_MAX_CYCLES}). The robot-side script counts the same way.
     */
    public int pollenDelivered() {
        return Math.min(count, FlowerWiggle.pollenFor(spec, cycles));
    }

    private FlowerPress(int flower, IntakeWheel wheel, IntakeWheel requested, FlowerWiggle.Spec spec, FlowerWiggle.Solution solution,
                        Pose2 lineUp, Pose2 start, Pose2 pushed, Pose2 retreat, int count, int cycles, double approachLengthIn,
                        double retreatLengthIn) {
        this.flower = flower;
        this.wheel = wheel;
        this.requested = requested;
        this.spec = spec;
        this.solution = solution;
        this.lineUp = lineUp;
        this.start = start;
        this.pushed = pushed;
        this.retreat = retreat;
        this.count = count;
        this.cycles = cycles;
        this.approachLengthIn = approachLengthIn;
        this.retreatLengthIn = retreatLengthIn;
        this.approach = DrivePath.straight(lineUp, start);
        this.swing = DrivePath.straight(start, pushed);
        this.retreatPath = DrivePath.straight(start, retreat);
    }

    /**
     * The pick-up of {@code count} POLLEN from FLOWER {@code flower} with the real wheel {@code wheel} (LEFT or RIGHT), for the numbers in
     * {@code spec}. {@code requested} is what the route asked for (AUTO or the wheel itself), kept only so the gallery can say how the wheel was chosen;
     * {@code retreatLengthIn} is how far the robot backs out.
     */
    static FlowerPress plan(AllianceField field, FlowerWiggle.Spec spec, int flower, IntakeWheel wheel, IntakeWheel requested, int count,
                            double retreatLengthIn) {
        return plan(field, spec, flower, FlowerWiggle.solve(wheel, spec), requested, count, retreatLengthIn);
    }

    /**
     * As above for a wiggle that is already worked out. The tests use it to hand the validator a pick-up that is NOT the derived one (for example a standoff
     * that forgets the swing) and check that the rules notice.
     */
    static FlowerPress plan(AllianceField field, FlowerWiggle.Spec spec, int flower, FlowerWiggle.Solution s, IntakeWheel requested, int count,
                            double retreatLengthIn) {
        IntakeWheel wheel = s.wheel;
        double approachLen = BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN;
        Pose2 lineUp = field.flowerPose(flower, s.standoffIn + approachLen, s.lateralIn, Math.PI);
        Pose2 start = field.flowerPose(flower, s.standoffIn, s.lateralIn, Math.PI);
        Pose2 pushed = field.flowerPose(flower, s.standoffIn, s.lateralIn, Math.PI + s.turnRad);
        Pose2 retreat = field.flowerPose(flower, s.standoffIn + retreatLengthIn, s.lateralIn, Math.PI);
        return new FlowerPress(flower, wheel, requested, spec, s, lineUp, start, pushed, retreat, count, FlowerWiggle.cyclesFor(spec, count),
                approachLen, retreatLengthIn);
    }

    /** As {@link #plan(AllianceField, FlowerWiggle.Spec, int, IntakeWheel, IntakeWheel, int, double)} for the real robot's numbers and the default retreat. */
    public static FlowerPress plan(AllianceField field, int flower, IntakeWheel wheel, int count) {
        return plan(field, FlowerWiggle.Spec.fromConstants(), flower, wheel, wheel, count, BioBuzzConstants.Auto.FLOWER_RETREAT_LENGTH_IN);
    }

    /**
     * The pick-up an INTAKE step makes, or null if the step does not take from a FLOWER or its target point is not near any FLOWER (the validator warns
     * about that separately). With a LEFT or RIGHT wheel the wheel is the one asked for; with AUTO it is the one {@link WheelChoice} picks when
     * there is no previous or next step to consider (so by legality and then clearance, LEFT on a tie). Inside a route the choice also weighs the legs before
     * and after: read it from {@code RouteAnalysis.StepInfo#press} instead.
     */
    public static FlowerPress of(Step.Intake step, AllianceField field) {
        return of(step, field, FlowerWiggle.Spec.fromConstants());
    }

    /** As {@link #of(Step.Intake, AllianceField)} for other numbers (a "what if the wheels were longer?"). */
    public static FlowerPress of(Step.Intake step, AllianceField field, FlowerWiggle.Spec spec) {
        if (step.source() != IntakeSource.FLOWER) {
            return null;
        }
        int f = field.nearestFlower(step.target());
        if (f < 0) {
            return null;
        }
        return WheelChoice.choose(field, new TimingModel().withFlowerSpec(spec), step, f, null, WheelChoice.Lookahead.none()).press();
    }

    // ---- derived values ----------------------------------------------------------------------------------

    /** The turn of the push in radians, positive = counter-clockwise (the way the wheel is carried toward the ball). */
    public double turnRad() {
        return solution.turnRad;
    }

    /** The turn of the push in degrees. */
    public double turnDeg() {
        return solution.turnDeg();
    }

    /** True if the wheel gets within the press offset of the ball without breaking a rule (see {@link FlowerWiggle.Solution#reaches}). */
    public boolean reaches() {
        return solution.reaches;
    }

    /** How many more inches the wheels would have to stick out for the pick-up to work (0 if it works; infinite if no length helps). */
    public double missingReachIn() {
        return solution.missingReachIn;
    }

    /** The distance from the pose point to the middle of the wheel, in inches: the lever arm of the turn. */
    public double leverArmIn() {
        return solution.leverArmIn;
    }

    /** The middle of the pressing wheel at the start of the wiggle, in field inches. */
    public Point2 wheelAtStart() {
        return Footprint.wheelCenter(start, wheel);
    }

    /** The middle of the pressing wheel at the end of the push, in field inches. */
    public Point2 wheelPushed() {
        return Footprint.wheelCenter(pushed, wheel);
    }

    /** The middle of the bottom POLLEN of this FLOWER, in field inches. */
    public Point2 ball(AllianceField field) {
        return field.flowerCenter(flower);
    }

    /** True if the program chose the wheel ({@link #requested} is AUTO). */
    public boolean wasChosen() {
        return requested.isAuto();
    }

    /** One short line for lists: how many POLLEN, which wheel, how far it turns and how many times. */
    public String describe() {
        return "FLOWER " + (flower + 1) + " with the " + wheel + " wheel: " + String.format(Locale.ROOT, "%.1f", Math.abs(turnDeg()))
                + " deg wiggle x" + cycles;
    }
}
