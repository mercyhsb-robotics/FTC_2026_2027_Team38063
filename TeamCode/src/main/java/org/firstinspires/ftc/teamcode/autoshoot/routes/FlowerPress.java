package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.Footprint;
import org.firstinspires.ftc.teamcode.autoshoot.core.IntakeWheel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Point2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * The geometry of taking the bottom POLLEN out of a FLOWER: a PRESS-IN. Route authors write only a simple
 * {@code intakeFlower(...)}; this class derives everything else from the FLOWER and from that step.
 *
 * <p><b>Why a press-in.</b> The FLOWER holds its POLLEN in a stack in the middle of a plate that stands on the field wall.
 * The bottom POLLEN sits in the hole of a ring that is about 0.4 in tall (0.354 in in the official CAD, 0.43 in in the manual),
 * so to get it out the ball has to be pushed UP AND OVER the edge of that ring (the "lip"). Our intake wheels lie flat (parallel to the floor) at the front corners of
 * the robot. They do the job only if they <b>press into</b> the POLLEN and roll it over the lip toward the robot. So the
 * robot faces the wall, drives straight in until the wheel overlaps the POLLEN, runs the intake while it stays pressed in,
 * then backs straight out. It is not a drive-by and not a drive through the plate.
 *
 * <p><b>One wheel, not two.</b> The two wheels are 12.2 in apart (center to center), far more than the 2.8 in POLLEN, so the
 * pair cannot straddle and pinch it. ONE wheel presses while the robot stands to one side of the FLOWER's center line, shifted
 * by {@code Robot.INTAKE_WHEEL_OFFSET_IN} (6.1 in) so that the middle of that wheel is on the POLLEN; the other wheel is
 * 12.2 in away along the wall, clear of the plate. The step names the wheel ({@link Step.Intake#wheel()}).
 *
 * <p>Top view of the press, FLOWER on the wall at the bottom, robot facing the wall (not to scale; the numbers are the
 * defaults of {@code BioBuzzConstants}):
 * <pre>
 *                    robot (front toward the wall)
 *            +-----------------------------+
 *            |           chassis           |        chassis front: 5.533 in from the wall
 *            +--+------+----------+------+-+        (plate edge 5.033 in + clearance 0.5 in)
 *               |wheel |          |wheel |           the two flat wheels stick out 2.7 in
 *               +--+---+          +------+           in front of the chassis
 *     plate        |
 *   +----------+   |  the pressing wheel is on the POLLEN, 1.201 in past its edge
 *   |  (POLLEN)|---+  front tip of the wheel: 2.833 in from the wall
 *   +----------+
 * ==================== wall ===================
 * </pre>
 * Measured out from the wall, with the default numbers:
 * <pre>
 *   0.000 in   the wall (the plate's back edge stands 0.162 in off it)
 *   2.634 in   center line of the POLLEN stack (the wheel never goes closer than this)
 *   2.833 in   PRESS STOP LINE: the front tip of the pressing wheel (4.034 - FLOWER_PRESS_DEPTH_IN of 1.201)
 *   4.034 in   field-side edge of the bottom POLLEN (so the wheel overlaps it by 1.201 in)
 *   5.033 in   field-side edge of the plate (4.871 in deep, 5.95 in along the wall)
 *   5.533 in   the closest the chassis front may be (plate + FLOWER_CHASSIS_CLEARANCE_IN of 0.5 in)
 * </pre>
 * The chassis front stands 2.7 in (the wheel reach) behind the wheel tips, i.e. at 2.833 + 2.7 = 5.533 in: exactly the
 * closest it may be. If the real wheels reach less than that, the validator says how many inches are missing.
 *
 * <p>The three poses of a press-in (the position is the pose point, the point the robot spins about):
 * <ol>
 *   <li>{@link #lineUp}: where the straight final approach starts. Written by the author (it is the INTAKE step's
 *       robot pose), or derived by {@link #idealLineUp}.</li>
 *   <li>{@link #press}: derived. Facing the wall squarely, with the middle of the pressing wheel on the FLOWER's center
 *       line and the wheel's front tip on the press stop line ({@link AllianceField#flowerPressStopLineIn()}: the POLLEN's
 *       field-side edge, 4.034 in from the wall, minus {@code Auto.FLOWER_PRESS_DEPTH_IN}).</li>
 *   <li>{@link #retreat}: derived. The press pose backed straight out by the step's retreat length. The INTAKE step
 *       ends here.</li>
 * </ol>
 *
 * <p><b>The invariants the validator enforces</b> (rules I1 to I4 of the press-in; walls, plate and HIVE are checked by
 * {@link RouteValidator}):
 * <ul>
 *   <li><b>I1</b> Nothing of the robot ever goes into or past a wall.</li>
 *   <li><b>I2</b> The CHASSIS (the footprint minus the front {@code Robot.INTAKE_REACH_IN} where only the wheels stick
 *       out, {@link Footprint#chassis(Pose2)}) never overlaps a FLOWER plate (it keeps {@code Auto.FLOWER_CHASSIS_CLEARANCE_IN}
 *       off the plate being pressed) or the HIVE keep-out, in any step, including the FLOWER's own INTAKE step.</li>
 *   <li><b>I3</b> Only the ONE pressing wheel disc ({@link Footprint#wheelDisc}) may enter the plate of the FLOWER being
 *       emptied, only during that FLOWER's INTAKE step, and only up to the press stop line. The other wheel stays out of
 *       the plate.</li>
 *   <li><b>I4</b> The approach is perpendicular to the wall (heading within {@code Auto.FLOWER_HEADING_TOLERANCE_DEG}), the
 *       middle of the pressing wheel is within {@code Auto.FLOWER_ALIGN_TOLERANCE_IN} of the FLOWER's center line, the
 *       final approach is a straight run of at least {@code Auto.FLOWER_APPROACH_LENGTH_IN} at the slow
 *       {@code Auto.FLOWER_APPROACH_SPEED_FRACTION}, then comes a dwell with the intake on, then a straight retreat of
 *       {@code Auto.FLOWER_RETREAT_LENGTH_IN}. The chassis stands at least {@code Auto.FLOWER_CHASSIS_CLEARANCE_IN}
 *       outside the plate, so the wheel must reach the rest of the way ({@link #requiredReachIn}).</li>
 * </ul>
 */
public final class FlowerPress {
    /** Index of the FLOWER (position in {@code Field.FLOWER_CENTERS_IN}). */
    public final int flower;
    /** The flat intake wheel that presses the POLLEN (the robot's own left or right). */
    public final IntakeWheel wheel;
    /** The pose the straight final approach starts from (the INTAKE step's own robot pose). */
    public final Pose2 lineUp;
    /** The pose with the wheel pressed into the POLLEN: derived from the FLOWER and the wheel. */
    public final Pose2 press;
    /** The pose after backing straight out: derived. The INTAKE step ends here. */
    public final Pose2 retreat;
    /** The straight, slow final approach {@code lineUp -> press}. */
    public final DrivePath approach;
    /** The straight, slow retreat {@code press -> retreat}. */
    public final DrivePath retreatPath;
    /** How far the line-up heading is from facing the wall squarely, in radians (0 to PI). */
    public final double headingErrorRad;
    /**
     * How far the MIDDLE OF THE PRESSING WHEEL is from the FLOWER's center line at the line-up pose, along the wall, in
     * inches (the sign only says which way along the wall). Near 0 when the robot is lined up.
     */
    public final double lateralIn;
    /** How far (inches) the line-up pose is from the press pose measured straight out from the wall: the length of the final approach. */
    public final double approachAxialIn;
    /** How far the robot backs out (inches), from the step. */
    public final double retreatLengthIn;
    /** The press stop line: distance of the wheel's front tip from the wall at the press, in inches. */
    public final double stopLineIn;
    /** How far the wheels have to stick out in front of the chassis (inches) for the press to work. */
    public final double requiredReachIn;

    private FlowerPress(int flower, IntakeWheel wheel, Pose2 lineUp, Pose2 press, Pose2 retreat, double headingErrorRad,
                        double lateralIn, double approachAxialIn, double retreatLengthIn, double stopLineIn,
                        double requiredReachIn) {
        this.flower = flower;
        this.wheel = wheel;
        this.lineUp = lineUp;
        this.press = press;
        this.retreat = retreat;
        this.approach = DrivePath.straight(lineUp, press);
        this.retreatPath = DrivePath.straight(press, retreat);
        this.headingErrorRad = headingErrorRad;
        this.lateralIn = lateralIn;
        this.approachAxialIn = approachAxialIn;
        this.retreatLengthIn = retreatLengthIn;
        this.stopLineIn = stopLineIn;
        this.requiredReachIn = requiredReachIn;
    }

    /**
     * The press-in an INTAKE step makes, or null if the step does not take from a FLOWER or its target point is not
     * near any FLOWER (the validator warns about that separately).
     */
    public static FlowerPress of(Step.Intake step, AllianceField field) {
        if (step.source() != IntakeSource.FLOWER) {
            return null;
        }
        int f = field.nearestFlower(step.target());
        if (f < 0) {
            return null;
        }
        IntakeWheel wheel = step.wheel();
        Pose2 lineUp = step.robotPose();
        double stop = field.flowerPressStopLineIn();
        Pose2 press = poseWithWheelOnCenterLine(field, f, wheel, stop);
        Point2 n = field.flowerInward(f);
        Pose2 retreat = new Pose2(press.x + n.x * step.retreatLengthIn(), press.y + n.y * step.retreatLengthIn(), press.heading);
        Point2 wheelAtLineUp = Footprint.wheelCenter(lineUp, wheel);
        double axial = field.flowerWallDistance(f, new Point2(lineUp.x, lineUp.y))
                - field.flowerWallDistance(f, new Point2(press.x, press.y));
        return new FlowerPress(f, wheel, lineUp, press, retreat, Math.abs(Angles.error(lineUp.heading, press.heading)),
                field.flowerLateral(f, wheelAtLineUp), axial, step.retreatLengthIn(), stop, field.flowerRequiredReachIn());
    }

    /**
     * The ideal line-up pose for FLOWER {@code flower} and the wheel that will press: facing the wall squarely, the middle
     * of that wheel on the FLOWER's center line, the wheel tip {@code Auto.FLOWER_APPROACH_LENGTH_IN} farther out than at the
     * press. The route builder uses it when the author gives only the FLOWER.
     */
    public static Pose2 idealLineUp(AllianceField field, int flower, IntakeWheel wheel) {
        return poseWithWheelOnCenterLine(field, flower, wheel,
                field.flowerPressStopLineIn() + BioBuzzConstants.Auto.FLOWER_APPROACH_LENGTH_IN);
    }

    /**
     * The wheel whose middle is nearer the FLOWER's center line when the robot stands at {@code pose}: the natural choice
     * when an author gives a line-up pose but not the wheel. (An exact tie goes to the LEFT wheel.)
     */
    public static IntakeWheel nearerWheel(AllianceField field, int flower, Pose2 pose) {
        double left = Math.abs(field.flowerLateral(flower, Footprint.wheelCenter(pose, IntakeWheel.LEFT)));
        double right = Math.abs(field.flowerLateral(flower, Footprint.wheelCenter(pose, IntakeWheel.RIGHT)));
        return right < left ? IntakeWheel.RIGHT : IntakeWheel.LEFT;
    }

    /**
     * The pose that faces FLOWER {@code flower}'s wall squarely, with the front tip of the wheels {@code frontTipFromWallIn}
     * inches from the wall and the middle of {@code wheel} exactly on the FLOWER's center line.
     */
    static Pose2 poseWithWheelOnCenterLine(AllianceField field, int flower, IntakeWheel wheel, double frontTipFromWallIn) {
        Point2 n = field.flowerInward(flower);
        Point2 wall = field.flowerWallPoint(flower);
        double heading = field.flowerFacingHeading(flower);
        double depth = frontTipFromWallIn + Footprint.frontExtentIn();           // wall to the pose point
        Pose2 centered = new Pose2(wall.x + n.x * depth, wall.y + n.y * depth, heading);
        // slide the robot along the wall until the middle of its wheel is on the center line (the wall's direction is (-n.y, n.x))
        double off = field.flowerLateral(flower, Footprint.wheelCenter(centered, wheel));
        return new Pose2(centered.x + n.y * off, centered.y - n.x * off, heading);
    }

    /** The heading error in degrees (0 to 180). */
    public double headingErrorDeg() {
        return Math.toDegrees(headingErrorRad);
    }

    /** How many inches short the wheels are of what the press needs (0 if {@code Robot.INTAKE_REACH_IN} is enough). */
    public double reachMissingIn() {
        return reachMissingIn(BioBuzzConstants.Robot.INTAKE_REACH_IN);
    }

    /** As {@link #reachMissingIn()} for wheels that stick out {@code reachIn} inches in front of the chassis. */
    public double reachMissingIn(double reachIn) {
        return Math.max(0.0, requiredReachIn - reachIn);
    }
}
