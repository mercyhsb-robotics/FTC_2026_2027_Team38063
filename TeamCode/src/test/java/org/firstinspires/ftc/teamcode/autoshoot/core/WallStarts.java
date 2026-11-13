package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds the awkward places a robot can be left standing in TeleOp: pressed against one of the four field walls, or in
 * a corner, facing any way. Tests use them to ask "can the planner still drive out of here without clipping the wall?".
 *
 * <p>Why it needs maths: the robot is a rectangle that is NOT centered on the point it spins about (its center of
 * rotation, the pose), and a rectangle that faces a diagonal way needs more room than one that faces along an axis. So
 * "touching the wall" puts the center of rotation a different distance from the wall at every heading AND for every wall:
 * facing the audience wall front-first it is 9.77 in from it, facing away from it 8.03 in, sideways 8.90 in. The box that
 * just holds the turned rectangle ({@link Footprint#bounds()}, built for a robot at the origin) says how far the center
 * must be from each wall for the rectangle to touch it.
 *
 * <p>Walls are numbered 0 = audience wall (y = 0), 1 = rear wall (y = 144), 2 = red wall (x = 0), 3 = blue wall
 * (x = 144). Corners are numbered by bits: bit 0 set = the x = 144 side, bit 1 set = the y = 144 side, so 0 is the
 * (0, 0) corner where the audience wall meets the red wall. A <b>gap</b> is the empty space, in inches, left between the
 * robot and the wall (0 = touching it).
 */
final class WallStarts {
    private WallStarts() {
    }

    /**
     * The robot against a wall: its center of rotation is placed so the turned rectangle is {@code gapIn} from the wall.
     *
     * @param robot      where the robot's edges lie around its center of rotation
     * @param wall       0 audience, 1 rear, 2 red, 3 blue
     * @param along      where along the wall, inches (x for the audience and rear walls, y for the side walls)
     * @param headingDeg which way the robot faces, degrees (0 = +x, counter-clockwise positive)
     */
    static Pose2 againstWall(FieldModel field, RobotShape robot, int wall, double along, double headingDeg, double gapIn) {
        double h = Math.toRadians(headingDeg);
        Rect box = Footprint.of(new Pose2(0.0, 0.0, h), robot).bounds();
        double far = field.size();
        switch (wall) {
            case 0:  return new Pose2(along, -box.minY + gapIn, h);
            case 1:  return new Pose2(along, far - box.maxY - gapIn, h);
            case 2:  return new Pose2(-box.minX + gapIn, along, h);
            default: return new Pose2(far - box.maxX - gapIn, along, h);
        }
    }

    /** The robot in a corner, {@code gapIn} from both walls. */
    static Pose2 inCorner(FieldModel field, RobotShape robot, int corner, double headingDeg, double gapIn) {
        double h = Math.toRadians(headingDeg);
        Rect box = Footprint.of(new Pose2(0.0, 0.0, h), robot).bounds();
        double far = field.size();
        double x = (corner & 1) == 0 ? -box.minX + gapIn : far - box.maxX - gapIn;
        double y = (corner & 2) == 0 ? -box.minY + gapIn : far - box.maxY - gapIn;
        return new Pose2(x, y, h);
    }

    /**
     * Every start next to a wall or in a corner: 12 headings (every 30 degrees), touching the wall and 1.5 in off it,
     * three places along each wall (30, 72 and 114 in) and the four corners. 384 poses.
     */
    static List<Pose2> all(FieldModel field, RobotShape robot) {
        List<Pose2> out = new ArrayList<Pose2>();
        double[] gaps = {0.0, 1.5};
        double[] places = {30.0, 72.0, 114.0};
        for (double gap : gaps) {
            for (int deg = -180; deg < 180; deg += 30) {
                for (int wall = 0; wall < 4; wall++) {
                    for (double along : places) {
                        out.add(againstWall(field, robot, wall, along, deg, gap));
                    }
                }
                for (int corner = 0; corner < 4; corner++) {
                    out.add(inCorner(field, robot, corner, deg, gap));
                }
            }
        }
        return out;
    }
}
