package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds the awkward places a robot can be left standing in TeleOp: pressed against one of the four field walls, or in
 * a corner, facing any way. Tests use them to ask "can the planner still drive out of here without clipping the wall?".
 *
 * <p>Why it needs maths: the robot is a square, and a square that faces a diagonal way needs more room than one that
 * faces along an axis (a 17.8 in square turned 45 degrees is about 25.2 in wide). So "touching the wall" puts the
 * rotation center a different distance from the wall at every heading. {@link ShotGeometry#footprintExtentX} and
 * {@code footprintExtentY} give the width and height of the box that just holds the turned square; half of that is the
 * distance from the center to the wall when they touch.
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
     * The robot against a wall: its rotation center is placed so the turned square is {@code gapIn} from the wall.
     *
     * @param size       the robot's side, inches (it is a square)
     * @param wall       0 audience, 1 rear, 2 red, 3 blue
     * @param along      where along the wall, inches (x for the audience and rear walls, y for the side walls)
     * @param headingDeg which way the robot faces, degrees (0 = +x, counter-clockwise positive)
     */
    static Pose2 againstWall(FieldModel field, double size, int wall, double along, double headingDeg, double gapIn) {
        double h = Math.toRadians(headingDeg);
        double halfX = 0.5 * ShotGeometry.footprintExtentX(size, size, h);
        double halfY = 0.5 * ShotGeometry.footprintExtentY(size, size, h);
        double far = field.size();
        switch (wall) {
            case 0:  return new Pose2(along, halfY + gapIn, h);
            case 1:  return new Pose2(along, far - halfY - gapIn, h);
            case 2:  return new Pose2(halfX + gapIn, along, h);
            default: return new Pose2(far - halfX - gapIn, along, h);
        }
    }

    /** The robot in a corner, {@code gapIn} from both walls. */
    static Pose2 inCorner(FieldModel field, double size, int corner, double headingDeg, double gapIn) {
        double h = Math.toRadians(headingDeg);
        double halfX = 0.5 * ShotGeometry.footprintExtentX(size, size, h);
        double halfY = 0.5 * ShotGeometry.footprintExtentY(size, size, h);
        double far = field.size();
        double x = (corner & 1) == 0 ? halfX + gapIn : far - halfX - gapIn;
        double y = (corner & 2) == 0 ? halfY + gapIn : far - halfY - gapIn;
        return new Pose2(x, y, h);
    }

    /**
     * Every start next to a wall or in a corner: 12 headings (every 30 degrees), touching the wall and 1.5 in off it,
     * three places along each wall (30, 72 and 114 in) and the four corners. 384 poses.
     */
    static List<Pose2> all(FieldModel field, double size) {
        List<Pose2> out = new ArrayList<Pose2>();
        double[] gaps = {0.0, 1.5};
        double[] places = {30.0, 72.0, 114.0};
        for (double gap : gaps) {
            for (int deg = -180; deg < 180; deg += 30) {
                for (int wall = 0; wall < 4; wall++) {
                    for (double along : places) {
                        out.add(againstWall(field, size, wall, along, deg, gap));
                    }
                }
                for (int corner = 0; corner < 4; corner++) {
                    out.add(inCorner(field, size, corner, deg, gap));
                }
            }
        }
        return out;
    }
}
