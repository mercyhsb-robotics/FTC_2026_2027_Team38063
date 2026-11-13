package org.firstinspires.ftc.teamcode.autoshoot.routes;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.CellSpec;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;

/**
 * Where a shot leaves the robot and how it relates to a HIVE cell. The shooter fires out the BACK, so
 * the launch point sits at a negative forward offset from the rotation center
 * ({@link BioBuzzConstants.Robot#LAUNCH_POINT_FORWARD_IN}) and the robot must stand with its back
 * toward the cell.
 */
public final class ShotGeometry {
    private ShotGeometry() {
    }

    /** Field position where the POLLEN leaves the shooter when the robot stands at {@code pose}. */
    public static Point2 launchPoint(Pose2 pose) {
        return new Point2(pose.fieldX(BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN),
                pose.fieldY(BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN,
                        BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN));
    }

    /** The floor point to aim at: the center of the cell opening when the cell is UP. */
    public static Point2 aimPoint(CellSpec cell) {
        return new Point2(cell.originX, cell.originY);
    }

    /** Distance from the launch point to the aim point of the cell, in the floor plane. */
    public static double standoff(Pose2 pose, CellSpec cell) {
        return launchPoint(pose).distanceTo(aimPoint(cell));
    }

    /**
     * How far the robot's BACK is turned away from the cell, in radians (0 = the back points straight at
     * the aim point, measured from the launch point).
     */
    public static double aimError(Pose2 pose, CellSpec cell) {
        Point2 l = launchPoint(pose);
        double bearing = Math.atan2(cell.originY - l.y, cell.originX - l.x);
        double backDirection = Angles.opposite(pose.heading);
        return Math.abs(Angles.error(backDirection, bearing));
    }

    /** True if the launch point lies on the side of the aim point that the cell opening faces (outside the HIVE). */
    public static boolean onOpeningSide(Pose2 pose, CellSpec cell) {
        Point2 l = launchPoint(pose);
        return (l.y - cell.originY) * cell.side.ySign() >= 0.0;
    }

    /**
     * The nominal shooting pose in front of a cell: on the HIVE axis, the launch point
     * {@link BioBuzzConstants.Auto#SHOOT_STANDOFF_NOMINAL_IN} out from the aim point, back toward the
     * cell. This is what the diagrams draw when a SHOOT leaves its pose to the runtime solver.
     */
    public static Pose2 nominalPose(CellSpec cell) {
        double heading = cell.facingHeading;      // the cell faces away from the HIVE; the robot faces the same way
        double exitX = cell.originX;
        double exitY = cell.originY + cell.side.ySign() * BioBuzzConstants.Auto.SHOOT_STANDOFF_NOMINAL_IN;
        double f = BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN;
        double l = BioBuzzConstants.Robot.LAUNCH_POINT_LEFT_IN;
        double cx = exitX - (f * Math.cos(heading) - l * Math.sin(heading));
        double cy = exitY - (f * Math.sin(heading) + l * Math.cos(heading));
        return new Pose2(cx, cy, heading);
    }
}
