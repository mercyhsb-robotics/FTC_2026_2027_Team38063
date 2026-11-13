package org.firstinspires.ftc.teamcode.autoshoot.routes;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.core.Rect;

/**
 * One of the four perimeter walls of the field, named the way the team talks about them: the RED wall is the one at x = 0 (the left wall of the
 * diagrams), the BLUE wall at x = field size, the AUDIENCE wall at y = 0 (the bottom of the diagrams) and the REAR wall at y = field size.
 *
 * <p>A robot may only be set up touching a wall of its OWN half of the field (rule G304.C), so each alliance has three usable walls: red uses the
 * red wall and the own halves of the audience and rear walls (x up to the field's middle), blue the blue wall and the other halves.
 *
 * <p>"Along the wall" means the coordinate that runs parallel to it: x for the audience and rear walls, y for the red and blue walls. It is the one
 * number, together with the heading, that decides where on a wall a robot stands once it touches the wall.
 */
public enum Wall {
    /** The red wall, x = 0. */
    RED("red wall", false),
    /** The blue wall, x = field size. */
    BLUE("blue wall", false),
    /** The audience wall, y = 0. */
    AUDIENCE("audience wall", true),
    /** The rear wall, y = field size. */
    REAR("rear wall", true);

    private final String plainName;
    private final boolean runsAlongX;

    Wall(String plainName, boolean runsAlongX) {
        this.plainName = plainName;
        this.runsAlongX = runsAlongX;
    }

    /** The name in plain words, for example {@code "audience wall"}. */
    public String plainName() {
        return plainName;
    }

    /** True if the wall runs along the x axis (the audience and rear walls); false if along y (the red and blue walls). */
    public boolean runsAlongX() {
        return runsAlongX;
    }

    /**
     * The direction, in radians, that points from the inside of the field toward this wall: pi for the red wall, 0 for the blue wall, -pi/2 for the
     * audience wall and pi/2 for the rear wall. A robot whose FRONT is on this wall faces this way.
     */
    public double outwardRad() {
        switch (this) {
            case RED:
                return Math.PI;
            case BLUE:
                return 0.0;
            case AUDIENCE:
                return -Math.PI / 2.0;
            default:
                return Math.PI / 2.0;
        }
    }

    /**
     * How far the box {@code bounds} is from this wall, in inches: positive = that many inches of floor between them, 0 = touching, negative = the box
     * pokes through the wall by that much.
     */
    public double gapIn(Rect bounds, double fieldSize) {
        switch (this) {
            case RED:
                return bounds.minX;
            case BLUE:
                return fieldSize - bounds.maxX;
            case AUDIENCE:
                return bounds.minY;
            default:
                return fieldSize - bounds.maxY;
        }
    }

    /** The coordinate of {@code p} along this wall: x for the audience and rear walls, y for the red and blue walls. */
    public double along(Pose2 p) {
        return runsAlongX ? p.x : p.y;
    }

    /**
     * {@code p} moved straight toward or away from this wall (never along it) by exactly as much as it takes for the box {@code boundsAtP}, the
     * footprint of the robot at {@code p}, to touch the wall: the way a robot that is pushed against a wall by hand ends up. The heading and the
     * coordinate along the wall do not change.
     */
    public Pose2 seated(Pose2 p, Rect boundsAtP, double fieldSize) {
        double move = gapIn(boundsAtP, fieldSize);       // how far the footprint has to travel toward the wall
        switch (this) {
            case RED:
                return new Pose2(p.x - move, p.y, p.heading);
            case BLUE:
                return new Pose2(p.x + move, p.y, p.heading);
            case AUDIENCE:
                return new Pose2(p.x, p.y - move, p.heading);
            default:
                return new Pose2(p.x, p.y + move, p.heading);
        }
    }

    /** The walls of this alliance's own half of the field, the only ones a robot may be set up against: red {RED, AUDIENCE, REAR}, blue {BLUE, AUDIENCE, REAR}. */
    public static List<Wall> own(Alliance alliance) {
        List<Wall> out = new ArrayList<Wall>();
        out.add(alliance == Alliance.RED ? RED : BLUE);
        out.add(AUDIENCE);
        out.add(REAR);
        return Collections.unmodifiableList(out);
    }
}
