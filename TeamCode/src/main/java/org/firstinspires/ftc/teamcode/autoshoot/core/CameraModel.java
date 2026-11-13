package org.firstinspires.ftc.teamcode.autoshoot.core;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Where the camera sits and how it is turned, and the maths to move between three frames:
 *
 * <ul>
 *   <li><b>camera space</b> (right, down, forward): +right is to the right of the image, +down is
 *       toward the bottom of the image, +forward points out of the lens along the optical axis;</li>
 *   <li><b>robot frame</b> (forward, left, up): forward is the direction the intake faces, left is 90
 *       degrees counter-clockwise from it, up is the height above the FLOOR for positions and the
 *       upward direction for directions.</li>
 * </ul>
 *
 * <p>The camera is described by its lens position in the robot frame, a yaw (0 = looks out the front,
 * 180 = looks out the back, counter-clockwise positive) and a pitch (degrees above level). Roll is
 * assumed to be zero: mount the camera upright. All distances are inches.
 *
 * <p>Why it exists: the Limelight reports where a tag is relative to the CAMERA ("2 ft in front of the lens,
 * a bit to the right"). The robot needs to know where the tag is relative to the ROBOT. Our camera sits behind
 * the robot's center, looks out the back, and is tilted up, so each of those facts changes the answer. This
 * class holds the mounting numbers (from {@code BioBuzzConstants.Vision}) and does that conversion once, in one
 * place. The Limelight's own "camera pose in robot space" setting is NOT used; these constants are.
 *
 * <p>Data flow: {@code LimelightVision} calls {@link #cameraToRobot} and {@link #azimuthOf} for every tag it
 * reads; the unit tests also use {@link #txTyOf} to simulate a camera. If the robot's idea of where the HIVE is
 * seems consistently off, wrong mounting numbers here are the first suspect (docs/START_HERE.md, "Set up the Limelight camera").
 */
public final class CameraModel {
    // The lens position in the robot frame (inches from the robot's rotation center; height above the floor).
    private final double forwardIn;
    private final double leftIn;
    private final double heightIn;
    // Cosine and sine of the yaw and pitch, worked out once so the per-tag maths is cheap.
    private final double cosYaw;
    private final double sinYaw;
    private final double cosPitch;
    private final double sinPitch;

    /**
     * @param forwardIn lens position, inches forward of the robot's rotation center (negative = behind it)
     * @param leftIn    lens position, inches left of the robot's center line (negative = right)
     * @param heightIn  lens height above the floor, inches
     * @param yawDeg    direction the camera looks, degrees counter-clockwise from robot forward (180 = out the back)
     * @param pitchUpDeg how far the camera is tilted up from level, degrees
     */
    public CameraModel(double forwardIn, double leftIn, double heightIn, double yawDeg, double pitchUpDeg) {
        this.forwardIn = forwardIn;
        this.leftIn = leftIn;
        this.heightIn = heightIn;
        double yaw = Math.toRadians(yawDeg);
        double pitch = Math.toRadians(pitchUpDeg);
        this.cosYaw = Math.cos(yaw);
        this.sinYaw = Math.sin(yaw);
        this.cosPitch = Math.cos(pitch);
        this.sinPitch = Math.sin(pitch);
    }

    /** The camera as mounted on the real robot: the {@code CAMERA_*} numbers in {@code BioBuzzConstants.Vision}. */
    public static CameraModel fromConstants() {
        return new CameraModel(BioBuzzConstants.Vision.CAMERA_FORWARD_IN, BioBuzzConstants.Vision.CAMERA_LEFT_IN,
                BioBuzzConstants.Vision.CAMERA_HEIGHT_IN, BioBuzzConstants.Vision.CAMERA_YAW_DEG,
                BioBuzzConstants.Vision.CAMERA_PITCH_UP_DEG);
    }

    /** Rotates a camera-space vector into the robot frame, without adding the lens position. */
    private double[] directionToRobot(double right, double down, double forward) {
        double up = -down;
        // Step 1: apply the pitch. Tilting the camera up swings its "forward" axis upward, so a ray straight
        // along the lens axis ends up pointing UP by the pitch angle when measured against a level frame.
        double levelForward = forward * cosPitch - up * sinPitch;
        double levelUp = forward * sinPitch + up * cosPitch;
        // Step 2: apply the yaw (a turn about the vertical axis). With yaw = 180 (out the back) this makes
        // image-right come out as the robot's LEFT and the lens axis as robot-backward.
        double f = levelForward * cosYaw + right * sinYaw;
        double l = levelForward * sinYaw - right * cosYaw;
        return new double[]{f, l, levelUp};
    }

    /** A point in camera space to the robot frame: {forward, left, height above the floor}. */
    public double[] cameraToRobot(double right, double down, double forward) {
        double[] d = directionToRobot(right, down, forward);
        return new double[]{d[0] + forwardIn, d[1] + leftIn, d[2] + heightIn};
    }

    /** The inverse of {@link #cameraToRobot}: a robot-frame point to camera space {right, down, forward}. */
    public double[] robotToCamera(double forward, double left, double height) {
        // The steps of cameraToRobot run backwards: subtract the lens position, then undo the yaw, then the pitch.
        double f = forward - forwardIn;
        double l = left - leftIn;
        double levelUp = height - heightIn;
        double levelForward = f * cosYaw + l * sinYaw;
        double right = f * sinYaw - l * cosYaw;
        double z = levelForward * cosPitch + levelUp * sinPitch;
        double up = -levelForward * sinPitch + levelUp * cosPitch;
        return new double[]{right, -up, z};
    }

    /**
     * The direction, in the robot's floor plan, of the ray through a pixel that the Limelight reports as
     * tx (degrees, positive = right) and ty (degrees, positive = up). Returns radians counter-clockwise
     * from the robot's forward direction. The camera's pitch is accounted for.
     *
     * <p>"tx" and "ty" are how far the tag is from the center of the image (the crosshair): tx sideways, ty
     * up/down, both in degrees. They need no distance, so they work even when the 3D position is missing. Because
     * this camera looks out the back, a tag straight behind the robot comes out near +/-PI (180 degrees), not 0.
     */
    public double azimuthOf(double txRightDeg, double tyUpDeg) {
        // Turn the two angles into a ray in camera space (the "1.0" is one unit straight out of the lens),
        // then rotate it into the robot frame and keep only its direction on the floor.
        double right = Math.tan(Math.toRadians(txRightDeg));
        double up = Math.tan(Math.toRadians(tyUpDeg));
        double[] d = directionToRobot(right, -up, 1.0);
        return Math.atan2(d[1], d[0]);
    }

    /**
     * The {tx right+, ty up+} in degrees at which a robot-frame point appears (the inverse of the above, used by
     * the unit tests to simulate what the camera would report).
     */
    public double[] txTyOf(double forward, double left, double height) {
        double[] c = robotToCamera(forward, left, height);
        return new double[]{Math.toDegrees(Math.atan2(c[0], c[2])), Math.toDegrees(Math.atan2(-c[1], c[2]))};
    }
}
