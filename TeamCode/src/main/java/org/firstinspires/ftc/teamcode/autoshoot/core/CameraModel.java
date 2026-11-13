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
 */
public final class CameraModel {
    private final double forwardIn;
    private final double leftIn;
    private final double heightIn;
    private final double cosYaw;
    private final double sinYaw;
    private final double cosPitch;
    private final double sinPitch;

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

    public static CameraModel fromConstants() {
        return new CameraModel(BioBuzzConstants.Vision.CAMERA_FORWARD_IN, BioBuzzConstants.Vision.CAMERA_LEFT_IN,
                BioBuzzConstants.Vision.CAMERA_HEIGHT_IN, BioBuzzConstants.Vision.CAMERA_YAW_DEG,
                BioBuzzConstants.Vision.CAMERA_PITCH_UP_DEG);
    }

    /** Rotates a camera-space vector into the robot frame, without adding the lens position. */
    private double[] directionToRobot(double right, double down, double forward) {
        double up = -down;
        double levelForward = forward * cosPitch - up * sinPitch;
        double levelUp = forward * sinPitch + up * cosPitch;
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
     */
    public double azimuthOf(double txRightDeg, double tyUpDeg) {
        double right = Math.tan(Math.toRadians(txRightDeg));
        double up = Math.tan(Math.toRadians(tyUpDeg));
        double[] d = directionToRobot(right, -up, 1.0);
        return Math.atan2(d[1], d[0]);
    }

    /** The {tx right+, ty up+} in degrees at which a robot-frame point appears (the inverse of the above for simulation). */
    public double[] txTyOf(double forward, double left, double height) {
        double[] c = robotToCamera(forward, left, height);
        return new double[]{Math.toDegrees(Math.atan2(c[0], c[2])), Math.toDegrees(Math.atan2(-c[1], c[2]))};
    }
}
