package org.firstinspires.ftc.teamcode.autoshoot.core;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * Every named device the robot code expects to find in the robot configuration (the list you edit on the Driver
 * Station under Configure Robot), with a plain-language description. The names themselves come from
 * {@code BioBuzzConstants.Hardware}, so this list can never disagree with the rest of the code.
 *
 * <p>Why it exists: the "Robot Check" OpMode asks the hardware map "is there a device with this name?" for each
 * entry and passes the answers to {@link RobotCheck}, which turns a missing one into advice a first-year student can
 * follow. This class is plain Java (no FTC SDK), so the logic can be tested on a PC.
 *
 * <p>Words used here. A <b>robot configuration</b> is the list of motors, servos and sensors, each with a name, that
 * you save on the Driver Station. The code finds every device by that exact name, so a typo or a capital letter in
 * the wrong place means "device not found". A <b>continuous-rotation servo</b> is a servo that spins like a motor
 * instead of turning to an angle.
 */
public enum RobotCheckDevice {
    FRONT_LEFT_MOTOR(BioBuzzConstants.Hardware.FRONT_LEFT_MOTOR, "FRONT_LEFT_MOTOR", Kind.MOTOR,
            "the front-left wheel motor", true),
    BACK_LEFT_MOTOR(BioBuzzConstants.Hardware.BACK_LEFT_MOTOR, "BACK_LEFT_MOTOR", Kind.MOTOR,
            "the back-left wheel motor", true),
    FRONT_RIGHT_MOTOR(BioBuzzConstants.Hardware.FRONT_RIGHT_MOTOR, "FRONT_RIGHT_MOTOR", Kind.MOTOR,
            "the front-right wheel motor", true),
    BACK_RIGHT_MOTOR(BioBuzzConstants.Hardware.BACK_RIGHT_MOTOR, "BACK_RIGHT_MOTOR", Kind.MOTOR,
            "the back-right wheel motor", true),
    PINPOINT(BioBuzzConstants.Hardware.PINPOINT, "PINPOINT", Kind.ODOMETRY_COMPUTER,
            "the Pinpoint odometry computer (it tracks where the robot is)", true),
    IMU(BioBuzzConstants.Hardware.IMU, "IMU", Kind.IMU,
            "the Control Hub's built-in gyro (only the backup drive uses it)", false),
    FLYWHEEL(BioBuzzConstants.Hardware.FLYWHEEL, "FLYWHEEL", Kind.MOTOR,
            "the flywheel motor at the back", true),
    FEEDER(BioBuzzConstants.Hardware.FEEDER, "FEEDER", Kind.CONTINUOUS_SERVO,
            "the feeder servo at the back", true),
    INTAKE_MOTOR(BioBuzzConstants.Hardware.INTAKE_MOTOR, "INTAKE_MOTOR", Kind.MOTOR,
            "the front intake roller motor", true),
    LEFT_INTAKE_SERVO(BioBuzzConstants.Hardware.LEFT_INTAKE_SERVO, "LEFT_INTAKE_SERVO", Kind.CONTINUOUS_SERVO,
            "the left corner intake servo at the front", true),
    RIGHT_INTAKE_SERVO(BioBuzzConstants.Hardware.RIGHT_INTAKE_SERVO, "RIGHT_INTAKE_SERVO", Kind.CONTINUOUS_SERVO,
            "the right corner intake servo at the front", true),
    LIMELIGHT(BioBuzzConstants.Hardware.LIMELIGHT, "LIMELIGHT", Kind.CAMERA,
            "the Limelight camera (without it the robot cannot confirm which HIVE cell is up)", false);

    /** What sort of device it is: the hardware adapter uses it to pick the right SDK type, and the advice uses its words. */
    public enum Kind {
        MOTOR("motor"),
        CONTINUOUS_SERVO("continuous-rotation servo"),
        ODOMETRY_COMPUTER("goBILDA Pinpoint odometry computer"),
        IMU("IMU"),
        CAMERA("Limelight 3A camera");

        /** The kind in plain words, for example "continuous-rotation servo". */
        public final String words;

        Kind(String words) {
            this.words = words;
        }
    }

    /** The exact name in the robot configuration (the value of the BioBuzzConstants.Hardware constant). */
    public final String configName;
    /** The name of that constant in BioBuzzConstants.Hardware, so advice can say where to change the name. */
    public final String constantName;
    public final Kind kind;
    /** What the device is for, in plain words ("the feeder servo at the back"). */
    public final String description;
    /** True if the robot cannot work without it (a missing one is a FAIL); false if a missing one is only a WARN. */
    public final boolean required;

    RobotCheckDevice(String configName, String constantName, Kind kind, String description, boolean required) {
        this.configName = configName;
        this.constantName = constantName;
        this.kind = kind;
        this.description = description;
        this.required = required;
    }

    /** The four wheel motors in the order the check tests them: front-left, back-left, front-right, back-right. */
    public static final RobotCheckDevice[] WHEELS = {FRONT_LEFT_MOTOR, BACK_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_RIGHT_MOTOR};
}
