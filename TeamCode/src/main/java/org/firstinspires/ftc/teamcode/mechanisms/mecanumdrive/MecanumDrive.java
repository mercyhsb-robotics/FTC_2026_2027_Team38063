package org.firstinspires.ftc.teamcode.mechanisms.mecanumdrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The LEGACY mecanum drive: it turns three numbers (forward, strafe, rotate) straight into power for the four wheel
 * motors. It has no odometry and no path following.
 *
 * <p>The normal drive is Pedro Pathing through {@code PedroDrive}. This class is only the fallback that
 * {@code BioBuzzRobot} uses if Pedro or the Pinpoint fails to start (automatic shooting is then off), and the
 * drive that the {@code MecanumDriveTest} diagnostic tests.
 *
 * <p><b>Words used here.</b> A <b>mecanum</b> wheel has angled rollers, so a robot with four of them can move in any
 * direction without turning. Moving straight sideways is called <b>strafing</b>. Each wheel's power is a mix of the
 * forward, strafe and rotate requests (see {@link #drive}). The <b>IMU</b> (inertial measurement unit) is the
 * Control Hub's built-in gyroscope and accelerometer.
 *
 * <p>The four motor names come from {@code BioBuzzConstants.Hardware} and must match the robot configuration on the
 * Driver Station.
 */
public class MecanumDrive {
    // The four wheel motors, named by where they are on the robot (front/back, left/right).
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    // The Control Hub's built-in IMU. It is set up in init() but never read: this drive is robot-centric
    // (the sticks mean "forward / left / right relative to the robot", not relative to the field).
    private IMU imu;

    /** Finds the four motors and the IMU in the robot configuration and sets them up. Call once, from init(). */
    public void init (HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, BioBuzzConstants.Hardware.FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotor.class, BioBuzzConstants.Hardware.BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotor.class, BioBuzzConstants.Hardware.FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.get(DcMotor.class, BioBuzzConstants.Hardware.BACK_RIGHT_MOTOR);

        // GoBilda: the left side is FORWARD and the right side REVERSE (the two sides are mirror images, so they
        // need opposite directions). With these settings a NEGATIVE power drives the robot forward (see drive()
        // below).
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // RUN_USING_ENCODER: the motor controller uses each wheel's encoder to regulate its speed (power is a
        // fraction of top speed).
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // BRAKE: when the power is zero the motors resist turning, so the robot stops quickly instead of coasting.
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, BioBuzzConstants.Hardware.IMU);

        // Tells the IMU how the Control Hub is mounted (which way its logo and its USB port face) so it can work
        // out which way the robot is turning.
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revOrientation));
    }

    /**
     * Drives the mecanum base. Inputs are the raw gamepad stick values (-1..1).
     *
     * <p>Sign convention: {@code forward} is {@code gamepad.left_stick_y}, which is NEGATIVE when
     * the stick is pushed forward. The left motors are set to FORWARD and the right motors to
     * REVERSE in {@link #init}, which together with the un-negated stick value below makes
     * "stick forward" drive the robot forward, "stick right" strafe right and "right stick right"
     * rotate clockwise. (The Pedro Pathing drivetrain config uses the opposite motor directions
     * with conventional signs; both are consistent for this robot.)
     *
     * <p>In short, for this method: a NEGATIVE {@code forward} drives forward, a POSITIVE {@code strafe} moves the
     * robot to its RIGHT and a POSITIVE {@code rotate} turns it clockwise. (Pedro's convention is different:
     * forward positive, strafe positive = LEFT, turn positive = counter-clockwise.)
     *
     * @param forward the forward request, -1..1 (negative = forward, the raw stick value)
     * @param strafe the sideways request, -1..1 (positive = right)
     * @param rotate the turn request, -1..1 (positive = clockwise)
     */
    public void drive(double forward, double strafe, double rotate) {
        // Calculate power for each wheel. This determines whether the drive goes forward,
        // strafes or rotates.
        double frontLeftPower = forward - strafe - rotate;
        double backLeftPower = forward + strafe - rotate;
        double frontRightPower = forward + strafe + rotate;
        double backRightPower = forward - strafe + rotate;

        // Normalize power: if mixing the three requests pushes any wheel past 1.0, divide all four wheels by the
        // largest one, so the wheels keep the same proportions (the robot still goes the intended direction,
        // only slower). maxPower starts at 1.0 so wheel powers below 1.0 are left as they are.
        double maxPower = 1.0;
        // Overall speed limit, 0..1. 1.0 = full speed; a smaller value would scale every wheel down.
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // Set power to motors
        frontLeftMotor.setPower(maxSpeed * frontLeftPower / maxPower);
        backLeftMotor.setPower(maxSpeed * backLeftPower / maxPower);
        frontRightMotor.setPower(maxSpeed * frontRightPower / maxPower);
        backRightMotor.setPower(maxSpeed * backRightPower / maxPower);

    }
}
