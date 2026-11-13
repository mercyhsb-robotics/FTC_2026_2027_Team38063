package org.firstinspires.ftc.teamcode.mechanisms.mecanumDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.robots.biobuzz.HardwareMapConstants;

public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private IMU imu;

    public void init (HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, HardwareMapConstants.FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotor.class, HardwareMapConstants.BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotor.class, HardwareMapConstants.FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.get(DcMotor.class, HardwareMapConstants.BACK_RIGHT_MOTOR);

        // GoBilda
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

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
     */
    public void drive(double forward, double strafe, double rotate) {
        // Calculate power for each wheel. This determines whether the drive goes forward,
        // strafes or rotates.
        double frontLeftPower = forward - strafe - rotate;
        double backLeftPower = forward + strafe - rotate;
        double frontRightPower = forward + strafe + rotate;
        double backRightPower = forward - strafe + rotate;

        // Normalize power
        double maxPower = 1.0;
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
