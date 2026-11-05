package org.firstinspires.ftc.teamcode.mechanisms.mecanumDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        // RevRobotics
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Works on Star Robot
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {

        // Calculate power for different motors.   This determines
        // whether drive will go forward, strafe or rotate. 
//        double frontLeftPower = forward + strafe + rotate;
//        double backLeftPower = forward - strafe + rotate;
//        double frontRightPower = forward - strafe - rotate;
//        double backRightPower = forward + strafe - rotate;

//        double frontLeftPower = forward - strafe + rotate;
//        double backLeftPower = forward + strafe + rotate;
//        double frontRightPower = forward + strafe - rotate;
//        double backRightPower = forward - strafe - rotate;



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
