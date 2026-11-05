package org.firstinspires.ftc.teamcode.robots.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.elevator.Elevator;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.mecanumDrive.MecanumDrive;

@TeleOp
public class SIDDecodeRobot extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private Intake intake = new Intake();
    private Elevator elevator = new Elevator();
    private Servo servoLeft;

    private Servo servoRight;

    double forward;
    double strafe;
    double rotate;

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        intake.init(hardwareMap);
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
    }

    @Override
    public void start() {
        intake.startIntake();
    }

    @Override
    public void stop() {
        intake.stopIntake();
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);

        mecanumDrive.drive(forward, strafe, rotate);

        // Servo stuff
        if (gamepad1.left_bumper)
            servoLeft.setPosition(1);
        else
            servoLeft.setPosition(0.5);


        if (gamepad1.right_bumper)
            servoRight.setPosition(1);
        else
            servoRight.setPosition(0.5);
    }
}
