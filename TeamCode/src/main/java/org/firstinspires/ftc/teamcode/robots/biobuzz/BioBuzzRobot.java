package org.firstinspires.ftc.teamcode.robots.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.elevator.Elevator;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.mecanumDrive.MecanumDrive;

@TeleOp
public class BioBuzzRobot extends OpMode {
    private MecanumDrive mecanumDrive;

    private Launcher launcher;
    private Intake intake;
    private Elevator elevator;
    private CRServo servoLeft;

    private CRServo servoRight;

    double forward;
    double strafe;
    double rotate;

    @Override
    public void init() {
        // Initialize mecanum drive
        mecanumDrive = new MecanumDrive();
        mecanumDrive.init(hardwareMap);

        // Initialize launcher
        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);

        // Initialize intake
        intake = new Intake();
        intake.init(hardwareMap);

        // Initialize feeders
        servoLeft = hardwareMap.get(CRServo.class, HardwareMapConstants.LEFT_INTAKE_SERVO);
        servoRight = hardwareMap.get(CRServo.class, HardwareMapConstants.RIGHT_INTAKE_SERVO);
    }

    @Override
    public void stop() {
        intake.stopIntake();
        launcher.stop();
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

        // Example controls for first iteration of BioBuzz robot.   It is
        // up to team to determine what controls they want.  
        // Controls for left and right feeders
        if (gamepad1.left_trigger > 0.5) {
            servoLeft.setPower(1);
            servoRight.setPower(1);
        } else {
            servoLeft.setPower(0);
            servoRight.setPower(0);
        }
        if (gamepad1.right_trigger > 0.5) {
            servoLeft.setPower(-1);
            servoLeft.setPower(-1);
        } else {
            servoLeft.setPower(0);
            servoRight.setPower(0);
        }

        // Controls for feeder
        if (gamepad1.left_bumper)
            launcher.feederReverse();

        if (gamepad1.right_bumper)
            launcher.feederForward();

        // Controls for launcher
        if (gamepad1.triangle)
            launcher.launch(true);
        if (gamepad1.square) // square
            launcher.launch(false);

        // Controls for intake
        if (gamepad1.cross)
            intake.startIntake();
        if (gamepad1.circle)
            intake.stopIntake();

        // Controls for DPad
        // TODO:  We can use this for automated features, such as automatically
        // search for pollen and intake.
        if (gamepad1.dpad_up) {
            // TODO
            telemetry.addLine("'dpad_up' was pressed");
        }
        if (gamepad1.dpad_down) {
            // TODO
            telemetry.addLine("'dpad_down' was pressed");
        }
        if (gamepad1.dpad_left) {
            // TODO
            telemetry.addLine("'dpad_left' was pressed");
        }
        if (gamepad1.dpad_right) {
            // TODO
            telemetry.addLine("'dpad_right' was pressed");
        }


    }
}
