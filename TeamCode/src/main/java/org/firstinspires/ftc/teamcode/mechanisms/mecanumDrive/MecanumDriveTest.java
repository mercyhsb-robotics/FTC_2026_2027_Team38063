package org.firstinspires.ftc.teamcode.mechanisms.mecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MecanumDriveTest extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    double forward;
    double strafe;
    double rotate;

    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
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
    }
}
