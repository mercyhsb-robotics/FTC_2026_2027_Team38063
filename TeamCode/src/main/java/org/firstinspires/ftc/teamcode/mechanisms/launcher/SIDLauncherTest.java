package org.firstinspires.ftc.teamcode.mechanisms.launcher;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SIDLauncherTest extends OpMode {
    private CRServo servo1;
    private CRServo servo2;
    private DcMotor flywheel;

    SIDLauncher launcher;

    @Override
    public void init() {
//        launcher = new SIDLauncher(hardwareMap, telemetry);
        float y;
        double rx;
        float x;
        double denominator;

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        servo1.setDirection(CRServo.Direction.REVERSE);
        servo2.setDirection(CRServo.Direction.FORWARD);
        servo1.setPower(0);
        servo2.setPower(0);
        flywheel.setPower(0);

    }

    @Override
    public void loop() {
        launch();
    }

    private void launch() {
        if (gamepad1.left_bumper) {
            launcher.startFlywheel(2300);
        } else {
            launcher.stopFlywheel();
        }
        if (gamepad1.right_bumper) {
            launcher.startServos();
        } else {
            launcher.stopServos();
        }
        if (gamepad1.left_trigger >= 0.3) {
            launcher.startFlywheel(1500);
        } else {
            flywheel.setPower(0);
        }
        if (gamepad1.right_trigger >= 0.3) {
            launcher.reverseServos();
        } else {
            launcher.stopServos();
        }
    }
}
