package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;

@TeleOp
public class LauncherTest extends OpMode {
    Launcher launcher;

    @Override
    public void init() {
        launcher = new Launcher();
        launcher.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.triangle) {
            launcher.launch(true);
        }

        if (gamepad1.dpad_up) {
            launcher.increaseRpm(BioBuzzConstants.Shooter.MANUAL_TARGET_TPS);
        }

        if (gamepad1.dpad_down) {
            launcher.stop();
        }

        if (gamepad1.dpad_left) {
            launcher.feederReverse();
        }

        if (gamepad1.dpad_right) {
            launcher.feederForward();
        }

        if (gamepad1.square) {
            launcher.stop();
        }
    }
}

