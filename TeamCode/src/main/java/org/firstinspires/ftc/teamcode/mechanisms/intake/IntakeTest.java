package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntakeTest extends OpMode {
    Intake intake = new Intake();

    @Override
    public void init() {
        intake.init(hardwareMap);
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

    }
}
