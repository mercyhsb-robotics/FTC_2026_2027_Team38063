package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;

/**
 * Intake test: spins the front intake roller by itself so you can check that it turns and that it pulls POLLEN (the
 * balls the robot collects and shoots) in. It does not touch the drive, the shooter or the two corner servos.
 *
 * <p>When to run it: after wiring or changing the intake motor, or when the intake does nothing in the TeleOp. An
 * "OpMode" is one program the Driver Station can run; this one appears in the TeleOp list as "IntakeTest".
 *
 * <p>What to do: press INIT, then PLAY. The roller starts at full power the moment you press PLAY and runs until you
 * press STOP. The gamepad is not used and nothing is shown in telemetry. What you should see: the roller turning
 * so that a POLLEN placed at the front is pulled toward the robot. If it pushes it away instead, the motor direction
 * has to be reversed (Intake.init() does not set a direction). If nothing turns, check the motor name in the robot
 * configuration against BioBuzzConstants.Hardware.INTAKE_MOTOR.
 *
 * <p>Where it fits: it is a diagnostic. The real robot uses the same {@code Intake} class from {@code BioBuzzRobot}.
 */
@TeleOp
public class IntakeTest extends OpMode {
    // The roller we are testing.
    Intake intake = new Intake();

    // Runs once when INIT is pressed: find the motor.
    @Override
    public void init() {
        intake.init(hardwareMap);
    }

    // Runs once when PLAY is pressed: start the roller (full power).
    @Override
    public void start() {
        intake.startIntake();
    }

    // Runs once when STOP is pressed: switch the roller off.
    @Override
    public void stop() {
        intake.stopIntake();
    }

    // Runs over and over while the OpMode is running. Nothing to do: the roller is already on.
    @Override
    public void loop() {

    }
}
