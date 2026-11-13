// ELEVATOR TEST OPMODE - DISABLED: THIS WHOLE FILE IS COMMENTED OUT
//
// Plainly: the current BioBuzz robot has NO elevator mechanism (see Elevator.java), so this test OpMode is commented
// out. The compiler ignores everything in this file, so the test does not appear on the Driver Station and nothing
// here runs. As written before, it crashed with a NullPointerException on START because the Elevator never
// initialised its motors. Re-enable it together with Elevator.java.

/*
// (Everything from here to the closing comment marker is inactive code. If you re-enable it, these notes become
// ordinary comments above the class.)
//
// ElevatorTest: a diagnostic OpMode, in the Driver Station TeleOp list as "ElevatorTest". An "OpMode" is one program
// the Driver Station can run. Press INIT (it finds the two lift motors), then PLAY: the elevator is raised at full
// power and keeps going until you press STOP. The gamepad is not used. Watch that the lift moves the right way and
// stop it before it reaches the end of its travel.
package org.firstinspires.ftc.teamcode.mechanisms.elevator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ElevatorTest extends OpMode {
    // The lift under test.
    Elevator elevator = new Elevator();

    // Runs once when INIT is pressed.
    @Override
    public void init() {
        elevator.init(hardwareMap);
    }

    // Runs once when PLAY is pressed: start raising.
    @Override
    public void start() {
        elevator.raise();
    }

    // Runs once when STOP is pressed: stop the lift.
    @Override
    public void stop() {
        elevator.stop();
    }

    // Runs over and over after PLAY. Nothing to do: the lift is already running.
    @Override
    public void loop() {
    }
}
*/
