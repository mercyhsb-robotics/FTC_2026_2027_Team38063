// ELEVATOR TEST OPMODE - DISABLED
//
// The current BioBuzz robot has no elevator mechanism (see Elevator.java), so this test OpMode is
// commented out. As written before, it crashed with a NullPointerException on START because the
// Elevator never initialised its motors. Re-enable it together with Elevator.java.

/*
package org.firstinspires.ftc.teamcode.mechanisms.elevator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ElevatorTest extends OpMode {
    Elevator elevator = new Elevator();

    @Override
    public void init() {
        elevator.init(hardwareMap);
    }

    @Override
    public void start() {
        elevator.raise();
    }

    @Override
    public void stop() {
        elevator.stop();
    }

    @Override
    public void loop() {
    }
}
*/
