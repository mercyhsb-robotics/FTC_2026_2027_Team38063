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
        elevator.startIntake();
    }

    @Override
    public void stop() {
        elevator.stopIntake();
    }

    @Override
    public void loop() {

    }
}
