// ELEVATOR - DISABLED
//
// The current BioBuzz robot (goBILDA StarterBot, mecanum) has no elevator mechanism, so this
// class is commented out. The previous version never initialised its motors (every call threw a
// NullPointerException) and reused the Intake method names. It is kept below, tidied up, as a
// starting point for when an elevator is actually built.
//
// To re-enable it: delete the opening and closing comment markers, add the two motor names to
// BioBuzzConstants.Hardware, and construct/init it in the TeleOp.

/*
package org.firstinspires.ftc.teamcode.mechanisms.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private DcMotor elevatorMotor1;
    private DcMotor elevatorMotor2;

    public void init(HardwareMap hardwareMap) {
        elevatorMotor1 = hardwareMap.get(DcMotor.class, "elevatorMotor1");
        elevatorMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");
        elevatorMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void raise() {
        elevatorMotor1.setPower(1.0);
        elevatorMotor2.setPower(1.0);
    }

    public void stop() {
        elevatorMotor1.setPower(0);
        elevatorMotor2.setPower(0);
    }
}
*/
