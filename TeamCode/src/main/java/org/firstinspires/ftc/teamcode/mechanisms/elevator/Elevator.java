// ELEVATOR - DISABLED: THIS WHOLE FILE IS COMMENTED OUT
//
// Plainly: the current BioBuzz robot (goBILDA StarterBot, mecanum) has NO elevator mechanism. The Elevator class
// below is therefore commented out, which means the compiler ignores everything in this file and nothing here
// runs. It is kept only as a starting point for the day an elevator is actually built.
//
// Why it is commented out instead of left active: with no elevator motors on the robot it could not do anything
// useful (its init() would fail to find the motors "elevatorMotor1" and "elevatorMotor2"). The previous version
// also never initialised its motors (every call threw a NullPointerException) and reused the Intake method names.
// The version below is the tidied-up one.
//
// To re-enable it: delete the opening and closing comment markers, add the two motor names to
// BioBuzzConstants.Hardware, and construct/init it in the TeleOp.

/*
// (Everything from here to the closing comment marker is inactive code. If you re-enable it, these notes become
// ordinary comments above the class.)
//
// Elevator: a lift worked by two motors that are always driven together. raise() runs both at full power and stop()
// stops them. There is no method to lower it yet. Power 1.0 means full power in the motors' FORWARD direction;
// check on the real robot whether that raises or lowers the lift and reverse the motors if needed.
package org.firstinspires.ftc.teamcode.mechanisms.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    // The two lift motors. Their names are typed in directly below; when this class is re-enabled they should move
    // to BioBuzzConstants.Hardware like every other hardware name.
    private DcMotor elevatorMotor1;
    private DcMotor elevatorMotor2;

    // Finds both motors in the robot configuration. Call once, from the OpMode's init(). RUN_USING_ENCODER means the
    // motor controller uses each motor's encoder to regulate its speed.
    public void init(HardwareMap hardwareMap) {
        elevatorMotor1 = hardwareMap.get(DcMotor.class, "elevatorMotor1");
        elevatorMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");
        elevatorMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Runs both lift motors at full power (1.0 = 100 %) until stop() is called.
    public void raise() {
        elevatorMotor1.setPower(1.0);
        elevatorMotor2.setPower(1.0);
    }

    // Stops both lift motors (power 0).
    public void stop() {
        elevatorMotor1.setPower(0);
        elevatorMotor2.setPower(0);
    }
}
*/
