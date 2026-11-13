package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;

/**
 * The intake roller: the motor at the FRONT of the robot that pulls POLLEN (the balls the robot collects and shoots)
 * in. It has only two actions, on (full speed) and off.
 *
 * <p>The two corner-pulling servos that sit beside the roller are NOT part of this class; {@code BioBuzzRobot}
 * drives them directly from the triggers.
 *
 * <p>Who uses it: {@code BioBuzzRobot} (cross turns it on, circle turns it off; it is also switched off while the
 * robot is shooting automatically, because the robot may hold at most 4 POLLEN) and the {@code IntakeTest}
 * diagnostic. The motor's name in the robot configuration is {@code BioBuzzConstants.Hardware.INTAKE_MOTOR}.
 */
public class Intake {
    // The roller motor. The code never sets its direction, so positive power is the motor's FORWARD direction.
    private DcMotor intakeMotor;

    /** Finds the roller motor in the robot configuration. Call once, from the OpMode's init(). */
    public void init (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, BioBuzzConstants.Hardware.INTAKE_MOTOR);
        // RUN_USING_ENCODER: the motor controller uses the motor's encoder to regulate its speed, so the power we
        // give is a fraction (-1..1) of the motor's top speed.
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Runs the roller at full power (1.0 = 100 %, the direction that pulls POLLEN in) until stopIntake() is called. */
    public void startIntake() {
        intakeMotor.setPower(1.0);
    }

    /** Stops the roller (power 0). */
    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
