package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.biobuzz.HardwareMapConstants;

public class Intake {
    private DcMotor intakeMotor;

    public void init (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, HardwareMapConstants.INTAKE_MOTOR);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startIntake() {
        intakeMotor.setPower(1.0);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
