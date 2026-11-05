package org.firstinspires.ftc.teamcode.mechanisms.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Elevator {
    private DcMotor elevatorMotor1;
    private DcMotor elevatorMotor2;
    private Servo servo;

    public void init (HardwareMap hardwareMap) {
        // pseudocode for now

//        elevatorMotor1 = hardwareMap.get(DcMotor.class, "elevatorMotor1");
//        elevatorMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");
//        elevatorMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startIntake() {
        elevatorMotor1.setPower(1.0);
        elevatorMotor2.setPower(1.0);
    }

    public void stopIntake() {
        elevatorMotor1.setPower(0);
        elevatorMotor2.setPower(0);
    }

}
