package org.firstinspires.ftc.teamcode.mechanisms.launcher;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SIDLauncher {
    private Telemetry telemetry;
    private DcMotor flywheel;
    private CRServo servo1;
    private CRServo servo2;

    public SIDLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize flywheel and servo
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPower(0);  // TODO: Replace "0" with constants

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo1.setDirection(CRServo.Direction.REVERSE);
        servo2.setDirection(CRServo.Direction.FORWARD);
        servo1.setPower(0); // TODO: Replace "0" with constants
        servo2.setPower(0); // TODO: Replace "0" with constants

    }

    public void startFlywheel(long rpm) {
        telemetry.addLine("Starting flywheel...");
        telemetry.addLine("Increasing rpm to " + rpm);
        telemetry.update();

        // Add code to increase rpm
        flywheel.setPower(1); // TODO: Replace "1" with constants
        ((DcMotorEx) flywheel).setVelocity(2300); // TODO: Replace 2500 with constants
    }
    public void stopFlywheel() {
        telemetry.addLine("Stopping flywheel..");
        telemetry.update();

        flywheel.setPower(0); // TODO: Replace "0" with constants
    }

    public void startServos() {
        telemetry.addLine("Starting servos...");
        servo1.setPower(1); // TODO: Replace "1" with constants
        servo2.setPower(1); // TODO: Replace "1" with constants
    }

    public void stopServos() {
        telemetry.addLine("Stopping servos..");
        servo1.setPower(0); // TODO: Replace "0" with constants
        servo2.setPower(0); // TODO: Replace "0" with constants
        servo1.setDirection(CRServo.Direction.REVERSE);
        servo2.setDirection(CRServo.Direction.FORWARD);
    }

    public void reverseServos() {
        servo1.setPower(1); // TODO: Replace "1" with constants
        servo2.setPower(1); // TODO: Replace "1" with constants
        servo1.setDirection(CRServo.Direction.FORWARD);
        servo2.setDirection(CRServo.Direction.REVERSE);
    }
}
