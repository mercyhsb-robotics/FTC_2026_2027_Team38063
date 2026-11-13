package org.firstinspires.ftc.teamcode.autoshoot.hardware;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.VisionFrame;
import org.firstinspires.ftc.teamcode.autoshoot.run.RouteExecutor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;

/**
 * The thin layer between the route runner ({@link RouteExecutor}, plain Java) and the robot: once per loop it reads the robot into an
 * {@link RouteExecutor.Inputs} and carries the {@link RouteExecutor.Outputs} out. It decides nothing.
 *
 * <p>Each loop the OpMode does three things, in this order: {@link #sense} (this also runs Pedro's follower, which reads the odometry and writes
 * the motor powers), {@code executor.update(inputs)}, and {@link #act}. It reuses the pieces the TeleOp uses: {@link PedroDrive} (the drive and
 * the odometry), {@link Launcher} (flywheel and feeder), {@link Intake} (the roller motor), the two corner intake servos and
 * {@link LimelightVision} (the camera, which may be missing: the runner then sees an "unavailable" frame and fails closed).
 *
 * <p>Writes to the robot are only sent when the command changes (the intake), or are safe to repeat every loop (the launcher and the drive
 * remember what they were told), so a long run does not flood the hub.
 */
public final class RouteRunnerHardware {
    private final PedroDrive drive;
    private final Launcher launcher;
    private final Intake intake;
    private final CRServo servoLeft;
    private final CRServo servoRight;
    private final LimelightVision vision;

    // What the intake was last told, so the hub is only written when it changes.
    private boolean intakeIsOn;

    /** @param vision the camera, or null if the Limelight could not be started */
    public RouteRunnerHardware(PedroDrive drive, Launcher launcher, Intake intake, CRServo servoLeft, CRServo servoRight, LimelightVision vision) {
        this.drive = drive;
        this.launcher = launcher;
        this.intake = intake;
        this.servoLeft = servoLeft;
        this.servoRight = servoRight;
        this.vision = vision;
    }

    /**
     * Runs the drive (Pedro's follower: odometry in, motor power out) and reads everything the runner needs.
     *
     * @param routeClockSec seconds since PLAY: the clock the runner works on
     * @param visionClockSec a clock that keeps running from INIT on, which the camera adapter uses to notice a frozen camera
     */
    public RouteExecutor.Inputs sense(double routeClockSec, double visionClockSec) {
        drive.update();
        RouteExecutor.Inputs in = new RouteExecutor.Inputs();
        in.nowSec = routeClockSec;
        in.pose = drive.pose();
        in.speedInS = drive.speedInPerSec();
        in.omegaRadS = drive.omegaRadPerSec();
        in.flywheelTps = launcher.velocity();
        in.frame = camera(visionClockSec);
        return in;
    }

    /** This loop's camera snapshot; without a Limelight it is the "not connected" frame. */
    public VisionFrame camera(double visionClockSec) {
        return vision != null ? vision.read(visionClockSec) : VisionFrame.unavailable(VisionFrame.Status.NOT_CONNECTED, visionClockSec);
    }

    /** Carries out what the runner asked for this loop. */
    public void act(RouteExecutor.Outputs out) {
        if (out.setPose != null) {
            drive.setPose(out.setPose);       // before the drive command: the next loop reads the new pose
        }
        launcher.setTargetVelocity(out.flywheelTps);
        launcher.setFeederOn(out.feederOn);
        setIntake(out.intakeOn);
        drive.apply(out.drive);
    }

    private void setIntake(boolean on) {
        if (on == intakeIsOn) {
            return;
        }
        intakeIsOn = on;
        if (on) {
            intake.startIntake();
            servoLeft.setPower(BioBuzzConstants.Run.INTAKE_LEFT_SERVO_POWER);
            servoRight.setPower(BioBuzzConstants.Run.INTAKE_RIGHT_SERVO_POWER);
        } else {
            intake.stopIntake();
            servoLeft.setPower(0.0);
            servoRight.setPower(0.0);
        }
    }

    /** Switches the flywheel, the feeder and the intake off and stops the drive. Safe to call more than once. */
    public void stopAll() {
        setIntake(false);
        launcher.stop();
        drive.stop();
        if (vision != null) {
            vision.stop();
        }
    }
}
