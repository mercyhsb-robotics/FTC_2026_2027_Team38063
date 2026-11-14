package org.firstinspires.ftc.teamcode.diagnostics;

import java.util.ArrayList;
import java.util.List;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Button;
import org.firstinspires.ftc.teamcode.autoshoot.core.CameraModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheck;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckConfig;
import org.firstinspires.ftc.teamcode.autoshoot.core.RobotCheckDevice;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.GamepadButtons;
import org.firstinspires.ftc.teamcode.autoshoot.hardware.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.launcher.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.mecanumdrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * "00 Robot Check": the first OpMode to run on the real robot. It walks the team through every important part, one
 * step at a time, with plain instructions on the Driver Station screen, and ends with a PASS / FAIL list that says what
 * to fix and where.
 *
 * <p><b>This class only does the hardware part.</b> Every decision (the order of the steps, the thresholds, the
 * PASS / WARN / FAIL verdicts, the advice texts) is in {@code autoshoot/core/RobotCheck*.java}, which is plain Java
 * with unit tests. Each loop this OpMode (1) reads the hardware and the gamepad into {@code RobotCheck.Inputs}, (2) asks
 * {@code RobotCheck.update} what to show and what the motors must do, (3) applies those motor commands (all of them,
 * every loop, so a zero really switches a part off) and (4) shows the text. Every number is in
 * {@code BioBuzzConstants.RobotCheck}.
 *
 * <p><b>The steps.</b> 0 safety, 1 battery, 2 hardware names, 3 drive wheels, 4 Pinpoint odometry, 5 intake, 6 flywheel,
 * 7 feeder, 8 Limelight camera, 9 gamepad, 10 summary. On the gamepad: A = yes / continue, B = no /
 * problem (and it stops a running motor at once), Y = skip the step, X = do it again. The buttons are
 * {@code BioBuzzConstants.RobotCheck.YES_BUTTON} and its neighbours.
 *
 * <p><b>Fail safe.</b> No motor or servo moves until the student presses the continue button; every test is short and at
 * low power; every motor and servo is switched off when a part ends, when you press STOP, and if anything in the loop
 * throws an error (the screen then says what went wrong and stays on until STOP).
 *
 * <p><b>The same set-up as the TeleOp.</b> The wheels use the directions that {@code pedro/Constants.java} gives Pedro (or,
 * if the Pinpoint cannot start, the ones {@code MecanumDrive.init()} sets, exactly like {@code BioBuzzRobot} falling
 * back to its backup drive). The shooter is set up by the real {@code Launcher} class (flywheel speed control and the
 * feeder direction), and the camera by {@code LimelightVision}. So a check that passes here means the TeleOp's own
 * set-up works.
 *
 * <p><b>Words used here.</b> An <b>OpMode</b> is one program the Driver Station can run. The <b>robot configuration</b> is
 * the list of devices (motors, servos, sensors) and their names that you save on the Driver Station. The <b>Pinpoint</b>
 * is the goBILDA odometry computer that tracks where the robot is; <b>Pedro</b> is the library that uses it to drive.
 * The <b>flywheel</b> is the fast wheel at the back that throws a POLLEN; the <b>feeder</b> is the servo that pushes
 * the POLLEN into it. The <b>Limelight</b> is the smart camera that finds AprilTags.
 *
 * <p><b>How to run it.</b> Pick "00 Robot Check" in the TeleOp list (group "1 Start here"), press INIT, connect gamepad 1
 * (press Start and A on it), put the robot on blocks, then press PLAY and follow the screen.
 */
@TeleOp(name = "00 Robot Check", group = "1 Start here")
public class RobotCheckOpMode extends OpMode {
    // The settings (all from BioBuzzConstants.RobotCheck) and the problems found in them, if any.
    private RobotCheckConfig cfg;
    private List<String> configProblems = new ArrayList<String>();
    // The logic, and the readings we hand it every loop.
    private RobotCheck check;
    private final RobotCheck.Inputs in = new RobotCheck.Inputs();

    // ---- drive and odometry
    // The four wheel motors, in the order front-left, back-left, front-right, back-right (null = not in the configuration).
    private final DcMotor[] wheels = new DcMotor[RobotCheckDevice.WHEELS.length];
    // Which drive the TeleOp would use, and why Pedro could not start (empty if it did).
    private RobotCheck.DriveKind driveKind = RobotCheck.DriveKind.PEDRO;
    private String driveNote = "";
    // Reads the Pinpoint through Pedro's own settings (pedro/Constants.java). null if it could not be created.
    private PinpointLocalizer localizer;
    // The Pinpoint itself, only to read its status (for example "FAULT_X_POD_NOT_DETECTED").
    private GoBildaPinpointDriver pinpoint;

    // ---- mechanisms
    private DcMotor intakeMotor;
    private CRServo leftServo;
    private CRServo rightServo;
    // The real shooter class: it sets up the flywheel's speed control and the feeder's direction. null if the flywheel or
    // the feeder is missing.
    private Launcher launcher;
    private CRServo feederServo;

    // ---- camera
    private LimelightVision vision;

    // System.nanoTime() (nanoseconds) when PLAY was pressed; the check's clock counts from there.
    private long startNanos;
    // Set if something in the loop threw an error: the text shown until STOP.
    private String faultText;

    // ------------------------------------------------------------------------------------------ INIT

    /** Runs once at INIT: finds every device (a missing one is not an error), sets the parts up, leaves everything off. */
    @Override
    public void init() {
        cfg = RobotCheckConfig.fromConstants();
        configProblems = cfg.problems();
        check = new RobotCheck(cfg);

        // Bulk caching, as in the TeleOp: the hub sends all its readings in one batch instead of one slow request each.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Which of the named devices are really in the robot configuration? tryGet answers null instead of throwing.
        in.present.clear();
        for (RobotCheckDevice d : RobotCheckDevice.values()) {
            if (isInConfiguration(d)) {
                in.present.add(d);
            }
        }

        // Each part is set up on its own, so one problem never stops the others from being checked.
        setUpDrive();
        setUpMechanisms();
        setUpCamera();
        stopEverything();
    }

    /** True if the robot configuration has a device of the right type with this device's name. */
    private boolean isInConfiguration(RobotCheckDevice d) {
        try {
            switch (d.kind) {
                case MOTOR:
                    return hardwareMap.tryGet(DcMotor.class, d.configName) != null;
                case CONTINUOUS_SERVO:
                    return hardwareMap.tryGet(CRServo.class, d.configName) != null;
                case ODOMETRY_COMPUTER:
                    return hardwareMap.tryGet(GoBildaPinpointDriver.class, d.configName) != null;
                case IMU:
                    return hardwareMap.tryGet(IMU.class, d.configName) != null;
                default:
                    return hardwareMap.tryGet(Limelight3A.class, d.configName) != null;
            }
        } catch (RuntimeException e) {
            return false;
        }
    }

    /**
     * Chooses the drive the way BioBuzzRobot does (Pedro with the Pinpoint if the Pinpoint starts, otherwise the backup
     * MecanumDrive) and gives the four wheel motors the directions that drive uses.
     */
    private void setUpDrive() {
        try {
            // Pedro builds its Pinpoint localizer from pedro/Constants.java (pod directions and offsets). It does not move the wheels.
            localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
            driveKind = RobotCheck.DriveKind.PEDRO;
        } catch (RuntimeException e) {
            localizer = null;
            driveKind = RobotCheck.DriveKind.LEGACY;
            driveNote = String.valueOf(e.getMessage());
        }
        try {
            pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, BioBuzzConstants.Hardware.PINPOINT);
        } catch (RuntimeException e) {
            pinpoint = null;
        }

        if (driveKind == RobotCheck.DriveKind.LEGACY) {
            // The backup drive sets its own motor directions in init(); calling it gives the wheels exactly those.
            try {
                new MecanumDrive().init(hardwareMap);
            } catch (RuntimeException ignored) {
                // A missing motor or IMU: the check reports that in plain words, so nothing to do here.
            }
        }
        for (int i = 0; i < wheels.length; i++) {
            try {
                wheels[i] = hardwareMap.tryGet(DcMotor.class, RobotCheckDevice.WHEELS[i].configName);
            } catch (RuntimeException e) {
                wheels[i] = null;
            }
            if (wheels[i] != null) {
                // Brake when the power is zero, so a wheel stops at once at the end of its test.
                wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (driveKind == RobotCheck.DriveKind.PEDRO) {
                    // Power is a plain fraction of full power; the direction is the one in pedro/Constants.java.
                    wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
        if (driveKind == RobotCheck.DriveKind.PEDRO) {
            setWheelDirection(0, Constants.drivetrainConfig.frontLeftDirection.get());
            setWheelDirection(1, Constants.drivetrainConfig.backLeftDirection.get());
            setWheelDirection(2, Constants.drivetrainConfig.frontRightDirection.get());
            setWheelDirection(3, Constants.drivetrainConfig.backRightDirection.get());
        }
    }

    private void setWheelDirection(int wheel, DcMotor.Direction direction) {
        if (wheels[wheel] != null) {
            wheels[wheel].setDirection(direction);
        }
    }

    /** Sets up the shooter with the real Launcher class, and finds the intake motor and the corner servos. */
    private void setUpMechanisms() {
        if (in.present.contains(RobotCheckDevice.FLYWHEEL) && in.present.contains(RobotCheckDevice.FEEDER)) {
            try {
                launcher = new Launcher();
                launcher.init(hardwareMap);
            } catch (RuntimeException e) {
                launcher = null;
            }
        }
        feederServo = tryServo(RobotCheckDevice.FEEDER);
        if (launcher == null && feederServo != null) {
            // Launcher.init normally sets this; without it do the same here so the feeder test still turns the right way round.
            feederServo.setDirection(BioBuzzConstants.Shooter.FEEDER_INVERTED
                    ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
        }
        try {
            intakeMotor = hardwareMap.tryGet(DcMotor.class, RobotCheckDevice.INTAKE_MOTOR.configName);
        } catch (RuntimeException e) {
            intakeMotor = null;
        }
        if (intakeMotor != null) {
            // Same mode as the Intake class uses.
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        leftServo = tryServo(RobotCheckDevice.LEFT_INTAKE_SERVO);
        rightServo = tryServo(RobotCheckDevice.RIGHT_INTAKE_SERVO);
    }

    private CRServo tryServo(RobotCheckDevice d) {
        try {
            return hardwareMap.tryGet(CRServo.class, d.configName);
        } catch (RuntimeException e) {
            return null;
        }
    }

    /** Starts the Limelight like the TeleOp does (selects the AprilTag pipeline and begins polling). A missing camera is fine. */
    private void setUpCamera() {
        if (!in.present.contains(RobotCheckDevice.LIMELIGHT)) {
            return;
        }
        try {
            vision = new LimelightVision(hardwareMap, CameraModel.fromConstants());
            vision.start();
        } catch (RuntimeException e) {
            vision = null;
        }
    }

    /** Runs over and over between INIT and PLAY: tells the team what to do first. */
    @Override
    public void init_loop() {
        telemetry.addLine("00 ROBOT CHECK: a guided check of every robot part.");
        telemetry.addLine("1. Connect gamepad 1: press Start and A on it. Gamepad 1 is now: " + gamepad1.type + ".");
        telemetry.addLine("2. Put the robot on blocks so all its wheels hang in the air.");
        telemetry.addLine("3. Press PLAY (the triangle) on the Driver Station. The first screen is the safety check.");
        telemetry.addLine("Drive in use: " + (driveKind == RobotCheck.DriveKind.PEDRO
                ? "Pedro with the Pinpoint (the same as the TeleOp)." : "the BACKUP drive (Pedro could not start)."));
        for (String problem : configProblems) {
            telemetry.addLine("SETTINGS PROBLEM: " + problem);
        }
    }

    /** Runs once at PLAY: starts the clock. Nothing moves yet. */
    @Override
    public void start() {
        startNanos = System.nanoTime();
        stopEverything();
    }

    // ------------------------------------------------------------------------------------------ LOOP

    /** Runs over and over after PLAY: read the hardware, ask the logic, apply the motor commands, show the text. */
    @Override
    public void loop() {
        boolean completed = false;
        try {
            if (!configProblems.isEmpty()) {
                // A typo in BioBuzzConstants.RobotCheck: do not run anything, say what it is.
                stopEverything();
                for (String problem : configProblems) {
                    telemetry.addLine("SETTINGS PROBLEM: " + problem);
                }
                telemetry.addLine("Fix it in BioBuzzConstants.RobotCheck, then run the check again.");
            } else if (faultText != null) {
                showFault();
            } else {
                double now = (System.nanoTime() - startNanos) / 1.0e9;
                readHardware(now);
                RobotCheck.Outputs out = check.update(in);
                applyCommands(out.commands);
                telemetry.addLine(out.title);
                for (String line : out.lines) {
                    telemetry.addLine(line);
                }
            }
            completed = true;
        } catch (RuntimeException e) {
            // Anything unexpected: every motor off first, then say what happened.
            stopEverything();
            faultText = e.getClass().getSimpleName() + ": " + e.getMessage();
            showFault();
            completed = true;
        } finally {
            if (!completed) {
                stopEverything();
            }
        }
    }

    private void showFault() {
        stopEverything();
        telemetry.addLine("The check stopped because of an error. Every motor is off.");
        telemetry.addLine(faultText);
        telemetry.addLine("Press STOP on the Driver Station. Tell a mentor what this message says.");
    }

    /** Copies the gamepad and every sensor reading into the logic's inputs. */
    private void readHardware(double now) {
        in.nowSec = now;
        in.buttons.clear();
        for (Button b : Button.values()) {
            if (GamepadButtons.pressed(gamepad1, b)) {
                in.buttons.add(b);
            }
        }
        in.leftTrigger = gamepad1.left_trigger > BioBuzzConstants.Controls.TRIGGER_THRESHOLD;
        in.rightTrigger = gamepad1.right_trigger > BioBuzzConstants.Controls.TRIGGER_THRESHOLD;
        in.gamepadType = String.valueOf(gamepad1.type);

        in.driveKind = driveKind;
        in.driveNote = driveNote;
        in.batteryVolts = readBatteryVolts();
        readPose();
        in.flywheelTps = launcher != null ? launcher.velocity() : 0.0;

        if (vision != null) {
            in.visionConnected = vision.connected();
            in.frame = vision.read(now);
            in.visionFrames = vision.framesSeen();
        } else {
            in.visionConnected = false;
            in.frame = null;
            in.visionFrames = 0;
        }
    }

    /** The lowest voltage any hub reports, in volts (NaN if none can be read). The lowest is the one that matters. */
    private double readBatteryVolts() {
        double lowest = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double volts = sensor.getVoltage();
            if (volts > 0.0 && (Double.isNaN(lowest) || volts < lowest)) {
                lowest = volts;
            }
        }
        return lowest;
    }

    /** Reads the Pinpoint pose through Pedro's localizer: x and y in inches, heading in radians. */
    private void readPose() {
        in.poseAvailable = false;
        if (localizer == null) {
            in.poseProblem = driveNote;
            return;
        }
        try {
            localizer.update();
            Pose pose = localizer.pose();
            in.poseXIn = pose.x();
            in.poseYIn = pose.y();
            in.poseHeadingRad = pose.heading();
            in.poseAvailable = true;
            in.poseProblem = "";
        } catch (RuntimeException e) {
            in.poseProblem = String.valueOf(e.getMessage());
        }
        in.pinpointStatus = "";
        if (pinpoint != null) {
            try {
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
                in.pinpointStatus = status == null ? "" : status.name();
            } catch (RuntimeException ignored) {
                in.pinpointStatus = "";
            }
        }
    }

    /** Applies EVERY command every loop, so a zero from the logic really switches that part off. */
    private void applyCommands(RobotCheck.Commands c) {
        for (int i = 0; i < wheels.length; i++) {
            if (wheels[i] != null) {
                wheels[i].setPower(c.wheel == i ? c.wheelPower : 0.0);
            }
        }
        if (intakeMotor != null) {
            intakeMotor.setPower(c.intakeMotorPower);
        }
        if (leftServo != null) {
            leftServo.setPower(c.leftServoPower);
        }
        if (rightServo != null) {
            rightServo.setPower(c.rightServoPower);
        }
        if (launcher != null) {
            launcher.setTargetVelocity(c.flywheelTps);
        }
        if (feederServo != null) {
            feederServo.setPower(c.feederPower);
        }
    }

    // ------------------------------------------------------------------------------------------ STOP

    /** Runs once at STOP: everything off, camera stopped. */
    @Override
    public void stop() {
        stopEverything();
        if (vision != null) {
            vision.stop();
        }
    }

    /**
     * Switches off every motor and servo. Each part is tried on its own, so a problem with one never leaves another running.
     * This method never throws.
     */
    private void stopEverything() {
        for (DcMotor wheel : wheels) {
            try {
                if (wheel != null) {
                    wheel.setPower(0.0);
                }
            } catch (RuntimeException ignored) {
                // keep switching the others off
            }
        }
        try {
            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }
        } catch (RuntimeException ignored) {
            // keep switching the others off
        }
        try {
            if (leftServo != null) {
                leftServo.setPower(0.0);
            }
        } catch (RuntimeException ignored) {
            // keep switching the others off
        }
        try {
            if (rightServo != null) {
                rightServo.setPower(0.0);
            }
        } catch (RuntimeException ignored) {
            // keep switching the others off
        }
        try {
            if (launcher != null) {
                launcher.stop();
            }
        } catch (RuntimeException ignored) {
            // keep switching the others off
        }
        try {
            if (feederServo != null) {
                feederServo.setPower(0.0);
            }
        } catch (RuntimeException ignored) {
            // nothing more to switch off
        }
    }
}
