# Team 38063 — BIOBUZZ (2026-27) robot code

Code for our goBILDA StarterBot (converted to mecanum) for the FIRST Tech Challenge game **BIOBUZZ**.
Built on the FTC SDK 12.0, Pedro Pathing 3.x (goBILDA Pinpoint odometry) and a Limelight 3A.

The headline feature: **hold one button and the robot drives to the closest feasible scoring pose for whichever
cell of your HIVE is tipped up, aligns on the AprilTags, and shoots every POLLEN it is holding.** It works for
both alliances and both sides of the HIVE.

**New to the robot? Start with [`docs/START_HERE.md`](docs/START_HERE.md).** It is the one guide for the robot day:
connect, check, measure, tune, set up the camera, build the shot table, try auto-shoot, and fix problems.

## The robot at a glance

| | |
|---|---|
| Drive | goBILDA mecanum, 17.80 × 17.80 in outline (its center of rotation is not the middle of the outline: about 9.77 in behind the front edge until you measure it, see `docs/START_HERE.md`), 104 mm wheels, Pinpoint odometry |
| Intake | front: roller + two corner-pulling servos |
| Shooter | back: one flywheel (1:1 Yellow Jacket) and a feeder "windmill" servo, fixed launch angle |
| Camera | Limelight 3A on top, facing the back with the shooter |
| Capacity | 4 POLLEN (rule G407) |

## OpModes (Driver Station list)

| OpMode | What it is for |
|---|---|
| **00 Robot Check** (TeleOp, group "1 Start here") | A guided test of every part (battery, names, wheels, Pinpoint, intake, flywheel, feeder, camera, gamepad). Run it first, with the robot on blocks. |
| **BioBuzzRobot** (TeleOp) | Driving. Hold **D-pad up** to auto-score. Alliance is chosen during INIT. |
| **Limelight Diagnostic** | Check the camera, its mounting constants and the tag readings with the robot standing still. |
| **Shooter Tuner** | Build the shot table: flywheel speed vs. distance. |
| RedAlliance_B1 / _C1 / _LeavePark | The existing autonomous routines (Pedro paths), in `autonomous/`. |
| PedroPathingAuto_*, AutoPath | Pedro path tests, in `autonomous/pathtests/`. |
| Pedro tuners | Mecanum, Pinpoint, **Foresight** and Tests procedures. They are a web page (`http://192.168.43.1:10158`, on the Robot Controller's Wi-Fi), not a Driver Station list item. See `docs/START_HERE.md`. |

### TeleOp controls (short version: every button of every OpMode is in the [Controller map](docs/START_HERE.md#controller-map); the shoot, flip, reset, alliance and jam buttons and the stick settings are in `BioBuzzConstants.Controls`)

| Control | Action |
|---|---|
| Left stick / right stick | drive / turn |
| **D-pad up (hold)** | drive to the scoring pose and shoot everything; release to take over |
| D-pad down | "the other cell is up": flip which cell the robot believes is tipped up |
| Share (hold ~0.6 s) | "I am standing on the default start pose": resets odometry |
| D-pad left / right (INIT only) | choose RED / BLUE alliance |
| Cross / circle | intake motor on / off |
| Triggers | the two corner servos |
| Triangle / square | manual launch on / off (feeds once the flywheel is fast enough) |
| Right bumper (hold) | feeder forward |
| **Left bumper (hold)** | **clear a jam:** run the indexer (feeder) slowly backward; also cancels auto-score |
| **D-pad right (hold, after PLAY)** | **clear a jam:** run the shooter flywheel slowly backward; also cancels auto-score |
| **Options (hold)** | **park:** drive into your own LOADING ZONE and hold there (5 PARK points at the end); release to take over |

Moving the left stick hard while the robot is driving itself cancels the automatic drive.

## The one rule: all numbers live in `BioBuzzConstants`

Every value that changes how the robot behaves on the field is in
[`BioBuzzConstants.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzConstants.java),
marked **FIXED** (from the manual/CAD), **ADD** (a placeholder waiting for a measurement), **VERIFY** (confirm
once on the robot) or **TUNE** (adjust from testing). The logic classes never need to be edited to retune the
robot. Two tests enforce this, and the TeleOp lists the values still marked ADD during INIT.
(`pedro/Constants.java` is the exception: Pedro's tuners generate it.)

## Where things are

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
  BioBuzzConstants.java        <- every tunable value
  teleop/                      <- BioBuzzRobot: the driver-controlled OpMode (wiring only)
  autonomous/                  <- the autonomous OpModes (RedAlliance_*); pathtests/ = Pedro path tests
  autoshoot/core/              <- the auto-shoot logic: field model, planner, vision analysis, state machine
                                  (plain Java, unit tested)
  autoshoot/hardware/          <- the only classes that touch the Limelight / Pedro / gamepad SDKs
  diagnostics/                 <- OpModes for checking hardware: Limelight Diagnostic, Shooter Tuner,
                                  IntakeTest, LauncherTest, MecanumDriveTest
  mechanisms/                  <- the robot's parts: Launcher, Intake, legacy MecanumDrive (no OpModes)
  pedro/                       <- Pedro Pathing config (written by its tuners), tuning menu, tuning procedures
TeamCode/src/test/             <- JUnit tests, including closed-loop simulations of the whole sequence
docs/                          <- guides (below)
```

## Guides

* [`docs/START_HERE.md`](docs/START_HERE.md) — **read this first.** The one guide for the robot day, in order: connect, Robot Check,
  measure, Pedro tuning, Limelight, shot table, the controller map, auto-shoot, autonomous, "if something goes wrong", and how it works
* [`docs/AUTOS.md`](docs/AUTOS.md) — autonomous routes as data: how to choose a route, where to place the robot, and the rules checks
  (the pictures are in [`docs/autos/index.html`](docs/autos/index.html))

## Build and test

```bash
./gradlew :TeamCode:assembleDebug          # build the robot controller app
./gradlew :TeamCode:testDebugUnitTest      # run the unit tests on a PC (about a minute)
```

The unit tests need no robot. They cover the geometry, vision analysis, shot gate and the full
press-and-hold sequence against a simulated robot, flywheel, HIVE and camera.

## Licence and credits

The FTC SDK (`FtcRobotController`) is © FIRST and its contributors under the licence in `LICENSE`; sound and
LEGO licence notices are in `doc/legal`. Release notes for the SDK are on the
[FTC SDK releases page](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases).
