# Team 38063 — BIOBUZZ (2026-27) robot code

Code for our goBILDA StarterBot (converted to mecanum) for the FIRST Tech Challenge game **BIOBUZZ**.
Built on the FTC SDK 12.0, Pedro Pathing 3.x (goBILDA Pinpoint odometry) and a Limelight 3A.

The headline feature (in the **BioBuzzRobot** OpMode): **hold one button and the robot drives to the closest feasible scoring pose for whichever
cell of your HIVE is tipped up, aligns on the AprilTags, and shoots every POLLEN it is holding.** It works for
both alliances and both sides of the HIVE.

**New to the robot? Start with [`docs/START_HERE.md`](docs/START_HERE.md).** It is the one guide for the robot day:
connect, check, measure, tune, set up the camera, build the shot table, try auto-shoot, and fix problems.

## The robot at a glance

| | |
|---|---|
| Drive | goBILDA mecanum, outline 17.80 × 17.62 in (length × width, measured) (its center of rotation is not the middle of the outline: about 9.77 in behind the front edge until you measure it, see `docs/START_HERE.md`), 104 mm wheels, Pinpoint odometry |
| Intake | front: roller + two corner-pulling servos |
| Shooter | back: one flywheel (1:1 Yellow Jacket) and a feeder "windmill" servo, fixed launch angle |
| Camera | Limelight 3A on top, facing the back with the shooter |
| Capacity | 4 POLLEN (rule G407) |

## OpModes (Driver Station list)

| OpMode | What it is for |
|---|---|
| **00 Robot Check** (TeleOp, group "1 Start here") | A guided test of every part (battery, names, wheels, Pinpoint, intake, flywheel, feeder, camera, gamepad). Run it first, with the robot on blocks. |
| **Manual TeleOp** (TeleOp, group "0 Competition") | **Use this at competition.** You aim and shoot by hand: hold the right trigger to shoot (the flywheel spins up and the feeder pushes POLLEN in as soon as the flywheel is fast enough, and the flywheel brakes when you let go), the left trigger to intake. Its slow-mode and shooting-speed buttons are in `BioBuzzConstants.Manual`, its shoot and reverse buttons in `BioBuzzConstants.Controls`. |
| **BioBuzzRobot** (TeleOp) | Driving with the automatic features: hold **D-pad up** to auto-score. Alliance is chosen during INIT. Same shooter, intake and reverse buttons as the Manual TeleOp (gamepad 2's work too). |
| **Limelight Diagnostic** | Check the camera, its mounting constants and the tag readings with the robot standing still. |
| **Shooter Tuner** | Build the shot table: flywheel speed vs. distance. |
| **Auto Route Picker** (Autonomous, group "2 Autonomous") | **The way to run an auto.** At INIT you choose a route and the alliance (D-pad up / down picks the route, D-pad left = RED, D-pad right = BLUE). The screen shows who tips the HIVE first, the time, our points, the route check, the Limelight, the values still to measure and, in plain words, where to put the robot (measured to its edges). If it says REFUSED the robot will not move: fix it or pick another route. After PLAY the robot drives the route by itself, never fires at a cell the camera has not seen UP, stops everything when AUTO ends and keeps its position for BioBuzzRobot. The routes have a tier word in their title: **Safe**, **Standard** or **Aggressive** (**Advanced** is for experts). |
| RedAlliance_B1 / _C1 / _LeavePark | The three old hand-written autonomous routines (Pedro paths), in `autonomous/`. With the current numbers they start the robot 0.57 in through the wall: use the Auto Route Picker instead. |
| PedroPathingAuto_*, AutoPath | Pedro path tests, in `autonomous/pathtests/`. |
| Pedro tuners | Mecanum, Pinpoint, **Foresight** and Tests procedures. They are a web page (`http://192.168.43.1:10158`, on the Robot Controller's Wi-Fi), not a Driver Station list item. See `docs/START_HERE.md`. |

### TeleOp controls (short version: every button of every OpMode is in the [Controller map](docs/START_HERE.md#controller-map); the shoot, flip, reset, alliance, park, jam and reverse buttons, the trigger setting and the stick settings are in `BioBuzzConstants.Controls`; the slow-mode and shooting-speed buttons are in `BioBuzzConstants.Manual`)

Gamepad 1 drives. Gamepad 2 (if you have one) can also press the shooter, intake, reverse and speed buttons.

| Control | Action |
|---|---|
| Left stick / right stick | drive / turn |
| **Right trigger (hold)** | **shoot by hand:** the flywheel spins up and the feeder pushes POLLEN in once it is fast enough; let go and the flywheel brakes to a stop |
| **Left trigger (hold)** | **intake:** the roller and both corner servos pull POLLEN in |
| Left bumper (hold) | slow drive, for careful aiming (Manual TeleOp only; `BioBuzzConstants.Manual`) |
| D-pad up / down / left | shooting speed faster / slower / normal (Manual TeleOp only; `BioBuzzConstants.Manual`) |
| **Right bumper (hold)** | reverse the **indexer** (feeder), slowly: clears a jam |
| **B (hold)** | reverse the **shooter flywheel**, slowly: clears a jam |
| **A (hold)** | reverse the **intake roller**: pushes a POLLEN out |
| **X (hold)** | reverse the **two corner servos** (the feelers): pushes a POLLEN out |
| D-pad up (hold) | BioBuzzRobot only: drive to the scoring pose and shoot everything; release to take over |
| D-pad down | BioBuzzRobot only: "the other cell is up": flip which cell the robot believes is tipped up |
| View (hold ~0.6 s) | BioBuzzRobot only: "I am standing on the default start pose": resets odometry |
| D-pad left / right (INIT only) | BioBuzzRobot only: choose RED / BLUE alliance |
| **Menu (hold)** | BioBuzzRobot only, **park:** drive into your own LOADING ZONE and hold there (5 PARK points at the end); release to take over |

Moving the left stick hard while the robot is driving itself cancels the automatic drive.

## The one rule: all numbers live in `BioBuzzConstants`

Every value that changes how the robot behaves on the field is in
[`BioBuzzConstants.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzConstants.java),
marked **FIXED** (from the manual/CAD), **ADD** (a placeholder waiting for a measurement), **VERIFY** (confirm
once on the robot) or **TUNE** (adjust from testing). The logic classes never need to be edited to retune the
robot. Two tests enforce this, and both TeleOps list the values still marked ADD during INIT.
(`pedro/Constants.java` is the exception: Pedro's tuners generate it.)

## Where things are

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
  BioBuzzConstants.java        <- every tunable value
  teleop/                      <- ManualTeleOp (competition) and BioBuzzRobot (automatic features): the driver-controlled OpModes (wiring only)
  autonomous/                  <- the autonomous OpModes: AutoRoutePicker, and the old RedAlliance_*; pathtests/ = Pedro path tests
  autoshoot/core/              <- the auto-shoot logic: field model, planner, vision analysis, state machine
                                  (plain Java, unit tested)
  autoshoot/routes/            <- autonomous routes as data: the catalog, the checker, the time and points estimate
  autoshoot/run/               <- the route runner and the Auto Route Picker's menu logic (plain Java, unit tested)
  autoshoot/hardware/          <- the only classes that touch the Limelight / Pedro / gamepad SDKs
  diagnostics/                 <- OpModes for checking hardware: Limelight Diagnostic, Shooter Tuner,
                                  IntakeTest, LauncherTest, MecanumDriveTest
  mechanisms/                  <- the robot's parts: Launcher, Intake, MecanumDrive (the simple drive of the Manual TeleOp); no OpModes
  pedro/                       <- Pedro Pathing config (written by its tuners), tuning menu, tuning procedures
TeamCode/src/test/             <- JUnit tests, including closed-loop simulations of the whole sequence
docs/                          <- guides (below)
```

## Guides

* [`docs/START_HERE.md`](docs/START_HERE.md) — **read this first.** The one guide for the robot day, in order: connect, Robot Check,
  measure, Pedro tuning, Limelight, shot table, the controller map, auto-shoot, autonomous, "if something goes wrong", and how it works
* [`docs/AUTOS.md`](docs/AUTOS.md) — autonomous routes as data: the table of all routes, how to choose a route, where to place the robot, the rules checks
  and how the Auto Route Picker runs a route (the pictures are in [`docs/autos/index.html`](docs/autos/index.html))

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
