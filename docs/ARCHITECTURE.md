# How the auto-shoot feature works

## The game facts it is built on

* Each alliance has one HIVE with **two cells**, one on the audience side and one on the far side. Exactly one is
  tipped **UP** (scorable). Launching enough POLLEN into the up cell **tips** the HIVE: the other cell is up next.
  The team's figure is about 8 POLLEN per tip.
* The robot's shooter fires out the **back**, so the robot stands on the side its up cell faces, with its back
  toward the HIVE. Standing too far back is limited by the wall: the farthest in-line shot from the audience wall is
  37.7 in.
* Launching at the outside (bottom, sides, top) of a cell is a foul (rule G417), so the robot must not feed while the
  cell it is aiming at is tipping down. The AprilTags are on the cells, so they **move** and cannot be used to locate
  the robot on the field; odometry does that, and the camera supplies relative alignment.
* The field is 180° rotationally symmetric. Everything is written once for RED and the blue alliance is the same
  numbers rotated half a turn about the field center.

## Data flow, every loop

```
 gamepad ──► BioBuzzRobot (wiring only)
 Pinpoint ─► PedroDrive ──► pose, speed ─┐
 Limelight ► LimelightVision ► VisionFrame ► ClusterAnalyzer ► ClusterViews ─► HiveTracker (which cell is up?)
                                                             │
                              ┌──────────────────────────────┘
                              ▼
                     AutoShootController (state machine)  ──► Outputs: drive command, flywheel speed, feeder, cue
                              │  uses ScoringPlanner, PathPlanner, ShotTable, FlywheelReady, ShotCounter
                              ▼
        PedroDrive.apply  ·  Launcher.setTargetVelocity / setFeederOn  ·  gamepad rumble
```

A second, optional loop runs beside this one: `PoseCorrector` compares the position the camera implies (from
`ClusterAnalyzer`) with the Pinpoint position and, if the robot is nearly still and the two differ by only a few
inches, nudges the Pinpoint position toward the camera's answer (see "Correcting drift" below).

Only the green-field classes touch the SDK: `LimelightVision`, `PedroDrive`, `GamepadButtons`, `Launcher`.
Everything in `autoshoot/core` is plain Java that takes measurements in and returns commands out, so the whole
sequence runs in unit tests against a simulated robot.

## Correcting drift (`PoseCorrector`)

The Pinpoint is accurate over short distances but slowly drifts (wheels slip, the heading creeps). Because the HIVE
is at a known place on the field, every sighting of its AprilTags also tells us where the robot is standing. When
that disagrees a little with the Pinpoint, the difference is mostly drift. `PoseCorrector` moves the Pinpoint
position a fraction of the way toward the camera's answer. It is cautious on purpose:

* only the **position** is corrected, never the heading (the Pinpoint's gyro is better than a tag angle);
* a sighting that disagrees by more than `POSE_CORRECTION_GATE_IN` is thrown away as a mistake (a wrong
  camera mount must not drag the robot across the field);
* each step is only `POSE_CORRECTION_GAIN` of the disagreement and never more than `POSE_CORRECTION_MAX_STEP_IN`;
* it works only while the robot is almost still, with a fresh image and at least `POSE_CORRECTION_MIN_TAGS` tags,
  and never in the middle of feeding a ball.

It is **off** (`Vision.POSE_CORRECTION_ENABLED = false`) until the camera mount has been checked on the real robot.

## The state machine (`AutoShootController`)

```
 IDLE --press--> PLAN --> APPROACH --> ALIGN --> FEED --> DONE
                   ^          |           |        |
                   +----------+-----------+--------+    the HIVE tipped / the other cell is up
 any state --release--> IDLE        any state --problem--> FAULT(reason), wait for release
```

* **PLAN** asks the `HiveTracker` which cell is up and the `ScoringPlanner` for the closest feasible pose; the
  flywheel starts spinning for that range at once.
* **APPROACH** follows a route (around the HIVE frame if needed) and hands over to ALIGN on **measured pose error**,
  never on Pedro's `isBusy()`, which does not mean "arrived".
* **ALIGN** holds the pose. The camera nudges the hold heading toward the aim point and the range toward the
  planned range, within small limits. It waits until the robot is settled.
* **FEED** runs the feeder only while a **fail-closed gate** says every condition holds (pose trusted, HIVE cell
  confirmed, tags in view, range inside the table, not moving, shooter pointing at the aim point, flywheel at
  speed). It counts shots by the speed dip each ball causes. When the up cell's tags vanish the feeder pauses
  immediately; the tracker then flips its belief and the sequence plans for the other side, keeping the shots
  already fired.
* Releasing the button returns to manual driving with the feeder off in the same loop. A button held through INIT
  cannot start a volley, and a finished volley does not restart until the button is released and pressed again.

## Which cell is up (`ClusterAnalyzer`, `HiveTracker`)

The stickers of both cells of a HIVE lie in one plane. Seen from the up cell's side, the four tag IDs run left to
right in increasing order; the other cell's sticker (seen across the gap) is turned half a turn, so its IDs run the
other way. That order is read from `tx` alone, so it does not depend on any 3D axis convention. Tag **height**
(about 50 in for an up cell, 36 in for a down cell) is a second vote when 3D positions are available; if the two
disagree the reading is "unknown". The tracker changes its belief only after several new frames agree, and treats
tags that were seen steadily and then vanish from the shooting side as a suspected tip.

From a cluster the analyzer also computes the range to the aim point, the turn that would point the shooter at it,
and the robot position the sighting implies, using the known tag offsets (any subset of the four tags works) and the
robot's heading from odometry.

## Where to stand (`ScoringPlanner`)

Candidates have the ball exit on the side the opening faces, inside a cone around the opening's center line, at a
range inside the shot table, with the back pointed at the aim point. A candidate is feasible if the robot's
footprint is clear of the walls, the HIVE frame and the FLOWERS and a collision-free route exists (`PathPlanner`, a
visibility graph around the keep-outs; the far side is reached through the left/right corridor). The cheapest
candidate wins: travel, plus small costs for turning and for being away from the preferred range.

## Rules the design follows

* **G407** at most 4 POLLEN: a volley is at most 4 shots and the intake stops while shooting.
* **G417** only launch into an up cell: confirmation before feeding, tag-loss pause, never fire blind.
* **G409** do not catch POLLEN released by a tipping HIVE: the robot never drives toward the cell that just tipped.
* **G402** (autonomous only) stay on your own half; the route validator checks it.
* **G401** no driver input in autonomous; the auto-shoot feature is TeleOp only.

## Testing

`./gradlew :TeamCode:testDebugUnitTest` runs, among others:

* geometry, planner, vision-analysis and tracker tests against a virtual camera that projects the real tag
  positions through the camera model;
* `AutoShootSequenceTest`, a closed loop of the real controller against a simulated robot, flywheel, HIVE that
  tips, and camera: both alliances and all four cells, a tip mid-volley, a wrong belief, no tags, no camera, noise and
  dropouts, release, stick override, a flywheel that never reaches speed, an empty hopper;
* `ConstantsDisciplineTest`, which keeps every tunable number in `BioBuzzConstants`.

None of this has run on the robot yet; `docs/DAY_ONE_CHECKLIST.md` is the order to verify it in.
