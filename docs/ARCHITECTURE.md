# How the auto-shoot feature works

## The game facts it is built on

* Each alliance has one HIVE with **two cells**, one on the audience side and one on the far side. Exactly one is
  tipped **UP** (scorable). Launching enough POLLEN into the up cell **tips** the HIVE: the other cell is up next.
  The team's figures are 5 NECTAR, or 8 POLLEN, or a mix. The match starts with 3 NECTAR already sitting in the
  up cell, so the **first** tip needs only one volley of 4 POLLEN (3 NECTAR + 4 POLLEN is enough, 3 NECTAR + 3 POLLEN
  is not). The cell that comes up after a tip has no NECTAR in it, so each later tip needs about 8 POLLEN, two
  volleys (our assumption: confirm it on the real HIVE).
* In autonomous the first tip may come from the **alliance partner** instead of from us. Every route in the autonomous
  catalog states which one it assumes (`FirstTip.SELF` or `FirstTip.PARTNER`, see `docs/AUTOS.md`). A route that
  assumes the partner waits for the camera to see the other cell tipped UP (the AprilTags on the bottom of the cell in
  view) before it shoots: a time never lets the robot shoot, and without a camera it skips the volley.
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

### Why the route is checked as a rectangle, not a point

The route finder (`PathPlanner`) plans for one dot, the robot's center, and keeps the dot a little way from walls and
obstacles. A dot has no corners. The real robot is a 17.8 x 17.8 in rectangle, and a rectangle that turns takes more
room than one that faces straight: facing the wall front-first its center of rotation must be 9.77 in from it, turned
25 degrees it needs 12.6 in, turned 45 degrees about 13.2 in (a front corner leads; see "Where the robot's edges are"
below). The robot does turn while it drives, so a route that is fine for the dot can swing a corner through a
wall, or through the HIVE frame or a FLOWER, in between two poses that each fit. A standing pose was always checked with
the real rectangle; the road between the poses was not.

Two small classes close that gap, and the drive and the check use the SAME ones so they cannot drift apart:

* `HeadingProfile` is the one rule for which way the robot faces at every point of a route (one turn, spread over the
  distance, optionally squeezed into a window). `PedroDrive` builds its Pedro path from it and the check reads it.
* `RouteSweep` slides the real rotated rectangle along the route and measures the gap to the four walls, the HIVE frame
  and every FLOWER plate. It looks every inch and every 3 degrees of turn, and looks more finely wherever the robot is
  close to something: a piece of route is accepted only when the gaps at its two ends, minus how far a corner can move
  in between, still leave room, so a thin obstacle cannot hide between two looks.

The planner runs the check on every candidate and repairs a route that fails: first the same road with the turn done only
where the robot is far enough from everything to spin freely (it keeps its heading while it leaves the wall, and is
finished turning before it comes back); then a detour out to open floor, with a bend added when the turn needs a longer
road to stay gentle. If no repair is safe the candidate is dropped like any blocked pose, and if every candidate is
dropped the controller stops with NO_FEASIBLE_POSE. A route may start touching a wall (the robot is set up there) but
may never squeeze closer than it started. The rule is tested with a separate, deliberately slow checker and with a
closed-loop simulation (see below). The numbers are `AutoShoot.PATH_*` in `BioBuzzConstants`.

What is NOT covered: the real robot never follows a route exactly, and the check keeps only the 0.25 in wall clearance
(plus 1.5 in around the HIVE and FLOWERS). The first time the robot drives out of a wall start, watch its corners and
retune `PATH_TURN_SETTLE_IN` and `PATH_MAX_TURN_DEG_PER_IN` if it turns late or clips.

## Where the robot's edges are (`RobotShape`)

The robot's **position** (the x, y and heading that Pedro Pathing and the Pinpoint report, and that every pose in the
code means) is its **center of rotation**: the one point that stays still while the robot spins on the spot. It is
**not** the middle of the 17.80 x 17.80 in outline. The intake sticks out in front of the front axle, so the outline
reaches **9.77 in in front** of that point, **8.03 in behind** it and **8.90 in to each side**
(`Robot.FRONT_EXTENT_IN`, `REAR_EXTENT_IN`, `LEFT_EXTENT_IN`, `RIGHT_EXTENT_IN`; they must add up to `LENGTH_IN` and
`WIDTH_IN`, and `ConstantsCheck` says so at INIT). The numbers come from the goBILDA drawing, taking the center of rotation
as the middle between the two axles; the team measures the real one (`docs/DAY_ONE_CHECKLIST.md`, section C).

`RobotShape` holds the four numbers and nothing else in the code writes `LENGTH_IN / 2`. `Footprint` builds the rotated
outline (the whole **perimeter**, the **chassis** without the front stretch where only the two flat wheels stick out, and
each flat **wheel disc**) from it, so every check that asks "does the robot fit here?" sees the same rectangle:
`RouteSweep`, `ScoringPlanner`, `RouteValidator`, the TeleOp planner, the start-pose check in `ConstantsCheck` and the
route diagrams. What it changes in practice:

* A robot facing a wall with its **front** touching it has its position **9.77 in** from the wall; with its **back** to the
  wall, **8.03 in**; sideways, **8.90 in**. (So the TeleOp default start is `y = FRONT_EXTENT_IN`, not 8.9.)
* A turning robot sweeps its farthest corner, a front corner, in a circle of **13.22 in** (`hypot(9.77, 8.90)`), not the
  12.59 in of a centered square.
* The ball leaves the **rear edge**, `Robot.LAUNCH_POINT_FORWARD_IN` = -8.03 in from the center of rotation (derived from
  the drawing, still to be measured). With the front on the audience wall the ball exit is 9.77 + 8.03 = 17.80 in from the
  wall, exactly the situation of the one measured shot (37.7 in from the aim point), so the physical shot did not change
  when the center of rotation moved.

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
* `ScoringPlannerWallSafetyTest`, which plans from every wall and corner (12 headings, both alliances) and walks every
  route in tiny steps with a separate checker, and `AutoShootWallSafetySimTest`, the closed loop that checks the
  robot's rectangle against the walls at every 20 ms tick while it drives out of a wall start (a planner with the check
  switched off must clip, or the tests prove nothing);
* `ConstantsDisciplineTest`, which keeps every tunable number in `BioBuzzConstants`.

None of this has run on the robot yet; `docs/DAY_ONE_CHECKLIST.md` is the order to verify it in.
