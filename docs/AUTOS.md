# Autonomous routes as data

An **auto route** is a short list of steps: where the robot starts, where it drives, what it picks up, what it
shoots, where it parks. This framework lets the team write that list as **one data entry**, and then gives back,
without a robot:

* a **picture** of the route on the whole field, for the red alliance and for the blue one;
* an **estimate** of how long it takes (the AUTO period is 30 s), how many POLLEN it handles and how many points it
  can claim;
* a **check** against the game rules (start placement, field walls, the HIVE, FLOWERS, the center line, the
  4 POLLEN limit, the LOADING ZONE) that lists every problem with the step it belongs to.

> **What it does not do (yet):** drive the robot. The OpModes (`RedAlliance_B1`, `RedAlliance_C1`,
> `RedAlliance_LeavePark`) still hold the Pedro code that moves the robot. The catalog *describes* those autos
> faithfully, so the picture and the rule check show what the OpMode really does, including its mistakes.

The pictures are in [`docs/autos/index.html`](autos/index.html) (open the file in any browser; it follows your light
or dark setting). They are generated, so never edit them by hand.

## Add an auto in five steps

1. **Pick a start.** The robot must start touching a field wall, completely inside its own half, not in the LOADING
   ZONE, not touching a FLOWER or the HIVE frame (rule G304). A pose is the robot's **center of rotation**, and that is not
   the middle of its 17.80 x 17.80 in outline: the front edge is 9.77 in ahead of it, the back edge 8.03 in behind it. So a
   robot facing the audience wall at tile B1 or C1 with its **front edge on the wall** has its center 9.77 in from the
   wall: `start(35.0, 9.77, -90)`. (Facing away from the wall, back edge on it: 8.03 in; sideways: 8.90 in.)
2. **Add one entry** to the file of its family (see "Who shoots the first tip" below):
   `SelfTipRoutes.java` if your own 4 pre-loads tip the HIVE, `PartnerTipRoutes.java` if the alliance partner tips it.
   Both are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autoshoot/routes/`, and each has a `routes()`
   method with a list to add to. (`RouteCatalog.java` only gathers the lists; the three conversions of the old
   OpModes and the two demonstration routes are written in it.) Write the route for the **RED** alliance only (see
   "Red and blue" below). Copy this and change it:

   ```java
   routes.add(AutoRoute.builder("my_route")                 // id: letters, digits, _ and - only; never changes
           .title("C1: shoot 4, then park")                  // what people read
           .description("Fires the 4 pre-loads, then drives up the left corridor into the LOADING ZONE.")
           .startTile("C1")                                  // must match the tile of the start pose
           .start(58.0, 9.77, -90)                           // x, y in inches (front edge on the wall), heading in degrees
           .shootUp(4)                                       // 4 POLLEN at the UP cell, pose chosen by the solver
           .driveReverseTangent(30.0, 45.0).via(30.0, 12.6)  // a curve: shooter first, bends toward (30, 12.6)
           .driveReverseTangent(30.0, 100.0).label("corridor")
           .driveLinear(12.0, 108.0, 0.0)                    // turns evenly to face +x while sliding in
           .park(12.0, 108.0, 0.0)                           // the final pose, partly inside the LOADING ZONE
           .build());
   ```

3. **Draw it.** Run `tools/render-autos.sh` (Git Bash, macOS, Linux) or `tools\render-autos.bat` (Windows). It
   needs only a JDK (8 or newer), takes a few seconds, and prints one line per route:

   ```
   route                 time  points  POLLEN  1st tip  validator
   my_route             12.3s       8      -3  SELF     no findings
   ```

   The `1st tip` column is who the route assumes shoots the first tip (`SELF` or `PARTNER`).
4. **Read the page.** Open `docs/autos/index.html`, find your route, look at both diagrams and read the findings.
   An **ERROR** means the route breaks a rule or cannot work (fix it). A **WARN** means "probably fine, check it on
   the field". Change the entry, run the script again, repeat.
5. **Commit** the catalog entry *and* the regenerated `docs/autos` files together. Then run the unit tests
   (`./gradlew :TeamCode:testDebugUnitTest`): one of them fails if the committed pictures do not show every route
   of the catalog.

## Red and blue

Everything is **written once, for RED**. The BLUE version is the same route turned **half a turn about the field
center**: `(x, y, heading)` becomes `(141.348 - x, 141.348 - y, heading + 180)`. It is *not* a mirror image, because the
field is rotationally symmetric. So a blue robot starts at the top of the picture, and the red HIVE's audience cell
becomes the blue HIVE's rear cell. Use `route.forAlliance(Alliance.BLUE)` to get the blue route; the diagrams and the
validator do it for you.

**Field frame** (the Pedro frame, the **wall-corner frame** of the official field CAD): inches, origin at the **inner**
corner where the audience wall meets the red wall (the face of the walls, which is what a robot touches), **+x toward
blue**, **+y away from the audience**. Heading 0 is +x and angles grow counter-clockwise. The field is **141.348 in**
across between the walls (the manual says "approximately 144 in", but its own CAD is 2.65 in smaller), so the center,
and the point the blue half turn is about, is (70.674, 70.674). The tiles are laid with a **23.5 in** pitch, not 24:
columns A-F run along x, rows 1-6 along y, A1 is the corner, and the seams between tiles are at 23.674, 47.174,
70.674, 94.174 and 117.674 in (the two outer tiles run to the walls and are 23.674 in wide), so B1 is x 23.674-47.174,
y 0-23.674. `core/TileGrid` turns a position into its tile name. A start that is "wall-referenced" (the robot touches a
wall) does not move with this frame; a number measured from the field center, a tile seam or a FLOWER does.

**The robot** has its intake at the **FRONT** and its shooter at the **BACK**, so a robot shooting at the HIVE
stands with its back toward it. The ball leaves 8.03 in behind the robot's center of rotation
(`BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN`, the rear edge according to the CAD drawing; to be measured).

**Where the outline is.** The pose of a route step (and everything Pedro and the odometry report) is the robot's **center
of rotation**: the point it spins about. The 17.80 x 17.80 in outline (every wheel included) is **not centered on it**,
because the intake sticks out in front of the front axle. The outline reaches **9.77 in in front of** that point, **8.03 in
behind** it and **8.90 in to the left and to the right** (`Robot.FRONT_EXTENT_IN`, `REAR_EXTENT_IN`, `LEFT_EXTENT_IN`,
`RIGHT_EXTENT_IN`: all VERIFY, the team measures them on day one). Everything that checks a pose uses them, so:

* with its **front edge on a wall** the robot's pose is **9.77 in** from the wall; with its **back** on the wall, **8.03 in**;
  with a **side** on the wall, **8.90 in**;
* the farthest corner is a front corner, **13.22 in** from the pose (`hypot(9.77, 8.90)`), so a robot spinning on the spot
  needs a pose at least 13.22 in from every wall;
* the picture draws the outline at its true place around the pose and marks the pose itself with a small ringed dot.

The intake is **two small wheels that lie flat** (their discs are parallel to the floor, like coins on a table) at the
two front corners, one on each side of the center line. In front of the chassis frame there is nothing else, so the
robot is drawn and checked as **three shapes** (`Footprint`, built from `RobotShape`): the whole **perimeter** (every wheel
included: used for walls, the HIVE, the start rule and the center line), the **chassis** (the body without the part where
only the wheels stick out in front, `Robot.INTAKE_REACH_IN`, so its front edge is 7.07 in ahead of the pose) and the
**two wheel discs** (2.8 in across, 6.1 in either side of the center line, `Robot.INTAKE_WHEEL_OFFSET_IN`). The reason
for the split is the FLOWER (see "Taking POLLEN from a FLOWER").

## The steps you can write

Every step is a method of the builder. The first step must be `start(...)`, and there is only one.

| Step | Builder call | What it means |
|---|---|---|
| **START** | `start(x, y, headingDeg)` | Where the robot is placed. Checked against rule G304. |
| **DRIVE** | `driveTangent(x, y)` | Follow a path to (x, y) with the **front** along the direction of travel. |
| | `driveReverseTangent(x, y)` | Same, but **backwards, shooter first** (heading is ignored, as in Pedro). |
| | `driveLinear(x, y, headingDeg)` | Turn evenly from the heading the leg starts with to the given one. |
| | `driveConstant(x, y, headingDeg)` | Turn first, then keep that heading all the way (strafing if needed). |
| | `driveFacing(x, y, px, py)` / `driveBackFacing(...)` | Keep the front / the back pointed at the point (px, py). |
| **INTAKE** | `intakeFloor(...)`, `intakeGarden(...)` | Take POLLEN: `(targetX, targetY, robotX, robotY, robotHeadingDeg, count)`. |
| | `intakeFlower(targetX, targetY, count)` | Empty a FLOWER by **wiggling one flat wheel into its bottom POLLEN** (see "Taking POLLEN from a FLOWER"). Give only the FLOWER's center and the count: where the robot stands, how far it turns, how many times and **which wheel** do the work are worked out for you (the cheaper wheel for the legs before and after). The robot goes to its line-up pose by a straight, checked move, so put a DRIVE before it if that straight line would hit something. |
| | `intakeFlower(targetX, targetY, IntakeWheel.LEFT, count)` | The same, forcing the wheel: `LEFT` or `RIGHT` (the robot's own sides), or `AUTO` (the default of the form above). |
| **SHOOT** | `shootUp(count)` | Fire at the cell that is UP; the solver picks the pose (the picture shows a nominal one). |
| | `shootUp(x, y, headingDeg, count)` | Same, from an explicit pose. |
| | `shootSide(Side.AUDIENCE, count)` | Fire at a fixed cell (a promise that it is UP at that moment). |
| **WAIT** | `waitSeconds(s)` | Do nothing for a while. |
| **WAIT_FOR_TIP** | `waitForTip(giveUpSec)` | Stand still until the camera has SEEN the HIVE tip; give up at the route time `giveUpSec`. For partner routes only (see "Who shoots the first tip"). |
| **PARK** | `park(x, y, headingDeg)` | The final pose, partly inside the own LOADING ZONE (5 points). |

Modifiers apply to the step added **just before** them:

| Modifier | Applies to | Effect |
|---|---|---|
| `.via(x, y)` | DRIVE | Adds a Bezier control point: the path bends toward it but does **not** pass through it. |
| `.speed(0.5)` | DRIVE | Drive at this fraction of the cruise speed. |
| `.label("corridor")` | DRIVE | Names the leg in the picture. |
| `.wallContact()` | DRIVE | A note that the leg touches the field wall on purpose (sweeping the GARDEN POLLEN). It gives **no** extra room: the robot may touch a wall but never push into it. |
| `.allowUnderHive()` | DRIVE | The leg may drive **under the HIVE**, along the lane between its two foot bars (see "Driving under the HIVE"). Off by default. |
| `.retreat(8.0)` | FLOWER INTAKE | How far the robot backs straight out after the last cycle of the wiggle (default 6 in; shorter is a WARN). |
| `.launchSeconds(5.0)` | SHOOT | The step takes exactly this long (instead of the estimate). |
| `.note("why")` | any | Free text for people; shown on the page. |

One more call sets the **assumption of the whole route** and goes right after `AutoRoute.builder("id")`:
`.partnerTipsFirst()` says "the alliance partner shoots the first tip". Without it the route assumes our own pre-loads
are the first tip: the 3rd of the 4 tips the HIVE (3 staged NECTAR + 3 POLLEN) and the 4th is the spare.

Two rules of thumb that the validator enforces:

* **A move is a DRIVE.** If an INTAKE, SHOOT or PARK has a pose far from where the previous step left the robot,
  the validator warns: add a `drive...` before it so the path is planned and checked.
* **Scoring.** A route claims LEAVE (3 points) when it ends with the whole robot off every wall, PARK (5) when it ends
  partly in the own LOADING ZONE, and a HIVE TIP (20) for every tip its OWN volleys cause. **LEAVE is judged at the end
  of AUTO and is lost by touching ANY wall then, also the wall inside the LOADING ZONE** (FIRST's Q&A QA21): a robot that
  parks there touching the wall keeps its PARK but not its LEAVE, so a route that wants both ends in the zone standing
  `Auto.LEAVE_WALL_MARGIN_IN` (1.5 in) or more off the wall (the zone is 11 in deep, a 17.8 in robot standing 1.5 in off its
  wall still reaches 9.5 in into it). A POLLEN squeezed between the robot and the wall does not count as the robot touching
  it (QA47), so only the robot's outline is looked at. The 3 NECTAR that sit in the UP cell at the start help: **3 POLLEN**
  tip the first HIVE (3 NECTAR + 3 POLLEN tip it, 3 NECTAR + 2 POLLEN do not: FIRST's calibration), so a volley of 4 tips it
  with 1 spare and stops at the tip, keeping the spare in the robot. The HIVE flips after a tip and the cell that comes
  up holds no NECTAR, so the next tip at the other cell needs 8 POLLEN (two volleys of 4, plan 9). See "How a HIVE tips"
  below. The partner's assumed tip (20 points for the alliance) is shown next to the route's points but is **never
  counted** in them: it is an assumption, not something our robot does.

## What the validator checks

| Check | Rule | ERROR when | WARN when |
|---|---|---|---|
| Start | G304 | outside the field (a start whose real outline pokes through a wall: the finding says how far the pose is from the wall and where the robot's edge that faces it lies), outside the own half, not touching a wall, in the LOADING ZONE, touching a FLOWER or the HIVE | |
| Field walls | 9.2, G412 | any part of the robot overlaps a wall by more than 0.1 in (rounding noise) at any moment: at the start, along every leg, while turning in place (the front corners sweep out to 13.22 in from the center of rotation), at an INTAKE, at the PARK pose and at the end. Touching is fine; no step, `wallContact` or not, may push in | |
| HIVE | G417, G409 | the robot footprint comes within 3 in of the HIVE frame footprint (the two foot bars and the box between them, 49.466 x 38.944 in). By default the gaps under and inside the frame count as solid. A DRIVE that says `allowUnderHive()` is checked against the **exact solid** instead (the foot bars and the leg stubs, with 3 in to spare on both sides): it may drive along the lane, but never across a foot bar or into a leg. A DOWN cell hangs out past the frame by up to 5.66 in, but only 35.5 in up or higher, so it is not an obstacle | a DRIVE under the HIVE while the HIVE could still tip (G409/G417, next section) |
| FLOWERS | G304.D, G415 | any part of the robot comes within 0.5 in of a FLOWER plate (4.871 x 5.95 in, its field-side edge 5.033 in from the wall; the 6.67 in long wall bracket above it counts too, see below). The only exception is the **one flat wheel** of a FLOWER INTAKE, during its wiggle (next row): the chassis, the other wheel and every other step stay out | |
| FLOWER wiggle | 9.7, G415 | at ANY pose of the straight approach, **the turn on the spot** or the retreat: the chassis comes within 0.5 in of the plate; the working wheel goes nearer the wall than the POLLEN's center line (the **press stop line**); the other wheel enters the plate; anything leaves the field. The wheel cannot REACH the POLLEN once the chassis has been stood far enough out to clear the plate through the swing (the finding says how many inches of wheel are missing). With the wheel left to `AUTO`: neither wheel is legal | the retreat is shorter than 6 in |
| Center line | G402 | the robot center crosses into the opponent half | the body ends over the line |
| POLLEN | G407 | more than 4 held (a volley that tips keeps its spare in the robot, so 4 pre-loads at the UP cell leave 1 and a GARDEN intake of 3 fits), firing more than held, taking from the other alliance's GARDEN or FLOWER | a source that is empty or that does not exist; a floor intake before anything was shot |
| Time | G401 | the estimate is over 30 s | within 2 s of the limit |
| End pose | 10.5.4, QA21 | the route ends inside the HIVE frame | the route ends in the LOADING ZONE touching (or within 1.5 in of) a wall: PARK counts, LEAVE is lost (QA21) |
| SHOOT | G417 | | pose off the 30-38 in standoff band, back more than 15 degrees off the cell, wrong side of the HIVE, or the target cell is DOWN at that moment |
| HIVE tips | G417 | in a partner route, a SHOOT at the cell that comes up with the partner's tip that has no WAIT_FOR_TIP before it, or whose wait gives up before the tip is confirmed (the assumed tip plus the 0.5 s settle time) | a volley still firing when the partner tips that cell; a TIMED volley (`launchSeconds`, an old OpMode) that goes on firing after the tip (QA12 allows adding to a moving HIVE, G417 does not like it) |
| Tip spare | QA46 | | a tip that puts in exactly the minimum (3 POLLEN into the staged cell, 8 into an empty one): a field that needs one more would not tip; plan 4 and 9 |
| Spill | G409 | the robot footprint is inside the spill zone of the cell that tipped, from the tip until the camera has confirmed it (0.5 s) plus 3 s | |
| First-tip assumption | design rule | | a WAIT_FOR_TIP in a route that shoots the first tip itself; a wait that gives up before the assumed tip is confirmed, or later than the rest of the route can afford; our own volley tipping before the partner's assumed time; less than 3 s of slack (see below); a route that would lose its LEAVE or PARK if the tip never comes (see "No camera, no shot") |
| PARK | 10.5.4 | the pose is not even partly in the LOADING ZONE | it overlaps by less than 3 in |

Every finding names its step number (the same number as in the picture) and quotes the numbers: how far through the
wall, where the footprint first enters the keep-out, how far the shot is from the cell.

**Nothing can slip between two samples.** A computer cannot test every one of the infinitely many poses of a leg, so
the validator tests the robot every 1 in along the path and every 5 degrees of turning. The farthest corner of the robot is
13.22 in from its center of rotation, so between two samples a corner can still move up to 1 + 13.22 x 5 degrees (in
radians) = 2.15 in, and a thin post could stand in the strip it sweeps. So the validator measures how far the robot is from
each obstacle (a plain distance) and grows every obstacle by half of that 2.15 in: if neither of two neighbouring
samples reaches the grown obstacle, nothing between them can touch the real one. Wherever it cannot prove the gap is
clear it cuts the gap in half and looks again, down to 1/1000 in. The result is the same as sampling infinitely
densely; the proof is the comment at the top of `routes/Sweep.java` and `SweepTest` tries to break it. The first and
the last pose of every leg are always tested, and so is the last pose of a turn in place.

## Driving under the HIVE (the lane)

Below the robot's height (13 in with the camera) the HIVE is **only its frame**: two **foot bars** on the floor (1.98 in
wide, 38.9 in long, 2.15 in tall: too tall to drive over, the chassis has 0.94 in of ground clearance) and four **legs**
that lean inward as they climb, so that below 13 in each leg is only a short slanted **stub** at the end of a bar. The
cells themselves are 30.65 in up or higher, and the crossbar 41 in. So between the feet the floor is **open**: a lane a
12 to 13 in tall robot can drive along, north-south, **under** the cells. In the wall-corner frame, for a 12.13 in tall
robot:

```
  x 45.9   47.9                      50.6        90.7                    93.4   95.4
     | bar |   ------ lane ------    |  stubs     |     lane at the stubs    | bar |
     +-----+   45.5 in wide between the bars                 40.1 in wide where the leg stubs narrow it
```

The official CAD gives these numbers and a second derivation reproduced them to 0.02 in (a 29 in tall robot would have a
30.9 in lane). `core/HiveSolid` builds the exact shape from them (`HiveSolidTest` checks it against the CAD).

**Why it is NOT the default.** It saves a trip of about **20 in** (below), but G409 says a robot may not catch what a tipped
HIVE releases and G417 forbids launching into a cell that is tipping, and a robot under a HIVE that is tipping is exactly
where nobody wants to be: the cells swing through 60 degrees over its head. So by default the whole footprint is solid and
every route goes **around** it.

**How to allow it.** Put `.allowUnderHive()` after the DRIVE that goes through the lane:

```java
.driveConstant(64.0, 38.0, -90).label("to the lane")
.driveConstant(64.0, 105.0, -90).label("the lane").allowUnderHive()   // x = 64: 6.7 in left of the center line
```

* The leg is checked against the foot bars and leg stubs with `Auto.HIVE_LANE_MARGIN_IN` (3.0 in) to spare on **both**
  sides. For a robot facing -90 or +90 (side edges 8.9 in from the center) the center must stay between x = 62.8 and 78.6 in;
  G402 keeps the center of a RED robot on its own half (x up to 70.674), so the lane is **7.9 in wide for the center of a red
  robot**, x 62.8 to 70.7 (blue: the half turn of it). A leg across a foot bar, or too close to a stub, is still a HIVE ERROR,
  and its message says how wide the lane is.
* The flag changes only that leg's HIVE check. Walls, FLOWERS, the center line and the end-pose rule apply as always, and
  a route may never **end** or **start** under the HIVE.
* **The WARN.** The checker WARNs (rule `UNDER_HIVE`, G409/G417) when the robot is over the HIVE frame while the HIVE could
  still tip. The HIVE is known to be at rest only after the camera has confirmed a tip fully (a `waitForTip` ends exactly
  then) and until the next volley of ours goes into it; so put a `waitForTip` before the lane leg with no SHOOT between them.
  A tip assumed during the move is named in the WARN.
* **The TeleOp planner** has the same option, `AutoShoot.ALLOW_UNDER_HIVE` (false by default, TUNE). With it on, the planner
  treats only the exact foot bars and leg stubs as obstacles and falls back to the way around when the lane route cannot be
  driven safely, so the flag can never make a trip impossible. The planner does not know whether the camera has confirmed the
  HIVE at rest, which is the reason it is off.

**The shorter loop, in numbers** (placeholder speeds; both printed by tests):

| Trip | Through the lane | Around the frame | Saved |
|---|---|---|---|
| A route author's legs, from the audience start (58, 9.77) to a pose beyond the HIVE (58, 125) (`HiveLaneTest`) | 116.7 in, 5.8 s | 139.4 in (the left corridor at x = 30), 6.4 s | 22.7 in, 0.6 s |
| The TeleOp planner, from the C1 start to the shooting pose in front of the red REAR cell (`PlannerLaneTest`) | 115.8 in | 148.5 in | 32.7 in |

That is the loop of a partner route (the robot must get from the audience side to the cell that comes up on the rear side
after the partner's tip): about 0.6 s at the placeholder cruise speed of 40 in/s, which is a tenth of the 6 s a 3-POLLEN
FLOWER pick-up is estimated to take. Worth having if the HIVE is known to be at rest; not worth a foul. Day one: measure the real
lane with a tape before the first run (`docs/DAY_ONE_CHECKLIST.md`).

## Taking POLLEN from a FLOWER (the wiggle)

The bottom POLLEN of a FLOWER sits in the hole of a ring that is about 0.4 in tall (0.354 in in the official CAD, about
0.43 in in the manual). The hole is 3.04 in wide in the CAD and 2.79 in in the manual: nobody has measured a real one, so
every decision about how much room the ball has uses the **smaller** figure (a decision that works with 2.79 in also works
with 3.04 in). To get the ball out, something has to roll it **up and over the edge of that ring** (the "lip"). Our two flat
wheels are 12.2 in apart, so they cannot pinch a 2.8 in ball. **ONE wheel does it, with a small oscillation, a "wiggle"**:
the spinning wheel and the push of the whole robot together roll the ball out, then the robot turns back so the next ball
can drop down into the hole. In order:

1. **Approach.** The robot lines up **facing the wall** and drives **straight in, slowly** (30 % speed, 6 in) standing
   **off center**, so that one flat wheel ends up **beside** the bottom POLLEN, not on it: the middle of the wheel is
   3.8 in from the middle of the ball along the wall (`Auto.FLOWER_WHEEL_START_OFFSET_IN`: the two radii plus 1.0 in of air),
   with the ball between the wheel and the robot's center line (`Auto.FLOWER_WHEEL_BALL_INBOARD`).
2. **Push.** It **turns on the spot** about its pose point (the point it spins about) **toward the ball**. The wheel is about
   10.4 in from that point, so a turn of a few degrees swings it sideways, and a little forward, into the ball, until the
   middle of the wheel is 1.8 in from the middle of the ball (`Auto.FLOWER_WHEEL_PRESS_OFFSET_IN`: it overlaps the ball by
   1.0 in). It holds there for 0.25 s with the wheel spinning (`FLOWER_WIGGLE_PUSH_HOLD_SEC`).
3. **Release.** It turns **back the same angle, never past straight**, and holds for 0.4 s so the next POLLEN can drop
   (`FLOWER_WIGGLE_RELEASE_HOLD_SEC`).
4. **Repeat** once per POLLEN wanted (`FLOWER_WIGGLE_CYCLES_PER_POLLEN`, at most `FLOWER_WIGGLE_MAX_CYCLES`), then **back
   straight out** 6 in.

Top view of what the working wheel does (the wall at the bottom, the robot facing it, wheels that stick out 4 in so that the
wiggle works; distances from the wall in inches). `(L)` is the middle of the LEFT flat wheel, `( o )` the bottom POLLEN, 2.8 in
wide with its middle 2.634 in from the wall. Not to scale:

```
 5.4 in                                  (L)    start of the wiggle: the wheel is 3.8 in to the side of the ball's middle,
                                          :                         the robot faces the wall, the chassis is 0.5 in off the plate
                                          :
                                          : the robot turns about its pose point (10.4 in from the wheel),
                                          :  17 degrees toward the ball: the wheel sweeps sideways and a little forward
 4.0 in                  (L)  . . . . . .'     end of the push: 1.8 in from the ball's middle, 1.0 in into the ball
                       ( o )                   the ball rolls out over the lip toward the robot (toward the top of the page)
 2.6 in
 ======================== wall ==========================================================================
```

You write only `intakeFlower(2.634, 47.281, 4)` (the center of the FLOWER on the red wall and the count). **Everything else is
derived from the constants**, so nobody computes it by hand:

* **The turn angle.** The middle of the wheel is the **lever arm** from the pose point, `hypot(how far ahead of it the wheel
  is, how far to the side)` = hypot(8.37 in, 6.1 in) = **10.36 in**, so while the robot turns in place the wheel travels
  on a circle of that radius. The turn is where that circle first gets within the press offset of the ball: the meeting
  point of two circles, solved exactly (`FlowerWiggle`). It always carries the wheel **toward** the ball: clockwise for the
  left wheel, counter-clockwise for the right one. It never exceeds `Auto.FLOWER_WIGGLE_MAX_TURN_DEG` (20 degrees).
* **The standoff (how far from the wall the robot stands).** A turning rectangle's corner moves toward the wall: when the robot turns,
  the corner of its body nearest the plate **swings toward it**. So the robot has to stand **farther out** than a straight
  press would need. The program finds the smallest distance for which the chassis keeps `Auto.FLOWER_CHASSIS_CLEARANCE_IN`
  (0.5 in) off the plate at **every pose of the swing**, the working wheel never goes nearer the wall than the POLLEN's center
  line (2.634 in, the **press stop line**), and nothing goes through the wall. With wheels that stick out 4 in that is 13.8 in
  from the wall to the pose point; the line-up pose is 6 in farther out (19.8 in). It uses the **same rules the validator uses**
  (`PressRules`), so the validator accepts what it derives.
* **Whether the wheel can still reach the ball.** Standing farther out costs reach. **With the numbers of the goBILDA drawing
  (wheels that stick out 2.7 in) the wheel falls 0.77 in short**: the validator says so as an ERROR, with the missing inches:
  *"the flat wheels would have to stick out 3.5 in in front of the chassis, 0.8 in more than they do"*. What helps is longer wheels,
  or a smaller `Auto.FLOWER_CHASSIS_CLEARANCE_IN`, or a smaller `Auto.FLOWER_WHEEL_START_OFFSET_IN` (a smaller swing): **measure the
  real robot** (the day-one checklist) before believing this ERROR, it follows from placeholder numbers. (If the wheel would
  have to turn more than 20 degrees, no length of wheel helps and the finding says so.)
* **Which wheel.** `intakeFlower(x, y, count)` leaves the wheel to `IntakeWheel.AUTO`; `LEFT` or `RIGHT` forces it. The program
  plans the whole pick-up for each wheel and **keeps only the legal ones** (the wheel reaches, nothing breaks a rule on the
  move in, the approach, the swing or the retreat, nor the HIVE or the center line). Among those it takes the **cheapest**:
  *the move from where the robot is to the line-up pose + the pick-up itself + the move from the retreat pose to the first
  pose of the next step* (one step ahead is looked at). A tie goes to the wheel with more room, then to the LEFT one. One shared
  function (`WheelChoice.choose`) decides, and the validator, the diagram, the timing and the robot's runner all use its answer.
  The gallery says why: *"chosen automatically: 0.1 s shorter than the LEFT wheel"*.
* **The time.** `approach + cycles x (turn + push hold + turn back + release hold) + retreat`: with the placeholder numbers
  3 POLLEN take about 6.1 s (2 x 1.0 s of slow driving, 3 x 1.35 s of wiggle). The gallery text reads *"FLOWER pick-up: 3 POLLEN
  in about 6.1 s with the RIGHT wheel"*.

What the validator holds you to (the numbers are `BioBuzzConstants`, the ones in the table below are marked **ADD** because
nobody has measured them on the robot yet):

* **The chassis never comes within 0.5 in of a plate**, at any pose of the approach, the swing or the retreat, and **only the one
  working wheel may be over the plate**, never nearer the wall than the POLLEN's center line. Every pose of the turn is swept
  with the real footprint, not only the two ends.
* **The wheels must be long enough** (above): ERROR, with the missing inches. With the wheel left to AUTO the ERROR comes only
  when **neither** wheel is legal.
* **Mind the wall bracket.** The FLOWER is held to the wall by a black bracket that sticks out 1.67 in from the wall at 11.44
  to 11.84 in above the floor. That is inside the robot's 12.13 in, so any part of the robot that is taller than 11.44 in
  and within 1.67 in of the wall can touch it. The bracket is 6.67 in long, 0.36 in longer than the plate at each end: the
  0.5 in keep-out margin around the plate covers that for driving, and a start that stands clear of the plate but over the
  end of the bracket gets a WARN (G304.D).
* **Straight, slow in and out.** The approach is square to the wall and the retreat is straight: they are **derived**, so they
  hold by construction (`Auto.FLOWER_APPROACH_LENGTH_IN`, `Auto.FLOWER_APPROACH_SPEED_FRACTION`, `Auto.FLOWER_RETREAT_LENGTH_IN`; a
  retreat shorter than 6 in is a WARN). The line-up pose is reached by a planned, checked straight move.
* **Walls and the HIVE still count in every one of these moves.** Nothing of the robot goes into a wall, and the
  body keeps out of the HIVE keep-out, during the wiggle as everywhere else.

**On the robot.** `FlowerWiggleSequence` is the script the route runner follows once the robot stands at the start pose: the
follower HOLDS the pose point and only the **target heading** alternates between "straight at the wall" and "turned toward
the ball". A turn ends when the heading is within `Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG`, the holds are timers, the POLLEN
count comes from the cycles done (never above 4, rule G407: the intake stops at 4), and it **fails closed**: if a turn does not get
there within `Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC` it turns back, backs out and skips the rest of that pick-up. It is tested in a
closed loop against a simulated heading controller (`FlowerWiggleSequenceTest`), also with an overshooting one and with a stuck one.

In the diagram a FLOWER INTAKE draws the robot where the wiggle **starts** (the working wheel has a green ring), a faint
second outline where the push **ends**, a **curved arrow** for the sweep of the wheel, a small label such as `wiggle x4`, a short
solid arrow for the straight approach and a dotted arrow for the retreat.

## How a HIVE tips

**FIRST's own calibration, not the manual.** The game manual never says how much it takes to tip a HIVE, but FIRST's field
documents do (the Event Field Setup Guide section 12.3, the Field Acceptance Checklist 2.3 to 2.6 and the Field Mitigation
Guide): every cell of every field is calibrated so that

* an **empty cell tips with the 8th POLLEN** (7 do not), and
* a cell holding the **3 staged NECTAR tips with the 3rd POLLEN** (3 NECTAR + 2 POLLEN do not).

To plan with those two facts the code counts a POLLEN as 1 and a NECTAR as **5/3 POLLEN** (`Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR`
and `_DENOMINATOR`, VERIFY), and a cell tips when what it holds adds up to **8 POLLEN-equivalents**
(`Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD`, VERIFY). A fraction like 5/3 is not exact as a decimal, so `TipModel` multiplies by 3 and
works in thirds of a POLLEN with whole numbers only:

```
NECTAR x 5  +  POLLEN x 3  >=  8 x 3 = 24
```

| The cell holds | In thirds | Tips? | FIRST's table |
|---|---|---|---|
| 3 NECTAR + 2 POLLEN | 15 + 6 = 21 | no | 3 NECTAR + 2 POLLEN must not tip |
| 3 NECTAR + 3 POLLEN | 15 + 9 = 24 | **yes** | the 3rd POLLEN must tip it |
| 7 POLLEN | 21 | no | 7 must not tip an empty cell |
| 8 POLLEN | 24 | **yes** | the 8th must tip it |
| 5 NECTAR | 25 | yes | (not official; the team's rule of thumb) |
| 4 NECTAR | 20 | no | (not official) |

Any NECTAR weight from 5/3 up to (not including) 2 POLLEN agrees with the official table; 5/3 is the lightest, and it also
gives the team's "5 NECTAR tip, 4 do not". Weigh your own balls (day-one checklist). `ConstantsCheck` refuses constants that do
not reproduce the table, and `TipModelTest` pins all six cases. (The first version of this planner used `NECTAR / 5 + POLLEN / 8 >= 1`, which
said that 3 NECTAR + 3 POLLEN do **not** tip and wanted 4 POLLEN for the first tip: that was wrong.)

* **The first tip needs 3 POLLEN.** The UP cell holds 3 NECTAR when the match starts (Fig 10-2), so the 3rd POLLEN tips it. The
  robot starts with 4 pre-loads, so **a route that fires its 4 pre-loads at the UP cell tips it with 1 spare**: the volley
  stops at the tip (the camera sees the cell's tags vanish and the feeder pauses, G417) and the 4th POLLEN **stays in the
  robot**, which can then take 3 more before the next volley (capacity 4, G407). A volley written for exactly
  3 POLLEN has no spare and gets a WARN (below).
* **Every later tip needs 8 POLLEN.** After a tip the OTHER cell is the UP one, and Fig 10-2 shows NECTAR only in the cell that is
  UP at the start, so the new UP cell is **empty**: it needs 8 POLLEN, two volleys of 4, and the robot can hold only 4 POLLEN at a
  time (G407), so it has to fetch more in between. A second volley of 4 at the SAME side as the first does not tip anything: that
  cell has just tipped DOWN.
* **Plan one POLLEN more (the spare).** A real field drifts: FIRST's own Q&A QA46 tells of a HIVE that needed 9 POLLEN instead of 8
  after 8 to 10 matches. So a route plans `Auto.TIP_PLANNING_MARGIN_POLLEN` (1, TUNE) POLLEN more than the minimum, **4 for the first
  tip and 9 for a later one**, and the validator WARNs for a tip that puts in exactly the minimum. The gallery says how many POLLEN a
  route has to spare ("1 POLLEN to spare, so a field that needs 1 more still tips" or "NO spare"), and the analysis reports it
  (`StepInfo.tipNeeded`, `tipSpare`, `RouteAnalysis.minimumTipSpare()`). 4 + 4 into an empty cell is exactly 8: no spare. 4 + 3 + 2 puts in 9.
* **A timed volley cannot stop.** A volley with `launchSeconds(...)` (an old OpMode feeds for a fixed time) cannot stop when the camera
  sees the tip, so the POLLEN still to be fired after the tip go into a cell that is already tipping down. FIRST's Q&A QA12 says elements
  may be added to a HIVE that is moving, but G417 allows launching only into an upward-facing CELL, so the validator WARNs. A volley
  without `launchSeconds` is gated by the camera and stops at the tip.
* **Our tips and the partner's tip.** `RouteAnalysis.tips()` and the points count only the tips OUR volleys cause. Our volleys and
  the partner's assumed tip are tracked separately, so a plan can never claim the partner's 20 points as its own. **A HIVE belongs to
  the alliance and a TIP is defined only by what the HIVE does** (the cells swap and the damper touches the frame; no rule counts
  launches per robot), so the partner's launches count for our HIVE and the partner-tips-first assumption is legal. The planner counts only OUR
  POLLEN toward our own tips: a partner that also launches into the same cell would only make ours easier.
* **AUTO scores only three things:** LEAVE 3, PARK 5 and a HIVE TIP 20 (Table 10-2). Everything else (the POLLEN left in a cell,
  the GARDEN, the FLOWERS) is end-of-match, so our pick-ups from a GARDEN or a FLOWER in AUTO are only for feeding POLLEN into a second tip.
* **Where the spill falls (G409).** When a HIVE tips, the cell that was UP turns down and everything in it (the 3
  NECTAR and the POLLEN launched into it) falls out of its opening onto the floor on that side of the HIVE. For the
  red HIVE the audience cell covers x 47.4 to 68.5 in and y 50.6 to 69.1 in (the CAD hull of the whole cell) and opens toward the audience wall. G409
  says a robot may not catch or deflect what a tipped HIVE releases, so for `Auto.TIP_SPILL_CLEAR_SEC` (3 s) after
  the camera has confirmed every tip the robot footprint must stay out of that cell's spill zone: the cell footprint grown by
  `Auto.TIP_SPILL_MARGIN_IN` (6 in). The other side of the HIVE is clear: that is where a robot waiting for the
  partner stands. On the real robot the rule is "stay out until the camera has confirmed the tip, plus those 3 s"
  (`TipConfirmation.spillClearAtSec()`); the checker's time-based version (the tip, the 0.5 s settle time, the 3 s) is for planning only.
* **G402 (AUTO sides).** The foul is disrupting the other alliance (controlling their elements or touching their robot on
  their side); merely crossing the center line is risky, not itself a foul. The validator keeps the stricter rule (the robot
  center stays on the own side), which is a safe choice.

## Who shoots the first tip

The HIVE belongs to the alliance, so our partner robot can tip it too. A route is written under one of two
assumptions, and the picture and the gallery print it for every route (`first tip: ...`) so the drive team can pick
the auto that matches what the partner's auto does.

| Assumption | Builder | What it means |
|---|---|---|
| **We shoot the first tip** (`FirstTip.SELF`, the default) | nothing | The 3rd of our 4 pre-loads, with the 3 staged NECTAR, is the first tip: ONE volley, and the 4th pre-load is the spare. Written in `SelfTipRoutes.java`. |
| **Partner tips first** (`FirstTip.PARTNER`) | `.partnerTipsFirst()` | The partner's volley is the first tip. The HIVE tips by itself at `Auto.PARTNER_FIRST_TIP_DONE_SEC` (12 s, VERIFY: agree the real time with the partner team) and the OTHER cell comes up, with no NECTAR in it. Our robot goes to that side. Written in `PartnerTipRoutes.java`. |

For the red alliance the UP cell at the start is on the audience side, so a partner tip brings the REAR cell up (for
blue it is the other way round). A partner route is therefore best started in the rear half of the field, close to
that cell. Our own volleys into the new cell need 8 POLLEN for the next tip (plan 9: two volleys of 4 and a spare, with a trip
to a FLOWER in between, because the robot holds only 4). The partner's launches count for our HIVE (it belongs to the alliance),
so this assumption is legal; the 20 points are the alliance's and are never counted in ours.

**The camera decides, never the clock.** Partner teams tip at different times, so the robot never shoots because
"12 seconds have passed". The step that makes it wait is `waitForTip(giveUpSec)`:

* The robot stands still until its camera has **seen** the cell UP and steady (the AprilTags on the bottom of the
  HIVE in view). Only then does it go on to the volley after the wait. "Fully tipped" means all of: the target cell's cluster is
  in view, the analyzer reads it as UP with a full geometry (range and bearing), the tracker believes it **from the camera** (the
  driver's toggle and the "tags never seen, assume the other side" flip do not count in AUTO: `HiveTracker.cameraConfirmed()`),
  and the cluster has been seen without a break for the **settle time** `Vision.HIVE_STABLE_SEEN_SEC` (0.5 s), so a HIVE that is
  still rocking is not shot at (a TIP is complete only when the damper touches the frame). That is `core/TipConfirmation`, and
  `core/TipWait` adds the deadline below. The plan includes the settle time: a wait ends 0.5 s AFTER the tip.
* `giveUpSec` is a **time on the route clock** (seconds since AUTO started), not a length of time and not a guess of
  the tip time. When the clock reaches it and the tip has not been confirmed, the robot skips the volleys that need the
  tip and carries on with the safe steps (LEAVE, PARK). So `waitForTip(22.0)` means "stop waiting at 22 s". The robot's real
  deadline is the smaller of that and the latest time the rest of the route allows (`RouteAnalysis.effectiveGiveUpSec`); a time can
  only END the wait, never allow a shot, and a tip confirmed after the deadline does not count.
* **No camera, no shot:** if the camera is missing, does not deliver frames, is stale or runs the wrong pipeline, nothing can be
  confirmed and the robot does not shoot any volley that depends on a tip: it waits until the deadline, skips it and carries on. At
  INIT the Auto Route Picker says so (`TipCameraPolicy`): a partner route is **refused** when the Limelight is not working, with the
  reason in plain words; a route whose later volleys depend on its own tip gets a warning.
* The checker insists on it: a SHOOT at the cell that comes up with the partner's tip **must** have a
  `waitForTip` before it, even if the SHOOT would start after the assumed 12 s, and the wait must last until the tip is confirmed.
* The same camera rule covers OUR OWN tips: the volley at the cell that came up with our own earlier tip waits at its pose until the
  camera has seen that cell steady (`StepInfo.confirmWaitSec`), and is skipped with no camera.

**How late may the partner be? (slack).** The route estimate still has to guess when the partner tips (12 s), to
count time and points. For every partner route the analysis also computes the **latest partner tip that still
works**: the latest time T so that the camera can have confirmed the tip (T plus the 0.5 s settle time) before the robot gives
up waiting, and the whole route, LEAVE and PARK after the volleys included, is still over by the AUTO limit minus the 2 s safety
margin (28 s). The gallery shows it next to the assumption, and the **slack** is T minus the assumed time. A big slack means a slow
partner is fine. Below `Auto.PARTNER_TIP_MIN_SLACK_SEC` (3 s, TUNE) the checker warns: "this route only works with a partner that
is about as fast as agreed"; with a negative slack the route does not work even at the assumed time. The same page shows, on the
WAIT_FOR_TIP step, the latest give-up time that the rest of the route allows: write that number, or a little less, into
`waitForTip(...)`.

**No camera, no shot: what the route does then.** For every route that has a volley which needs a confirmed tip,
`RouteAnalysis.giveUpPlan()` plays the route as the robot does it when the tip never comes (the partner is late or never tips, or the
camera is dead): the wait lasts until its deadline, the tip-dependent volleys are left out, an intake takes only what fits in the
robot (G407), and the rest goes on. The gallery shows it ("If the tip is never confirmed": where it ends, LEAVE and PARK, points) and the
checker WARNs when that plan loses a LEAVE or PARK the normal plan earns: **put the safe steps (the drive off the wall and the
PARK) after the volley**, so they are done whether or not the tip comes.

## Reading a diagram

* **Top view**, the audience wall at the **bottom**, the red alliance on the **left**. Tiles are labelled A-F and 1-6.
* The route is drawn in the alliance's **ink**: red ink for the red alliance, blue ink for the blue one.
* The **robot is drawn to scale**, in two tones: the **body** is tinted in the alliance ink and the **two flat intake
  wheels** are the dark discs at the front. Its **FRONT edge (the wheel tips) is green** and its **BACK edge (shooter)
  is orange**. The small **ringed dot** is the robot's **center of rotation**, the point the route's poses refer to; it is
  9.77 in behind the front edge and 8.03 in in front of the back edge, so it is not in the middle of the outline. The START footprint has an arrow along its heading. A robot drawn with an **amber outline** breaks a
  rule (an ERROR) at that step.
* **DRIVE legs** are solid lines or curves with arrowheads. **Dotted arrows** are the moves to a pose that a step makes
  by itself (a SHOOT pose, an INTAKE pose, a PARK pose).
* Every step has a **numbered circle**, in route order. A red triangle on it is an ERROR, a yellow one a WARN.
* **INTAKE** is a dashed circle around the target, labelled like `INTAKE GARDEN +4`. A FLOWER INTAKE shows the robot where
  the wiggle starts (the working wheel ringed in green), a faint second outline where the push ends, a curved arrow for the
  sweep of the wheel, a label like `wiggle x4`, a short solid arrow for the straight approach and a dotted retreat arrow.
  The step list on the page says `FLOWER pick-up: 4 POLLEN in about 6.1 s with the LEFT wheel` and why that wheel.
* **SHOOT** is the robot at the shooting pose and a dashed arrow from the **ball exit** to the **aim point** in the
  cell opening, labelled `SHOOT 4`. A dashed footprint means the pose is the solver's *nominal* pose. `HIVE TIP +20`
  marks the volley that tips the HIVE.
* **WAIT** is a small clock with its seconds; **WAIT FOR TIP** is the same clock with "give up at ..." (the route time
  at which the robot stops waiting). **PARK** is a bold dashed outline of the final footprint. A route that
  ends on a DRIVE gets an `END` outline instead.
* The line under the title names the route, its start tile and **who shoots the first tip**; for a partner route it
  also gives the latest partner tip that still works.
* The **UP cell** of each HIVE at the start of the match is solid and filled, the **DOWN cell** dashed; both show their
  AprilTag IDs. The hatched half is the opponent's: the robot center must not go there in AUTO (G402).
* The footer shows the estimated time, the POLLEN balance, **our** AUTO points (LEAVE, PARK and our own tips), the
  partner's assumed tip "(not ours)" when the route has one, and the first findings. The gallery page has all of
  them with their full text, and works out the tip arithmetic of every volley (for example `3 NECTAR x 5/3 + 3 POLLEN = 8
  POLLEN-equivalents, the 8 that tip a cell, so the HIVE tips on POLLEN 3 of the 4 and the volley stops there; the last 1 stays in
  the robot as the spare`).

## About the numbers

* **Times are placeholders.** Nobody has measured the robot's cruise speed, acceleration, turn rate or shooting
  times yet. They live in `BioBuzzConstants.Auto` marked **ADD** (and are listed among the placeholders the TeleOp
  prints at INIT): `CRUISE_SPEED_IN_PER_SEC`, `ACCEL_IN_PER_SEC2`, `TURN_RATE_DEG_PER_SEC`, `SETTLE_SEC`,
  `INTAKE_DWELL_*_SEC`, `AIM_SPINUP_SEC`, `TIME_PER_SHOT_SEC`. Compare routes with them, but do not trust a route
  that is within a few seconds of the limit. When you measure one, replace it, change its marker to VERIFY or TUNE
  and delete its name from `BioBuzzConstants.PLACEHOLDERS`, then regenerate the pictures.
* **The FLOWER wiggle numbers are placeholders too** (marked **ADD**): how far the wheel tips stick out in front
  of the chassis (`Robot.INTAKE_REACH_IN`, 2.7 in from the drawing), how far each wheel is from the center line
  (`Robot.INTAKE_WHEEL_OFFSET_IN`, 6.1 in), how far beside the ball the wheel starts (`Auto.FLOWER_WHEEL_START_OFFSET_IN`) and how
  close it gets (`Auto.FLOWER_WHEEL_PRESS_OFFSET_IN`), which side of the wheel the ball is on (`Auto.FLOWER_WHEEL_BALL_INBOARD`), the holds
  and the cycles (`Auto.FLOWER_WIGGLE_*`), and how far the chassis stays off the plate (`Auto.FLOWER_CHASSIS_CLEARANCE_IN`). **With the
  numbers of the drawing the validator reports that the wheel falls 0.77 in short of the ball** (see "Taking POLLEN from a
  FLOWER"): that is the finding to check first with the real robot. They are on the day-one checklist (section C); measure
  them with the real robot at a real FLOWER, both wheels.
* **The tip figures** (`Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR` / `_DENOMINATOR` = 5/3 and `Auto.TIP_POLLEN_EQUIVALENT_THRESHOLD` = 8,
  VERIFY) come from FIRST's calibration (see "How a HIVE tips"), not from the manual. `Auto.TIP_PLANNING_MARGIN_POLLEN` = 1 (TUNE) is
  the spare a route plans. `Auto.NECTAR_STAGED_IN_UP_CELL` = 3 is FIXED (Fig 10-2).
* **The partner numbers** are guesses to be replaced: `Auto.PARTNER_FIRST_TIP_DONE_SEC` = 12 s (VERIFY: agree it with
  the partner team), `Auto.TIP_SPILL_CLEAR_SEC` = 3 s and `Auto.TIP_SPILL_MARGIN_IN` = 6 in (VERIFY: watch a real tip),
  `Auto.PARTNER_TIP_MIN_SLACK_SEC` = 3 s (TUNE). The settle time before the robot believes a tip is complete is
  `Vision.HIVE_STABLE_SEEN_SEC` = 0.5 s (TUNE), the same constant the tracker uses for "tags seen steadily".
* **Rule limits** (30 s, 4 POLLEN, the 3/5/20 points) are FIXED from the manual. The safety margins (3 in around the
  HIVE, 2 s before the end, the standoff band) are TUNE or VERIFY.
* The look of the diagrams (scale, inks, tile colours, label sizes) is in `BioBuzzConstants.Auto` with the
  `RENDER_` prefix.

## What the validator says about today's autos

(For the current list open `docs/autos/index.html`; this is the situation with the robot's outline reaching 9.77 in in
front of its center of rotation, 8.03 in behind it and 8.90 in to each side.)

**All three old OpModes start with the robot's center 9.2 in from the wall** (`RedAlliance_B1` and `RedAlliance_C1` at
y = 9.2, `RedAlliance_LeavePark` at x = 9.21). That number is from before the center of rotation was known. The real front
edge is 9.77 in from the center of rotation, so a robot placed there would reach **0.57 in through the wall**: every
conversion therefore has a start ERROR (G304.A), and the first leg that leaves the wall has a wall ERROR too. The note on the
START step of each route says it: **place the real robot with its FRONT edge ON the wall** (center 9.77 in out) and change
the OpMode's start y (or x) to match once the real center of rotation is measured. (The OpMode comments say the front was
"about 0.3 in from the wall", which would mean a front extent of about 9.5 in: the measurement decides.)

* **`red_leave_park`** has only that start finding (the start and its first leg 0.6 in through the red wall). It still
  earns LEAVE and PARK.
* **`red_c1`** and **`red_b1`** feed for a fixed 5 s (`launchSeconds(5.0)`, as the old OpModes do): the HIVE tips on the 3rd of
  their 4 POLLEN and the 4th goes into a cell that is already tipping down, so each has a WARN for it. A camera-gated volley
  (without `launchSeconds`) would stop at the tip and keep the 4th.
* **`red_c1`** drives its first leg, straight from (58, 9.2) to (62, 118.1), **through the HIVE frame**. The route cannot
  work as written. At y = 9.2 its shot is also 38.3 in from the aim point, just outside the 30-38 in band (placed with the
  front on the wall, 37.7 in: the measured shot).
* **`red_b1`** turns to -115 degrees right against the audience wall (a corner pokes **3.4 in through the wall**: with its
  center 9.2 in out the robot needs 12.6 in to turn that far), shoots from 44 in (the good band is 30-38 in), drives so close
  to the HIVE that the robot is inside the 3 in keep-out (the numbers were written for a narrower robot), and parks only
  **2.9 in** into the LOADING ZONE.
* **`wall_shoot_park`** and **`garden_cycle`** are new demonstration routes that pass: both start with the **front edge on the
  audience wall** (`start(x, 9.77, -90)`, written with `Robot.FRONT_EXTENT_IN`). The first shoots the pre-loads (the first
  tip: 3 POLLEN, the 4th stays in the robot as the spare), leaves through the left corridor and parks deep in the zone, standing about
4 in off the red wall so that it keeps its LEAVE (28 points); the second shoots the pre-loads (the first tip, same spare), sweeps up
3 of the 4 GARDEN POLLEN (the robot already holds 1; standing 0.8 in off the wall) and steps off the wall (23 points). It does not
shoot the GARDEN POLLEN: the UP cell is on the far side of the HIVE after the tip and needs 8 POLLEN (plan 9).

One thing to know about a start with the front exactly on the wall: the robot cannot turn toward that wall at all, not even
a degree, because the corner that leads would poke through (0.1 in is forgiven as measurement noise). A route that begins
with a reverse-tangent leg, which turns the robot a degree or two while it leaves, should start a few tenths of an inch off
the wall (the start check allows 0.75 in), or begin with a SHOOT that the solver plans.

These are findings about the **route data**. Whether the old OpModes should be corrected is the team's call; the
catalog deliberately does not "improve" them.

## How the pieces fit

| Package | What is in it |
|---|---|
| `autoshoot.routes` | The data model (`AutoRoute`, `Step`, `DrivePath`, `HeadingMode`, `IntakeSource`, `ShootTarget`, `FirstTip`), the simulation (`RouteAnalysis`, `TimingModel`, and `TipModel` for the tip arithmetic), the checks (`RouteValidator`, `TipRules`, `Sweep`, `LegSweeps`, `PressRules`, `Issue`), what the Auto Route Picker says about the camera at INIT (`TipCameraPolicy`), the field geometry (`AllianceField`, `ShotGeometry`), the FLOWER wiggle (`FlowerWiggle`: the derived turn, standoff and reach; `FlowerPress`: one planned pick-up; `WheelChoice`: the one function that picks the wheel; `FlowerWiggleSequence`: the robot-side script, simulated in closed loop) and the lists of routes: **`SelfTipRoutes`** (we shoot the first tip) and **`PartnerTipRoutes`** (the partner does), gathered by `RouteCatalog`. The robot's own shape (`RobotShape`: the four distances from the center of rotation to its edges; `Footprint`: perimeter, chassis, flat wheel discs; `IntakeWheel`) lives in `autoshoot.core`, together with the robot's camera-only decision that a HIVE has fully tipped (`TipConfirmation`, `TipWait`, `HiveTracker.cameraConfirmed()`). |
| `autoshoot.render` | `RouteSvgRenderer` (one route to one SVG), `RouteGallery` (the page), `RenderRoutesMain` (the command line), `SvgWriter` (escaping and number formatting). |
| `tools/` | `render-autos.sh` and `render-autos.bat`. |
| `docs/autos/` | The generated page and the SVG files. |

`BioBuzzConstants` and the packages `autoshoot.core`, `autoshoot.routes` and `autoshoot.render` are **plain Java**: they
must not import the FTC SDK, Android or Pedro, and must use only the Java 8 library. That is what lets the scripts
compile them with a bare `javac`, and a unit test (`PureJavaBuildTest`) compiles them exactly that way, so a stray import
fails the build instead of the script.

The validator, the timing estimate, the points and the pictures all read **one** simulation (`RouteAnalysis`), so
they can never disagree with each other.

## If something goes wrong

| Symptom | Cause and fix |
|---|---|
| `javac was not found` | Install a JDK (8 or newer) or set `JAVA_HOME` to one. Android Studio's bundled JDK works. |
| `route id '...' cannot be a file name` | Ids become file names: use only letters, digits, `_` and `-`, and no two routes with the same id. |
| `IllegalStateException: via() applies to a DRIVE, but the last step is ...` | A modifier (`.via`, `.speed`, `.label`, `.wallContact`) must come right after a `drive...` step; `.launchSeconds` right after a `shoot...`. |
| A unit test says `docs/autos/... is missing` | You added or renamed a route without regenerating: run the script and commit `docs/autos`. |
| A diagram label overlaps something | Labels avoid each other and the step circles, not every line of the field. Open the SVG in a browser and zoom; the step list on the page says what each number is. |
| The script printed a `Note: ... belongs to no route` | You removed a route. Delete its two old SVG files by hand (the tool never deletes files). |
