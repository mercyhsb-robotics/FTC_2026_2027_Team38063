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
   ZONE, not touching a FLOWER or the HIVE frame (rule G304). The robot is a 17.80 x 17.80 in square, so a robot
   facing the audience wall at tile B1 or C1 has its *center* 9.2 in from the wall: `start(35.0, 9.2, -90)`.
2. **Add one entry** to `build()` in
   `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autoshoot/routes/RouteCatalog.java`. Write it for the
   **RED** alliance only (see "Red and blue" below). Copy this and change it:

   ```java
   routes.add(AutoRoute.builder("my_route")                 // id: letters, digits, _ and - only; never changes
           .title("C1: shoot 4, then park")                  // what people read
           .description("Fires the 4 pre-loads, then drives up the left corridor into the LOADING ZONE.")
           .startTile("C1")                                  // must match the tile of the start pose
           .start(58.0, 9.2, -90)                            // x, y in inches, heading in degrees
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
   route                 time  points  POLLEN  validator
   my_route             12.3s       8      -4  no findings
   ```
4. **Read the page.** Open `docs/autos/index.html`, find your route, look at both diagrams and read the findings.
   An **ERROR** means the route breaks a rule or cannot work (fix it). A **WARN** means "probably fine, check it on
   the field". Change the entry, run the script again, repeat.
5. **Commit** the catalog entry *and* the regenerated `docs/autos` files together. Then run the unit tests
   (`./gradlew :TeamCode:testDebugUnitTest`): one of them fails if the committed pictures do not show every route
   of the catalog.

## Red and blue

Everything is **written once, for RED**. The BLUE version is the same route turned **half a turn about the field
center**: `(x, y, heading)` becomes `(144 - x, 144 - y, heading + 180)`. It is *not* a mirror image, because the
field is rotationally symmetric. So a blue robot starts at the top of the picture, and the red HIVE's audience cell
becomes the blue HIVE's rear cell. Use `route.forAlliance(Alliance.BLUE)` to get the blue route; the diagrams and the
validator do it for you.

**Field frame** (the Pedro frame): inches, origin at the corner where the audience wall meets the red wall, **+x
toward blue**, **+y away from the audience**. Heading 0 is +x and angles grow counter-clockwise. Tiles are 24 in:
columns A-F run along x, rows 1-6 along y, so A1 is the corner and B1 is x 24-48, y 0-24.

**The robot** has its intake at the **FRONT** and its shooter at the **BACK**, so a robot shooting at the HIVE
stands with its back toward it. The ball leaves 8.9 in behind the robot's center
(`BioBuzzConstants.Robot.LAUNCH_POINT_FORWARD_IN`).

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
| **INTAKE** | `intakeFloor(...)`, `intakeFlower(...)`, `intakeGarden(...)` | Take POLLEN: `(targetX, targetY, robotX, robotY, robotHeadingDeg, count)`. |
| **SHOOT** | `shootUp(count)` | Fire at the cell that is UP; the solver picks the pose (the picture shows a nominal one). |
| | `shootUp(x, y, headingDeg, count)` | Same, from an explicit pose. |
| | `shootSide(Side.AUDIENCE, count)` | Fire at a fixed cell (a promise that it is UP at that moment). |
| **WAIT** | `waitSeconds(s)` | Do nothing for a while. |
| **PARK** | `park(x, y, headingDeg)` | The final pose, partly inside the own LOADING ZONE (5 points). |

Modifiers apply to the step added **just before** them:

| Modifier | Applies to | Effect |
|---|---|---|
| `.via(x, y)` | DRIVE | Adds a Bezier control point: the path bends toward it but does **not** pass through it. |
| `.speed(0.5)` | DRIVE | Drive at this fraction of the cruise speed. |
| `.label("corridor")` | DRIVE | Names the leg in the picture. |
| `.wallContact()` | DRIVE | The robot may graze the field wall on purpose (sweeping the GARDEN POLLEN). |
| `.launchSeconds(5.0)` | SHOOT | The step takes exactly this long (instead of the estimate). |
| `.note("why")` | any | Free text for people; shown on the page. |

Two rules of thumb that the validator enforces:

* **A move is a DRIVE.** If an INTAKE, SHOOT or PARK has a pose far from where the previous step left the robot,
  the validator warns: add a `drive...` before it so the path is planned and checked.
* **Scoring.** A route claims LEAVE (3 points) when it ends with the whole robot off every wall, PARK (5) when it ends
  partly in the own LOADING ZONE, and a HIVE TIP (20) for every 8 POLLEN it launches into the UP cell. The HIVE
  flips after a tip, so the next volley counts from zero at the other cell.

## What the validator checks

| Check | Rule | ERROR when | WARN when |
|---|---|---|---|
| Start | G304 | outside the field or the own half, not touching a wall, in the LOADING ZONE, touching a FLOWER or the HIVE | |
| Field walls | 9.2, G412 | any footprint along a leg or while turning pokes through a wall (steps marked `wallContact` get 1 in) | |
| HIVE | G417, G409 | the robot footprint comes within 3 in of the HIVE frame | |
| FLOWERS | G304.D, G415 | a leg runs into a FLOWER plate (except the one an INTAKE takes from) | |
| Center line | G402 | the robot center crosses into the opponent half | the body ends over the line |
| POLLEN | G407 | more than 4 held, firing more than held, taking from the other alliance's GARDEN or FLOWER | a source that is empty or that does not exist; a floor intake before anything was shot |
| Time | G401 | the estimate is over 30 s | within 2 s of the limit |
| End pose | 10.5.4 | the route ends inside the HIVE frame | |
| SHOOT | G417 | | pose off the 30-38 in standoff band, back more than 15 degrees off the cell, wrong side of the HIVE, or the target cell is DOWN at that moment |
| PARK | 10.5.4 | the pose is not even partly in the LOADING ZONE | it overlaps by less than 3 in |

Every finding names its step number (the same number as in the picture) and quotes the numbers: how far through the
wall, where the footprint first enters the keep-out, how far the shot is from the cell.

## Reading a diagram

* **Top view**, the audience wall at the **bottom**, the red alliance on the **left**. Tiles are labelled A-F and 1-6.
* The route is drawn in the alliance's **ink**: red ink for the red alliance, blue ink for the blue one.
* The **robot is drawn to scale**. Its **FRONT edge (intake) is green** and its **BACK edge (shooter) is orange**.
  The START footprint has an arrow along its heading.
* **DRIVE legs** are solid lines or curves with arrowheads. **Dotted arrows** are the moves to a pose that a step makes
  by itself (a SHOOT pose, an INTAKE pose, a PARK pose).
* Every step has a **numbered circle**, in route order. A red triangle on it is an ERROR, a yellow one a WARN.
* **INTAKE** is a dashed circle around the target, labelled like `INTAKE GARDEN +4`.
* **SHOOT** is the robot at the shooting pose and a dashed arrow from the **ball exit** to the **aim point** in the
  cell opening, labelled `SHOOT 4`. A dashed footprint means the pose is the solver's *nominal* pose. `HIVE TIP +20`
  marks the volley that tips the HIVE.
* **WAIT** is a small clock with its seconds. **PARK** is a bold dashed outline of the final footprint. A route that
  ends on a DRIVE gets an `END` outline instead.
* The **UP cell** of each HIVE at the start of the match is solid and filled, the **DOWN cell** dashed; both show their
  AprilTag IDs. The hatched half is the opponent's: the robot center must not go there in AUTO (G402).
* The footer shows the estimated time, the POLLEN balance, the claimable points and the first findings. The
  gallery page has all of them with their full text.

## About the numbers

* **Times are placeholders.** Nobody has measured the robot's cruise speed, acceleration, turn rate or shooting
  times yet. They live in `BioBuzzConstants.Auto` marked **ADD** (and are listed among the placeholders the TeleOp
  prints at INIT): `CRUISE_SPEED_IN_PER_SEC`, `ACCEL_IN_PER_SEC2`, `TURN_RATE_DEG_PER_SEC`, `SETTLE_SEC`,
  `INTAKE_DWELL_*_SEC`, `AIM_SPINUP_SEC`, `TIME_PER_SHOT_SEC`. Compare routes with them, but do not trust a route
  that is within a few seconds of the limit. When you measure one, replace it, change its marker to VERIFY or TUNE
  and delete its name from `BioBuzzConstants.PLACEHOLDERS`, then regenerate the pictures.
* **The HIVE needs 8 POLLEN** to tip (`Auto.TIP_POLLEN_THRESHOLD`, VERIFY): the team's figure is 5 NECTAR or 8 POLLEN.
* **Rule limits** (30 s, 4 POLLEN, the 3/5/20 points) are FIXED from the manual. The safety margins (3 in around the
  HIVE, 2 s before the end, the standoff band) are TUNE or VERIFY.
* The look of the diagrams (scale, inks, tile colours, label sizes) is in `BioBuzzConstants.Auto` with the
  `RENDER_` prefix.

## What the validator says about today's autos

(For the current list open `docs/autos/index.html`; this is the situation with the 17.80 in square robot.)

* **`red_leave_park`** has no findings.
* **`red_c1`** drives its first leg, straight from (58, 9.2) to (62, 118.1), **through the HIVE frame**. The route
  cannot work as written.
* **`red_b1`** turns to -115 degrees right against the audience wall (a corner pokes **2.6 in through the wall**), shoots
  from 43 in (the good band is 30-38 in), drives so close to the HIVE that the robot is **0.4 in inside the 3 in
  keep-out** (the numbers were written for a narrower robot), and parks only **2.9 in** into the LOADING ZONE.
* **`wall_shoot_park`** and **`garden_cycle`** are new demonstration routes that pass: the first leaves through the
  left corridor and parks deep in the zone (8 points); the second shoots the pre-loads, sweeps up the 4 GARDEN POLLEN and
  shoots them too, so 8 POLLEN reach one cell and the HIVE tips (23 points).

These are findings about the **route data**. Whether the old OpModes should be corrected is the team's call; the
catalog deliberately does not "improve" them.

## How the pieces fit

| Package | What is in it |
|---|---|
| `autoshoot.routes` | The data model (`AutoRoute`, `Step`, `DrivePath`, `HeadingMode`, `IntakeSource`, `ShootTarget`), the simulation (`RouteAnalysis`, `TimingModel`), the checks (`RouteValidator`, `Issue`), the geometry (`Footprint`, `AllianceField`, `ShotGeometry`) and **`RouteCatalog`**, the list you edit. |
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
