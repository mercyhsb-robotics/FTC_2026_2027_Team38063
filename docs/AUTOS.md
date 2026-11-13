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
   route                 time  points  POLLEN  1st tip  validator
   my_route             12.3s       8      -4  SELF     no findings
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
| **WAIT_FOR_TIP** | `waitForTip(giveUpSec)` | Stand still until the camera has SEEN the HIVE tip; give up at the route time `giveUpSec`. For partner routes only (see "Who shoots the first tip"). |
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

One more call sets the **assumption of the whole route** and goes right after `AutoRoute.builder("id")`:
`.partnerTipsFirst()` says "the alliance partner shoots the first tip". Without it the route assumes our own 4
pre-loads are the first tip.

Two rules of thumb that the validator enforces:

* **A move is a DRIVE.** If an INTAKE, SHOOT or PARK has a pose far from where the previous step left the robot,
  the validator warns: add a `drive...` before it so the path is planned and checked.
* **Scoring.** A route claims LEAVE (3 points) when it ends with the whole robot off every wall, PARK (5) when it ends
  partly in the own LOADING ZONE, and a HIVE TIP (20) for every tip its OWN volleys cause. The 3 NECTAR that sit in the
  UP cell at the start help: ONE volley of 4 POLLEN tips the first HIVE (3 NECTAR + 4 POLLEN is enough, 3 NECTAR +
  3 POLLEN is not). The HIVE flips after a tip and the cell that comes up holds no NECTAR, so the next tip at the
  other cell needs 8 POLLEN (two volleys of 4). See "How a HIVE tips" below. The partner's assumed tip (20 points
  for the alliance) is shown next to the route's points but is **never counted** in them: it is an assumption, not
  something our robot does.

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
| HIVE tips | G417 | a volley that goes on firing after the tip; in a partner route, a SHOOT at the cell that comes up with the partner's tip that has no WAIT_FOR_TIP before it, or whose wait gives up before the assumed tip | a volley still firing when the partner tips that cell |
| Spill | G409 | the robot footprint is inside the spill zone of the cell that tipped, within 3 s of the tip | |
| First-tip assumption | design rule | | a WAIT_FOR_TIP in a route that shoots the first tip itself; a wait that gives up before the assumed tip, or later than the rest of the route can afford; our own volley tipping before the partner's assumed time; less than 3 s of slack (see below) |
| PARK | 10.5.4 | the pose is not even partly in the LOADING ZONE | it overlaps by less than 3 in |

Every finding names its step number (the same number as in the picture) and quotes the numbers: how far through the
wall, where the footprint first enters the keep-out, how far the shot is from the cell.

## How a HIVE tips

The manual does not say how much it takes to tip a HIVE, so the planner uses the team's figures: **5 NECTAR, or 8
POLLEN, or a mix**. A mix works like a recipe: each NECTAR is worth 1/5 of a tip, each POLLEN 1/8 of a tip, and the
cell tips when the shares add up to 1. The code multiplies this by 5 x 8 so that it needs only whole numbers: a cell
tips when

```
NECTAR x 8  +  POLLEN x 5  >=  40
```

* **The first tip.** The UP cell holds 3 NECTAR when the match starts (Fig 10-2). 3 NECTAR + 4 POLLEN is
  3 x 8 + 4 x 5 = 44, so **one volley of 4 POLLEN is the first tip**; 3 NECTAR + 3 POLLEN is 24 + 15 = 39, which is
  not. Two volleys are NOT needed for the first tip.
* **Every later tip.** After a tip the OTHER cell is the UP one, and Fig 10-2 shows NECTAR only in the cell that is
  UP at the start, so the new UP cell is empty: it needs 8 POLLEN (0 x 8 + 8 x 5 = 40), two volleys of 4, and the
  robot can hold only 4 POLLEN at a time (G407), so it has to fetch more in between. A second volley of 4 at the SAME
  side as the first does not tip anything: that cell has just tipped DOWN.
* **A volley that is too big.** If a tip happens in the middle of a volley, the POLLEN still to be fired would go
  into a cell that is already tipping down (a G417 violation). The validator reports that as an ERROR.
* **Our tips and the partner's tip.** `RouteAnalysis.tips()` and the points count only the tips OUR volleys cause.
  Our volleys and the partner's assumed tip are tracked separately, so a plan can never claim the partner's 20
  points as its own. The planner does not model the partner also launching into the HIVE in some other way.
* **Where the spill falls (G409).** When a HIVE tips, the cell that was UP turns down and everything in it (the 3
  NECTAR and the POLLEN launched into it) falls out of its opening onto the floor on that side of the HIVE. For the
  red HIVE the audience cell covers x 49.25 to 69.25 in and y 52.3 to 69.9 in and opens toward the audience wall. G409
  says a robot may not catch or deflect what a tipped HIVE releases, so for `Auto.TIP_SPILL_CLEAR_SEC` (3 s) after
  every tip the robot footprint must stay out of that cell's spill zone: the cell footprint grown by
  `Auto.TIP_SPILL_MARGIN_IN` (6 in). The other side of the HIVE is clear: that is where a robot waiting for the
  partner stands. On the real robot the rule is "stay out until the camera has confirmed the tip, plus those 3 s";
  the checker's time-based version is for planning only.

## Who shoots the first tip

The HIVE belongs to the alliance, so our partner robot can tip it too. A route is written under one of two
assumptions, and the picture and the gallery print it for every route (`first tip: ...`) so the drive team can pick
the auto that matches what the partner's auto does.

| Assumption | Builder | What it means |
|---|---|---|
| **We shoot the first tip** (`FirstTip.SELF`, the default) | nothing | Our 4 pre-loads, with the 3 staged NECTAR, are the first tip: ONE volley. Written in `SelfTipRoutes.java`. |
| **Partner tips first** (`FirstTip.PARTNER`) | `.partnerTipsFirst()` | The partner's volley is the first tip. The HIVE tips by itself at `Auto.PARTNER_FIRST_TIP_DONE_SEC` (12 s, VERIFY: agree the real time with the partner team) and the OTHER cell comes up, with no NECTAR in it. Our robot goes to that side. Written in `PartnerTipRoutes.java`. |

For the red alliance the UP cell at the start is on the audience side, so a partner tip brings the REAR cell up (for
blue it is the other way round). A partner route is therefore best started in the rear half of the field, close to
that cell. Our own volleys into the new cell need 8 POLLEN for the next tip (two volleys of 4, with a trip to a
FLOWER in between, because the robot holds only 4).

**The camera decides, never the clock.** Partner teams tip at different times, so the robot never shoots because
"12 seconds have passed". The step that makes it wait is `waitForTip(giveUpSec)`:

* The robot stands still until its camera has **seen** the cell UP and steady (the AprilTags on the bottom of the
  HIVE in view). Only then does it go on to the volley after the wait.
* `giveUpSec` is a **time on the route clock** (seconds since AUTO started), not a length of time and not a guess of
  the tip time. When the clock reaches it and the tip has not been seen, the robot skips the volleys that need the
  tip and carries on with the safe steps (LEAVE, PARK). So `waitForTip(22.0)` means "stop waiting at 22 s".
* No camera, no shot: if the camera does not work, the robot does not shoot any volley that depends on a tip.
* The checker insists on it: a SHOOT at the cell that comes up with the partner's tip **must** have a
  `waitForTip` before it, even if the SHOOT would start after the assumed 12 s.

**How late may the partner be? (slack).** The route estimate still has to guess when the partner tips (12 s), to
count time and points. For every partner route the analysis also computes the **latest partner tip that still
works**: the latest time T so that the robot has not given up waiting yet and the whole route, LEAVE and PARK after
the volleys included, is still over by the AUTO limit minus the 2 s safety margin (28 s). The gallery shows it next
to the assumption, and the **slack** is T minus the assumed time. A big slack means a slow partner is fine. Below
`Auto.PARTNER_TIP_MIN_SLACK_SEC` (3 s, TUNE) the checker warns: "this route only works with a partner that is about
as fast as agreed". The same page shows, on the WAIT_FOR_TIP step, the latest give-up time that the rest of the route
allows: write that number, or a little less, into `waitForTip(...)`.

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
* **WAIT** is a small clock with its seconds; **WAIT FOR TIP** is the same clock with "give up at ..." (the route time
  at which the robot stops waiting). **PARK** is a bold dashed outline of the final footprint. A route that
  ends on a DRIVE gets an `END` outline instead.
* The line under the title names the route, its start tile and **who shoots the first tip**; for a partner route it
  also gives the latest partner tip that still works.
* The **UP cell** of each HIVE at the start of the match is solid and filled, the **DOWN cell** dashed; both show their
  AprilTag IDs. The hatched half is the opponent's: the robot center must not go there in AUTO (G402).
* The footer shows the estimated time, the POLLEN balance, **our** AUTO points (LEAVE, PARK and our own tips), the
  partner's assumed tip "(not ours)" when the route has one, and the first findings. The gallery page has all of
  them with their full text, and works out the tip arithmetic of every volley (for example `3 x 8 + 4 x 5 = 44, at least
  40, so the HIVE tips`).

## About the numbers

* **Times are placeholders.** Nobody has measured the robot's cruise speed, acceleration, turn rate or shooting
  times yet. They live in `BioBuzzConstants.Auto` marked **ADD** (and are listed among the placeholders the TeleOp
  prints at INIT): `CRUISE_SPEED_IN_PER_SEC`, `ACCEL_IN_PER_SEC2`, `TURN_RATE_DEG_PER_SEC`, `SETTLE_SEC`,
  `INTAKE_DWELL_*_SEC`, `AIM_SPINUP_SEC`, `TIME_PER_SHOT_SEC`. Compare routes with them, but do not trust a route
  that is within a few seconds of the limit. When you measure one, replace it, change its marker to VERIFY or TUNE
  and delete its name from `BioBuzzConstants.PLACEHOLDERS`, then regenerate the pictures.
* **The tip figures** (`Auto.NECTAR_TIP_THRESHOLD` = 5 and `Auto.POLLEN_TIP_THRESHOLD` = 8, VERIFY) are the team's:
  5 NECTAR, or 8 POLLEN, or a mix. The manual does not give them. `Auto.NECTAR_STAGED_IN_UP_CELL` = 3 is FIXED (Fig 10-2).
* **The partner numbers** are guesses to be replaced: `Auto.PARTNER_FIRST_TIP_DONE_SEC` = 12 s (VERIFY: agree it with
  the partner team), `Auto.TIP_SPILL_CLEAR_SEC` = 3 s and `Auto.TIP_SPILL_MARGIN_IN` = 6 in (VERIFY: watch a real tip),
  `Auto.PARTNER_TIP_MIN_SLACK_SEC` = 3 s (TUNE).
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
* **`wall_shoot_park`** and **`garden_cycle`** are new demonstration routes that pass: the first shoots the pre-loads
  (the first tip), leaves through the left corridor and parks deep in the zone (28 points); the second shoots the
  pre-loads (the first tip), sweeps up the 4 GARDEN POLLEN and steps off the wall (23 points). It does not shoot the
  GARDEN POLLEN: the UP cell is on the far side of the HIVE after the tip and needs 8 POLLEN.

These are findings about the **route data**. Whether the old OpModes should be corrected is the team's call; the
catalog deliberately does not "improve" them.

## How the pieces fit

| Package | What is in it |
|---|---|
| `autoshoot.routes` | The data model (`AutoRoute`, `Step`, `DrivePath`, `HeadingMode`, `IntakeSource`, `ShootTarget`, `FirstTip`), the simulation (`RouteAnalysis`, `TimingModel`, and `TipModel` for the tip arithmetic), the checks (`RouteValidator`, `TipRules`, `Issue`), the geometry (`Footprint`, `AllianceField`, `ShotGeometry`) and the lists of routes: **`SelfTipRoutes`** (we shoot the first tip) and **`PartnerTipRoutes`** (the partner does), gathered by `RouteCatalog`. |
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
