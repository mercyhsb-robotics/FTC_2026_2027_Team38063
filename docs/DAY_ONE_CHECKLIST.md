# Day-one checklist (first time on the real robot)

Nothing in the auto-shoot feature has run on the robot yet. Work through this list in order; each step makes the
next one trustworthy. Every number you find goes into `BioBuzzConstants` (change its marker from ADD to VERIFY or
TUNE and delete its name from `PLACEHOLDERS`, otherwise the TeleOp keeps listing it).

## A. Before powering the drive

- [ ] Build and install the app (`./gradlew :TeamCode:assembleDebug`, or Android Studio). INIT the TeleOp: it
      lists the **placeholders still to measure** and any constants **warnings**; an **error** stops INIT with a
      message that names the constant.
- [ ] Robot configuration names match `BioBuzzConstants.Hardware` (drive motors, `pinpoint`, `flywheel`, `feeder`,
      `intakeMotor`, `servoLeft`, `servoRight`, `limelight`).

## B. Pedro (drive and odometry) — see `docs/TUNING.md` §1

- [ ] **Mecanum Tuner**: every wheel spins the right way.
- [ ] **Pinpoint Tuner**: pod directions and scalar; paste the offsets into `pedro/Constants.java`.
- [ ] **Foresight Tuner** (about 6 ft of clear floor): paste its output over `foresightConfig`.
- [ ] **Tests** procedure: the robot holds and drives a line where it should.
- [ ] TeleOp drives the right way. If forward/strafe/turn feel reversed, flip `Controls.MANUAL_*_SIGN`.

## C. Measurements

- [ ] **Find the robot's center of rotation, then measure from it to the four edges of the robot.** This matters more than it
      looks. The robot's *position* (the x and y that Pedro and the Pinpoint report, and that every pose in the code means) is the
      point the robot **spins about**, and on this robot that point is **not** in the middle of the 17.80 × 17.80 in outline,
      because the intake sticks out in front. Every check of "does the robot fit here?" uses these four distances.
      1. Tape a big sheet of paper to a smooth floor and set the robot on it. Tape a marker pen to the robot, pointing down, at
         your best guess: halfway between the left and right wheels, and about halfway between the front and rear wheels.
      2. Turn the robot slowly on the spot (right stick only). If the pen draws only a dot, you found the point. If it draws a
         circle, the point is at the middle of that circle: move the pen there and try again until it draws only a dot. Then put a
         strip of tape on the robot over that spot and write "center of rotation" on it.
      3. The Pinpoint pod offsets in `pedro/Constants.java` were measured from some point of the robot. Check that it is this same
         point. If it is not, tell a mentor before going on: the position the robot reports is that point.
      4. With a tape measure, measure in a straight line from the tape mark to the **front edge** (the outer edge of the two flat
         intake wheels), the **rear edge** (the back lip, where the ball leaves), the **left edge** and the **right edge** (the
         outer faces of the wheels), in inches. Write them into `Robot.FRONT_EXTENT_IN` (the drawing says 9.77),
         `REAR_EXTENT_IN` (8.03), `LEFT_EXTENT_IN` (8.90) and `RIGHT_EXTENT_IN` (8.90), and change their marker to FIXED if you
         are sure. Front + rear must equal `LENGTH_IN` and left + right must equal `WIDTH_IN`; if they do not, the TeleOp refuses to
         start at INIT and the message names the two numbers that disagree.
      5. Measure the whole outline too: `LENGTH_IN` and `WIDTH_IN` (the CAD says 17.80 × 17.80; it must fit the 18 in start cube).
      6. Stand the robot with its **front edge on the audience wall** (the wheel tips touching the wall, facing the wall) and
         read the position it reports: its y should be about `FRONT_EXTENT_IN` (9.77). If it shows something else, the center of
         rotation you marked and the point the odometry reports are not the same point. The old autonomous OpModes
         (`RedAlliance_B1`, `RedAlliance_C1`) start at y = 9.2 and their comments say the front was about 0.3 in from the wall: that
         would make the real front extent about 9.5 in. Your measurement decides; then change the OpModes' start y to match.
- [ ] Ball exit position: `Robot.LAUNCH_POINT_FORWARD_IN` (negative, measured straight back from the center of rotation to where the
      ball leaves the shooter; the starting value, -8.03, is only the rear edge from the CAD drawing) and `LAUNCH_POINT_LEFT_IN`.
      Then redo the one measured shot (front on the audience wall, in line with the HIVE: 37.7 in from the aim point) and check
      that the ball exit is still about 17.8 in from the wall.
- [ ] **How to place the robot at the start of AUTO.** The robot must touch a wall with its outline, not its center. Set it so
      that the edge that faces the wall is ON the wall (for the audience wall facing it: the wheel tips touching; with the back
      to the wall: the back lip touching), measure from the field wall to the robot's EDGES with a tape measure, never to its
      center, and check the position the robot reports against the route's start pose. A start with the front 0.5 in off the wall is
      still legal (the check allows 0.75 in), but a start with the front *through* the wall is not.
- [ ] **The two flat intake wheels** (the discs that lie flat at the front corners). Three numbers, with a ruler:
      `Robot.INTAKE_REACH_IN` is how far a wheel's front tip sticks out in front of the chassis frame (measure from
      the front edge of the frame, the part that must never touch a FLOWER plate, to the front tip of the wheel; the
      drawing says 2.7 in), `Robot.INTAKE_WHEEL_OFFSET_IN` is how far the middle of each wheel is from the robot's
      center line (6.1 in on the drawing), and `Robot.INTAKE_WHEEL_DIAMETER_IN` is how wide a wheel disc is (2.8 in).
- [ ] **A FLOWER press-in** (robot at a real FLOWER, facing the wall; the auto routes assume this move, see
      `docs/AUTOS.md`, "Taking POLLEN from a FLOWER"). Drive straight in slowly until one wheel rolls the bottom
      POLLEN over the lip of its ring, and write down:
      - Which wheel touches the POLLEN, the left or the right one (the routes name it: `IntakeWheel.LEFT` or `RIGHT`),
        and how far the robot has to stand to that side of the FLOWER's center line (should equal the wheel offset).
      - How far the wheel presses into the POLLEN: `Auto.FLOWER_PRESS_DEPTH_IN` (1.25 in is the guess: the wheel tip stops
        1.25 in past the POLLEN's near edge). Too shallow does not roll it, too deep jams it against the closed wall.
      - The **standoff**: with the wheel properly pressed in, the gap between the chassis frame and the FLOWER plate,
        `Auto.FLOWER_CHASSIS_CLEARANCE_IN` (0.5 in is the guess). The plate is 4.75 in deep, so the chassis front
        should stand at about 5.25 in from the wall; if it is closer, the chassis touches the plate.
      - How straight and how centered the robot must be for it to work: `Auto.FLOWER_HEADING_TOLERANCE_DEG` (5°)
        and `Auto.FLOWER_ALIGN_TOLERANCE_IN` (1 in), and how long the slow approach and the retreat should be
        (`FLOWER_APPROACH_LENGTH_IN`, `FLOWER_RETREAT_LENGTH_IN`, 6 in each) at what speed
        (`FLOWER_APPROACH_SPEED_FRACTION`, 0.3), plus how long it takes (`INTAKE_DWELL_FLOWER_SEC`).
- [ ] Camera: `CAMERA_FORWARD_IN` and `CAMERA_LEFT_IN` (measured from the **center of rotation**, the tape mark above, not from the middle of the outline), `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_UP_DEG` (+30 to +40°).
- [ ] Launch angle `Robot.LAUNCH_ANGLE_DEG` is about 68.5° (team measurement): confirm it with a protractor or the CAD (it only feeds the physics check on the Shooter Tuner screen).
- [ ] The real HIVE: heights of the opening and tag rows, and how many POLLEN tip it. The team's figures: 5 NECTAR,
      8 POLLEN or a mix, and 3 NECTAR are staged in the up cell at the start, so the first tip needs only 4 POLLEN.
      Check that, and check how many POLLEN the second tip needs (we assume 8). The TeleOp does not count tips (the
      camera sees them), but the autonomous route planner does.
- [ ] Watch a real HIVE tip from the side. How many seconds until the NECTAR and POLLEN have landed and stopped
      rolling, and how far from the cell do they end up? The planner keeps the robot out of the tipped cell's spill zone
      for `Auto.TIP_SPILL_CLEAR_SEC` (3 s) and 6 in around the cell (`Auto.TIP_SPILL_MARGIN_IN`): replace both with
      what you see (rule G409: a robot may not catch what a tipped HIVE releases).
- [ ] Ask the alliance partner team: does their auto shoot the first tip of the HIVE, and how many seconds into AUTO
      is it done? Write the answer into `Auto.PARTNER_FIRST_TIP_DONE_SEC` (12 s is a guess). Then pick the auto that
      matches: the gallery (`docs/autos/index.html`) says for every route whether it assumes "we shoot the first tip" or
      "partner tips first", and for a partner route it shows how late the partner may tip and the route still works.

## D. Limelight — `docs/LIMELIGHT_SETUP.md`

- [ ] Pipeline 0 configured as listed; Driver Station name `limelight`.
- [ ] **Limelight Diagnostic**: frames arrive, IDs 30–45, left/right and tx signs right, UP/DOWN reads correctly,
      implied position within about 1 in, ranges match a tape measure at three distances.

## E. Shooter — `docs/TUNING.md` §2

- [ ] **Shooter Tuner**: find the speed that scores at three or more distances, starting from the one known good
      shot (robot front against the audience wall, in line with the HIVE = 37.7 in, about 1200 ticks/s, a little
      high). Record rows and paste them into `Shooter.SHOT_TABLE`.
- [ ] Note the dip each ball causes and set `SHOT_DIP_FRAC` to about half the smallest dip.
- [ ] Confirm the feeder power (`FEEDER_FORWARD_POWER`) feeds one ball at a time, and check `FEED_SECONDS_PER_SHOT`.

## F. The feature

- [ ] Robot on the **default start pose** (center of tile C1 for RED, front against the audience wall). INIT,
      choose the alliance with D-pad left/right, START.
- [ ] Hold D-pad up. Watch the telemetry: `Driving to the scoring pose` → `Aligning` → `Feeding` → `Done: 4 shots`.
- [ ] Release the button mid-way: the robot must stop and give control back immediately.
- [ ] Tip the HIVE by hand (or shoot 4 POLLEN on a fresh HIVE, then about 8 for each later tip) and shoot again: it
      should drive to the other side.
- [ ] Repeat for the BLUE alliance.
- [ ] Drift correction (only after section D agrees within about 1 in): set `Vision.POSE_CORRECTION_ENABLED = true`,
      push the robot a few inches off its pose while it faces the HIVE, and watch the "Drift correction" telemetry
      line pull the position back (`LIMELIGHT_SETUP.md` step 9).
- [ ] Practice rule: shoot only into the UP cell. The robot is built to pause when the tags disappear, but the
      rule (G417) is the driver's responsibility too.
