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
- [ ] **Measure YOUR field once, with a tape.** Every field number in the code is the official CAD (the wall-corner frame:
      141.348 in between the inner faces of the walls, tiles 23.5 in, see `docs/AUTOS.md`). Real fields vary by about 1 in
      (the manual says "approximately 144 in", the CAD says 141.348 in). Measure: the distance between the red wall and the blue
      wall, inner face to inner face, along the middle of the field; the distance from the audience wall to the **foot bar end**
      of the HIVE frame (the CAD says 51.2 in; the old round figure was 52.5 in); the distance from the red wall to the **inner
      face of the left foot bar** (CAD 47.9 in); and the tile seams (CAD 23.67, 47.17, 70.67, 94.17, 117.67 in from the red wall).
      If your field is more than about 1.5 in from the CAD, change `Field.SIZE_IN` (and tell a mentor: the other numbers move with
      it); if it matches, the 1.3 in anchor-shot difference below is the robot, not the field.
- [ ] Ball exit position: `Robot.LAUNCH_POINT_FORWARD_IN` (negative, measured straight back from the center of rotation to where the
      ball leaves the shooter; the starting value, -8.03, is only the rear edge from the CAD drawing) and `LAUNCH_POINT_LEFT_IN`.
      Then redo the one measured shot (front on the audience wall, in line with the HIVE: 37.7 in from the aim point) and check
      that the ball exit is still about 17.8 in from the wall.
      **The model and the measurement disagree by 1.34 in, so re-measure the distance from the ball exit to the cell opening center
      with a tape.** With the official field CAD the center of the UP cell's opening is 54.16 in from the audience wall (the old
      144 in field said 55.52), so the code computes 54.16 - 17.80 = **36.36 in** for the very place where the team measured
      37.7 in. Stand the robot with its front on the wall and in line with the HIVE, hold a tape from the point where the ball
      leaves the shooter (look from the side: the lip of the ramp) horizontally to the middle of the cell's opening, and compare:
      - If the tape says about 36.4 in, the old measurement was a little long: keep the code, and move the table row
        `{37.7, 1200}` to the range the tape says (its speed stays 1200: it is what scored).
      - If the tape says about 37.7 in, the ball leaves the robot about 1.3 in farther forward than `LAUNCH_POINT_FORWARD_IN`
        says (the robot CAD puts it 6.58 in behind the center of rotation, which gives 37.8 in): measure it and fix that constant.
      The table row is **not** changed until you know which one it is (it is a measurement, and the ranges next to it come from it).
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
- [ ] **A FLOWER wiggle** (robot at a real FLOWER, facing the wall; the auto routes assume this move, see
      `docs/AUTOS.md`, "Taking POLLEN from a FLOWER"). The robot drives in **off center** so that ONE flat wheel is beside the
      bottom POLLEN, **turns on the spot** toward the ball (the spinning wheel and the push of the whole robot roll it over the
      lip of its ring), turns back so the next ball drops into the hole, and repeats. Do it by hand first with **both** wheels
      (left and right: the program picks the cheaper one, so BOTH must work) and write down, for each wheel, any difference:
      - **Which side of the wheel the ball must be on**: between the wheel and the robot's center line ("inboard", the guess,
        `Auto.FLOWER_WHEEL_BALL_INBOARD = true`) or on the far side of the wheel ("outboard", `false`). Turning toward the ball
        should also swing the wheel a little forward, toward the wall; if the ball only comes out the other way, change the flag.
      - **The start offset**: how far beside the ball, along the wall, the middle of the wheel has to be so that the robot can
        drive straight in without the wheel touching or dragging the ball, but a small turn still reaches it:
        `Auto.FLOWER_WHEEL_START_OFFSET_IN` (3.8 in is the guess: the two radii plus 1 in of air).
      - **How far the turn must go**: the middle of the wheel must come this close to the middle of the ball for the ball to
        roll out over the lip, without jamming it against the closed side of the FLOWER: `Auto.FLOWER_WHEEL_PRESS_OFFSET_IN`
        (1.8 in is the guess: the wheel overlaps the ball by 1 in). The program turns the robot just far enough (about 17 degrees
        with the guesses); do NOT type the angle anywhere. If the real turn has to be more than `Auto.FLOWER_WIGGLE_MAX_TURN_DEG`
        (20 degrees), the validator says so.
      - **The holds**: how long to stay turned in with the wheel pressed on the ball (`Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC`, 0.25 s)
        and how long to stay turned back so the next POLLEN drops down the chute (`Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC`, 0.4 s);
        how long one half of the turn takes (`Auto.FLOWER_WIGGLE_TURN_SEC`, 0.35 s); how many cycles one POLLEN needs
        (`Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN`, 1).
      - **The wheel spin direction**: does the flat wheel spin so that it **pulls the ball toward the robot**, out over the lip
        (check `RobotCheck` and the intake servo signs, `RobotCheck.INTAKE_LEFT_SERVO_SIGN` and `INTAKE_RIGHT_SERVO_SIGN`)? A
        wheel that pushes the ball back into the hole defeats the whole wiggle. Check both wheels.
      - **The standoff**: with the robot turned in and the wheel properly on the ball, the gap between the chassis frame and the
        FLOWER plate, `Auto.FLOWER_CHASSIS_CLEARANCE_IN` (0.5 in is the guess). The plate's field-side edge is 5.033 in from the wall
        (the CAD: 4.871 in deep, standing 0.162 in off the wall; the manual says 4.75). The corner of the chassis nearest the plate
        swings toward it while the robot turns, so measure the gap at the **worst moment of the turn**, not only when standing
        straight. **With the numbers of the drawing the validator reports that the wheel falls 0.77 in short of the ball** (the
        wheels would have to stick out 3.5 in instead of 2.7 in once the chassis keeps 0.5 in off the plate through the swing):
        measure `Robot.INTAKE_REACH_IN` and this clearance carefully, they decide whether a FLOWER pick-up is possible at all.
      - The real FLOWER, with a tape: the distance from the wall to the middle of the POLLEN stack (the CAD says 2.634 in, the
        manual 2.4 in), the diameter of the hole the bottom POLLEN sits in (CAD 3.04 in, manual 2.79 in: the code plans with the
        smaller one) and the height of the ring's lip (CAD 0.35 in, manual 0.43 in).
      - The wall bracket: which parts of YOUR robot are taller than 11.44 in, and how far from the front or side edge they are. The
        FLOWER's black bracket sticks out 1.67 in from the wall at 11.44 to 11.84 in above the floor and is 6.67 in long (0.36 in
        longer than the plate at each end); a robot part in that height band within 1.67 in of the wall touches it.
      - How long the slow straight approach and the straight retreat should be (`FLOWER_APPROACH_LENGTH_IN`,
        `FLOWER_RETREAT_LENGTH_IN`, 6 in each) at what speed (`FLOWER_APPROACH_SPEED_FRACTION`, 0.3). The robot's turn is checked
        against `Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG` (2 degrees) and gives up after `Auto.FLOWER_WIGGLE_TURN_TIMEOUT_SEC` (1.5 s):
        check on the real robot that a normal turn finishes well inside that, and that a blocked one really backs out.
- [ ] Camera: `CAMERA_FORWARD_IN` and `CAMERA_LEFT_IN` (measured from the **center of rotation**, the tape mark above, not from the middle of the outline), `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_UP_DEG` (+30 to +40°).
- [ ] Launch angle `Robot.LAUNCH_ANGLE_DEG` is about 68.5° (team measurement): confirm it with a protractor or the CAD (it only feeds the physics check on the Shooter Tuner screen).
- [ ] **The lane under the HIVE** (only if a route or `AutoShoot.ALLOW_UNDER_HIVE` will use it, see `docs/AUTOS.md`, "Driving under
      the HIVE"). With a tape: the distance between the **inner faces of the two foot bars** (CAD 45.5 in) and between the **tips of
      the leg stubs** at the height of your robot's tallest part, 13 in (CAD 40.1 in at 12.13 in, a little less at 13 in); and push the
      robot up the lane by hand with its camera on: nothing may touch. A robot that touches a foot bar (a 2.15 in step) is stopped
      by it, so test at slow speed first.
- [ ] The real HIVE: heights of the opening and tag rows, and how many POLLEN tip it. **FIRST's own calibration** (Event
      Field Setup Guide 12.3, Field Acceptance Checklist 2.3 to 2.6) says an empty cell tips with the **8th POLLEN** (7 do
      not) and a cell with the **3 staged NECTAR tips with the 3rd POLLEN** (2 do not): the first tip needs only 3 POLLEN, so
      4 pre-loads tip it with 1 spare, and every later tip needs 8 (the planner plans 9, `Auto.TIP_PLANNING_MARGIN_POLLEN`,
      because a real field drifts: one needed 9 instead of 8 after 8 to 10 matches, Q&A QA46). Check all of it on the real
      HIVE, with the 3 NECTAR placed gently at the back of the up cell: tip it with 2 POLLEN then 3, and an empty cell with 7
      then 8. Weigh a few POLLEN and NECTAR on a kitchen scale (AndyMark lists 0.055 lb and 0.091 lb): one NECTAR counts as
      5/3 POLLEN in the code (`Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR` / `_DENOMINATOR`, anything from 1.667 up to 2.0
      agrees with FIRST's table); and see whether 5 NECTAR tip an empty cell and 4 do not. `ConstantsCheck` refuses numbers
      that contradict FIRST's table. The TeleOp does not count tips (the camera sees them), but the autonomous route planner
      does.
- [ ] Watch a real HIVE tip from the side. How many seconds until the NECTAR and POLLEN have landed and stopped
      rolling, and how far from the cell do they end up? The planner keeps the robot out of the tipped cell's spill zone
      for `Auto.TIP_SPILL_CLEAR_SEC` (3 s) and 6 in around the cell (`Auto.TIP_SPILL_MARGIN_IN`): replace both with
      what you see (rule G409: a robot may not catch what a tipped HIVE releases).
- [ ] Ask the alliance partner team: does their auto shoot the first tip of the HIVE, and how many seconds into AUTO
      is it done? Write the answer into `Auto.PARTNER_FIRST_TIP_DONE_SEC` (12 s is a guess). Then pick the auto that
      matches: the gallery (`docs/autos/index.html`) says for every route whether it assumes "we shoot the first tip" or
      "partner tips first", and for a partner route it shows how late the partner may tip and the route still works (the
      latest partner tip and the slack). The partner's launches count for our HIVE (a HIVE belongs to the alliance and the
      20 points are the alliance's), so a partner that shoots the first tip is legal. The time is only a plan: the robot goes
      on when its camera has **seen** the other cell UP and steady for `Vision.HIVE_STABLE_SEEN_SEC` (0.5 s), never because a
      time has passed.
- [ ] **No camera, no shot.** Unplug the Limelight and start a partner route at INIT: the Auto Route Picker must refuse it
      with the reason (`TipCameraPolicy`); with the camera back, run it and let a partner (or a person) tip the HIVE at a
      time you do not tell the robot: it must wait, and shoot only after the tags are steady in view. Then let the HIVE
      never tip: the robot must give up at its deadline, skip the volley and still do its LEAVE and PARK.
- [ ] **LEAVE and the wall in the LOADING ZONE** (FIRST's Q&A QA21): park in the LOADING ZONE once touching the wall and once
      standing `Auto.LEAVE_WALL_MARGIN_IN` (1.5 in) off it, and ask a referee which one counts as LEAVE: a robot that touches
      any wall at the end of AUTO loses LEAVE, but keeps PARK.

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
- [ ] Tip the HIVE by hand (or shoot 3 POLLEN on a fresh HIVE, then about 8 for each later tip) and shoot again: it
      should drive to the other side.
- [ ] Repeat for the BLUE alliance.
- [ ] Drift correction (only after section D agrees within about 1 in): set `Vision.POSE_CORRECTION_ENABLED = true`,
      push the robot a few inches off its pose while it faces the HIVE, and watch the "Drift correction" telemetry
      line pull the position back (`LIMELIGHT_SETUP.md` step 9).
- [ ] Practice rule: shoot only into the UP cell. The robot is built to pause when the tags disappear, but the
      rule (G417) is the driver's responsibility too.
