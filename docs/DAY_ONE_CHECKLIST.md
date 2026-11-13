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

- [ ] Ball exit position: `Robot.LAUNCH_POINT_FORWARD_IN` (negative: it is at the back), `LAUNCH_POINT_LEFT_IN`.
- [ ] Re-measure the robot's `LENGTH_IN` and `WIDTH_IN` (the CAD says 17.80 × 17.80; it must fit the 18 in start cube).
- [ ] Camera: `CAMERA_FORWARD_IN`, `CAMERA_LEFT_IN`, `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_UP_DEG` (+30 to +40°).
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
