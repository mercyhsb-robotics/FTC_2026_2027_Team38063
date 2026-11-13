# Tuning guide

All values are in `BioBuzzConstants` unless a section says otherwise. Change one thing at a time, and write down
what you saw.

## 1. Pedro Pathing (drive and odometry) — do this first

Pedro's tuners live in the Pedro tuning menu on the Driver Station (registered in `pedro/Tuning.java`). Each step
needs the one before it to be right.

| Order | Procedure | What it gives you | Where the result goes |
|---|---|---|---|
| 1 | **Mecanum Tuner** | confirms motor names and directions | `pedro/Constants.drivetrainConfig` |
| 2 | **Pinpoint Tuner** | pod directions, scalar, offsets | `pedro/Constants.localizerConfig` |
| 3 | **Foresight Tuner** | max speed, braking and heading/translation gains | replaces `pedro/Constants.foresightConfig` (it prints a complete block) |
| 4 | **Tests** | a hold test and a line test with your values | nothing: it shows whether it worked |

Foresight Tuner starting inputs are already sensible: 48 in drive for top speed, 30 in/s for deceleration, 36 in
for braking (minimum 15 in). Clear about 6 ft of straight floor forward and sideways, use a fresh battery, and
expect the robot to coast past its target on every run. The Foresight numbers now in the file are Pedro's documentation
examples (the file says PROVISIONAL); the auto-drive works with them but stops less accurately.

## 2. The shot table (`Shooter.SHOT_TABLE`)

The table maps **range** → **flywheel speed in ticks per second**. Range is the horizontal distance from the **ball
exit** to the **aim point** (the middle of the cell opening).

**Measuring the range.** Put a tape measure from the ball exit (the back face of the robot unless
`LAUNCH_POINT_FORWARD_IN` says otherwise) to the **foot line of the HIVE frame** at the end you are shooting into,
then **add 2.95 in** (the aim point is 2.95 in inside the foot line). Check against the one known shot: the robot's
front against the audience wall, in line with the HIVE: the ball exit is 17.8 in from the wall. The official CAD puts the
foot line at **51.2 in** from the audience wall, so 51.2 − 17.8 + 2.95 = **36.35 in**; the old figures (foot line 52.5 in,
add 3.0 in) gave the 37.7 in the table is anchored on. The two differ by 1.3 in: **measure the distance from the audience
wall to the foot line on your own field** (the CAD says 51.2 in, the manual's round numbers 52.5 in, real fields vary by
about 1 in) and use that to settle which is right; see `docs/DAY_ONE_CHECKLIST.md`. That shot passed through the middle of
the opening, slightly high, at about 1200 ticks/s.

**Procedure (Shooter Tuner OpMode).**

1. Put the robot at a known range, in line with the cell (the cell axis is the center of the 20 in wide opening:
   x = 57.92 for the red HIVE, 83.42 for blue).
2. Cross: start the flywheel. D-pad up/down: change the speed (hold the left bumper for bigger steps). Wait for
   `at speed: true`.
3. Hold circle to feed a ball. Note where it went. Repeat until you find the lowest speed that goes in the middle of
   the opening. Too fast flies over the cell and is a foul if it hits the outside of the cell (rule G417).
4. Set the range with D-pad left/right, press triangle to record the row; the rows also go to the robot log.
5. Do at least three ranges across **30–44 in** (a robot against the audience wall can reach at most 37.7 in in
   line). Paste the rows, sorted by range, into `SHOT_TABLE`, change its marker from ADD to TUNE, and remove
   `"Shooter.SHOT_TABLE"` from `PLACEHOLDERS`.

The planner only stands the robot inside the table: from `first range + RANGE_MARGIN_IN` to
`last range − RANGE_MARGIN_IN`. A table that is too narrow gives `NO_FEASIBLE_POSE`.

**Dip numbers** (also on the tuner screen): `SHOT_DIP_FRAC` should be about half the smallest dip you see, and
the feeder waits for the flywheel to recover into its window (`AT_SPEED_*`).

## 3. "The robot does X → change Y"

Telemetry shows `Auto-shoot: …` with the current state, and `Waiting: <reason>` or `Stopped: <fault>` when it
cannot continue.

### While it feeds ("Waiting: …")

| Reason | Meaning | What to check / change |
|---|---|---|
| `POSE_UNTRUSTED` | odometry was never given a start pose | START with the robot on the default start pose, or hold **Share**; check the Pinpoint is connected |
| `HIVE_UNCONFIRMED` | camera running but has not seen the up cell's tags | `docs/LIMELIGHT_SETUP.md`; or press D-pad down once; `AutoShoot.REQUIRE_HIVE_CONFIRMATION` |
| `TAGS_LOST` | the up cell's tags vanished (tip, or something in the way) | wait; `AutoShoot.VISION_LOSS_PAUSE_SEC` up if balls in flight pause it too often |
| `OUT_OF_RANGE` | range outside the shot table | add table rows; the planner band |
| `ROBOT_MOVING` | still moving or turning | `SHOT_MAX_SPEED_IN_S`, `SHOT_MAX_OMEGA_DEG_S`; tune Foresight |
| `HEADING_ERROR` | shooter not pointing at the aim point | `SHOT_HEADING_TOL_DEG`; `ALIGN_HEADING_TOL_DEG`; check Pinpoint heading; `VISION_HEADING_*` |
| `FLYWHEEL_NOT_AT_SPEED` | speed outside its window | PIDF (`FLYWHEEL_PIDF_*`); speed above what the motor can do (max about 2800 ticks/s); `AT_SPEED_BELOW_FRAC` / `ABOVE_FRAC` / `RISE_SEC`; battery |

### When it stops ("Stopped: …")

| Fault | Meaning | What to check / change |
|---|---|---|
| `NO_POSE` | odometry not trusted | as `POSE_UNTRUSTED` |
| `NO_FEASIBLE_POSE` | nowhere legal to stand | table range band; `SHOOT_CONE_HALF_ANGLE_DEG`; `KEEPOUT_MARGIN_IN`; `WALL_CLEARANCE_IN`; robot size |
| `APPROACH_TIMEOUT` / `APPROACH_STALLED` | did not arrive / stuck | `APPROACH_TIMEOUT_*`, `APPROACH_STALL_*`; something in the way; Foresight |
| `ALIGN_TIMEOUT` | would not settle | mecanum rarely settles under 1 in: `ALIGN_POS_TOL_IN`, `ALIGN_HEADING_TOL_DEG`, `ALIGN_TIMEOUT_SEC`, `SHOT_RELAXED_POS_TOL_IN` |
| `SPINUP_TIMEOUT` | flywheel never got up to speed | `SPINUP_TIMEOUT_SEC`, PIDF, target too high |
| `DRIVER_OVERRIDE` | the stick was pushed hard | `DRIVER_OVERRIDE_STICK`, `DRIVER_OVERRIDE_SEC` (if it triggers by accident) |
| `HIVE_NOT_VISIBLE` | stood at the pose and saw no tags on either side | camera setup; `HIVE_UNSEEN_FLIP_SEC` |
| `FEED_TIMEOUT` | no ball seen (only when `SHOT_COUNT_MODE = DIP`) | use `HYBRID` or `TIMED` |

### Shots and feeding

| What you see | Change |
|---|---|
| Every shot at one distance is high / short | that row of `SHOT_TABLE` (down for high, up for short) |
| Shots wrong only beyond some distance | add rows out to that distance |
| Every shot lands to one side | `Robot.LAUNCH_POINT_LEFT_IN` (is the exit really on the center line?) |
| Volley ends with 3 or fewer although 4 were fed | `SHOT_DIP_FRAC` too big (shallow dips not seen), or `EMPTY_AFTER_LAST_SHOT_SEC` too short; or use `TIMED` |
| Counts shots that did not happen | `SHOT_DIP_FRAC` too small; `SHOT_REFRACTORY_SEC` up |
| Feeds two balls at once / jams | `FEEDER_FORWARD_POWER`, `FEED_MODE = GATED_PER_BALL`, `FEED_SECONDS_PER_SHOT` |
| Feeder runs the wrong way | `FEEDER_INVERTED` |
| Stutters: one shot, long pause | `AT_SPEED_RISE_SEC` down, `AT_SPEED_BELOW_FRAC` up a little |

### Driving and the HIVE

| What you see | Change |
|---|---|
| Stick directions are reversed | `Controls.MANUAL_FORWARD_SIGN` / `_STRAFE_SIGN` / `_TURN_SIGN` |
| Robot holds still when you let go of the sticks and drifts when it should not | `Controls.STICK_DEADBAND` |
| Auto-drive swings wide near the wall or HIVE | `AutoShoot.KEEPOUT_MARGIN_IN`, `WALL_CLEARANCE_IN` (smaller), robot size constants |
| Picks a far-away pose when a nearer one is fine | planning costs `PLAN_*` |
| Goes to the wrong side after a tip | press D-pad down; `Vision.HIVE_CONFIRM_FRAMES` / `HIVE_CONFIRM_SEC` / `HIVE_LOST_SEC` |
| Switches sides in the middle of a volley when nothing tipped | `Vision.HIVE_LOST_SEC` up (tags blocked for a moment) |
| An AUTO that waits for the partner's tip never believes the HIVE has tipped (it gives up and skips the volley) | the camera is not seeing the new cell's tags steadily: check the Limelight Diagnostic first; `Vision.HIVE_STABLE_SEEN_SEC` down only if the tags are steady but the 0.5 s settle time is too long for the plan |
| An AUTO shoots before the HIVE has stopped rocking | `Vision.HIVE_STABLE_SEEN_SEC` up (the camera must see the new cell's tags this long without a break) |
| Range reported by the camera is off | the camera constants in `LIMELIGHT_SETUP.md` §4 |
| Pose settles slowly after a camera correction / correction is too jumpy | `Vision.POSE_CORRECTION_GAIN` (up = faster, down = smoother), `POSE_CORRECTION_MAX_STEP_IN` |
| "Drift correction: ignored: camera disagrees by N in" | N is bigger than `POSE_CORRECTION_GATE_IN`: either the drift really is that big (press the pose-reset button on the start pose) or the camera constants are wrong |
| Camera says DOWN for the up cell | `Vision.TX_POSITIVE_IS_RIGHT`; camera roll |

### FLOWER pick-up (the wiggle, autonomous)

The pick-up is derived from these numbers (`docs/AUTOS.md`, "Taking POLLEN from a FLOWER"); the turn angle and the standoff are
never typed in.

| What you see | Change |
|---|---|
| The validator says the wheel cannot reach the POLLEN, "N in more than they do" | measure `Robot.INTAKE_REACH_IN` (the wheels must stick out that much in front of the chassis); or `Auto.FLOWER_CHASSIS_CLEARANCE_IN` down if the chassis really can stay closer to the plate; or `Auto.FLOWER_WHEEL_START_OFFSET_IN` down (a smaller swing) |
| The ball does not roll out over the lip | `Auto.FLOWER_WHEEL_PRESS_OFFSET_IN` down (the wheel overlaps the ball more), `Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC` up; check the flat wheel spins so that it pulls the ball toward the robot |
| The ball is jammed against the closed side of the FLOWER | `Auto.FLOWER_WHEEL_PRESS_OFFSET_IN` up (a lighter push) |
| The wheel drags the ball on the way in | `Auto.FLOWER_WHEEL_START_OFFSET_IN` up |
| The wheel misses the ball: it only turns toward it and nothing happens | `Auto.FLOWER_WHEEL_BALL_INBOARD` (is the ball between the wheel and the center line, or outside the wheel?), the real `Robot.INTAKE_WHEEL_OFFSET_IN` |
| The next POLLEN does not drop down the chute before the next push | `Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC` up |
| The pick-up takes too long in the estimate | `Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN`, the holds, `FLOWER_WIGGLE_TURN_SEC`; the slow approach `FLOWER_APPROACH_SPEED_FRACTION` |
| The robot gives up the pick-up and backs out | the heading never got within `Auto.FLOWER_WIGGLE_HEADING_TOLERANCE_DEG` of its target in `FLOWER_WIGGLE_TURN_TIMEOUT_SEC`: the robot is blocked (a chassis corner on the plate?) or the heading gains are too soft |

If a value you want to change is not in `BioBuzzConstants`, that is a bug: add it there.
