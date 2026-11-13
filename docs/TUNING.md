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
then **add 3.0 in** (the aim point is 3.0 in inside the foot line). Check against the one known shot: the robot's
front against the audience wall, in line with the HIVE: the ball exit is 17.8 in from the wall and the foot line is
at 52.5 in, so 52.5 − 17.8 + 3.0 = **37.7 in**. That shot passed through the middle of the opening, slightly high,
at about 1200 ticks/s.

**Procedure (Shooter Tuner OpMode).**

1. Put the robot at a known range, in line with the cell (the cell axis is the center of the 20 in wide opening:
   x = 59.25 for the red HIVE, 84.75 for blue).
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
| Range reported by the camera is off | the camera constants in `LIMELIGHT_SETUP.md` §4 |
| Camera says DOWN for the up cell | `Vision.TX_POSITIVE_IS_RIGHT`; camera roll |

If a value you want to change is not in `BioBuzzConstants`, that is a bug: add it there.
