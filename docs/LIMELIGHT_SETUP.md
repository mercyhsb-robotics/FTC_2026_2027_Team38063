# Limelight 3A setup

The code expects the Limelight to be set up exactly this way. The values here are the ones in
`BioBuzzConstants.Vision`; if you change one there, change it here too. Items marked **(measure)** are
placeholders until you take the real measurement.

## 1. Hardware and connection

1. Mount the camera rigidly on the robot, **facing the back** (the same way as the shooter), upright (not
   rolled), on a part that does not flex.
2. **Tilt it up.** The tag rows are 36–50 in above the floor and the Limelight 3A only sees 42° vertically, so it
   needs roughly **+30 to +40° of upward pitch** to keep them in view from the shooting distance. The code starts
   with `CAMERA_PITCH_UP_DEG = 35` **(measure)**.
3. Measure and write into `BioBuzzConstants.Vision` **(measure)**:
   * `CAMERA_FORWARD_IN` — lens position from the robot's rotation center, forward positive (it is behind
     center, so negative);
   * `CAMERA_LEFT_IN` — sideways offset, left positive;
   * `CAMERA_HEIGHT_IN` — lens height above the floor;
   * `CAMERA_PITCH_UP_DEG` — tilt above level.
4. USB-C cable from the Limelight to the Control Hub's **blue (USB 3) port**; strain-relief both ends.
5. Power the robot and wait about 20 s for the Limelight's light.

## 2. Driver Station configuration

Configure Robot → scan → the Limelight shows up as an Ethernet/USB device. **Rename it `limelight`** (this is
`BioBuzzConstants.Hardware.LIMELIGHT`), save and activate the configuration.

## 3. Pipeline 0 (in the Limelight web UI)

Connect a laptop to the Limelight (USB) and open `http://limelight.local:5801` (or `http://172.28.0.1:5801`).
The code uses **pipeline index 0** (`Vision.PIPELINE_INDEX`) and checks at run time that it is selected.

| Tab | Setting | Value |
|---|---|---|
| Input | Pipeline type | **Fiducial Markers** |
| Input | Resolution | 1280×960 (best 3D accuracy; use 640×480 only if the frame rate is too low) |
| Input | Black level | 0 |
| Input | Sensor gain | start at 15 |
| Input | Exposure | start low; raise until a cluster of 4 tags is detected solidly from the shooting distance, then drive and rotate and lower it until detection starts to fail, then back off a little |
| Standard | Family | **AprilTag Classic 36h11** (the code expects the family string `36H11C`) |
| Standard | Marker size | **82.55 mm** (3.25 in). The 3A default is 101.6 mm, so this MUST be changed |
| Standard | ID filter | **30, 31, 32, … 45** |
| Standard | Detector downscale | 2 to start (try 1 if range is short) |
| Standard | Crop / area | none; keep the minimum area at its lowest value |
| Advanced | **Full 3D** | **ON** (without it the code only gets bearings, no range) |

The camera pose you can enter on the Advanced tab ("Limelight pose in robot space") is **not used** by the code:
it applies its own mount constants (step 1). It is harmless to leave it at zero.

Back up the pipeline after every change: use the UI's *download* button next to the pipeline name, and also from
a laptop connected to the camera:

```bash
curl "http://limelight.local:5807/pipeline-atindex?index=0" -o pipeline0.json
```

(port 5801 is the web UI, port 5807 is the REST interface the robot code also uses). Keep the file in the repo
under `docs/limelight/` so it can be restored; flashing new firmware erases pipelines. The Limelight's
pipeline file format is not publicly documented, so the code never uploads a pipeline: it only checks that
pipeline 0 is selected.

## 4. Check it with the Limelight Diagnostic OpMode

Put the robot on the field in the **default start pose** (center of tile C1 for RED, front against the audience
wall; BLUE is the half-turn of that), run **Limelight Diagnostic**, choose the alliance with D-pad left/right
during INIT, then check, in this order:

1. **The frame line** says `OK`, `new=true` flickering, and `tags` > 0. If `tags` is 0: pipeline, exposure,
   pitch, or the HIVE tilted so the camera sees nothing from this side.
2. **Tag IDs** are 30–45, in the right group (red audience side 34–37, red far side 30–33, blue audience side
   38–41, blue far side 42–45).
3. **No 3D?** If a tag line says "no 3D", Full 3D is off.
4. **Left/right.** Slide the robot to its LEFT. The cluster's `left` value must grow. If it shrinks, flip
   `Vision.CAMERA_SPACE_CONVENTION`. (The Limelight's own documentation contradicts itself on this, which is why
   it is a setting.)
5. **tx sign.** `tx` must be positive for a tag on the right of the image. If not, flip `TX_POSITIVE_IS_RIGHT`.
6. **UP / DOWN.** For the cell that is tipped up toward the robot the diagnostic must say `UP`. (The red
   audience-side cell is UP at the start of a match; the blue far-side cell is UP.) If it says `DOWN` or
   `UNKNOWN`, check the tx sign and the camera roll (the camera must be upright).
7. **Implied position.** "IMPLIED robot position … error vs stated" should be within about an inch. If it is
   consistently off, correct `CAMERA_FORWARD_IN`, `CAMERA_LEFT_IN`, `CAMERA_HEIGHT_IN` and `CAMERA_PITCH_UP_DEG`
   until it agrees. A height error shows up as a range error; a pitch error shows up as a range error that grows
   with distance.
8. **Range check.** Hold the robot at 38, 59 and 79 in from a cluster and compare the reported range with a tape
   measure.

9. **Drift correction (optional, last).** Once step 7 agrees within about an inch at several places on the
   field, set `Vision.POSE_CORRECTION_ENABLED = true`. Then drive away, let the robot drift (or push it a few
   inches by hand), stop in view of the HIVE and watch the TeleOp's "Drift correction" line: it should say
   `applied` and the Pose should settle back to the true spot within a second or two. If the pose jumps the wrong
   way, the camera constants from step 7 are still off: turn it back off and fix them.

Then delete the finished items from `BioBuzzConstants.PLACEHOLDERS`, change their markers from ADD to VERIFY or
TUNE, and commit.

## 5. If the robot refuses to shoot

The TeleOp's telemetry says why. The two camera-related reasons are:

* **HIVE_UNCONFIRMED** — the camera is working but has not seen the up cell's tags. Fix the pipeline/mounting
  above, or press the "other cell is up" button (D-pad down) once to confirm the robot's belief yourself. While
  you are still setting the camera up you can set `AutoShoot.REQUIRE_HIVE_CONFIRMATION = false`; turn it back on
  before a match, because firing at the down cell is a foul (rule G417).
* **TAGS_LOST** — the tags vanished mid-volley. That is what a HIVE tip looks like from the shooting side, so
  the feeder pauses and the robot re-targets the other side.
