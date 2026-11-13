# Start here: your first day with the robot

This is the one guide for the robot day. You do not need to have run an FTC robot before. Do the steps in
order, tick each box, and read the "You should see" line after every step. If you do not see it, read the
"If not" line. If that does not help, ask a mentor before you go on.

**Contents**

* [What this robot does and what you need](#what-this-robot-does-and-what-you-need)
* [How to read this guide](#how-to-read-this-guide)
* [Get the code onto the robot and connect](#get-the-code-onto-the-robot-and-connect) (steps 1 to 6)
* [Run the Robot Check](#run-the-robot-check) (steps 7 to 10)
* [Measure the robot](#measure-the-robot) (steps 11 to 21)
* [Tune driving with Pedro](#tune-driving-with-pedro) (steps 22 to 31)
* [Set up the Limelight camera](#set-up-the-limelight-camera) (steps 32 to 44)
* [Build the shot table](#build-the-shot-table) (steps 45 to 51)
* [Controller map](#controller-map) (every button, for every OpMode)
* [Try auto-shoot on the field mat](#try-auto-shoot-on-the-field-mat) (steps 52 to 59)
* [Autonomous](#autonomous) (steps 60 to 62)
* [If something goes wrong](#if-something-goes-wrong)
* [How it works, in ten lines](#how-it-works-in-ten-lines)
* [Measure later, for the planned autonomous routes](#measure-later-for-the-planned-autonomous-routes)

## What this robot does and what you need

**The game.** The game is BIOBUZZ. Each alliance has one **HIVE** with two **cells**, one on the audience side and
one on the far side. Exactly one cell is tipped **UP** (it can score). You score by launching **POLLEN** (the balls)
into the UP cell. Enough POLLEN **tips** the HIVE: the UP cell swings down and the other cell comes up. At the
start of a match the UP cell holds 3 **NECTAR** (heavier balls), so the first tip needs only 3 POLLEN. We start with 4
POLLEN, so one is spare. After a tip the new UP cell is empty, so every later tip needs 8 POLLEN. A robot may hold at most
4 POLLEN. Launching into the DOWN cell, or into the outside of a cell, is a foul (rule G417).

**The robot.** It is a goBILDA StarterBot with mecanum wheels (wheels that let it slide sideways). The
**intake** is at the front. The **shooter** is at the back: a **flywheel** (a fast spinning wheel that throws the
POLLEN) and a **feeder** (also called the indexer: a servo that pushes one POLLEN at a time into the flywheel). A
**Limelight** camera on top faces the back, like the shooter. It finds **AprilTags** (black and white square codes
with a number) on the HIVE.

**What it can do for you.** In TeleOp (the driver-controlled part of a match), **hold D-pad up** and the robot drives
itself to the closest legal spot in front of the UP cell, turns its back to the cell, aims with the camera and
shoots every POLLEN it holds. Let go and you have control again.

**You need:**

* a laptop with Android Studio and this project;
* the Robot Controller (**RC**: the Control Hub, or a phone, on the robot) and a **Driver Station** (**DS**: the phone or tablet app you use to start programs);
* a PlayStation-style gamepad;
* a charged robot battery, USB cables, a tape measure, painter's tape, a ruler;
* blocks to hold the robot with its wheels in the air;
* a big sheet of paper and a marker pen (for step 12);
* a clear stretch of smooth floor, about 6 ft (180 cm) in each direction;
* a field mat or field with a HIVE, and some POLLEN.

**Safety rules for the whole day:**

* Put the robot on blocks (all four wheels in the air) whenever a step says so.
* Keep hands, hair and loose sleeves away from the wheels, the front rollers and the back flywheel.
* Take the POLLEN out of the robot before a motor test (the shot table steps need one POLLEN).
* One person always holds the Driver Station and is ready to press **STOP**.
* When the robot drives itself, stand clear. Let go of D-pad up to take control back.
* Shoot only into the cell that is UP.

## How to read this guide

* Steps have a box and a number: `- [ ] 12.` Numbers go from 1 to 62 without a gap.
* Names in `code style` are real names in the code. Units are in the name: `_IN` is inches, `_DEG` degrees, `_SEC` seconds, `_TPS`
  ticks per second. A **tick** is one count of a motor's encoder (its built-in rotation counter), so "ticks per
  second" is a speed.
* An **OpMode** is one program that the Driver Station can start. Its name is shown in quotes, like "00 Robot Check".
* The **robot configuration** is the list of motors, servos and sensors, each with a name, that you save on the Driver
  Station. The code finds every part by that exact name, capital letters included.
* ★ marks the measurements that matter most for shooting.

**The constants file.** Every number you will change is in one file. In Android Studio press `Shift` twice and type
`BioBuzzConstants`. Its path is `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzConstants.java`. A **constant** is a named number.
Names are written `Group.NAME`, for example `Robot.FRONT_EXTENT_IN` is the constant `FRONT_EXTENT_IN` inside the
block called `Robot`. Press `Ctrl+F` to find a name.

Above every number is a comment that starts with one word. You will see these words on the robot screen too:

| Word | Meaning |
|---|---|
| FIXED | From the rule book or the field drawing. Do not change. |
| ADD | A placeholder that waits for your measurement. |
| VERIFY | Probably right. Check it once. |
| TUNE | A start value that you will adjust by testing. |

**How to change a number** (every time):

1. Type the new number in the constants file.
2. If its word was ADD, change the word in the comment to VERIFY (or TUNE), and delete the number's name from the
   list called `PLACEHOLDERS` near the top of the file. (The TeleOp prints that list at INIT as "Placeholders still to
   measure". A test checks that the list and the words agree.)
3. Press the green Run button again (step 2) to install the new code on the robot. It takes about a minute.

A second file, `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`, holds the settings for **Pedro**, the
driving library. We call it the **Pedro file**. Pedro's own tuners write its numbers (steps 22 to 31).

## Get the code onto the robot and connect

- [ ] **1. Open the project and charge the battery.** Open this project in Android Studio and wait until the
  progress bar at the bottom ends. Charge the robot battery.
  **You should see:** no red errors in the build messages.
  **If not:** ask a mentor. Do not change code files to "fix" a build error.

- [ ] **2. Install the code on the robot.** Plug the laptop into the Robot Controller with a USB cable. Choose `TeamCode`
  in the box next to the green Run triangle and press Run.
  **You should see:** "BUILD SUCCESSFUL" and then the app opens on the Robot Controller.
  **If not:** try another cable (some cables only charge). Make sure the Robot Controller is switched on.

- [ ] **3. Switch the robot on and pair the Driver Station.** On the Driver Station open the three-dot menu, then Settings,
  then the pairing page for your Robot Controller, and choose your robot.
  **You should see:** the Driver Station shows the robot battery voltage and a ping time (in milliseconds).
  **If not:** switch the robot off and on, and pair again.

- [ ] **4. Open the robot configuration** (Driver Station, three-dot menu, Configure Robot). Make one entry for each
  part in this table. **The names must be exactly as written.** The port does not matter, the name does.

| Name | Part | Type |
|---|---|---|
| `frontLeft`, `backLeft`, `frontRight`, `backRight` | the four wheel motors | motor |
| `pinpoint` | goBILDA Pinpoint (position computer) | I2C device |
| `flywheel` | the flywheel at the back | motor |
| `feeder` | the feeder at the back | continuous-rotation servo |
| `intakeMotor` | the front roller | motor |
| `servoLeft`, `servoRight` | the two corner intake servos | continuous-rotation servo |
| `limelight` | the Limelight 3A camera | USB device |
| `imu` | the Control Hub's built-in gyro (only the backup drive uses it) | built in |

  These are `BioBuzzConstants.Hardware`. A **continuous-rotation servo** spins like a motor instead of turning to an
  angle. Save the configuration and make it the active one.
  **You should see:** the configuration name on the Driver Station main screen.
  **If not:** a name that is wrong by one letter is the usual reason. Check the capital letters.

- [ ] **5. Look at the OpMode lists.**
  **You should see:** under TeleOp: "00 Robot Check" (group "1 Start here"), "BioBuzzRobot", "Limelight Diagnostic" and "Shooter Tuner" (group "AutoShoot").
  Under Autonomous: "RedAlliance_B1", "RedAlliance_C1", "RedAlliance_LeavePark", "PedroPathingAuto_Forward", "PedroPathingAuto_Strafe" and "AutoPath".
  **If not:** step 2 did not finish. Run again.

- [ ] **6. Find your gamepad buttons.** Open the [Controller map](#controller-map) below once and look at the picture. You
  will use "cross" (the X button) and "circle" (the O button) a lot.

## Run the Robot Check

The Robot Check is a guided test of every part of the robot. It walks you through it one check at a time. **Nothing moves until you
press a button**, and every test is short and gentle.

- [ ] **7. Prepare.** Put the robot on blocks so all four wheels hang in the air. Take the POLLEN out. Use a fresh
  battery. One person holds the Driver Station ready to press STOP.

- [ ] **8. Start it.** On the Driver Station choose TeleOp, group "1 Start here", "00 Robot Check". Press INIT.
  Connect the gamepad: press **Options and cross** together (on other gamepads: Start and A). Press PLAY.
  **You should see:** "Step 0 of 10: Safety first".
  **If not:** the gamepad is not connected. The INIT screen says "Gamepad 1 is now:" and the gamepad type.

- [ ] **9. Follow the screen, one check at a time.** The screen says "Step 3 of 10" and so on. In this guide we call those **checks**, so they are not mixed up with the numbered steps of the guide.
  Press cross (yes, continue) to start each check. The checks are:

  * **Check 0, Safety first:** you promise the robot is on blocks and everyone is clear.
  * **Check 1, Battery:** reads the volts. 12.4 V or more passes. Below 11.0 V fails: charge the battery.
  * **Check 2, Hardware names:** checks that every name from step 4 is in the configuration.
  * **Check 3, Drive wheels:** each wheel spins gently for 0.6 s. You say if the top of the wheel moved toward the FRONT (the intake end).
  * **Check 4, Pinpoint odometry:** **odometry** means tracking where the robot is with small wheels that roll on the
    floor. Take the robot off the blocks. Push it straight forward about 12 in (a ruler). Then turn it on the spot
    a quarter turn counter-clockwise. The motors do not move: you move the robot by hand.
  * **Check 5, Intake:** the roller, then each corner servo, spin for 1.5 s. You say if each pulled POLLEN inward.
  * **Check 6, Flywheel:** it spins up to 1200 ticks per second. Keep hands away from the back.
  * **Check 7, Feeder:** it turns forward for 1 s and then backward for 0.5 s. You say if it turned the right way.
  * **Check 8, Limelight camera:** hold a HIVE tag (number 30 to 45) in front of the camera at arm's length. The camera faces the BACK.
  * **Check 9, Gamepad:** press every button and both triggers once. Each one shows SEEN.
  * **Check 10, Summary:** the list of results (step 10).

  Put the robot back on the blocks for checks 5 to 7 if you took it off for check 4.
  **You should see:** a result for each check: PASS, WARN or FAIL.
  **If not:** press circle (no) if anything looks wrong. Circle also stops a running motor at once.

- [ ] **10. Read the summary and fix every FAIL.** The last screen lists every check with PASS, WARN or FAIL, and says what
  to fix. Use the D-pad up and down buttons to scroll. Fix each FAIL (the table [If something goes wrong](#if-something-goes-wrong) helps), run the
  check again, and repeat until there is no FAIL. If you skipped a check with triangle, run the whole Robot Check again
  without skipping before a match.
  **You should see:** a line that says "RESULT: ... 0 FAIL".
  **If not:** a wheel that turned the wrong way is fixed in step 23. A missing Limelight is fixed in steps 32 to 44. You can come back here after those steps.

## Measure the robot

The code treats the robot as **one point** with a direction. That point is the **center of rotation**: the
spot that stays still when the robot turns on the spot. The point plus the direction is called the **pose**
(position in inches, and **heading**: the way the robot faces). The Pinpoint reports the pose.

**The center of rotation is not the middle of the robot.** The outline is 17.80 in long and 17.80 in wide, but the
intake sticks out at the front. From the center of rotation the outline is about 9.8 in to the front, 8.0 in to the
rear and 8.9 in to each side. Every check of "does the robot fit here?" uses these four distances. A wrong number here
makes the robot hit walls or stand in the wrong place.

**The field frame.** Positions are measured in inches from the inside corner where the red wall meets the audience wall.
The wall means the inside face, the face a robot can touch. A robot that touches a wall with its front has its center point
at its own front distance from that wall (the file's `Robot.FRONT_EXTENT_IN` is the distance from the center point to the front edge).
(Stand behind the audience wall, looking at the field: the red wall is on your left.)

- [ ] **11. Open the constants file** (see "How to read this guide") and find the block called `Robot`.
  **You should see:** the numbers you will change, each with a comment that starts with FIXED, ADD, VERIFY or TUNE.

- [ ] **12. Find the center of rotation.** Tape the big sheet of paper to a smooth floor. Set the robot on it. Tape the
  marker pen to the robot, pointing down, at your best guess: halfway between the left and right wheels, and about halfway
  between the front and rear wheels. Switch the robot on and turn it **slowly, by hand, on the spot** (do not drive it).
  **You should see:** the pen draws only a dot. That dot is the center of rotation.
  **If not:** if the pen draws a circle, the point is at the middle of that circle. Move the pen there and try again. Then put
  a strip of painter's tape on the robot over the dot and write "center of rotation" on it.

- [ ] **13. ★ Measure the four distances from that mark to the edges of the robot.** Use the tape measure, in a straight line, in inches.
  Write the numbers in the constants file, block `Robot`:

  | Constant | Measure from the mark to the... | File starts at |
  |---|---|---|
  | `Robot.FRONT_EXTENT_IN` | front edge (the outer edge of the two flat intake wheels) | 9.77 |
  | `Robot.REAR_EXTENT_IN` | rear edge (the back lip, where the ball leaves) | 8.03 |
  | `Robot.LEFT_EXTENT_IN` | left edge (the outer face of the left wheels) | 8.90 |
  | `Robot.RIGHT_EXTENT_IN` | right edge (the outer face of the right wheels) | 8.90 |

  Example: you measure 9.8 in to the front, so type `9.8`.
  The sums must match the size of the robot: front + rear = `Robot.LENGTH_IN` and left + right = `Robot.WIDTH_IN` (both 17.80 in).
  **You should see:** later, at INIT of the TeleOp, no error.
  **If not:** if the sums do not match, the TeleOp refuses to start at INIT and the message names the two numbers that disagree.
  Measure again, or fix the size in step 14.

- [ ] **14. Measure the whole outline.** With the robot standing normally, measure the longest length and width in inches.
  Write them in `Robot.LENGTH_IN` and `Robot.WIDTH_IN` (the drawing says 17.80 and 17.80). The robot must fit in the 18 in start cube (rule R102).

- [ ] **15. ★ Measure where the ball leaves the robot.** The ball leaves the ramp at the back. Tape a measure straight
  back from the center-of-rotation mark to the point where the ball leaves the ramp. Write it as a **negative** number
  (behind the center) in `Robot.LAUNCH_POINT_FORWARD_IN`. Example: the tape says 6.6 in, so type `-6.6`. The robot drawing says
  about 6.58 in behind the center, but use your tape, not the drawing. If the exit is not on the center line, measure how far it is to the left (positive) or right
  (negative) in `Robot.LAUNCH_POINT_LEFT_IN` (0.0 if it is centered).
  **You should see:** a negative number, usually a little smaller than your rear distance in step 13 (the ball leaves inside the outline).
  **If not:** measure again. A positive number means you measured in front of the center.

- [ ] **16. Measure how high the ball leaves.** With the tape from the floor to where the ball leaves the ramp (in
  inches), write it in `Robot.LAUNCH_HEIGHT_IN`. The drawing says about 8.71 in. Check the launch angle
  `Robot.LAUNCH_ANGLE_DEG` (about 68.5 degrees) with a protractor or a phone level app. These two numbers only feed the
  physics check on the Shooter Tuner screen.

- [ ] **17. Measure the two flat intake wheels** (the discs that lie flat at the front corners). Three numbers, with a ruler, in inches:
  `Robot.INTAKE_REACH_IN` (how far a wheel's front tip sticks out in front of the metal frame; the file says 2.7),
  `Robot.INTAKE_WHEEL_OFFSET_IN` (how far the middle of each wheel is from the robot's center line; 6.1) and
  `Robot.INTAKE_WHEEL_DIAMETER_IN` (how wide a wheel disc is; 2.8).

- [ ] **18. Measure your field once.** Every field number in the code comes from the official field drawing. Real fields differ by about 1 in.
  Measure, along the middle of the field: (a) from the inside face of the red wall to the inside face of the blue wall (the code says
  `Field.SIZE_IN` = 141.348 in; the rule book says "about 144"); (b) from the audience wall to the near end of the HIVE's two foot bars (the
  metal bars of the HIVE frame that lie on the floor): the code uses 51.2 in.
  **You should see:** numbers within about 1.5 in of the code.
  **If not:** tell a mentor before you change `Field.SIZE_IN`: other numbers move with it.

- [ ] **19. Check the one measured good shot.** The team once measured a good shot with the robot's front touching the audience wall, in line with the HIVE: **37.7 in** from the ball exit to the cell, at about 1200 ticks per second.
  Work out the same range from your numbers: 51.2 (audience wall to the HIVE foot bar, or your own number from step 18) minus (`Robot.FRONT_EXTENT_IN` plus the distance from step 15, without its minus sign) plus 2.95.
  Example: 51.2 - (9.77 + 6.58) + 2.95 = 37.8 in.
  Then re-measure the distance from the ball exit to the cell opening center with a tape: stand the robot front-on to the audience wall, in line with the HIVE, and hold the tape level from the lip of the ramp to the middle of the cell's opening (look from the side).
  **You should see:** the sum and the tape agree within about 0.5 in, near 37.7 in.
  **If not:** if they disagree, measure steps 15 and 18 again. If the sum and the tape agree with each other but say about 36.4 in, the old 37.7 in was a little long: in step 47 change the table row `{37.7, 1200}` to the range your
  tape says. Its speed stays 1200.

- [ ] **20. Where the Pinpoint pods are.** The two small tracking wheels ("pods") of the Pinpoint touch the floor. The x pod measures forward
  and backward motion. The y pod measures sideways motion. With a tape, measure from the center-of-rotation mark to each pod
  (forward or back, left or right, in inches) and write the four numbers down. You need them in step 25.

- [ ] **21. Finish the list.** For every ADD number you replaced in steps 13 to 17, change ADD to VERIFY in its comment and delete its name from `PLACEHOLDERS` (see "How to change a number"). Run the code again (step 2).
  **You should see:** the TeleOp's INIT screen no longer lists the names you did. (Names that start with `Auto.` are for planning autonomous routes on a computer. Ignore them today.)
  **If not:** a name still listed means it is still in `PLACEHOLDERS`.

## Tune driving with Pedro

**Pedro Pathing** is the library that drives the robot to a point or along a line using its pose. **Foresight** is
Pedro's way of turning "go there" into wheel power. It needs to know how this robot speeds up and brakes. Pedro has four **tuners**
(small programs that measure the robot). They are not in the Driver Station list. They run in a **web page**.
Run them in this order: each one needs the one before it to be right.

- [ ] **22. Open the tuner page.** Join the laptop to the same Wi-Fi network as the Driver Station (the Robot Controller's own
  network). **Keep the Driver Station connected.** In a web browser open `http://192.168.43.1:10158`.
  **You should see:** a page that lists the tuners: Mecanum Tuner, Pinpoint Tuner, Foresight Tuner and Tests.
  **If not:** check that the laptop is on the Robot Controller's Wi-Fi and the Driver Station shows the robot connected. Reload the page.

- [ ] **23. Mecanum Tuner** (robot on blocks). It asks for the four motor names, then spins each wheel one at a time. Enter
  `frontLeft`, `frontRight`, `backLeft` and `backRight`. After each wheel, say whether it spun **Forward** (the top of the wheel moved toward the front
  of the robot) or **Reversed**. At the end it shows a block of code. Copy only the four `Direction` lines (for example
  `c.frontLeftDirection.set(...)`) into `drivetrainConfig` in the Pedro file.
  **You should see:** four lines with FORWARD or REVERSE.
  **If not:** if the wrong wheel spun, a motor is plugged into the wrong port. Fix the cable and start again.

- [ ] **24. Pinpoint Tuner, first half: directions.** Enter the name `pinpoint` and choose your pod type (the code uses the goBILDA 4-bar pod).
  Then push the robot straight **forward** when it asks, and straight to the **left** when it asks.
  The tuner tells you whether each pod must be Forward or Reversed.

- [ ] **25. Pinpoint Tuner, second half: offsets.** The last screen is "Offsets Identification". Turn the robot slowly on the spot **counter-clockwise** until
  the "heading" on the screen reads about **3.14** (half a turn, in radians), then press stop. It prints the two pod offsets in inches.
  **Run the whole Pinpoint Tuner three times** and use the **average** of the three results for each offset. Compare them with your tape
  numbers from step 20: they should agree within about 0.5 in. Copy `xPodOffset`, `yPodOffset`, `xPodDirection` and `yPodDirection` into `localizerConfig` in the Pedro file.
  **You should see:** offsets that are not tiny. (The numbers the file had at the start, -0.167 and 0.037 in, are far smaller than a real robot's.)
  **If not:** if the three runs differ by more than about 0.5 in, the spin was uneven. Spin slower and exactly to 3.14.

- [ ] **26. Foresight Tuner** (6 ft / 180 cm of clear floor in front of and beside the robot, fresh battery). It runs a long list of short tests:
  top speed forward and sideways, braking, turning and position correction. The robot will drive, brake and **coast past its target**
  on every run. Keep the start values it shows (drive 48 in; 30 in/s; braking 36 in, minimum 15 in). Stand clear.
  At the end it prints a complete `foresightConfig` block. **Paste it over the old `foresightConfig`** in the Pedro file and delete the old "PROVISIONAL" note.
  The numbers in the file at the start are Pedro's example numbers, not yours.
  **You should see:** the tuner ends and shows the block. (Pedro's example numbers in the file say 72.7 in per second forward and 52.3 sideways. Yours will be different.)
  **If not:** if a test stops or the robot barely moves, check the battery and start the Foresight Tuner again.

- [ ] **27. Run Tests: Hold Test and Line Test.** In the tuner page choose Tests. The Hold Test makes the robot hold its place. The Line Test (48 in) makes it drive a line.
  Push the robot a little during the Hold Test.
  **You should see:** the robot returns to its place, and ends the Line Test within about 1 to 2 in of its target with no wobble.
  **If not:** run the Foresight Tuner again (step 26).

- [ ] **28. Run "PedroPathingAuto_Forward".** On the Driver Station choose Autonomous, "PedroPathingAuto_Forward", INIT, PLAY, with 4 ft (120 cm) of clear floor in front. The robot drives 48 in forward and 48 in back.
  **You should see:** it ends within about 1 to 2 in of where it began.
  **If not:** run steps 25 and 26 again.

- [ ] **29. Run "PedroPathingAuto_Strafe".** The same, sideways: 48 in to the robot's right and back, so keep 4 ft (120 cm) of clear floor to the right.
  **You should see:** it ends within about 1 to 2 in of where it began.

- [ ] **30. Drive the TeleOp.** Choose TeleOp, "BioBuzzRobot", INIT. Choose the alliance with D-pad left (red) or D-pad right (blue). Press PLAY. Drive with the sticks on the floor.
  **You should see:** left stick forward = robot forward, left stick right = robot slides right, right stick right = robot turns clockwise.
  **If not:** if the whole robot goes the opposite way, change `Controls.MANUAL_FORWARD_SIGN`, `Controls.MANUAL_STRAFE_SIGN` or `Controls.MANUAL_TURN_SIGN`
  from `1.0` to `-1.0` (one at a time). If the robot twists or jerks instead of driving straight, one wheel direction is wrong: redo step 23.

- [ ] **31. Run the Robot Check again** (step 8) and check that "Drive wheels" and "Pinpoint odometry" pass.
  **You should see:** PASS for both. In the Pinpoint check, turning the robot on the spot should not slide its position more than about 3 in.
  **If not:** a position that slides while turning means the pod offsets are wrong: repeat steps 20 and 25.

## Set up the Limelight camera

A **pipeline** is one saved set of detection settings inside the Limelight. The code uses **pipeline 0** and checks that it is selected.

- [ ] **32. Mount the camera.** Fix it firmly, **facing the back** (the same way as the shooter), upright (not tilted sideways), on a part that does
  not flex. **Tilt it up** by 30 to 40 degrees: the tags are 36 to 50 in above the floor and the Limelight 3A sees only 42 degrees up and down.
  Connect it with a USB-C cable to the Control Hub's **blue** USB 3 port, with strain relief at both ends.
  **You should see:** the Limelight's light, about 20 s after the robot is switched on.
  **If not:** a different cable or port. The blue USB 3 port is the right one.

- [ ] **33. ★ Measure the camera** from the center-of-rotation mark, and write the numbers in the block `Vision`:

  | Constant | What to measure | File starts at |
  |---|---|---|
  | `Vision.CAMERA_FORWARD_IN` | lens position, forward is positive. The camera is behind the center, so the number is negative | -6.0 |
  | `Vision.CAMERA_LEFT_IN` | sideways offset, left is positive | 0.0 |
  | `Vision.CAMERA_HEIGHT_IN` | lens height above the floor | 13.0 |
  | `Vision.CAMERA_PITCH_UP_DEG` | tilt above level (use a phone level app) | 35.0 |

  Example: the lens is 5.5 in behind the mark, so type `-5.5`.

- [ ] **34. Name it in the robot configuration.** Configure Robot, scan for devices, and rename the Limelight `limelight` (it appears as a USB device). Save and activate.
  **You should see:** the name `limelight` in the configuration.

- [ ] **35. Open the Limelight settings page.** Connect the laptop to the Limelight (USB) and open `http://limelight.local:5801` (or `http://172.28.0.1:5801`).
  **You should see:** the Limelight web page with the camera picture.
  **If not:** wait 20 s after power-on and try the second address.

- [ ] **36. Set up pipeline 0** exactly like this:

  | Tab | Setting | Value |
  |---|---|---|
  | Input | Pipeline type | Fiducial Markers |
  | Input | Resolution | 1280x960 (640x480 only if the frame rate is too low) |
  | Input | Black level | 0 |
  | Input | Sensor gain | start at 15 |
  | Standard | Family | AprilTag Classic 36h11 (the code expects the name `36H11C`) |
  | Standard | Marker size | **82.55 mm** (3.25 in). The default is 101.6 mm, so you MUST change it |
  | Standard | ID filter | 30, 31, 32 ... 45 |
  | Standard | Detector downscale | 2 (try 1 if the range is short) |
  | Standard | Crop / area | none; keep the minimum area at its lowest |
  | Advanced | **Full 3D** | **ON** (without it the robot gets angles but no distances) |

  **Exposure:** start low. Raise it until a cluster of 4 tags is detected solidly from the shooting distance. Then drive and turn the robot, lower it until detection
  starts to fail, and back off a little. The camera pose on the Advanced tab is not used by the code. Leave it at zero.
  **You should see:** a green box around each HIVE tag in the picture.

- [ ] **37. Back up the pipeline.** Use the download button next to the pipeline name, or run this on the laptop:
  `curl "http://limelight.local:5807/pipeline-atindex?index=0" -o pipeline0.json`
  Keep the file in the folder `docs/limelight/` of this project. New camera firmware erases pipelines. The code never uploads a pipeline: it only checks that pipeline 0 is selected.

- [ ] **38. Put the robot on the start place** (for the Limelight Diagnostic). For the RED alliance: front touching the audience wall, the robot centered on tile C1 (the third tile
  from the red wall, in the row along the audience wall), in line with the HIVE. For BLUE: the same place turned half a turn around the field center, with the front touching the wall opposite
  the audience wall, in tile D6. (The code calls this the default start pose.)

- [ ] **39. Run "Limelight Diagnostic".** TeleOp, group "AutoShoot". Press INIT. Choose the alliance with D-pad left (red) or D-pad right (blue). Wait until the "Limelight" line shows the camera running.
  Press PLAY. The robot does not drive. **A frame** is one result from the camera. A **tag** is one AprilTag. A **cluster** is the four tags of one HIVE cell.
  **You should see:** a "Frame" line with `OK`, and a line for every tag it sees.
  **If not:** see "If something goes wrong", the camera rows.

- [ ] **40. Frame and tag checks.** (a) The Frame line says `OK`, `new=true` flickers on and off, and `tags` is more than 0. If `tags` is 0: check the pipeline, exposure and pitch, and that the camera can see the HIVE tags.
  (b) The tag numbers are 30 to 45, in the right group: red audience side 34-37, red far side 30-33, blue audience side 38-41, blue far side 42-45.
  (c) If a tag line says "no 3D", Full 3D is off (step 36).

- [ ] **41. The three-move test.** This takes one minute and shows the camera's directions are right. The camera looks out of the BACK, so the picture's **right** side is the robot's **left**.
  Hold the robot (by hand) and point the camera straight at a tag. Look at the tag's line on the screen (`tx`, `ty` in degrees: `tx` is how far right of the middle of the picture, `ty` how far above).
  1. **Distance.** Hold the lens about **24 in** from the tag (use the tape). One of the three numbers on the "Raw" line is about 24: it must be the **last** one (the distance straight out of the lens).
  2. **Sideways.** Slide the robot about 6 in to **its right**. The tag moves toward the right of the picture, so `tx` goes **up**, and the `left` number of the cluster **grows**.
     Slide the robot about 6 in to **its left** instead: `tx` goes **down**, and the cluster's `left` number **shrinks**.
  3. **Up and down.** Lift the back of the robot (the camera end) a few inches with a block. The tag sinks in the picture, so `ty` goes **down**. Lower it: `ty` goes up.
  **You should see:** all three moves behave as written.
  **If not:** do not change anything yet. Ask a mentor: the settings are `Vision.CAMERA_SPACE_CONVENTION`, `Vision.TX_POSITIVE_IS_RIGHT` and `Vision.TY_POSITIVE_IS_UP`. The values in the file are
  believed correct.

- [ ] **42. UP or DOWN.** For the cell that is tipped up toward the robot, the screen must say `UP`. At the start of a match the RED audience-side cell is UP, and the BLUE far-side cell is UP.
  **You should see:** "CLUSTER ... UP  tags=4".
  **If not:** `DOWN` or `UNKNOWN` means the camera is not upright, or step 41 is not right.

- [ ] **43. Implied position and range.** Find the line "IMPLIED robot position ... error vs stated: dx= dy=". It should be within about **1 in**. Then hold the robot at 38, 59 and 79 in from a cluster
  and compare the "range to aim point" with the tape.
  **You should see:** errors of about 1 in or less.
  **If not:** correct `Vision.CAMERA_FORWARD_IN`, `CAMERA_LEFT_IN`, `CAMERA_HEIGHT_IN` and `CAMERA_PITCH_UP_DEG` until they agree. A height error shows as a range error. A pitch error shows as
  a range error that grows with distance.

- [ ] **44. Tidy up.** Change the markers of the four camera numbers from ADD to VERIFY, and delete `Vision.CAMERA_FORWARD_IN`, `Vision.CAMERA_LEFT_IN`, `Vision.CAMERA_HEIGHT_IN` and
  `Vision.CAMERA_PITCH_UP_DEG` from `PLACEHOLDERS`. Run the code again (step 2).
  The camera also corrects the robot's position when the Pinpoint drifts. That is **off** (`Vision.POSE_CORRECTION_ENABLED` is `false`) until you have checked the mount (step 59).

## Build the shot table

The shooter has one fixed angle. To reach the HIVE from different distances, you change how fast the flywheel spins. The **shot table** lists, for several
distances, the flywheel speed that scores. It is `Shooter.SHOT_TABLE`. Each row is `{range in inches, speed in ticks per second}`. The three rows in the file are
placeholders except one: `{37.7, 1200}`, the team's one measured good shot.

**The range.** It is the distance from the **ball exit** (step 15) to the aim point (the middle of the cell's opening). To measure it: hold a tape from the ball exit to the
**foot line** of the HIVE frame at the end you shoot into (the near edge of the foot bar), level with the floor, then **add 3 in** (2.95 in exactly: the aim point is that far inside the foot line). The one measured good shot: robot **front against the audience wall**,
in line with the HIVE, range 37.7 in, speed about 1200 ticks per second, slightly high (the best speed is probably a little lower).

- [ ] **45. Set up.** Put the robot at a measured range from the HIVE, in line with the cell (the cell axis is the middle of the 20 in wide opening). Load one POLLEN. Everyone stands clear of the back of the robot, where the POLLEN is thrown.

- [ ] **46. Run "Shooter Tuner".** TeleOp, group "AutoShoot". INIT, PLAY. Press cross to start the flywheel. D-pad up and down change the speed by 10 ticks per second (hold the left bumper for 50). Wait for `at speed: true`.
  Hold circle to feed one POLLEN and watch where it goes.
  **You should see:** a screen with "Flywheel RUNNING", the target and measured speeds, "Shots counted by dip", and your recorded rows.
  **If not:** if `at speed` never becomes true, see "flywheel never at speed" in the problem table.

- [ ] **47. Find the speed that scores, for at least three ranges between 30 and 44 in.** For each range, find the **lowest** speed that sends the POLLEN through the middle of the opening. Too fast flies over the cell. Hitting the outside of a cell is a
  foul. Set the range to your tape number with D-pad left and right (0.5 in per press, 2 in with the left bumper held). Press triangle to **record a row**. The rows appear on the screen and in the robot log. (Square clears them.)
  A robot against the audience wall, in line, reaches at most 37.7 in. For longer ranges stand to one side of the line (no more than 20 degrees: `AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG`). For shorter ones stand closer to the HIVE.
  If step 19 told you to move the 37.7 row, do it now: use the range your tape says and keep the speed.
  **You should see:** at least three rows like `{30.0, 1150},`.

- [ ] **48. Paste the rows into `Shooter.SHOT_TABLE`**, sorted by range, replacing the old rows. Change its marker from ADD to TUNE and delete `Shooter.SHOT_TABLE` from `PLACEHOLDERS`.
  The robot only stands where the table covers: from the first range + `Shooter.RANGE_MARGIN_IN` (1.5 in) to the last range - 1.5 in. A table that is too narrow gives `NO_FEASIBLE_POSE`.

- [ ] **49. Look at the dip.** Each POLLEN makes the flywheel slow down a little. The tuner screen shows "Last feed: lowest speed ... (dip x%)". Set `Shooter.SHOT_DIP_FRAC` (file: 0.05) to about **half the smallest dip** you see. A dip of 4 % is 0.04, so you would type 0.02.
  The robot uses the dip to count shots.

- [ ] **50. Check the feeder.** `Shooter.FEEDER_FORWARD_POWER` (0.3) should push one POLLEN at a time. Check `Shooter.FEED_SECONDS_PER_SHOT` (0.9 s).
  If the feeder turns the wrong way when it feeds, set `Shooter.FEEDER_INVERTED` to `true`.

- [ ] **51. Run the code again** (step 2).

## Controller map

All buttons use gamepad 1, a PlayStation-style gamepad. "Cross" is the X button and "circle" is the O button. (L1, R1 are the bumpers on top; L2, R2 are the triggers; L3, R3 are the stick buttons.)

```
  L2 = left trigger                      R2 = right trigger
  L1 = left bumper                       R1 = right bumper
.----------------------------------------------------------.
|                                                          |
|               SHARE  [ touchpad ]  OPTIONS               |
|                                                          |
|  D-PAD                                     FACE BUTTONS  |
|     UP                                         TRIANGLE  |
|  LEFT  RIGHT                             SQUARE  CIRCLE  |
|    DOWN                                           CROSS  |
|                            PS                            |
|                                                          |
|    LEFT STICK                               RIGHT STICK  |
|    (push = L3)                              (push = R3)  |
|                                                          |
'----------------------------------------------------------'
```

### TeleOp: "BioBuzzRobot"

**Hold** means: keep it pressed. **Press** means: one quick press.

| Control | What it does | Setting that changes it |
|---|---|---|
| Left stick | Drive: forward and back, slide left and right (the directions are the robot's own). Pushing it hard (past 0.6 for 0.15 s) while the robot drives itself cancels the automatic drive. | `Controls.MANUAL_FORWARD_SIGN`, `MANUAL_STRAFE_SIGN`, `MANUAL_DRIVE_SCALE`, `STICK_DEADBAND`; `AutoShoot.DRIVER_OVERRIDE_STICK`, `DRIVER_OVERRIDE_SEC` |
| Right stick | Left and right: turn (right = clockwise). Up and down: not used. | `Controls.MANUAL_TURN_SIGN`, `MANUAL_TURN_SCALE`, `STICK_DEADBAND` |
| D-pad up | **Hold:** auto-score. The robot drives to the scoring spot, aims and shoots. Let go to stop and take over. | `Controls.SHOOT_BUTTON` |
| D-pad down | **Press:** "the other cell is up". Flips which cell the robot thinks is UP. | `Controls.TOGGLE_HIVE_BUTTON` |
| D-pad left | Before PLAY (INIT) only: choose the RED alliance. After PLAY: not used. | `Controls.ALLIANCE_RED_BUTTON` |
| D-pad right | Before PLAY (INIT): choose the BLUE alliance. After PLAY, **hold:** run the shooter flywheel slowly **backward** to free a jammed POLLEN. It stops when you let go and it cancels auto-score. | `Controls.ALLIANCE_BLUE_BUTTON` (INIT); `Controls.UNJAM_SHOOTER_BUTTON` and `Shooter.FLYWHEEL_UNJAM_POWER` (play) |
| Cross (X) | **Press:** intake roller on. It stays on. | fixed in the code |
| Circle (O) | **Press:** intake roller off. | fixed in the code |
| Triangle | **Press:** manual launch on. The flywheel spins up to 1200 ticks per second and the feeder pushes POLLEN once it is above 950. | fixed in the code; speeds `Shooter.MANUAL_TARGET_TPS`, `Shooter.MANUAL_MIN_TPS` |
| Square | **Press:** manual launch off. | fixed in the code |
| Left bumper (L1) | **Hold:** run the indexer (the feeder) slowly **backward** to free a jammed POLLEN. It stops when you let go and it cancels auto-score. | `Controls.UNJAM_INDEXER_BUTTON`; `Shooter.FEEDER_REVERSE_POWER` |
| Right bumper (R1) | **Hold:** run the feeder **forward** (push a POLLEN toward the flywheel). | fixed in the code; `Shooter.FEEDER_FORWARD_POWER` |
| Left trigger (L2) | **Hold** (pull past 0.3): the left corner intake servo pulls POLLEN in. | fixed in the code; `Controls.TRIGGER_THRESHOLD` |
| Right trigger (R2) | **Hold** (pull past 0.3): the right corner intake servo pulls POLLEN in. | fixed in the code; `Controls.TRIGGER_THRESHOLD` |
| Share | **Hold 0.6 s:** "the robot is on the default start pose". Resets its position. Works only when it is not auto-scoring. | `Controls.RESET_POSE_BUTTON`, `Controls.RESET_POSE_HOLD_SEC` |
| Options, PS, touchpad, left stick push (L3), right stick push (R3) | Not used. | none |

While the robot auto-scores, the intake, the corner servos, the manual launch buttons and the right bumper are switched off, because it may hold at most 4 POLLEN. A jam button cancels the auto-score and then works as written.
If you still hold D-pad up when you let go of a jam button, the auto-score can start again: let go of D-pad up first. The screen says **CLEARING A JAM** while you hold a jam button.

### "00 Robot Check"

| Control | What it does | Setting that changes it |
|---|---|---|
| Cross (X) | Yes. Continue. Start the check you are on. | `RobotCheck.YES_BUTTON` |
| Circle (O) | No, or "there is a problem". Stops a running motor at once. | `RobotCheck.NO_BUTTON` |
| Triangle | Skip this check (the safety check cannot be skipped). | `RobotCheck.SKIP_BUTTON` |
| Square | Do the check again. | `RobotCheck.REPEAT_BUTTON` |
| Touchpad | In the gamepad check: finish early. | `RobotCheck.GAMEPAD_FINISH_BUTTON` |
| D-pad up, D-pad down | Scroll the summary when it is longer than the screen. | `RobotCheck.SCROLL_UP_BUTTON`, `RobotCheck.SCROLL_DOWN_BUTTON` |
| Every button and both triggers | In the gamepad check they are only tested, they do nothing. | `RobotCheck.GAMEPAD_BUTTONS_TO_CHECK` |
| Sticks, L3, R3, PS | Not used. | none |

A press that comes less than 0.4 s after the screen changed is ignored (`RobotCheck.BUTTON_LOCKOUT_SEC`).

### "Limelight Diagnostic"

| Control | What it does | Setting that changes it |
|---|---|---|
| D-pad left | During INIT only: choose RED. | `Controls.ALLIANCE_RED_BUTTON` |
| D-pad right | During INIT only: choose BLUE. | `Controls.ALLIANCE_BLUE_BUTTON` |
| Everything else | Not used. The robot does not drive. | none |

### "Shooter Tuner"

| Control | What it does | Setting that changes it |
|---|---|---|
| D-pad up / down | **Press:** flywheel target speed +10 / -10 ticks per second. | `Shooter.TUNER_SPEED_STEP_TPS` |
| D-pad right / left | **Press:** recorded range +0.5 / -0.5 in. | `Shooter.TUNER_RANGE_STEP_IN` |
| Left bumper (L1) | **Hold** while you press D-pad: big steps (50 ticks per second, 2 in). | `Shooter.TUNER_SPEED_BIG_STEP_TPS`, `Shooter.TUNER_RANGE_BIG_STEP_IN` |
| Cross (X) | **Press:** flywheel start / stop. | fixed in the code |
| Circle (O) | **Hold:** feed POLLEN (only while the flywheel runs). | fixed in the code; `Shooter.FEEDER_FORWARD_POWER` |
| Triangle | **Press:** record a row with the current range and speed. | fixed in the code |
| Square | **Press:** forget all recorded rows. | fixed in the code |
| Everything else | Not used. | none |

## Try auto-shoot on the field mat

- [ ] **52. Place the robot** on the default start place (step 38): RED, front touching the audience wall, centered on tile C1, in line with the HIVE, facing the wall. Put in the POLLEN the way you do in a match (at most 4).
  Measure the robot's **edges**, never the tape mark, when you place it.

- [ ] **53. Start the TeleOp.** "BioBuzzRobot". INIT. Choose the alliance with D-pad left (red) or D-pad right (blue). The gamepad light turns red or blue.
  **You should see:** "Drive: Pedro + Pinpoint", any constants warnings, "Placeholders still to measure", the Vision line (the camera running), and "Start pose: default: center of tile C1, front against the audience wall".
  **If not:** "LEGACY drive" means Pedro or the Pinpoint failed to start (check the name `pinpoint` and the cable). An error that names a constant means that number is wrong: fix it and run again.

- [ ] **54. Press PLAY.** The robot now believes it stands on the start place. (If you restart within 5 minutes, it keeps the old position. If you moved the robot by hand, put it on the start place and **hold Share for 0.6 s**.)

- [ ] **55. Hold D-pad up.** Watch the screen and your hands.
  **You should see, in this order:** `Planning` (it may flash by), `Driving to the scoring pose` (skipped if you are already in place), `Aligning`, `Feeding`, `Done: 4 shot(s)`. The gamepad buzzes briefly when the robot arrives, and twice when it is done.
  **If not:** read the line `Waiting: ...` or `Stopped: ...` and use the tables below.

- [ ] **56. Let go mid-way.** Start again and let go of D-pad up while the robot drives.
  **You should see:** the robot stops and gives control back at once.

- [ ] **57. Tip the HIVE and shoot again.** At the start of a match the UP cell holds 3 NECTAR and tips with the 3rd POLLEN. After a tip the new UP cell is empty and needs 8 POLLEN (7 do not). Tip the HIVE by hand or by shooting, then hold D-pad up again.
  **You should see:** the robot drives to the other side and shoots at the other cell. Press D-pad down if it goes to the wrong side.
  **If not:** see the row "wrong side" in the problem table.

- [ ] **58. Repeat for BLUE.** Choose BLUE at INIT with D-pad right. Start in tile D6, with the front against the wall opposite the audience wall.
  **You should see:** the same messages as in step 55.

- [ ] **59. Try drift correction** (optional, last, **only after step 43 agreed within about 1 in at several spots on the field**). Set `Vision.POSE_CORRECTION_ENABLED` to `true` and run again.
  Push the robot a few inches off its place while it faces the HIVE, and watch the "Drift correction" line.
  **You should see:** `applied`, and the "Pose" settling back to the true spot within a second or two.
  **If not:** if the pose jumps the wrong way, set it back to `false` and fix the camera numbers (step 43). "ignored: camera disagrees by N in" means the difference is bigger than `Vision.POSE_CORRECTION_GATE_IN` (10 in).
  If the pose settles too slowly or too jumpily, change `Vision.POSE_CORRECTION_GAIN` (up = faster, down = smoother) or `POSE_CORRECTION_MAX_STEP_IN`.

**Rule while you practise:** shoot only into the UP cell. The robot pauses when the tags disappear, but rule G417 is also the driver's job.

**What the screen says.** `Ready` (waiting for you), `Planning`, `Driving to the scoring pose`, `Aligning`, `Feeding`, `Waiting: <reason>` (the robot will go on when the reason is gone), `Stopped: <fault>` (it gave up; the gamepad buzzes
long and hard; let go of D-pad up and try again), `Done: N shot(s)`. The screen also shows the shots so far, the range and aim error, the flywheel speed, "HIVE up side" (which cell the robot thinks is up, and whether
the camera has seen it), the Vision line and the Pose.

**`Waiting:` reasons**

| Reason | What to try |
|---|---|
| `POSE_UNTRUSTED` | The robot does not know where it is. Put it on the start place and hold Share 0.6 s. Check the Pinpoint cable. |
| `HIVE_UNCONFIRMED` | The camera has not seen the UP cell's tags. Do the Limelight steps. Or press D-pad down until "HIVE up side" names the cell that really is up. While you set up the camera you may set `AutoShoot.REQUIRE_HIVE_CONFIRMATION` to `false`. **Set it back to `true` before a match**: shooting into the DOWN cell is a foul. |
| `TAGS_LOST` | The tags vanished (a tip, or something in the way). Wait. If it pauses too often, raise `AutoShoot.VISION_LOSS_PAUSE_SEC`. |
| `OUT_OF_RANGE` | The range is outside the shot table. Add rows (steps 45 to 48). |
| `ROBOT_MOVING` | Still moving. Raise `AutoShoot.SHOT_MAX_SPEED_IN_S` or `SHOT_MAX_OMEGA_DEG_S` a little, or run the Foresight Tuner again. |
| `HEADING_ERROR` | The shooter does not point at the cell. Check the Pinpoint heading (Robot Check). `AutoShoot.SHOT_HEADING_TOL_DEG`. |
| `FLYWHEEL_NOT_AT_SPEED` | See "flywheel never at speed" in the problem table. |

**`Stopped:` faults**

| Fault | What to try |
|---|---|
| `NO_POSE` | Same as `POSE_UNTRUSTED`. |
| `NO_FEASIBLE_POSE` | There is no legal place to stand. Widen the shot table (steps 45 to 48). Check the robot size numbers (step 13). `AutoShoot.SHOOT_CONE_HALF_ANGLE_DEG`, `KEEPOUT_MARGIN_IN`, `WALL_CLEARANCE_IN`. |
| `APPROACH_TIMEOUT`, `APPROACH_STALLED` | It did not arrive or got stuck. Clear the way. Run the Foresight Tuner again. |
| `ALIGN_TIMEOUT` | It would not settle. A mecanum robot rarely settles under 1 in. `AutoShoot.ALIGN_POS_TOL_IN`, `ALIGN_HEADING_TOL_DEG`, `ALIGN_TIMEOUT_SEC`. |
| `SPINUP_TIMEOUT` | The flywheel never got to speed. See the problem table. |
| `DRIVER_OVERRIDE` | You pushed the left stick hard. If it happens by accident: `AutoShoot.DRIVER_OVERRIDE_STICK`, `DRIVER_OVERRIDE_SEC`. |
| `HIVE_NOT_VISIBLE` | At the spot it saw no tags on either side. Check the camera steps. `AutoShoot.HIVE_UNSEEN_FLIP_SEC`. |
| `HIVE_UNCONFIRMED` | Same as the `Waiting:` reason. |
| `FEED_TIMEOUT` | No POLLEN seen (only when `Shooter.SHOT_COUNT_MODE` is `DIP`). Use `HYBRID` or `TIMED`. |

## Autonomous

**Autonomous** is the first 30 seconds of a match, when the robot drives itself. In today's code there are three hand-written programs, all for the **RED** alliance (there is no blue version yet), in the Autonomous list.
The **LOADING ZONE** is the taped area beside the red wall where a robot ends and parks.

| OpMode | Where to place the robot | What it does |
|---|---|---|
| "RedAlliance_B1" | tile B1, front touching the audience wall. Right edge 26.1 in from the red wall | turns, shoots, then drives backward up the left side to the LOADING ZONE |
| "RedAlliance_C1" | tile C1, front touching the audience wall. Right edge 49.1 in from the red wall | shoots, then drives backward to the far end and toward the red wall |
| "RedAlliance_LeavePark" | tile A3, front touching the red wall. Left edge 61.4 in from the audience wall | drives out in two short lines. No shooting |

The edge distances use the file's `Robot.RIGHT_EXTENT_IN` and `Robot.LEFT_EXTENT_IN` of 8.90 in: the program starts with its center of rotation 35.0, 58.0 and 70.3 in from the wall, and the edge is that minus 8.90 in.
Left and right are the robot's own: stand behind the robot and look where it faces.

- [ ] **60. Place the robot with a tape measure.** Put the side that faces the wall **on** the wall, square to it. Measure from the wall to the robot's **edge**, never to the tape mark. Use the table, with your own numbers
  from step 13 (center distance minus your extent).

- [ ] **61. Fix the start line.** These three programs start with the center of rotation 9.2 in from the wall (the start line `poseFactory.of(..., 9.2, ...)` in `RedAlliance_B1.java` and `RedAlliance_C1.java`, and 9.2112 in `RedAlliance_LeavePark.java`).
  With the front touching the wall, the center of rotation is `Robot.FRONT_EXTENT_IN` away. If your measured number is not 9.2, change that number in the start line to match.

- [ ] **62. Choose and run one.** Open `docs/autos/index.html` in a browser. It draws each route for both alliances, with its start place, its time and any rule problems. How to place the robot is explained in [`docs/AUTOS.md`](AUTOS.md),
  section "Where to set the robot up (rule G304)". Clear the path, stand back, then choose Autonomous, your OpMode, INIT, PLAY.
  **You should see:** the robot follows the route and stops.
  **If not:** press STOP. Check the start place against the table.
  The diagrams also show routes that are only plans (for example "wall_shoot_park"). **No program drives those yet.** Only the three OpModes above move the robot.

## If something goes wrong

| What you see | What to try |
|---|---|
| The TeleOp refuses to start at INIT, with "BioBuzzConstants has errors" | The message names the constant. Fix that number (for example front + rear distance must equal the robot length). |
| The Driver Station does not list the OpModes | Run step 2 again. |
| The robot drives the wrong way: stick forward, robot goes back (or slides or turns the wrong way) | Flip `Controls.MANUAL_FORWARD_SIGN`, `MANUAL_STRAFE_SIGN` or `MANUAL_TURN_SIGN` between `1.0` and `-1.0`, one at a time. |
| The robot twists or jerks instead of driving straight, or the Robot Check says a wheel went backward | A wheel direction is wrong. Redo the Mecanum Tuner (step 23): the four `Direction` lines in `drivetrainConfig` in the Pedro file. Run the Robot Check again. |
| The Pedro tuner page does not open | The laptop must be on the Robot Controller's Wi-Fi, and the Driver Station must stay connected. Try `http://192.168.43.1:10158` again. |
| The TeleOp says "LEGACY drive" | Pedro or the Pinpoint did not start. Check the name `pinpoint`, the cable, and the Robot Check Pinpoint check. |
| The camera is not seen: "NO Limelight", or the status is `NOT_CONNECTED`, `NO_RESULT`, `STALE`, `STALLED` or `WRONG_PIPELINE` | Check the USB cable in the blue port. Wait 20 s after power-on. The configuration name must be exactly `limelight`. Pipeline 0 must be the AprilTag pipeline (step 36). Open the Limelight page (step 35). |
| The camera sees no tags | Check pitch, exposure, marker size 82.55 mm and Full 3D (step 36). Is the HIVE in view from this spot? |
| The flywheel never gets to speed (`FLYWHEEL_NOT_AT_SPEED`, `SPINUP_TIMEOUT`) | Use a fresh battery. The target must be below what the motor can do (about 2800 ticks per second). Check the flywheel cable and encoder (Robot Check, flywheel). `Shooter.FLYWHEEL_PIDF_P` (40) and `FLYWHEEL_PIDF_F` (12.5) are goBILDA's starting numbers. `Shooter.AT_SPEED_BELOW_FRAC`, `AT_SPEED_ABOVE_FRAC`, `AT_SPEED_RISE_SEC` widen the "at speed" window. |
| The robot will not shoot | Read the `Waiting:` or `Stopped:` line and use the tables in "Try auto-shoot". |
| A POLLEN is **jammed** in the indexer or the flywheel | **Let go of D-pad up first** (it can start the auto-score again). Then **hold left bumper** (the indexer runs slowly backward) or **hold D-pad right** (the flywheel runs slowly backward). They stop when you let go. If it will not free, switch the robot off, then free it by hand with a mentor. The speeds are `Shooter.FEEDER_REVERSE_POWER` and `Shooter.FLYWHEEL_UNJAM_POWER` (0.15 each). Raise one a little at a time. The TeleOp refuses numbers above 0.5 (`Controls.UNJAM_MAX_POWER`). |
| Every shot at one distance is high or short | Change that row of `Shooter.SHOT_TABLE`: a lower speed for high, a higher speed for short. |
| Shots are wrong only beyond some distance | Add table rows out to that distance. |
| Every shot lands to one side | `Robot.LAUNCH_POINT_LEFT_IN`: is the ball exit really on the center line? |
| The volley ends with 3 or fewer although 4 were fed | `Shooter.SHOT_DIP_FRAC` is too big (shallow dips not seen), or raise `Shooter.EMPTY_AFTER_LAST_SHOT_SEC`. |
| It counts shots that did not happen | `Shooter.SHOT_DIP_FRAC` too small, or raise `Shooter.SHOT_REFRACTORY_SEC`. |
| It feeds two POLLEN at once, or jams | Change `Shooter.FEEDER_FORWARD_POWER`. Use `Shooter.FEED_MODE` = `GATED_PER_BALL` and check `Shooter.FEED_SECONDS_PER_SHOT`. |
| The feeder runs the wrong way | Set `Shooter.FEEDER_INVERTED` to `true`. |
| The robot creeps when you let go of the sticks | Raise `Controls.STICK_DEADBAND` a little. |
| One shot, then a long pause | Lower `Shooter.AT_SPEED_RISE_SEC`, or raise `Shooter.AT_SPEED_BELOW_FRAC` a little. |
| It goes to the wrong side after a tip | Press D-pad down. If it happens often, check the camera steps. Settings: `Vision.HIVE_CONFIRM_FRAMES`, `HIVE_CONFIRM_SEC`, `HIVE_LOST_SEC`. |
| It switches sides in the middle of a volley when nothing tipped | Raise `Vision.HIVE_LOST_SEC` (the tags were blocked for a moment). |
| It swings wide near the wall or the HIVE | Lower `AutoShoot.KEEPOUT_MARGIN_IN` or `WALL_CLEARANCE_IN`. Check the robot size numbers. |
| The camera says DOWN for the up cell | Step 41, and make sure the camera is upright. |
| The position slowly drifts | Put the robot on the start place and hold Share for 0.6 s. After step 43, try drift correction (step 59). |

## How it works, in ten lines

1. The Pinpoint (two small wheels and a gyro) tracks the robot's pose. Pedro drives to a pose using it.
2. The Limelight finds the four AprilTags of each HIVE cell. Their order and height tell the code which cell is UP.
3. The tags are on the cells, and the cells move, so the camera cannot say where the robot is on the field. The Pinpoint does that. The camera gives alignment, and an optional small correction of drift.
4. Hold D-pad up and the controller **plans**: it picks the closest legal spot in front of the UP cell, with the robot's back (the shooter) toward the cell and the range inside the shot table.
5. It **drives** there around the walls, the HIVE frame and the FLOWERS (the POLLEN holders on the walls), checking the whole robot rectangle, not just its center.
6. It **aligns** and holds the spot. The camera trims the heading toward the aim point.
7. It **feeds**: the feeder runs only while every condition is true: the pose is trusted, the UP cell is confirmed, the tags are in view, the range is in the table, the robot is still, the shooter points at the cell, and the flywheel is at speed.
8. It counts shots by the dip in flywheel speed, and stops at 4 (rule G407). If the tags vanish, the cell may be tipping, so the feeder pauses and the robot plans for the other cell.
9. Let go of D-pad up and you have manual control at once. A button held through INIT cannot start a volley.
10. The field is the same after a half turn, so everything is written for RED and BLUE is the same numbers turned half a turn. The rules built in: G407 (4 POLLEN), G417 (only the UP cell), G409 (do not catch what a tipping HIVE drops), and in autonomous G402 (stay on your own half) and G401 (no driver input).

The logic is plain Java that is tested on a laptop with no robot (`./gradlew :TeamCode:testDebugUnitTest`). Nothing of auto-shoot has run on the robot before today. The folders are in the README.

## Measure later, for the planned autonomous routes

You do not need these today. They feed the planner that checks routes on a computer (`docs/AUTOS.md`). Do them when you build autonomous routes. All names are in the block `Auto` unless noted.

* **HIVE tip numbers.** FIRST's own field documents say an empty cell tips with the 8th POLLEN (7 do not) and a cell with the 3 staged NECTAR tips with the 3rd POLLEN (2 do not). Check it on your HIVE:
  3 NECTAR placed gently at the back of the UP cell, then 2 POLLEN (no tip), then 3 (tip); an empty cell with 7, then 8. The planner plans one POLLEN more than the minimum (`Auto.TIP_PLANNING_MARGIN_POLLEN`) because a real field drifts.
  Weigh a few POLLEN and NECTAR on a kitchen scale (AndyMark lists 0.055 lb and 0.091 lb): the code counts one NECTAR as 5/3 POLLEN (`Auto.NECTAR_POLLEN_EQUIVALENT_NUMERATOR` and `_DENOMINATOR`).
* **Where tipped balls land.** Tip a HIVE and mark where the first balls touch the floor, measured out from the HIVE's middle line, and how many seconds until everything stops rolling: `Auto.TIP_SPILL_REACH_IN` (40 in),
  `Auto.TIP_SPILL_MARGIN_IN` (6 in), `Auto.TIP_SPILL_CLEAR_SEC` (3 s). A robot may not catch what a tipped HIVE releases (rule G409).
* **Does the 4th POLLEN go?** Film a volley of 4 at a HIVE that the 3rd POLLEN tips. Count the frames: how long a POLLEN is in the air (`Auto.BALL_FLIGHT_SEC`, 0.5 s) and how long from the tipping POLLEN landing until the feeder stops (`Auto.TIP_STOP_LATENCY_SEC`, 0.85 s).
* **The partner team.** Ask whether their autonomous shoots the first tip of the HIVE, and when it is done: `Auto.PARTNER_FIRST_TIP_DONE_SEC` (12 s is a guess).
* **LEAVE and the wall.** Ask a referee whether a robot that touches the wall in the LOADING ZONE at the end of autonomous keeps LEAVE (`Auto.LEAVE_WALL_MARGIN_IN`, 1.5 in).
* **The FLOWER pick-up.** One flat intake wheel turns into the bottom POLLEN of a FLOWER and rolls it out over the ring's lip. Do it by hand with both wheels. The numbers: `Auto.FLOWER_WHEEL_START_OFFSET_IN`,
  `Auto.FLOWER_WHEEL_PRESS_OFFSET_IN`, `Auto.FLOWER_WHEEL_BALL_INBOARD`, `Auto.FLOWER_WIGGLE_PUSH_HOLD_SEC`, `Auto.FLOWER_WIGGLE_RELEASE_HOLD_SEC`, `Auto.FLOWER_WIGGLE_TURN_SEC`, `Auto.FLOWER_WIGGLE_CYCLES_PER_POLLEN`,
  `Auto.FLOWER_CHASSIS_CLEARANCE_IN` and `Robot.INTAKE_REACH_IN`. The spinning wheel must pull the ball toward the robot. Print or watch the robot's pose during one wiggle: the pose point must not slide more than about 0.3 in. Also measure the real FLOWER: the distance from the wall to the middle of the POLLEN stack (the drawing says 2.634 in), the hole the bottom POLLEN sits in (3.04 in; the rule book says 2.79 in), the height of the ring's lip (0.35 in; the rule book says 0.43 in), and which parts of your robot are taller than 11.44 in (the FLOWER's wall bracket sticks out 1.67 in from the wall at 11.44 to 11.84 in above the floor). Details: "Taking POLLEN from a FLOWER" in [`docs/AUTOS.md`](AUTOS.md).
* **The lane under the HIVE** (only if a route uses it): the distance between the inner faces of the two foot bars (about 45.5 in) and between the leg stubs at your tallest part's height. Push the robot through by hand with the camera on: nothing may touch.
