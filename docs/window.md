# The window, its layers, and what the readout line carries

The key table is on the front page. This page is what the layers are for, why the window starts
clean, and what hiding a layer does — and does not do — to the messages on the bus.

The window starts in the **clean view**: the robot, the world and the readout line, without the four
*raw measurement* layers (lidar scan, odometry trail, gps fix, odometry ghost) that otherwise cover the
map a student is meant to look at. Nothing is hidden on the bus — a `ros2 topic echo` sees all of it —
only on screen. `--view sensors` starts with everything on, `--layers scan,ghost` turns single layers on
and `--layers -hud` turns one off, and one demo config that is about a raw layer (`config/demo_*.json`)
asks for that layer itself, so a demo always shows what the demo is about: `./lab sim
--config config/demo_wifi.json`, or `ros2 launch launch/demo.launch.py demo:=wifi` with RViz beside it.

The menu switches **drawing only**. `/<robot>/scan`, `/odom`, `/gps`, `/imu`, `/poi` and your `kf/pose`
keep running at full rate; `ros2 topic hz /alice/scan` does not care what the window shows. That
is deliberate: hide the dots, keep the data, and see which layer belongs to which topic. The
per-robot readout line also carries the IMU (`ax`, `ay`, `gz` and the chip temperature) — it is live
in **every** run, not only in Experiment 2: default `imu.rate` is 100 Hz with bias random walk,
scale error, tilt cross-coupling and vibration, and `az ≈ +9.81 m/s²` while standing still. The GPS
part of the same line says what the fix is worth (`q2 8 sats`, or `q0 0 sats` in a blackout) and how
many messages were lost (`lost 3`), and the LIDAR part how many beams came back with no echo — all
of it without a second terminal running `ros2 topic echo` beside the window.

Two things the window shows that no key switches. The **pointer** carries the world coordinate under
it — `12.40, 6.20 m` — read through the same mapping the frame is drawn with (`Camera.wx()` is its
inverse), so a coordinate in a lab report is read off the screen instead of interpolated between grid
lines. It appears only inside the world box: outside the hall the number counts metres of void.

**The right button places a robot.** Dragging with the left button pans, the wheel zooms to the cursor,
the middle button centres again — and the right button asks a different question: *put a robot here*. It
opens a menu at the pointer with one row per robot that drives, each row wearing the colour the robot is
drawn in, and one click on a row moves that robot to the spot the menu was opened for. The heading stays
as it was: placement moves, turning is a command. The odometry goes along (`odometer.reset()` at the new
spot), because a belief left behind at the old one would be the thing the whole exercise then fights
against; the drive that happened before stays, because `contacts` and `distance` are facts about a run
and a robot that drove into three walls has not un-driven them by being picked up.

Two things that menu does not do. It does not open over a rack or over the void — a menu whose every row
can only answer "no" is not an offer — using the one rule `World.free()` also hands a radiation source, so
a spot cannot be legal for a source and impossible for a robot in the same hall. And the window moves
nothing by itself: the click is reported, `node.run_loop` asks the engine, the engine decides (a renderer
that moved robots would be a second place that knows what a pose is). `test_spawn_service.py` runs that
chain once end to end, from a posted mouse button to the pose in the engine, because a chain tested at
its two ends is a chain nobody has run. `esc` closes the menu instead of ending the run, and a second
right click moves the menu rather than needing a close-click first.

Every letter above that — and the coordinate is one of them — goes through one function,
`Renderer._text()`, which does two things that are not decoration. It converts the colour: this code
uses two colour spaces (`types.PALETTE` and the literal HUD colours are 0..1, `rgb()` and `mix()` hand
back 0..255), and pygame does not read 0..1, it truncates. The amber of the goal, `(1, .85, .3)`,
arrived at the font as `(1, 0, 0)` and painted the word `goal` black on a floor of (33, 35, 43) —
1.3:1, which is not a dim label but no label. `to255()` is where the two spaces meet now, so a caller
cannot forget the rule again.

The second thing is a one-pixel dark edge behind the letters, the way a map keeps its place names
readable. The reason is not the floor but the **wall**: (86, 91, 107), and no colour of
`types.PALETTE` reaches 4.5:1 against that — a red robot's name measured 1.98:1, a blue one 2.19:1 —
while a robot in a hall built out of walls drives along walls most of the time. No hue repairs that
(no red clears 2.5:1 on grey), so the pixels directly behind the letters are darkened first: on floor
and void that is dark on dark and invisible, on a wall it is the difference, 1.98:1 → 5.55:1 for the
red name and 4.89:1 → 13.7:1 for `goal`. It costs one extra render and four blits per label and stays
inside the frame budget `tests/test_render_c.py` measures with eight robots and lidar running. The
contrast itself is asserted there for the labels on the map and in `tests/test_gps_zones_integrator.py`
for the zone names, the WCAG ratio being `tests/support_contrast.py`: a label either reaches the 4.5:1
small text needs, or the build says so.

Layer **`c`** paints the **radio coverage** of the hall, one square per world cell, coloured by the
quality `wifi.budget()` answers at that spot — green where the access point has the room, dark red
where a rack is in the way. Measured on `production` with `config/demo_wifi.json`, at 8 m from the
antenna: `0.67` on the open floor, `0.32` with one rack in the line. It is a picture of the *model*,
not of a measurement, which is why it is off in every default view, why it is drawn under the racks
that cast the shadows, and why it is sampled once per run instead of once per frame (§6.14). The live
number of one robot — including the slow fade the map leaves out — stays the bar in the network panel
and the `wifi q 0.42 -63.1 dBm 1 wall` on the `/link` line.

Layer **`i`** paints the **radiation dose** of the same kind of picture — one square per cell, amber where
a counter would barely tick and red at the source — but for the opposite reason, and that difference is
the sentence worth saying in the lab. The radio map goes under the racks because a rack *shadows* a wave;
the dose map goes under the walls because the counter model has no wall in it at all (`Source.intensity()`
is a distance law), so the field runs through the rack untouched and the wall painted over it is the one
thing left that tells a student where in the hall a painted cell is. The numbers on it are the numbers of
the counter (`1.0` at the source, `0.5` at `d0`, `0.1` at `3·d0`, sources adding), and a student who
expects a shadow behind the rack has found the assumption of the model to discuss (§6.13, `docs/poi.md`).

The layer keys come from `mecanum_lab/keys.py`; `./lab docs` prints the table the window answers with, and a test fails when the front page and the code stop agreeing.
