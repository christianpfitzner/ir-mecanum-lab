# The window, its layers, and what the readout line carries

The key table is on the front page. This page is what the layers are for, why the window starts clean,
and what hiding a layer does — and does not do — to the messages on the bus.

The window opens in the **clean view**: robot, world, readout line, and none of the four *raw measurement*
layers (lidar scan, odometry trail, gps fix, odometry ghost) that otherwise cover the map you are meant to
look at. Nothing is hidden on the bus — `ros2 topic echo` sees all of it. Only the screen changed.

`--view sensors` starts with everything on. `--layers scan,ghost` turns single layers on, `--layers -hud`
turns one off. A demo config about a raw layer (`config/demo_*.json`) asks for that layer itself, so a
demo always shows what the demo is about: `./lab sim --config config/demo_wifi.json`, or
`ros2 launch launch/demo.launch.py demo:=wifi` with RViz beside it.

The menu switches **drawing only**. `/<robot>/scan`, `/odom`, `/gps`, `/imu`, `/poi` and your `kf/pose`
keep running at full rate; `ros2 topic hz /alice/scan` does not care what the window shows. Hide the dots,
keep the data, and see which layer belongs to which topic.

The readout line is live in **every** run, and every instrument reports itself in it. The IMU gives `ax`,
`ay`, `gz` and the chip temperature — default `imu.rate` is 100 Hz, with bias random walk, scale error,
tilt cross-coupling and vibration, and `az ≈ +9.81 m/s²` standing still. The GPS gives what the fix is
worth (`q2 8 sats`, or `q0 0 sats` in a blackout) and what the transport lost (`lost 3`). The LIDAR counts
the beams that came back with no echo.

The **pointer** carries the world coordinate under it — `12.40, 6.20 m` — read through the same mapping the
frame is drawn with (`Camera.wx()` is its inverse), so a coordinate for a lab report comes off the screen
instead of between grid lines. Only inside the world box: outside the hall the number counts void.

**The right button places a robot.** Left drag pans, the wheel zooms to the cursor, the middle button
centres again. The right button asks *put a robot here*: a menu at the pointer, one row per robot that
drives, each row in the colour that robot is drawn in. One click moves it to the spot the menu was opened
for, heading unchanged — placement moves, turning is a command.

The odometry goes along (`odometer.reset()` at the new spot): a belief left behind at the old one is the
thing the exercise would then fight. The drive before stays — `contacts` and `distance` are facts about a
run, and a robot that drove into three walls has not un-driven them by being picked up.

Two limits. The menu does not open over a rack or over the void — a menu whose every row can only answer
"no" is not an offer — so it asks the one rule `World.free()` also hands a radiation source. A spot cannot
be legal for a source and impossible for a robot in the same hall. And the window moves nothing by itself:
the click is reported, `node.run_loop` asks the engine, the engine decides. A renderer that moved robots
would be a second place that knows what a pose is, so `test_spawn_service.py` runs that chain from a posted
mouse button to the pose in the engine. `esc` closes the menu instead of ending the run, and a second right
click moves it rather than needing a close-click first.

Every letter on the screen goes through `Renderer._text()`, which does two unglamorous things. The first
converts the colour, because this code uses two colour spaces (`types.PALETTE` and the literal HUD colours
are 0..1, `rgb()` and `mix()` hand back 0..255) and pygame does not read 0..1, it truncates. So the amber
of the goal, `(1, .85, .3)`, once arrived at the font as `(1, 0, 0)` and painted the word `goal` black on a
floor of (33, 35, 43) — 1.3:1, which is not a dim label but no label. `to255()` is where the two spaces
meet, so a caller cannot forget the rule again.

The second thing is a one-pixel dark edge behind the letters, the way a map keeps its place names readable.
The reason is the **wall**: (86, 91, 107). No colour of `types.PALETTE` reaches the 4.5:1 small text needs
against that, so hue cannot be the fix — a red robot's name measured 1.98:1 on grey. Darkening the pixels
behind the letters is: 1.98:1 → 5.55:1 for that red name, 4.89:1 → 13.7:1 for `goal`. On floor and void the
edge is dark on dark and invisible; on a wall it is the difference. `tests/support_contrast.py` computes
the ratio and the tests assert it — `tests/test_render_c.py` for the labels on the map,
`tests/test_gps_zones_integrator.py` for the zone names. A label either reaches 4.5:1, or the build says so.

Layer **`c`** paints the **radio coverage** of the hall, one square per world cell, coloured by the quality
`wifi.budget()` answers at that spot — green where the access point has the room, dark red where a rack is
in the way. Measured on `production` with `config/demo_wifi.json`, at 8 m from the antenna: `0.67` on the
open floor, `0.32` with one rack in the line. It pictures the *model*, not a measurement, so it is off by
default, drawn under the racks that cast the shadows, and sampled once per run rather than per frame
(§6.14). One robot's live number, fade and all, stays the bar in the network panel and the
`wifi q 0.42 -63.1 dBm 1 wall` on the `/link` line.

Layer **`i`** paints the **radiation dose** the same way — one square per cell, amber where a counter would
barely tick, red at the source — and, unlike the radio map, *under* the walls. The counter model has no
wall in it at all, so the field runs through a rack untouched and the wall painted over it is the only
thing left that says where in the hall you are. `docs/poi.md` has the numbers and the assumption worth
discussing.

The layer keys come from `mecanum_lab/keys.py`; `./lab docs` prints the table the window answers with, and
a test fails when the front page and the code stop agreeing.
