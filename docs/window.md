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

Layer **`c`** paints the **radio coverage** of the hall, one square per world cell, coloured by the
quality `wifi.budget()` answers at that spot — green where the access point has the room, dark red
where a rack is in the way. Measured on `production` with `config/demo_wifi.json`, at 8 m from the
antenna: `0.67` on the open floor, `0.32` with one rack in the line. It is a picture of the *model*,
not of a measurement, which is why it is off in every default view, why it is drawn under the racks
that cast the shadows, and why it is sampled once per run instead of once per frame (§6.14). The live
number of one robot — including the slow fade the map leaves out — stays the bar in the network panel
and the `wifi q 0.42 -63.1 dBm 1 wall` on the `/link` line.

The layer keys come from `mecanum_lab/keys.py`; `./lab docs` prints the table the window answers with, and a test fails when the front page and the code stop agreeing.
