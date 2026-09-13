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

The layer keys come from `mecanum_lab/keys.py`; `./lab docs` prints the table the window answers with, and a test fails when the front page and the code stop agreeing.
