# Mecanum lab — two experiments in one 2D simulator

A Python simulator for the lab course: **Experiment 1** teaches mecanum kinematics (the
student drives the tasks), **Experiment 2** teaches state estimation (the simulator drives,
the student estimates with a Kalman filter). Both run headless, without ROS 2 and without
numpy — ROS 2 is an optional layer, not a requirement.

## Install in one command

```bash
./install.sh              # checks python/pygame/pytest/ROS, offers to install what is missing
./install.sh --check      # report only, install nothing
./install.sh --mit-tests  # plus a short self test (simulation + unit tests, headless)
```

`./install.sh` writes nothing into your home directory and needs no sudo. Without an
internet connection: `sudo apt install python3-pygame` (or `./install.sh --user`).

## First steps — Experiment 1 (kinematics)

```bash
./lab run --robot alice --controller student/controller_template.py   # window, drive yourself
./lab grade --task v1 --controller student/solution.py               # grade all tasks
```

`./lab run` is the one-process form: the simulator, your node as a thread and the keyboard, all three
on the same `/cmd_vel`. A node publishes every tick and the keys publish only while one is held, so
your node drives and the keys interrupt it — and the readout line says which of the two was last
(`cmd topic 0.04 s`, `cmd keys 0.02 s`, `cmd none` once `cmd_timeout` passed without a frame).

**On `kinematik` your node's job is the conversion alone, and the robot does not drive itself.** For the
task `""` or `kinematik` the runner behind `serve()` never calls `mission()`: it reads every `cmd_vel`
that arrives, pushes it through your `inverse_kinematics()` and publishes the four wheel speeds — full
stop, because T1 is graded by the grader sending commands blind and measuring what the wheels do, not by
anything your program decides. So `./lab run --task kinematik --controller student/solution.py` shows a
robot standing still, with `/mission_state` at `idle`: nothing is broken, nothing is waiting for a goal,
and the way to see the conversion work is to hold an arrow key (a key *is* a `cmd_vel`, and it is your IK
that turns it into wheels — which is exactly what the window is for). Missions start at T2: `--task
quadrat` runs `mission()` once and the robot drives the square by itself.

## First steps — Experiment 2 (state estimation)

The simulation drives; your node only measures and reports its estimate on
`/<robot>/kf/pose` — position **and** its own 1σ, because the grade hangs on the uncertainty.

```bash
./lab run --task kf_gps --robot alice --controller student/kf_template.py --truth
./lab grade --task kf_alle --controller student/kf_template.py --log messung.csv
python3 tools/kfplot.py messung.csv       # numbers + ASCII plot, no matplotlib
```

Four tasks: `kf_gps` (CV model, GPS only) → `kf_fusion` (GPS + odometry + IMU through an
8 s GPS outage) → `kf_kovarianz` (the stated σ must match the error, NEES) → `kf_dynamik`
(fast, 200 Hz IMU). Handout: `docs/praktikum/kalman.tex` (`make kalman`).

## With ROS 2 (Kilted or newer)

```bash
source /opt/ros/kilted/setup.bash
./lab sim --robots alice,bob                      # simulator as a real ROS node
./lab spawn --name carlo                          # add a robot to a running sim
ros2 launch launch/kf.launch.py                   # Experiment 2, everything configurable
ros2 launch launch/kf.launch.py grade:=kf_alle controller:=student/kf_solution.py
ros2 launch launch/sim.launch.py robots:=alice,bob  # Experiment 1
ros2 topic echo /alice/imu --once                 # az at rest ≈ +9.81 — that is correct
ros2 topic echo /tf --once                        # map → alice/odom → alice/base_link → laser
rviz2 -d rviz/kf.rviz --ros-args -p use_sim_time:=true   # the sim stamps TF in sim seconds
```

To keep the in-process bus even with ROS: `MECANUM_ROS=stub ./lab sim --headless`.

## The simulator window (what you hide is not what is published)

| input | what it does |
|---|---|
| mouse wheel | zoom **to the cursor** · `+`/`-` zoom to the middle · `f` shows the whole world |
| drag with the left button | pan (the map follows the mouse) · right button centres again |
| `1`…`9` / `0` | follow one robot / show everything · window resizable, the camera keeps up |
| `Up`/`Down` drive · `Left`/`Right` **strafe** | keyboard driving, on unless `--no-teleop`; the keys set body speeds, this is not a game |
| `q` or `,` turn right · `e` or `.` turn left | ±0.9 rad/s yaw. With teleop on, `q` **turns** instead of quitting — `ESC` or the window's close button ends the run |
| `SPACE` pause | `q` quits only when teleop is off |
| `m` | opens the layer menu (starts closed so it covers nothing): lidar scan, odometry trail, gps fix, estimate + σ ellipse, wheels, velocity vector, floor markings, goal, readout lines, gps shadow zones, odometry ghost + drift, radiation source + field, radio link + access point |
| `l t g k w v d z h s o p n` | switch a single layer — the same as clicking its row (`s` GPS shadow, `o` odometry ghost, `p` radiation source and its field rings, `n` the radio link) |

The menu switches **drawing only**. `/<robot>/scan`, `/odom`, `/gps`, `/imu`, `/poi` and your `kf/pose`
keep running at full rate; `ros2 topic hz /alice/scan` does not care what the window shows. That
is deliberate: hide the dots, keep the data, and see which layer belongs to which topic. The
per-robot readout line also carries the IMU (`ax`, `ay`, `gz` and the chip temperature) — it is live
in **every** run, not only in Experiment 2: default `imu.rate` is 100 Hz with bias random walk,
scale error, tilt cross-coupling and vibration, and `az ≈ +9.81 m/s²` while standing still. The GPS
part of the same line says what the fix is worth (`q2 8 sats`, or `q0 0 sats` in a blackout) and how
many messages were lost (`lost 3`), and the LIDAR part how many beams came back with no echo — all
of it without a second terminal running `ros2 topic echo` beside the window.

### GPS that gets bad by place: the shadow demo (not the default)

`gps.zones` degrades the fix **where the robot is**, not when: the first rectangle that contains
it multiplies `sigma_xy`, adds a bias (multi-path pushes the fix away from the reflector) or
suppresses the fix completely. It ships empty, because the graded tasks in `config/tasks.json`
are calibrated on the plain GPS of each experiment — a permanent shadow would silently move what
every filter is graded against. `config/demo_gps_shadow.json` is the demo that turns it on:

```bash
./lab sim --world production --config config/demo_gps_shadow.json     # drive into it yourself
./lab run --world production --config config/demo_gps_shadow.json \
          --robot muster --controller student/solution.py --seconds 30
```

Measured with 400 fixes per spot (σ=0.06 m is the lab default):

| spot | σ of a fix | median error | mean offset | fixes |
|---|---|---|---|---|
| open floor | 0.06 / 0.06 m | 0.07 m | (−0.01, 0.00) m | 400/400 |
| under the high shelf (σ×6, bias +0.8/−0.5) | 0.37 / 0.36 m | 1.03 m | (+0.83, −0.51) m | 400/400 |
| multipath in the corner (σ×3) | 0.18 / 0.18 m | 0.52 m | (−0.39, +0.30) m | 400/400 |
| loading dock (`"block": true`) | — | — | — | **no fix at all** |

The window draws the zones as hatched shadow (`s`), the blackout in the error colour, and labels
each with what it does to the fix. Together with the odometry ghost (`o`) and the rubber that
slipping wheels leave on the floor, one frame shows a student the three ways a position can be
wrong: noisy, biased, or missing — and how much of it the odometry invented (`overlays.py`).

### Wheel slip: odometry you can watch lying

Press the robot against a wall and the odometry keeps counting metres that were never driven.
That is the drift of Experiment 1 in its purest form — no sensor noise needed. The model is
physical: against an obstacle the body stands still while the wheels keep the speed the motor
demands, and odometry integrates wheel speeds (`sensors.py` integrates what the wheels report,
never the truth). `robot.slip` scales it: `1` = full slip (default), `0` = ideal static friction
with honest odometry. Measured in `arena`, 4 s of full throttle east into the wall, seed 5:

| `robot.slip` | body really moved | odometry counted | phantom distance |
|---|---|---|---|
| `1` (default) | 0.69 m | 1.55 m | **+0.86 m** |
| `0` | 0.69 m | 0.69 m | −0.003 m |

By hand: `./lab sim --world arena`, then `Up` into the east wall — the `odom x` in the readout
climbs while the robot's dot stays where it is. To drive it yourself with nothing but the counter —
no `/gps`, no truth, no `distance` field — `student/poi_seek_example.py` is the worked example:

```bash
./lab run --world open --config config/demo_poi_exploration.json \
          --controller student/poi_seek_example.py --seconds 100 --log poi.csv
```

It climbs while the reading rises, arcs when it stops rising, and drives back to where it was loudest
when the field goes silent; the why of all three rules is in the file, and `intensity_poi`/`name_poi`
in `poi.csv` is what `tools/kfplot.py` plots afterwards. To record the IMU instead: `./lab grade --task
kinematik --controller student/solution.py --log messung.csv` writes `ax_imu, ay_imu, gz_imu` next to
the other columns (`python3 tools/kfplot.py messung.csv --list` shows all of them, and
`sensor_state:` prints `q_gps`, `lost_gps`, `temp_imu` and `intensity_poi` as four sparklines — what
the instruments reported about themselves, each on its own scale).

### Odometry with the wrong wheel radius: the model-error demo (not the default)

Slip is one way to make odometry lie; the commonest one is that the wheel constants are wrong —
worn tyres, a reprinted hub, the diameter used where the radius belongs. `odom.geometry` lets the
odometry integrator believe a geometry of its own (`wheel_radius_scale`, `lever_scale`,
`wheel_base_scale`, `scale_xy`, `bias_xy`; see `docs/CONTRACT.md` §6.4), while the chassis keeps
the true one. Default `{}` = correct wheel constants, so nothing graded moved.
`config/demo_odom_error.json` is the demo (radius 1.05, lever 0.97):

```bash
./lab sim --world track --config config/demo_odom_error.json      # drive straight, watch `o`
./lab grade --config config/demo_odom_error.json --task alle --controller student/solution.py
```

Measured with the reference solution: after 12 m of straight lane the ghost is **0.61 m** ahead of
the robot (5 % too many metres), one commanded revolution ends **28°** rotated, and the graded run
drops to 70/100 — T1 and T2 stay green (T2 corrects its own drift), but T3 navigates by odometry
between the tables and comes out in one: **26 wall contacts**. That is the argument for why this is
a demo file and not a default.

### What a sensor says besides its number: the reality demo (not the default)

Every knob here is off in `DEFAULT_CONFIG`, and off means that not one random number is drawn that
the sensors of the graded tasks did not draw — measured message for message, not assumed
(`tests/test_sensor_reality.py`). `config/demo_sensor_reality.json` asks for all of them at once,
so one drive shows what a real sensor delivers on top of its number:

```bash
./lab sim --world production --config config/demo_sensor_reality.json    # drive it yourself
./lab grade --world production --config config/demo_sensor_reality.json \
            --task kinematik --controller student/solution.py --log messung.csv
```

| knob in the demo | what it changes | measured on one 25 s straight drive |
|---|---|---|
| `gps.sats = 8`, `gps.sats_min = 4`, `gps.zones` | what the fix is worth **where the robot is standing** | q2 with 8 anchors on open floor, q1 with 3 between the racks, and in the dock (`"block": true`) the window reads q0 while `/gps` publishes nothing at all — from x = 16.6 m |
| `gps.dropout = 0.15` | probability per message that the transport loses it | 12 of 122 emissions gone, `lost 12` in the readout; the pattern belongs to `--seed`, not to where the robot stood |
| `gps.latency = 0.25` | seconds on the wire, ±50 % jittered | a fix is 0.45 s old when it arrives (0.25 s of wire + the wait for the next slot), at most 1.0 s, and it is still the position it measured then |
| `imu.temp_*` | the chip warms up and the bias walks with it | 24.00 → 29.52 °C, `az` bias +0.024 m/s², `gz` bias +0.00069 rad/s; standing still stays cold and `az` stays +9.81 |
| `lidar.reflectivity_min = 0.25` | a wall echoes only if the cosine of the incidence angle reaches the threshold | 0.5 m off a 30 m wall: seen 7.17 m down its length by default, 1.93 m here; 20 of 360 beams come back empty (`Scan.missing`) |
| `odom.jitter = 0.35` | an encoder report arrives when it arrives | stamps 13.3…26.8 ms instead of exactly 20.0 ms (σ 2.8 ms), values and message count unchanged |

Two of these are the exercise, not the decoration. **No fix and no message are different faults.**
Quality 0 belongs to a *place*, not to a message: `GpsSensor.sky()` says what the sky at a position is
worth, the window, the log and `/sensor/info` repeat that answer, and a receiver standing in a place
worth 0 sends **nothing** — `fix()` returns `None` and there is no `/gps` message to attach a quality
to. A dropped packet looks the same from outside and is counted separately, in `lost`. So a filter that
treats every message as equally good is a filter tuned for a sensor that does not exist, and one that
treats every gap as a blackout cannot tell the dock from a radio failure. With `q_gps`, `sats_gps`,
`lost_gps`, `temp_imu` and `noecho_scan` in the same CSV as the errors of the student's own filter, the
drive becomes an argument instead of a demo.

Over ROS 2 none of the three standard messages can carry any of it — `/gps` is a `PoseStamped`, `/imu`
an `Imu`, `/scan` a `LaserScan` — which is what `/<robot>/sensor/info` is for: one JSON object per
`gps.rate` with `quality`, `sats`, `lost`, `latency_ms`, `temp` and `scan_gaps`, so
`ros2 topic echo /alice/sensor/info` answers what the readout line answers (`docs/CONTRACT.md` §6.4).

### How fast a run goes: `--speed` and `--fixed-step`

`./lab grade` runs in real time, because that is what the lab course does. For supervisors who want
the same run again and again: `--speed 4` takes four simulation seconds per wall second,
`--fixed-step` steps exactly 1/`rate` per round and never sleeps (≈36× realtime, and it switches
the sleeps of the in-process bus off, so a student node can keep up). Both keep the fixed physics
step and the seed, and a run that cannot keep up says so once with the seconds it lost instead of
quietly dropping them. What each pace measures — including why experiment 2 must stay near real
speed (`rate_hz` is the node's message rate per **sim** second): `docs/CONTRACT.md` §9.1 and
`docs/CONTRACT-KF.md` §5.1.

```bash
./lab grade --task alle --controller student/solution.py --speed 4      # 104 s of sim in 26 s
./lab grade --task alle --controller student/solution.py --fixed-step   # the same in 5 s
python3 tools/fastgrade.py --task kf_alle --speed 8                     # without the window at all
```

The same determinism is what makes a **screenshot** reproducible, and these two pictures are built
that way rather than photographed off a monitor: `--frame-max N` stops after N drawn frames and
`--screenshot FILE.png` saves the last one through `pygame.image.save`, so a figure is a build product
of the build that checks it —

```bash
./lab sim --world production --config config/demo_gps_shadow.json --headless --fixed-step \
          --frame-max 30 --screenshot docs/img/readout-gps-shadow.png
./lab sim --world production --config config/demo_wifi.json --headless --fixed-step \
          --frame-max 30 --screenshot docs/img/readout-radio.png
```

Both lines run in the headless CI (dummy video driver) and `tests/test_readout_pictures_w7.py` runs
them, checks the PNG and its size against the config, checks that the frame is not one flat colour, and
checks that the sim time printed for frame 30 is the same number twice. `tools/worldpic.py` draws the
arenas the same way and `check.sh` draws both, so a stale picture cannot survive a build.

![Frame 30 of `config/demo_gps_shadow.json` on `production`, 0.60 s of simulation time: the robot at
its spawn pose with the readout line under it. The shadow zones are the two grey rectangles at the
right edge of the hall — this frame is before the robot reaches one, and the `ellipse 2σ: half-axis
… m = … px at 54 px/m` segment is what one pixel of that ellipse is worth.](docs/img/readout-gps-shadow.png)

![Frame 30 of `config/demo_wifi.json`, the same 0.60 s: the access point at the left wall, the robot at
the spawn pose with the link bar above it and the `link …` segment in the readout line — the best the
link gets in this hall, which is the other half of the story the table below measures.](docs/img/readout-radio.png)

*What these two frames are and are not:* they are the readout line and the map at the spawn pose, which
is the state a headless build reaches without anyone driving. A picture of a robot **inside** the shadow
or **under** the failsafe needs a controller and a few thousand frames — `--frame-max` counts drawn
frames, and 30 of them is 0.60 s — so the in-flight numbers in this file stay quoted transcripts of
driven runs (`./lab run … --controller student/link_autonomy_example.py`, `./lab grade … --log`), and
`tests/test_wifi_w6.py` asserts the positions those transcripts name. What the two frames above prove
is the part a screenshot can prove: that the segments exist, that they are drawn from the same numbers
the topics carry, and that the picture is rebuildable.

*Both frames are 1120 × 700 pixels — `view.width` × `view.height` of `config/default.json`, which is
what the `--screenshot` line saves and what the test compares against — at the default `view.scale` of
54 px/m, the ruler the σ segment prints.* The size is a config value and not a constant: change the
window, and both the picture and the assertion change with it.

## Second drive train: steering (Ackermann)

`--variant steering` is a different kinematic world, not a worse mecanum robot: one driven axle, one
steered axle, and therefore **one** degree of freedom in the velocity — metres per second along the
car, and a turn rate that is not free but follows from the steering angle. Nothing else changes:
`--variant` stays empty by default, which is still the stock mecanum robot, so every graded task and
both reference solutions drive exactly what they drove before. The whole car is three numbers:

| number in `DEFAULT_CONFIG["steering"]` | default | what it decides |
|---|---|---|
| `wheel_base` (L) | 1.00 m | the lever between the axles: `omega = v·tan(delta)/L`, `R = L/tan(delta)` |
| `steer_max_deg` | 32° | the physical end stop of the rack, which moves at `steer_rate_deg_s` = 60°/s |
| `v_max` | 0.8 m/s | drive speed; `track` 0.62 m is what makes the two front angles differ |

From the first two follows the number that shapes every path: `R_min = L / tan(delta_max)` = **1.60 m**.
A tighter corner is not a driving mistake here, it is a car that does not exist. Measured as the chord
of a half turn — not as `v/omega`, which would compare the model with itself — demanded 14°/22°/30°
come out as 4.011/2.475/1.732 m against `L/tan(delta)`: ratio 1.0000 each, and a command of 45° still
drives 1.600 m (`tests/test_steering_w4.py`). The two front wheels never steer equally, because the
car turns about one point on the rear axle line: `atan(L/(R − W/2))` inside, `atan(L/(R + W/2))`
outside. All four wheel speeds are still published under the old labels `VL/VR/HL/HR` (the rear pair
rolls, its mean *is* `v`); the angles are the extra `steer_deg` field of `/sim/robots`.

```bash
./lab run --world production --robot car --variant steering \
          --controller student/steering_example.py --headless --seconds 60 --truth
```

That example drives one lap of a rounded rectangle — every corner `R_min + 0.15 m`, because the rack
needs half a second to get from straight ahead to 30° — and then parks in front of a shelf by LIDAR.
Measured on that command: 20.3 m driven, **0 wall contacts**, mission `done` after 45.2 s of the 60 s
budget, gap to the shelf 0.959 m. `vy` appears in it once, in the comment saying why it never helps:
**a steering car cannot strafe.** The simulator answers
`steering robot cannot strafe, vy=0.25 dropped` once per robot and keeps driving on `vx` and `omega`.
Four wheel speeds cannot steer it either — their mean becomes `v` and the rack stays straight, which
is the legible answer to a student who sends lab 1's inverse kinematics to this car.

The odometry is the second lesson. The mecanum integrator averages four wheel speeds; this one
integrates a single steering angle that it only *believes*, because the car has no steering encoder.
`odom.steer_max_scale` is that belief (default 1.0 = the true end stop). 20 s at full lock, the other
sensor noise switched off, truth against odometry:

| `odom.steer_max_scale` | yaw error after 20 s | place error |
|---|---|---|
| 1.0 | +0.0° | 0.00 m |
| 1.1 — rack believed at 35.2° | **+44.9°** | 1.12 m |
| 0.9 | −42.0° | 1.34 m |

`./lab run --variant steering --set odom.steer_max_scale=1.1 --truth --seconds 20` shows it: the
odometry ghost (`o`) leaves a circle of its own under the car. The error is systematic, so it does not
average out — every corner adds the same wrong degree.

Against an obstacle this car behaves like the mecanum one, and that is what makes the demo transfer:
the body stops, the drive wheels keep the speed the motor demands, and the wheel counters integrate
what the wheels did, not what the car did. 30 s of 0.6 m/s into a wall (`steering.slip` = 1.0, the
default): **4.38 m driven, 17.91 m counted**, `contacts 1`, the wheels still at 12.0 rad/s; with
`steering.slip=0` (static friction) the wheels stand still and the odometry stays honest to 1 cm.
Model, wiring and what is published where: `docs/CONTRACT.md` §5.1 and §6.12.

## An empty hall, and a source to find

`--world open` is 30 × 20 m of floor with a border wall and nothing else: no obstacle, no floor
marking, no goal, two spawns on the centre line. Odometry drift is the subject there — in every other
arena a wrong wheel constant shows up as a collision first, here it stays what it is: a number.
Measured on 20 m driven straight at 0.5 m/s (seed 1, 40.1 s):

| `odom.geometry.wheel_radius_scale` | odometry counted | ghost away from the robot | wall contacts |
|---|---|---|---|
| 1.0 (default) | 20.01 m | 0.01 m | 0 |
| 1.05 — `config/demo_open_odrift.json`, GPS switched off | 21.01 m | **1.00 m** | 0 |

```bash
./lab sim --world open --config config/demo_open_odrift.json    # hold Up, watch `o` walk away
./lab sim --world open                                          # the same hall with honest wheels
```

### A radiation source: `/poi`

`pois` plants a Point of Interest in a world and `mecanum_lab/pois.py` gives it a field:
`intensity = activity / (1 + (d/d0)²)` up to the source's `range`, 0 beyond it, with counting noise
on top (`poi.counts` per unit, so the sigma of one reading is `sqrt(counts)/counts` — the reading is
rough where the field is weak). The robot gets `/<robot>/poi` at `poi.rate` (5 Hz) with
`t, intensity, name, distance`, and the readout line prints `poi src1 0.803` so the sensor is
observable without a second terminal. Measured for the shipped source, 4000 readings per spot:

| distance | intensity | σ of one reading | relative |
|---|---|---|---|
| 0.5 m | 0.800 | 0.045 | 5.6 % |
| 2 m | 0.200 | 0.022 | 11 % |
| 4 m (= `range`) | 0.059 | 0.012 | 21 % |
| 4.5 m | 0.000 | 0.000 | — silence |

**The field goes through the furniture.** Nothing in the model asks what stands between the robot and
the source, because a gamma source does not care about a shelf: in `production` the LIDAR pointed at
the source reports the table in front of it at 0.60 m while the counter, 3.6 m away, still reports
0.072. Two sensors that disagree because one of them needs a straight line and the other does not —
and neither of them is wrong. That is why the layer `p` draws the source as a symbol rather than as
an obstacle, and with `debug_truth` the rings where the reading is half and a tenth of the activity.

`distance` is **not** in the message: `poi.publish_distance` is false by default, because turning an
intensity series into a distance is the exercise. `pois` is empty in the defaults, so no detector is
built, no message is published and no random number is drawn in any graded run.

```bash
./lab sim --world open --config config/demo_poi_exploration.json         # drive past it, watch `p`
./lab sim --world production --config config/demo_poi_exploration.json   # same source, tables between
./lab sim --world open --config config/demo_poi_exploration.json --truth # + the field rings
```

Model, validation (a source outside the walls is refused, not dropped) and the noise model:
`docs/CONTRACT.md` §6.13.

## One access point per hall: the radio link `/link`

Every command your robot receives arrives by radio in this simulator too — with `wifi.enabled` true,
`/cmd_vel` and `/wheels` go through one access point per hall, and the two things that kill a real
link kill it here: **distance and walls in the straight line to that access point**. The model is one
line of arithmetic (`mecanum_lab/wifi.py`, stdlib `math`, no WiFi stack and no wish to have one):

```
rssi = tx_dbm − 10 · n · log10(d / d0) − (walls crossed) · wall_db + shadow      [dBm]
q    = clamp((rssi − floor_dbm) / (good_dbm − floor_dbm), 0, 1)
```

Wall crossings use the LIDAR's own ray test, so a beam and a radio wave cannot disagree about which
rectangles exist. Measured with the shipped numbers (`tx_dbm −40`, `n 2.4`, `wall_db 12`, floor
−85 dBm, good −50 dBm) at spots in `production`, whose access point hangs at (1.0, 2.0):

| distance | free floor | 1 wall | 2 walls |
|---|---|---|---|
| 2 m | −47.2 dBm · q 1.00 | −59.2 · q 0.74 | −71.2 · q 0.39 |
| 8 m | −61.7 · q 0.67 | −73.7 · q 0.32 | −85.7 · q 0.00 |
| 15 m | −68.2 · q 0.48 | −80.2 · q 0.14 | −92.2 · q 0.00 |

Six of these nine cells are open floor in `production` and measurable by driving there; the six spots
and the three that the furniture does not offer are all asserted in `tests/test_wifi_w6.py`. What
quality is worth: `p_drop = (1 − q)²` — q 0.90 loses a frame in 100, q 0.50 one in four, q 0.32 one
in 2.2 — and the wire costs `latency_ms · (1 + 2(1 − q))`, so 20 ms on a good link and 47 ms at
q 0.32. **Walls, not metres, are what kills a link in a furnished hall**: one rack between you and
the AP at 8 m is worse than open floor at 15 m, and 23 % of this hall's free floor is below the level
a robot can be driven on.

Below `wifi.link_up_q` (0.15) a countdown runs; if the level stays under it for `wifi.link_timeout`
(1.5 s of **simulation** time), the link is down: nothing external arrives any more, the robot's
outline turns amber, `/sim/robots` says `mode: autonomy`, and `wifi.autonomy` decides what it does
then — `stop` holds the place it was left at (what a command watchdog is), `dead_reckoning` keeps
executing the last command that really arrived (what "the robot walks itself home" means, wall in the
way or not). Which of the two a practice robot should ship is a question for the practice team, and
the run below lets you drive both.

`/<robot>/link` comes out at `wifi.rate` (5 Hz) with `t, quality, rssi_dbm, ap, up, dropped,
latency_ms` — `ap` included on purpose, so a controller can drive back into coverage *before* the
failsafe fires. Layer `n` draws the access point, the line to each robot on the very segment the
budget is computed on and a quality bar over each; the readout line prints
`wifi q 0.42 -72.3 dBm 1 wall lost 4/120 43 ms`. The access point belongs to the hall
(`wifi.ap_by_world`), and `--set wifi.ap=[10,6]` moves it for one run — `wifi.effective_ap` on
`/sim/config` names whichever layer won, so nobody has to guess.

```bash
./lab sim --world production --config config/demo_wifi.json          # drive it, look at layer n
./lab run --world production --config config/demo_wifi.json --robot muster \
        --controller student/link_autonomy_example.py --seconds 60    # it drives into the shadow
./lab sim --world production --config config/demo_wifi.json --set wifi.autonomy=dead_reckoning
ros2 launch launch/wifi.launch.py wall_db:=20 ap:=[10,6]             # the same knobs as --set
```

`wifi.enabled` is false in `mecanum_lab/types.py`, and that is not a hint but a guarantee: with the
option off no radio is built, no random number is drawn for it and no `/link` is published, so the
command stream and the recorded CSV of a graded run are byte for byte what they were before a radio
existed (`tests/test_wifi_w6.py` compares two recorded files). The exercise, the four numbers to
check with a tape measure and what this model deliberately does **not** do — no second access point,
no roaming, no shared airtime, no multipath and no retransmission — is spelled out line by line in
`config/demo_wifi.json`. Model and delivery rules: `docs/CONTRACT.md` §6.14.

## Task and arena belong together

`--task` takes task ids, groups or a comma list: `v1` and `alle` (Experiment 1),
`kf_alle`/`v2` (Experiment 2), `beide` (really all). Every task names its arena in
`config/tasks.json` (`"world"`); `./lab` and `tools/fastgrade.py` follow that setting,
`--world` overrides it. Grading in the wrong arena gets you wall contacts
instead of points.

![The five arenas at one common scale, in three columns: arena (24 × 16 m, open hall, the four
state-estimation tasks), maze (13 × 11 m built on 1 m grid cells), open (30 × 20 m, border walls
only — the hall for drift work), production (20 × 12 m hall with six tables, the four kinematics
and odometry tasks) and track (18 × 11 m ring around a central island). Solid blocks are walls that
collide, dashed lines are painted floor markings without collision, dots are the start poses of
robots 1–4 with their heading, the bullseye is the goal.](docs/img/worlds.png)

*The five arenas, drawn by `python3 tools/worldpic.py` — one metre has the same thickness in
every panel, so the halls are comparable.* The numbers under the panels come from the same
sources the checks use: task titles from `config/tasks.json`, and the width of the tightest
passage on the widest start→goal path from `tools/worldcheck.py`. `arena` is deliberately open
(5.25 m at its narrowest) because state estimation wants free space and a GPS outage in a
corner; `maze` is deliberately tight (0.50 m free where the robot needs 0.46 m) because that
is what makes odometry hard. `open` has no goal and nothing in it, so there is no path to quote a
passage for and its panel states what the same check measures without one: 9.25 m of free space in
the widest spot, 0.46 m of it needed — the number is the hall, not a gap between two walls. Add an
arena or move a task and the figure follows when you regenerate it — `tools/check.sh` draws it as a
check, so a stale image cannot survive a build, and two tests compare these numbers with the tool
rather than with this file.

## Where configuration lives

| Layer | File / option | Note |
|---|---|---|
| Defaults | `mecanum_lab/types.py` → `DEFAULT_CONFIG` | every sensor number of both experiments |
| Site | `config/default.json` | overrides the defaults |
| Task profile | `config/tasks.json` → `sim` | measured sensing per task (GPS rate, σ, outage, IMU) |
| Command line | `--set gps.sigma_xy=1.2 --set imu.rate=400 --set gps.gap='[14,8]'` | always wins |
| Launch file | `ros2 launch launch/kf.launch.py --show-args` | 47 arguments, all mapped onto `--set` |
| Launch file, radio | `ros2 launch launch/wifi.launch.py --show-args` | the 15 `wifi` keys |

Arenas: `arena` (open, Experiment 2), `production`, `maze`, `track`, `open` (nothing but floor and
border, for drift work) — or your own
`worlds/name.txt` (`python3 tools/worldcheck.py --world name` checks it, including that the
world is closed). One grid cell is 0.5 m; `"worlds": {"cell_by_world": {"maze": 1.0}}` in
`config/default.json` makes a single world coarser without touching the robot — that is why the
maze has room to drive in while the graded arenas stay as they are.

## Useful commands

```bash
./lab docs                # topics, tasks, examples
./lab robots              # who is driving right now?
./lab grade --task kf_gps --controller student/kf_solution.py --json bericht.json
tools/check.sh            # teaching team's gate (tests, grading, budgets, hygiene)
python3 tools/fastgrade.py --task kf_alle --controller student/kf_solution.py --speed 25
```

## When something does not work

* **`pygame is missing`** → `./install.sh`, or run everything with `--headless`.
* **No display / SSH** → `--headless` (or `SDL_VIDEODRIVER=dummy`).
* **`ros2: command not found`** → source `/opt/ros/<distro>/setup.bash`; everything except
  `ros2 …` works without it too.
* **Grading reports `no measurement pairs`** → your node must publish `/<robot>/kf/pose`,
  and the sim must publish truth (`--truth`, automatic under `./lab grade`).
* **Estimate lags behind the measurements** → you used the wall clock. What counts are the
  message stamps (`fix.t`, `odom.t`), never `time.time()`.
* **Second task starts far away** → discard the previous task's measurements
  (`fix.t >= mission start`), see the handout, section "Three rules".
* **Lists** → `./lab docs`.

## Rules of this codebase

Stdlib + pygame (no numpy/scipy/yaml in the simulator or in the student files), **everything
written and everything named in English** — comments, docstrings, report text, handouts,
identifiers, launch arguments, the keys of `config/tasks.json`. Two tools check it, both a step in
`tools/check.sh`: `python3 tools/langcheck.py` reads the prose, `python3 tools/germanids.py` reads
the names — definitions, arguments, attributes and the short strings that carry data. **Local
variables are outside its scope**, and its docstring says so with the measured size of that open end
(reading them would report 114 spots over 23 identifiers, 29 of them in the student example files —
renaming those is a package with grading runs around it, not a line in a checker). What stays German is
what the tool lets through: ROS topic and message field
names, the wheel names `VL/VR/HL/HR`, the task groups on the command line (`alle`, `beide`,
`kf_alle`), and the German side of the two compatibility maps that keep old files and old shell
histories working (`tasks._LEGACY_KEYS`, the `DEPRECATED` tables in `launch/`) — the tables are in
`docs/CONTRACT.md` §6.11. Deterministic from `--seed`, headless-capable.
Details: `docs/CONTRACT.md` (Experiment 1), `docs/CONTRACT-KF.md` (Experiment 2).
