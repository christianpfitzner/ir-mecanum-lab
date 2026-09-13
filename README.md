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

## How it works

![The chassis on the left, the message loop on the right. Left: the four wheels on their axle points,
each with its roller axis at 45°, and the three body velocities. Right: one `cmd_vel` becoming four wheel
speeds becoming a pose becoming six sensor messages becoming an estimate.](docs/img/howitworks.png)

Both halves are drawn by `tools/labmap.py` out of the modules the exercise is graded with — the wheel
mounts from `render.wheel_mounts()`, the roller axes from `physics.inverse_kinematics()`, the topic names
from `types.topic()` — and the tool refuses to draw if a roller axis is not perpendicular to the velocity
its own wheel produces. The one rule the left half is about: a mecanum wheel pushes only across its
roller axis, so with all four rollers of one sign the sideways parts cancel and the robot goes forward,
and with the front pair against the rear pair the forward parts cancel and it strafes. The two red cards
on the right are the two places where the simulator lets a number be wrong on purpose — wheel slip at a
wall, and wheel constants in `odom.geometry` — because an estimator that never has to doubt a sensor is
not being taught anything. Signs and topic contracts: `docs/CONTRACT.md` §5 and §6.

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
./install.sh                                      # also builds the ROS package (colcon, ~1 min)
source install/setup.bash                         # every new terminal, until ROS is the only one
ros2 launch mecanum_lab lab.launch.py             # sim + window, keyboard drives 'muster'
ros2 launch mecanum_lab demo_wifi.launch.py       # one demo by name — six of them, table below
ros2 launch mecanum_lab kf.launch.py              # Experiment 2, 47 arguments, all mapped onto --set
ros2 topic echo /muster/imu --once                # az at rest ≈ +9.81 — that is correct
ros2 topic echo /tf --once                        # map → muster/odom → muster/base_link → laser
```

Nothing needs that build. Every file also launches by path from this tree
(`ros2 launch launch/demo_wifi.launch.py`), and the plain commands stay:
`./lab sim --robots alice,bob`, `./lab spawn --name carlo`, `./lab rviz --robot alice`. To keep the
in-process bus even with ROS sourced: `MECANUM_ROS=stub ./lab sim --headless`.

## The simulator window (what you hide is not what is published)

| input | what it does |
|---|---|
| mouse wheel | zoom **to the cursor** · `+`/`-` zoom to the middle · `f` shows the whole world |
| drag with the left button | pan (the map follows the mouse) · right button centres again |
| `1`…`9` / `0` | follow one robot / show everything · window resizable, the camera keeps up |
| `w`/`s` drive · `a`/`d` turn | car-style keyboard driving, on unless `--no-teleop`; the keys set body speeds, this is not a game |
| `Up`/`Down` drive · `Left`/`Right` **strafe** | the same keys as the four degrees of freedom of a mecanum chassis instead of a car's two |
| `q`/`e` (or `,`/`.`) turn left/right | ±0.9 rad/s yaw. With teleop on, `q` **turns** instead of quitting — `ESC` or the window's close button ends the run |
| `SPACE` pause | `q` quits only when teleop is off |
| `m` | opens the layer menu (starts closed so it covers nothing): lidar scan, odometry trail, gps fix, estimate + σ ellipse, wheels, velocity vector, floor markings, goal, readout lines, gps shadow zones, odometry ghost + drift, radiation source + field, radio link + access point |
| `l t g k r v c z h x o p n` | switch a single layer — the same as clicking its row (`r` wheels, `c` floor markings, `x` GPS shadow, `o` odometry ghost, `p` radiation source and its field rings, `n` the radio link) |

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
### The demos: four configurations that make one instrument lie

Each file changes one block of the config and leaves the graded defaults alone, so starting a demo is
a `--config`, not an edit. Each demo has a launcher of its own — `ros2 launch mecanum_lab
demo_gps_shadow.launch.py` (or `launch/demo_gps_shadow.launch.py` by path, no build needed) — and
`ros2 launch launch/demo.launch.py demo:=gps_shadow` is the same thing with the name as an argument.

| demo (config, `demo:=` name, launcher) | what it turns on | what you see |
|---|---|---|
| `demo_gps_shadow` | `gps.zones`: two rectangles and a blackout | hatched shadow (`x`), fixes that scatter, then none at all |
| `demo_odom_error` | `odom.geometry`: the odometry believes radius ×1.05, lever ×0.97 | the ghost (`o`) drifts ahead: 0.61 m per 12 m of straight lane |
| `demo_open_odrift` | no GPS at all (`gps.gap` past the end of the run) on top of the wrong radius | dead reckoning in an empty hall: the trail (`t`) shows where odometry thinks it has been |
| `demo_sensor_reality` | latency, dropout, staleness, chip temperature | `sensor_state` in the readout: `lost_gps`, `q_gps`, `temp_imu` |
| `demo_wifi` | the radio link and its access points | the `link` panel (`n`), and a robot that loses its autonomy mid-lane |
| `demo_poi_exploration` | a radiation source somewhere in the hall | the `poi` panel (`p`) and a robot hunting by one number |

Wheel slip has no file of its own: it is `robot.slip`, and `./lab sim --world arena` with the robot
driven into the east wall shows the odometry counting metres that were never driven.

The commands, the measured numbers behind those rows and what each demo costs a graded run:
**[docs/demos.md](docs/demos.md)**.
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
| Launch file, demos | `ros2 launch launch/demo.launch.py --show-args` | the six demo configs, `rviz:=auto` |
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
the names — definitions, arguments, attributes, the constants a file assigns at module level, and the
short strings that carry data. **Local variables are counted and printed on every run, and fail it only
with `--locals`** (294 spots in 56 names on today's tree, most of them in the student example files;
renaming those is a package with the grading runs of both experiments around it, not a line in a
checker). What stays German is
what the tool lets through: ROS topic and message field
names, the wheel names `VL/VR/HL/HR`, the task groups on the command line (`alle`, `beide`,
`kf_alle`), and the German side of the two compatibility maps that keep old files and old shell
histories working (`tasks._LEGACY_KEYS`, the `DEPRECATED` tables in `launch/`) — the tables are in
`docs/CONTRACT.md` §6.11. Deterministic from `--seed`, headless-capable.
Details: `docs/CONTRACT.md` (Experiment 1), `docs/CONTRACT-KF.md` (Experiment 2).
