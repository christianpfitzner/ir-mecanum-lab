# Mecanum lab

2D simulator of a mecanum robot (also differential drive and Ackermann) for the course Intelligent
Robotics. ROS 2 Jazzy, Python, pygame.

## Install

```bash
./install.sh
```

Gets pygame and pytest if missing and builds the ROS packages. `--check` only looks. By hand, in a
ROS 2 workspace:

```bash
colcon build --symlink-install --packages-select mecanum_lab_interfaces mecanum_lab
```

## Run

| command | what |
|---|---|
| `ros2 launch mecanum_lab lab.launch.py` | simulator, window, rviz; the keyboard drives |
| `ros2 launch mecanum_lab kf.launch.py` | experiment 2, Kalman filter |
| `ros2 launch mecanum_lab demo.launch.py demo:=wifi` | a demo: `gps_shadow`, `odom_error`, `open_odrift`, `sensor_reality`, `wifi`, `poi_exploration` |
| `./lab sim` | the simulator without ROS |
| `./lab grade --task alle --controller student/solution.py` | grade a controller |
| `./lab docs` | tasks and topics |

Keys: `w`/`s` drive, `a`/`d` strafe, `q`/`e` turn, `SHIFT` faster, `m` layer menu, `SPACE` pause.

## Parameters

| launch argument | |
|---|---|
| `world:=` | hall: `production`, `arena`, `open`, `track`, `maze`, `rooms` |
| `variant:=` | drive: empty = mecanum, `diff`, `steering` |
| `robot:=` | robot name, default `muster` (topics `/muster/...`) |
| `controller:=` | your node; empty = keyboard only |
| `config:=` | config file under `config/` |
| `rviz:=`, `headless:=`, `truth:=`, `seed:=` | |

All arguments: `ros2 launch mecanum_lab lab.launch.py --show-args`. Any config value can be set with
`--set gps.sigma_xy=1.2`.

## More

Tasks: `config/tasks.json`, handouts in `docs/praktikum/` (`docs/praktikum/anleitung.tex`,
`docs/praktikum/kalman.tex`). Topics and units: [docs/CONTRACT.md](docs/CONTRACT.md). Everything else:
[docs/overview.md](docs/overview.md). Teaching team's checks: `tools/check.sh`.
