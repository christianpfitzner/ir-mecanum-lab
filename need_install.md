# need_install.md — what is still missing on this machine to test the simulator

Checked **2026-09-13** in `/home/pfitzner/git/mecanum-lab` (Ubuntu 24.04 Noble, Python 3.12.3). Every
number below was run here, not copied out of the documentation. Written in English because CONTRACT §1
puts the whole repository in English — `tools/langcheck.py` fails the build over a single umlaut.

## Short answer

**ROS is not missing — it is already installed** (`/opt/ros/kilted`, 147 `ros-kilted-*` packages). The
simulator is testable **without installing anything**:

```bash
cd ~/git/mecanum-lab
python3 -m pytest tests -q
# 461 passed, 2 skipped, 5 warnings in 53.29s
```

Two apt packages are missing and **neither is needed for the tests**: `ros-kilted-rosidl-default-generators`
(§3, it is the real `/sim/spawn_robot` service) and `ros-kilted-ros2run` (§3.2, `install.sh` warns about
it, no documented command uses `ros2 run`). The 2 `skipped` are not a missing install either — they are
a plugin conflict (§2); with one environment variable all 463 run.

## 1. Status as measured

| part | status | version / evidence |
|---|---|---|
| python3 | ok | 3.12.3 |
| pygame (the only hard dependency) | ok | 2.6.1, pip `--user` |
| pytest | ok | 9.1.1 |
| numpy | ok | 1.26.4 (the simulator does not need it, it is there anyway) |
| PyYAML | ok | 6.0.1 |
| ROS 2 | ok | `/opt/ros/kilted`, `rclpy` importable |
| message/service types in use | ok | `Twist, Imu, LaserScan, Odometry, Clock, Trigger, String, Float64, TFMessage` |
| colcon | ok | `~/.local/bin/colcon` (pip `--user`) |
| `mecanum_lab` package built | ok | `ros2 pkg prefix mecanum_lab` → `install/mecanum_lab` |
| build tools | ok | cmake, g++, make |
| LaTeX (handouts) | ok | `/usr/bin/pdflatex`, `latexmk` |
| **`mecanum_lab_interfaces`** | **missing** | blocked by `rosidl_default_generators` (§3.1) |
| **`ros2 run`** | **missing** | `ros-kilted-ros2run` (§3.2) — cosmetic |
| rviz2 | missing | optional, only for `./lab rviz` (§4) |
| matplotlib | missing | optional, only for the PNG of `tools/kfplot.py` (§4) |

The repository's own check agrees, except for the last two lines:

```
$ ./install.sh --check
  ok      python3 3.12.3
  ok      pygame 2.6.1
  ok      pytest 9.1.1
  ok      ROS 2 (kilted) — ros2 launch and ros2 topic work
  WARNING ros2 run is not in this ROS base (apt install ros-kilted-ros2run) — every documented
          command here is a ros2 launch
  WARNING not built — in the lab room: ./install.sh (takes ~1 min), then source install/setup.bash
result: ready.
```

## 2. What really blocks testing is a plugin conflict, not an install

In a shell that has ROS sourced, pytest no longer starts at all:

```
pluggy._manager.PluginValidationError: Plugin 'launch_testing' for hook 'pytest_pycollect_makemodule'
hookimpl definition: pytest_pycollect_makemodule(path, parent)
Argument(s) {'path'} are declared in the hookimpl but can not be found in the hookspec
```

Reason: ROS Kilted ships two pytest plugins (`launch_testing`, `launch_ros`) whose hook signature is
from pytest ≤ 8, while pytest 9.1.1 is installed. `apt install python3-pytest` does **not** repair it —
apt provides 7.4.4, but `python3 -m pytest` prefers the newer one in `~/.local`, and `/usr/bin/pytest`
fails the same way (verified). `tools/check.sh` solved this once and for all and is worth copying:

```bash
# A) tests without ROS (default, unchanged behaviour)
cd ~/git/mecanum-lab && python3 -m pytest tests -q
# 461 passed, 2 skipped
#   SKIPPED tests/test_package_f.py:224   ROS 2 package 'launch' not sourced
#   SKIPPED tests/test_sensor_info.py:84  no ROS 2 in this shell

# B) tests WITH ROS — all 463, nothing skipped
source /opt/ros/kilted/setup.bash && source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
# 463 passed, 5 warnings in 58.23s
```

`-p no:launch_testing -p no:launch_ros` also works but pins the plugin names of today; disabling the
autoload covers whatever the next ROS release advertises. A display is **not** needed for the tests —
this ran over SSH with `DISPLAY` empty, and no test opens a window. `SDL_VIDEODRIVER=dummy` becomes
relevant only where a pygame **window** is supposed to open (§5).

## 3. What is missing

### 3.1 `rosidl_default_generators` — the one package with a consequence

`interfaces/mecanum_lab_interfaces` (the `.srv` behind `/sim/spawn_robot`) cannot be built today. The
build was reproduced in a temporary directory, error unambiguous:

```
CMake Error at CMakeLists.txt:5 (find_package):
  Could not find a package configuration file provided by "rosidl_default_generators"
Failed   <<< mecanum_lab_interfaces
```

`rosidl_default_runtime` is present, the **generator** is not. The ROS apt source is configured
(`ros2.sources` → packages.ros.org), candidate `1.7.2-1noble.20260813.030946`.

```bash
sudo apt update
sudo apt install -y ros-kilted-rosidl-default-generators
source /opt/ros/kilted/setup.bash
cd ~/git/mecanum-lab
./install.sh                       # builds the lab package and the interface package, ~1 min
```

By hand, without the rest of `install.sh`:

```bash
cd ~/git/mecanum-lab
source /opt/ros/kilted/setup.bash
colcon build --paths . --symlink-install
colcon build --paths interfaces/mecanum_lab_interfaces --symlink-install
source install/setup.bash
python3 -c "import mecanum_lab_interfaces; print('ok')"
ros2 pkg list | grep mecanum       # mecanum_lab  mecanum_lab_interfaces
```

> **Without this package nothing is broken.** `ros_bridge.py` answers `/sim/spawn_robot` through a JSON
> handshake on `/sim/spawn_next` (a `Trigger`), and `install.sh` says so out loud: `interfaces not
> built — /sim/spawn_robot answers over the JSON fallback, which works`. No test fails over it.

### 3.2 `ros2 run` — a warning without a consequence

`./install.sh --check` reports `ros2 run is not in this ROS base`. True, and harmless: every command
this repository documents is a `ros2 launch` or `./lab`. Fill it in whenever apt runs anyway:

```bash
sudo apt install -y ros-kilted-ros2run
```

### 3.3 Without internet

`sudo apt install -y python3-pygame python3-pytest` covers the entire non-ROS mode (`./lab sim`,
`./lab run`, `./lab grade`) and all 461 unit tests.

## 4. Optional — only if the feature is wanted

```bash
sudo apt install -y ros-kilted-rviz2      # a second view beside the window: ./lab rviz
pip3 install --user matplotlib            # PNG plots in tools/kfplot.py
```

Both are comfort, not a prerequisite: `./install.sh --check` does not look for them, and the 461 tests
passed before either was considered. `mecanum_lab/rviz_view.py` prints the hint as
`sudo apt install ros-$ROS_DISTRO-rviz2` — **careful**, this `setup.bash` does *not* export
`ROS_DISTRO` (`echo $ROS_DISTRO` is empty after sourcing it here), so the expansion produces
`ros--rviz2`. Either write the distro literally or `export ROS_DISTRO=kilted` first.

## 5. Window versus SSH

This is an SSH session, `DISPLAY` and `WAYLAND_DISPLAY` empty. The window still cannot be seen, but
nothing in the test suite needs it, and a headless run is verified (`./lab sim --headless --robots
alice --seconds 3` → exit 0). To look at the window: work at the machine, or forward X with
`ssh -X/-Y`. Checking the topics does not need a window:

```bash
./lab sim --headless --robots alice,bob     # terminal 1
ros2 topic echo /muster/imu --once         # az at rest ≈ +9.81 — that is correct
MECANUM_ROS=stub ./lab sim --headless      # without the ROS bus at all
```

## 6. Everything at once

```bash
cd ~/git/mecanum-lab
sudo apt update
sudo apt install -y ros-kilted-rosidl-default-generators ros-kilted-ros2run ros-kilted-rviz2
pip3 install --user matplotlib
source /opt/ros/kilted/setup.bash
./install.sh                                              # checks and builds both ROS packages
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q   # target: 463 passed
```

`sudo` asks for a password here (`sudo -n true` → "a password is required"), so run these in a terminal
rather than from a script.

## 7. Deliberately **not** needed

Gazebo/Ignition (the apt source `gazebo-stable.list` exists, nothing in this repository uses it), Isaac
Sim / Isaac Lab, CARLA, ROS 1, a venv, Docker, or `ros-base`/`desktop` as a metapackage. The simulator
is a 2D pygame model; ROS 2 is an optional layer above it and `mecanum_lab/stub.py` takes over the bus
when ROS is not sourced.
