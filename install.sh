#!/usr/bin/env bash
# One-command installation for the lab course (lab 1 kinematics, lab 2 state estimation).
#
#   ./install.sh               check the environment, get missing pygame, build the interfaces
#   ./install.sh --check       only check and explain, install nothing
#   ./install.sh --without-ros skip the ROS branch on purpose (the simulator needs no ROS)
#   ./install.sh --user        pygame/pytest through `pip --user` instead of the system package
#   ./install.sh --mit-tests   then a self-test: short simulation + unit tests (headless)
#
# No sudo and no internet needed: without either, the script says what is missing and who has
# to supply it — it fails loudly, not silently. It writes nothing to files in your home folder.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYGAME_HIDE_SUPPORT_PROMPT=1        # pygame otherwise thanks you loudly on every import
cd "$here"

check_nur=0; ohne_ros=0; mit_pip_user=0; mit_tests=0
gebaut=0        # did this run build the ROS package? the final hints are written accordingly
for a in "$@"; do
  case "$a" in
    --check) check_nur=1 ;;
    --without-ros) ohne_ros=1 ;;
    --user) mit_pip_user=1 ;;
    --mit-tests) mit_tests=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option: $a (accepted: --check --without-ros --user --mit-tests)" >&2; exit 2 ;;
  esac
done

problems=0; hinweise=()
ok()   { printf '  \033[32mok\033[0m     %s\n' "$1"; }
wann () { printf '  \033[33mWARNING\033[0m %s\n' "$1"; }
fehlt(){ printf '  \033[31mMISSING\033[0m %s\n' "$1"; problems=1; }

echo "Mecanum simulator lab course — environment"
echo "------------------------------------------------------------------"

# ---------------------------------------------------------------- Python (required, 3.10+)
if python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null; then
  ok "python3 $(python3 -c 'import platform;print(platform.python_version())')"
else
  fehlt "python3 needs to be at least 3.10 (dataclasses with slots, pattern matching)"
fi

# ---------------------------------------------------------------- pygame (required for the window)
if python3 -c 'import pygame' >/dev/null 2>&1; then
  ok "pygame $(python3 -c 'import pygame;print(pygame.ver)' 2>/dev/null | tail -1)"
else
  fehlt "pygame is missing — without pygame the simulator only runs with --headless"
  if [[ $check_nur == 0 ]]; then
    if [[ $mit_pip_user == 1 ]]; then
      echo "        trying: pip3 install --user pygame"
      python3 -m pip install --user pygame \
        || python3 -m pip install --user --break-system-packages pygame \
        || echo "        no network connection? Then: sudo apt install python3-pygame"
    else
      hinweise+=("install pygame once:  sudo apt install python3-pygame   (or: ./install.sh --user)")
    fi
    python3 -c 'import pygame' >/dev/null 2>&1 && problems=0 && ok "pygame present now"
  fi
fi

# ---------------------------------------------------------------- Tests (only for the quality gate)
if python3 -m pytest --version >/dev/null 2>&1; then
  ok "pytest $(python3 -m pytest --version 2>/dev/null | head -1 | awk '{print $2}')"
else
  wann "pytest is missing — only tools/check.sh and the supervisor's tests need it"
  if [[ $check_nur == 0 && $mit_pip_user == 1 ]]; then
    python3 -m pip install --user pytest \
      || python3 -m pip install --user --break-system-packages pytest || true
  else
    hinweise+=("for the unit tests:  sudo apt install python3-pytest   (or: ./install.sh --user)")
  fi
fi

# ---------------------------------------------------------------- ROS 2 (optional, for ros2 launch)
ros_setup=""
if [[ $ohne_ros == 0 ]]; then
  for f in "${ROS_SETUP:-}" "/opt/ros/${ROS_DISTRO:-kilted}/setup.bash" \
           /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/rolling/setup.bash; do
    [[ -n "$f" && -f "$f" ]] && { ros_setup="$f"; break; }
  done
  ros_name="$(basename "$(dirname "$ros_setup")" 2>/dev/null || echo ros)"
  if [[ -n "$ros_setup" ]]; then
    # set +u is required: the ROS setup scripts read AMENT_* variables that are unset on
    # the first source — with `set -u` the script dies silently right here.
    set +u; source "$ros_setup" 2>/dev/null || true; set -u
    if python3 -c 'import rclpy' >/dev/null 2>&1; then
      ok "ROS 2 (${ROS_DISTRO:-$ros_name}) — ros2 launch and ros2 topic work"
      # `ros2 run` is an optional CLI extension (ros2run), not part of a ROS base install. The lab is
      # started with launch files, so this is only the note that says which of the two words a reader
      # of `ros2 run mecanum_lab mecanum-lab …` may type here.
      if ros2 run --help >/dev/null 2>&1; then
        ok "ros2 run works too — 'ros2 run mecanum_lab mecanum-lab <command>' is ./lab under its own name"
      else
        wann "ros2 run is not in this ROS base (apt install ros-$ros_name-ros2run) — every documented command here is a ros2 launch"
      fi
      if python3 -c 'import mecanum_lab_interfaces' >/dev/null 2>&1 \
         && ros2 pkg prefix mecanum_lab >/dev/null 2>&1; then
        ok "mecanum_lab and mecanum_lab_interfaces installed — ros2 launch mecanum_lab <demo> works"
      elif [[ $check_nur == 1 ]]; then
        wann "not built — in the lab room: ./install.sh (takes ~1 min), then source install/setup.bash"
      else
        # Both packages in one pass. `--paths .` finds the root package.xml (the lab itself, whose launch
        # files and config/ end up in share/mecanum_lab) and the interfaces below it — which is what makes
        # `ros2 launch mecanum_lab demo_gps_shadow.launch.py` work from any directory: without this build
        # only the *paths* of the source tree launch, and every demo command in the documentation that
        # starts with a package name is a promise this machine cannot keep.
        #
        # colcon comes from apt (`ros-$ROS_DISTRO-dev-tools`, the way that needs no network) or from pip,
        # which writes into ~/.local — so the pip route is taken only when --user was asked for, and this
        # script's promise of leaving the home directory alone holds for a plain run.
        colcon_bin="$(command -v colcon || true)"
        if [[ -z "$colcon_bin" && $mit_pip_user == 1 ]]; then
          echo "        trying: pip3 install --user colcon-common-extensions"
          python3 -m pip install --user -q colcon-common-extensions \
            || python3 -m pip install --user --break-system-packages -q colcon-common-extensions \
            || true
          if [[ -x "$HOME/.local/bin/colcon" ]]; then
            colcon_bin="$HOME/.local/bin/colcon"
          fi
        fi
        if [[ -n "$colcon_bin" ]]; then
          echo "  building the ROS package (colcon, --symlink-install) …"
          # The lab package is built first and on its own, and the interface package is a second attempt:
          # one colcon pass over both means a machine without `rosidl_default_generators` reports
          # "1 package not processed" and ends up with no lab package either — while the lab package is
          # exactly what makes `ros2 launch mecanum_lab <demo>` work, and the spawn service it would use
          # has a JSON fallback for precisely this case.
          # (`--paths` names package paths and does not search below them, so the interface package has to
          # be spelled out; `--paths .` alone never found it.)
          if ( cd "$here" && "$colcon_bin" build --paths . --symlink-install ); then
            ok "built — for new shells: source install/setup.bash (it also overlays ROS)"
            gebaut=1
            if ( cd "$here" && "$colcon_bin" build --paths interfaces/mecanum_lab_interfaces \
                   --symlink-install ) >/dev/null 2>&1; then
              ok "mecanum_lab_interfaces built — /sim/spawn_robot is a real service"
            else
              wann "interfaces not built — /sim/spawn_robot answers over the JSON fallback, which works"
              hinweise+=("for the real spawn service:  sudo apt install ros-${ROS_DISTRO:-$ros_name}-rosidl-default-generators,"
                         "  then build it:  colcon build --paths interfaces/mecanum_lab_interfaces --symlink-install")
            fi
          else
            wann "colcon build failed — from the source tree everything still works with ros2 launch launch/…"
          fi
        else
          wann "colcon is missing — the demos still run by path: ros2 launch launch/demo_wifi.launch.py"
          hinweise+=("for the demos by package name (ros2 launch mecanum_lab demo_wifi.launch.py), build the"
                     "  package once: sudo apt install ros-${ROS_DISTRO:-$ros_name}-dev-tools — or without sudo, writing to"
                     "  ~/.local: ./install.sh --user — and then run ./install.sh again")
        fi
      fi
    else
      wann "ROS 2 is installed, but rclpy is not importable — ROS parts skipped"
    fi
  else
    wann "no ROS 2 found — the simulator still runs (./lab run, ./lab grade, ./lab sim --stub)"
    hinweise+=("ROS 2 would bring ros2 launch and RViz: install it per docs.ros.org, then restart")
  fi
else
  hinweise+=("this run left ROS 2 out on purpose (--without-ros)")
fi

# ---------------------------------------------------------------- Final
echo "------------------------------------------------------------------"
for h in "${hinweise[@]:-}"; do [[ -n "$h" ]] && echo "  note: $h"; done

if [[ $mit_tests == 1 ]]; then
  echo "self-test (no window, no ROS):"
  SDL_VIDEODRIVER=dummy MECANUM_ROS=stub ./lab sim --headless --seconds 3 --robot test \
    && ok "simulation runs" || fehlt "simulation does not run"
  SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q \
    && ok "unit tests green" || fehlt "unit tests not green"
fi

if [[ $problems == 1 ]]; then
  echo "result: environment incomplete — see the MISSING lines above."
  exit 1
fi
echo "result: ready. Continue in directory $here with"
# Which prefix the hints use: the package name is the form that works from any directory and in a terminal
# that sourced ROS and the workspace — but only once the package is really installed.
launch_pref="launch"
if [[ $gebaut == 1 ]] || ros2 pkg prefix mecanum_lab >/dev/null 2>&1; then
  launch_pref="mecanum_lab"
fi
echo "  ros2 launch $launch_pref/lab.launch.py                (experiment 1: window, the keys drive)"
echo "  ros2 launch $launch_pref/kf.launch.py                 (experiment 2: every knob an argument)"
echo "  ./lab run --robot alice --controller student/controller_template.py     (the same run, no ROS 2)"
if [[ $launch_pref == mecanum_lab ]]; then
  echo "  ros2 launch $launch_pref/demo_gps_shadow.launch.py     (demos: $(cd "$here" && ls config | sed -n 's/^demo_\(.*\)\.json$/\1/p' | tr '\n' ' '))"
  if [[ $gebaut == 1 ]]; then
    echo "        in this terminal first: source install/setup.bash (and in every new one)"
  fi
else
  echo "  ros2 launch launch/demo_gps_shadow.launch.py          (or ./lab sim --config config/demo_wifi.json)"
fi
[[ -n "$ros_setup" ]] && echo "In every new terminal, if ROS does not come up automatically:"
echo "  source ${ROS_SETUP:-$ros_setup}"
exit 0
