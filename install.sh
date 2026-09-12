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
      if python3 -c 'import mecanum_lab_interfaces' >/dev/null 2>&1; then
        ok "mecanum_lab_interfaces built (spawn service with its own type)"
      elif [[ $check_nur == 1 ]]; then
        wann "interfaces not built — in the lab room: ./install.sh (takes ~20 s)"
      elif command -v colcon >/dev/null 2>&1; then
        echo "  building mecanum_lab_interfaces (colcon, --symlink-install) …"
        ( cd "$here" && colcon build --paths interfaces/mecanum_lab_interfaces --symlink-install ) \
          && ok "interfaces built — for new shells: source install/setup.bash" \
          || wann "colcon build failed — without the interfaces everything runs over the JSON handshake"
      else
        wann "colcon is missing — without built interfaces the sim answers the JSON handshake (see docs/CONTRACT.md §4)"
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
echo "  ./lab run --robot alice --controller student/controller_template.py"
echo "  ./lab grade --task kf_alle --controller student/kf_template.py --log messung.csv"
echo "  ros2 launch launch/kf.launch.py            (with ROS 2)"
[[ -n "$ros_setup" ]] && echo "In every new terminal, if ROS does not come up automatically:"
echo "  source ${ROS_SETUP:-$ros_setup}"
exit 0
