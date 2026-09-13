#!/usr/bin/env bash
# Supervisor quality gate: everything that must be green before the lab is released.
# Runs without ROS and without a window.
#
#   tools/check.sh            base checks (no ROS)
#   tools/check.sh --ros      plus real ROS 2 evidence (expects ROS to be sourced)
set -uo pipefail
cd "$(dirname "$0")/.."; export SDL_VIDEODRIVER=dummy MECANUM_LOG=warning PYGAME_HIDE_SUPPORT_PROMPT=1
fail=0
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }
run() { if eval "$1"; then echo "  ok: $1"; else echo "  FAIL: $1"; fail=1; fi; }

step "Static check (syntax of every module)"
run "python3 -m compileall -q mecanum_lab student tools"
step "Unit tests (no ROS, no window)"
run "python3 -m pytest tests -q"
step "LOC budget"
run "python3 tools/loc.py"
step "Launch arguments (every one read is declared, every one declared has one line of help)"
# `ros2 launch … --show-args` prints exactly these three things, and an argument that is read but
# not declared is silently ignored — the classic "my setting does nothing" hour.
run "python3 tools/launchargs.py --quiet"
step "Language (CONTRACT section 1: written prose is English)"
# Not cosmetics: handouts, comments and report text are what students read, and German creeps
# back in with every new feature. langcheck reports umlauts anywhere and German words in prose.
run "python3 tools/langcheck.py --quiet"
step "Identifiers (CONTRACT section 1: names are English too)"
# The prose was English long before the identifiers were. This parses every module and fails on a
# German identifier or dict key outside the documented exceptions (ROS names, the wheel labels,
# the two compatibility maps) — without it, the rename of 2024-09 would quietly undo itself.
run "python3 tools/germanids.py --quiet"
step "Repository hygiene (.gitignore keeps build junk out of git)"
# Once a byte-code or LaTeX by-product is committed, every student clone carries it forever and
# every regeneration shows up as a diff. This fails when one reappears in the index.
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  junk=$(git ls-files | grep -E '\.(py[cod]|aux|fls|fdb_latexmk|synctex\.gz|toc|out|lof|lol|log)$' || true)
  if [ -n "$junk" ]; then
    echo "  FAIL: build junk is tracked:"; printf '%s\n' "$junk" | sed 's/^/         /'; fail=1
  else
    echo "  ok: no byte-code or LaTeX junk in the index"
  fi
else
  echo "  skipped: not a git worktree"
fi
step "Arena picture for the documentation (worlds/*.txt -> docs/img/worlds.png)"
# Generated, not hand-drawn: this proves every world still draws and that worldpic agrees with
# worlds/*.txt. It writes to /tmp — docs/img/worlds.png is updated on purpose by the tool.
run "python3 tools/worldpic.py --out /tmp/worlds_check.png"
step "Short stub simulation, maze world, with the reference solution"
run "./lab run --world maze --robot muster --controller student/solution.py --headless --seconds 6"
step "Grading the reference solution on all tasks (experiment 1)"
# --controller is not a detail: without a node nobody drives, and T2..T4 come out as "barely
# moved" — a test rig that runs without the reference solution checks nothing.
run "./lab grade --robot muster --task alle --controller student/solution.py --json /tmp/grade.json --seconds 150"
step "Grading the same seed again at four times real time (a loaded host must not be the variable)"
# The same reference solution through the same grader, only at a defined pace: --speed N takes N
# simulation seconds per wall second, --fixed-step leaves the wall clock out of the loop entirely.
# A limit that only holds on an idle machine is not a limit. Measured spread and the reason why
# this extra run is experiment 1: docs/CONTRACT.md §9 (experiment 2 has rate_min limits, which are
# a limit on the node's CPU share, not on the filter - so they cannot be graded faster than the
# node can tick).
run "./lab grade --robot muster --task alle --controller student/solution.py --headless \
    --speed 4 --json /tmp/grade_fast.json --seconds 150"

step "Teleop path (pass-through) without a student node"
# Two real names on purpose: `a` and `b` are shorter than NAME_RE allows, so the step used to pass
# while the simulator answered two "invalid robot name" errors — a gate that reports errors and
# calls it green is the one thing a gate must not be.
run "./lab sim --world track --robots alice,bob --headless --seconds 3"

step "Radio link: the demo config on the real-time path"
# The one option of the feature is off in the defaults, so nothing above exercises it: this is the
# run that shows `wifi.enabled` builds a radio, publishes /link and keeps the run alive for 20 s of
# simulated driving. The graded thresholds of both experiments are calibrated without a radio, which
# is exactly why this step and the pytest guard are separate: one proves the option works, the other
# that switching it off changes nothing.
run "./lab sim --world production --config config/demo_wifi.json --headless --seconds 20"
step "Radio link: the example drive walks out of coverage and the failsafe stops it"
# Not a wall-clock race: --fixed-step with the same physics step means the autonomy flip lands the
# same sim second every time (tests/test_wifi_w6.py asserts the number, this asserts the exit code).
run "./lab run --world production --config config/demo_wifi.json --robot muster \
    --controller student/link_autonomy_example.py --headless --fixed-step --seconds 45"

step "Experiment 2: arena world, installation, measurement log tool"
run "python3 tools/worldcheck.py --world arena"
run "bash -n install.sh"
run "./install.sh --check"
if [[ -f student/kf_solution.py ]]; then
  step "Experiment 2: grading the reference solution (kf_alle)"
  run "./lab grade --robot kf --task kf_alle --controller student/kf_solution.py --json /tmp/kf.json --seconds 230"
  step "Experiment 2: measurement log and evaluation"
  run "./lab grade --robot kf --task kf_gps --controller student/kf_solution.py --log /tmp/messung.csv --seconds 60"
  run "python3 tools/kfplot.py /tmp/messung.csv"
else
  echo "  skipped: student/kf_solution.py is not there yet"
fi
if [[ -f student/kf_template.py ]]; then
  step "Experiment 2: the template runs without crashing"
  run "./lab run --world arena --task kf_gps --robot tpl --controller student/kf_template.py --headless --seconds 10 --truth"
else
  echo "  skipped: student/kf_template.py is not there yet"
fi

if [[ "${1:-}" == "--ros" ]]; then
  step "ROS 2: topics and services against a running simulation"
  # set +u is required: the ROS setup scripts read AMENT_* variables that are not set
  # before the first source — with `set -u` the source call aborts (as in ./lab).
  set +u
  source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash || { echo "ROS not sourced"; exit 1; }
  set -u
  # The sim has to outlive all ROS checks below together (topics, services, three launch
  # runs); otherwise a `ros2 topic hz` afterwards waits for messages forever.
  ./lab sim --world track --robots ros_test --headless --seconds 300 & sim=$!
  sleep 6
  run "ros2 topic list | grep -q /ros_test/odom"
  run "timeout 25 ros2 topic echo /ros_test/odom --once"
  run "timeout 25 ros2 topic echo /ros_test/scan --once | head -20"
  # A rate test never ends on its own: the timeout is the abort, not the failure case. So
  # measure into a file and check its content — through the pipe, pipefail would report the
  # timeout as a failure of the simulation.
  run "timeout 25 ros2 topic hz --window 10 /ros_test/scan >/tmp/hz_check.txt 2>/dev/null; grep -q average\\ rate /tmp/hz_check.txt"
  # The typed spawn service needs the built interfaces (mecanum_lab_interfaces); without
  # them the bus falls back to the JSON handshake, and `ros2 service call` then finds no
  # service of that type. So only check it when the interface is importable.
  if python3 -c 'from mecanum_lab_interfaces.srv import SpawnRobot' >/dev/null 2>&1; then
    run "./lab spawn --name gast"
  else
    echo "  skipped: interfaces not built — ./install.sh builds them, until then spawn runs as a JSON handshake"
    run "./lab robots"
  fi
  run "timeout 25 ros2 service call /sim/reset std_srvs/srv/Trigger || true"
  run "timeout 25 ros2 topic echo /sim/robots --once"
  step "ROS 2: launch files"
  run "timeout 25 ros2 launch launch/sim.launch.py headless:=true seconds:=15"
  step "ROS 2: the radio link through launch/wifi.launch.py"
  # Same rule as the kf step below: ros2 launch also waits for the node it started, so running into
  # the timeout is the healthy end of the run and an early exit is a broken launch file.
  mkdir -p .runs
  timeout 45 ros2 launch launch/wifi.launch.py headless:=true seconds:=15 wifi:=true \
      > .runs/wifi_launch.log 2>&1
  rc=$?
  if [[ $rc == 0 || $rc == 124 ]]; then
    echo "  ok: wifi.launch.py ran 45 s with the example driver (rc $rc)"
    grep -q "link to 'alice' down" .runs/wifi_launch.log \
      && echo "  note: the link went down during that run (the demonstration worked)"
  else
    echo "  FAIL: wifi.launch.py exited with $rc"; tail -12 .runs/wifi_launch.log; fail=1
  fi
  step "ROS 2: own student node against the real simulation"
  timeout 40 ros2 launch launch/lab.launch.py headless:=true seconds:=35 controller:=student/solution.py || true
  if [[ -f student/kf_template.py ]]; then
    step "ROS 2: lab 2 through kf.launch.py (KF topics in the real sim)"
    # Not run() with a plain timeout: ros2 launch also waits for the student node, so running
    # over the timeout (124) is the expected end of a healthy run — but an exit before that is a
    # crashed launch file, which is exactly what no step used to catch.
    mkdir -p .runs
    timeout 45 ros2 launch launch/kf.launch.py headless:=true seconds:=25 task:=kf_gps \
        controller:=student/kf_template.py > .runs/kf_launch.log 2>&1
    rc=$?
    if [[ $rc == 0 || $rc == 124 ]]; then
      echo "  ok: kf.launch.py ran for 45 s without crashing (rc $rc)"
    else
      echo "  FAIL: kf.launch.py exited with $rc"; tail -12 .runs/kf_launch.log; fail=1
    fi
  fi
  kill $sim 2>/dev/null
fi

step "Result"
[[ $fail == 0 ]] && echo "ALL CHECKS GREEN" || echo "AT LEAST ONE CHECK FAILED"
exit $fail
