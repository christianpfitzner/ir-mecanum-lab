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
# Plugin autoload is off, and not because of these tests: with ROS sourced, `launch_testing` advertises
# a pytest plugin whose hook no longer matches the hook spec of the pluggy that pip put in ~/.local, and
# the run dies in a PluginValidationError before a single test has been collected. A gate that depends
# on which plugins the machine happens to advertise is not a gate; `-p no:launch_testing` would fix
# the plugin of today and leaves the next one.
run "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q"
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
# One command per code block: the pages are read at a machine with one hand on a robot, and a block with
# two commands in it is a block where a reader types one of them and gets something else than the sentence
# above it promised. Same reason the pages lead with `ros2 launch`: it is what the lab room types.
run "python3 tools/docblocks.py"
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
step "How-it-works figure for the documentation (modules -> docs/img/howitworks.png)"
# Also generated, and it checks its own subject: the tool asks the kinematics whether every roller axis
# it is about to draw stands perpendicular to the velocity that wheel produces, and refuses the figure
# if not. tests/test_labmap_docs.py compares the file in the repository against a fresh run.
run "python3 tools/labmap.py --out /tmp/howitworks_check.png"
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
  # Both covariance topics, and not as decoration: a message field of a fixed length is filled by the
  # bridge, and the rclpy of a recent ROS copies a list of the wrong length into it without counting
  # the entries. Humble's does not — its setter raises inside the publisher and the simulator dies at
  # its first IMU message. Only the machine that has to run this in the lab room (the students'
  # humble install) sees that, so every topic with a covariance in it is echoed here.
  run "timeout 25 ros2 topic echo /ros_test/imu --once"
  run "timeout 25 ros2 topic echo /ros_test/gps_cov --once | grep -q covariance"
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
  step "ROS 2: adding and removing a robot over the wire"
  # The round trip is demonstrated with the service type every ROS 2 install has. `/sim/spawn_robot`
  # needs the own interface, which is optional and here not built: a call that works only on a machine
  # that compiled an extra package is a maybe, and whoever presses the button does not know which
  # machine that is. A Trigger has no input and one string out: the check reads the sentence.
  mkdir -p .runs
  live=.runs/spawn_live.log
  timeout 40 ros2 launch launch/lab.launch.py headless:=true seconds:=35 > "$live" 2>&1 &
  sleep 10
  call=.runs/spawn_call.log
  if timeout 20 ros2 service call /sim/spawn_next std_srvs/srv/Trigger > "$call" 2>&1 \
     && grep -q "spawned 'robot1'" "$call"; then
    echo "  ok: $(grep -o "spawned '[^']*'[^\"]*" "$call" | head -1)"
  else
    echo "  FAIL: /sim/spawn_next did not answer as CONTRACT 4 promises"
    tail -6 "$call"; fail=1
  fi
  gone=.runs/despawn_call.log
  # The name taken from the answer and not from what was hoped for: two simulators on one machine answer
  # the same service name, and whichever replied to the spawn has to be the one that takes it away.
  born=$(grep -o "spawned '[^']*'" "$call" | head -1 | cut -d"'" -f2)
  if timeout 20 ros2 service call /sim/despawn_last std_srvs/srv/Trigger > "$gone" 2>&1 \
     && grep -q "'$born' removed" "$gone"; then
    echo "  ok: $(grep -o "'[^']*' removed" "$gone" | head -1)"
  else
    echo "  FAIL: /sim/despawn_last did not answer"; tail -6 "$gone"; fail=1
  fi
  wait || true
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
  step "ROS 2: a demo through launch/demo.launch.py (the include chain, and rviz:=auto)"
  # `demo:=gps_shadow` is the first launch line a student copies, and it goes through two files: the demo
  # file includes lab.launch.py, which starts the processes. A leftover import in the outer file failed
  # only *here* — under pytest the repository is already on sys.path, so every unit test passed while
  # `ros2 launch` raised ModuleNotFoundError. rviz2 is absent on this machine, so the same run also has
  # to print the one honest line about it and still end without a crashed process.
  timeout 45 ros2 launch launch/demo.launch.py demo:=gps_shadow headless:=true seconds:=12 \
      > .runs/demo_launch.log 2>&1
  rc=$?
  if [[ $rc == 0 || $rc == 124 ]]; then
    if grep -q "ModuleNotFoundError\|Traceback" .runs/demo_launch.log; then
      echo "  FAIL: demo.launch.py raised"; tail -12 .runs/demo_launch.log; fail=1
    elif ! grep -q "\[demo\] gps_shadow:" .runs/demo_launch.log; then
      # The line the launcher prints, not merely the word "demo": a launch file that starts *a* sim but
      # not the one it was named for is the failure a student notices as "my demo did nothing".
      echo "  FAIL: the run does not say which demo it started"; tail -8 .runs/demo_launch.log; fail=1
    else
      echo "  ok: demo.launch.py starts the demo it names, with the rviz rule applied"
    fi
  else
    echo "  FAIL: demo.launch.py exited with $rc"; tail -12 .runs/demo_launch.log; fail=1
  fi
  step "ROS 2: each of the six demo launchers, by its own name"
  # `demo.launch.py demo:=x` and `demo_x.launch.py` reach the same engine through different files, and the
  # thin one has to find `mecanum_lab` *and* its config on its own — which is what broke after a
  # `colcon build`, when the data stopped sitting next to the module. Every name is taken from the same
  # glob the launcher uses, so a seventh config is launched the day it appears.
  for demo in $(python3 -c "from mecanum_lab import demo_launch; print(' '.join(demo_launch.demos()))"); do
    timeout 30 ros2 launch "launch/demo_${demo}.launch.py" headless:=true seconds:=6 \
        > ".runs/demo_${demo}.log" 2>&1
    rc=$?
    if [[ $rc != 0 && $rc != 124 ]] ||
       grep -q "ModuleNotFoundError\|FileNotFoundError\|Traceback" ".runs/demo_${demo}.log"; then
      echo "  FAIL: demo_${demo}.launch.py"; tail -8 ".runs/demo_${demo}.log"; fail=1
    else
      echo "  ok: demo_${demo}.launch.py"
    fi
  done
  if [[ -f install/setup.bash ]]; then
    step "ROS 2: the installed package name, which is what install.sh promises"
    # From the built package, not from this tree: the file then lives in share/, and a launch file that
    # resolves its config relative to its own source path works here and fails there.
    ( set +u
      source install/setup.bash >/dev/null
      timeout 30 ros2 launch mecanum_lab demo_wifi.launch.py headless:=true seconds:=6 \
          > .runs/demo_installed.log 2>&1 )
    if grep -q "ModuleNotFoundError\|FileNotFoundError\|Traceback" .runs/demo_installed.log; then
      echo "  FAIL: ros2 launch mecanum_lab demo_wifi.launch.py"; tail -10 .runs/demo_installed.log; fail=1
    else
      echo "  ok: the same launcher runs from the built package under its package name"
    fi
  else
    echo "  skipped: no install/ here — ./install.sh builds the ROS package this step would check"
  fi
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
