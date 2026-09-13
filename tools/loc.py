#!/usr/bin/env python3
"""LOC guard: compares the module sizes with the budget in docs/CONTRACT.md.

For the supervisor only (not part of the student lab):
    python3 tools/loc.py            # table + deviations
    python3 tools/loc.py --strict   # exit 1 when a budget is exceeded
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# File -> budget (CONTRACT §7). Comments and blank lines count too — that is the
# size a student sees while reading.
# Experiment 1 (CONTRACT §7) plus experiment 2 (CONTRACT-KF §6): the budgets of the
# second experiment are justified because the IMU model, KF grading, config layers and
# the measurement log are new constraints — not because modules may be padded out.
# What students have to read (robot_io.py, topics in types.py) stays small.
#
# Addendum, integration of experiment 2 (justified per CONTRACT §7, numbers measured):
#   engine.py  300 -> 340   one robot is dropped at the spawn pose per KF task
#                           (reset_robot), sensor clocks follow the simulation time
#                           (_clock_to_sim), a task with its own GPS outage (set_task),
#                           config_json/robots_info for topics and the measurement log.
#   node.py    465 -> 520   arena from the tasks (cfg_get_world), first_task for
#                           /sim/task, the robot list published continuously (run_loop),
#                           and the settling kf/pose in the measurement log. That is
#                           pacing, which was missing before — experiment 1 was silently off.
#   grade.py   610 -> 620   KF step plan, blind command run, RMSE/NEES/GPS outage.
#                           The alternative (a second file kf_grade.py) only moves the
#                           lines and doubles the sampling path.
#   student/kf_template.py 160 -> 190   the adapter code now also explains why a fix
#                           from the previous task has to be discarded — a trap that
#                           would make every other run stumble.
#   student/kf_template.py 190 -> 200   the same trap twice more: a task must not start
#   student/kf_solution.py 295 -> 310   on the stamps the bus still remembers, and a node
#                           that slept must not extrapolate five seconds of CV model.
#                           Both showed up as one KF task collapsing (rmse 3.5 m, NEES 332)
#                           in a run that was fine the time before.
#   tools/kfplot.py        190 -> 250   ASCII diagram with axis labels and the
#                           diagnostic lines; replaces matplotlib, which is not available.
#
# Addendum, GPS shadow zones + the overlays that make them visible (numbers measured):
#   sensors.py 295 -> 330    gps.zones in GpsSensor.fix (place-based sigma, multipath bias,
#                            blackout) and _zones(), which drops a broken demo entry instead
#                            of killing a lab run. Empty by default, so every graded task
#                            measures exactly what it measured before this existed.
#   overlays.py new, 140    three effects that belong to no existing file: hatched GPS
#                            shadow, the odometry ghost with the drift in metres, rubber
#                            from wheel slip. A new file rather than 135 more lines inside
#                            render.py — that one builds a frame, this is a kit drawn on top.
#   overlays.py 140 -> 150   a label has to clear the readout block (one line per robot).
#   node.py 540 -> 550     `teleop` becomes the command that CONTRACT §6.9 and the handout
#                          already promise (one robot + keyboard), instead of the handout
#                          line ending in "unrecognized arguments".
#   CORE_TOTAL 4300 -> 4450 sensors +35 and overlays +150; nothing else grew.
#
# Addendum, the English sweep (CONTRACT §1: identifiers are English too; numbers measured):
#   tasks.py 170 -> 210     _LEGACY_KEYS is 46 lines of old key -> new key, and it has to be
#                           complete to be any good: a task file printed before the sweep must
#                           still load. Plus the one warning per key that is found.
#   launch/kf.launch.py 160 -> 185   declares the arguments it used to forward blindly (47 of
#                           them) and resolves the deprecated names of CONTRACT §6.11. The
#                           argument table itself used to live only in the printed handout, and
#                           the file died on the first launch without arguments because of it.
#   tools/fastgrade.py 140 -> 145    keeps --wanduhr-max as an alias for --wallclock-max.
#   CORE_TOTAL 4450 -> 4500  only tasks.py grew in the core; every other module is the same size
#                           it was, because a rename does not add lines (the diff was symmetric).
# Addendum, grading that does not depend on host load + four dead knobs (numbers measured):
#   types.py   345 -> 365     the `odom.geometry` block with one line of physical meaning per term
#                             (what error a wrong wheel radius or lever arm produces),
#                             `gps.delay_ticks`, and `_merge`'s docstring: `None` means "nothing
#                             overridden", which is what the CLI sends for every option that was
#                             not given on the command line.
#   sensors.py 330 -> 400     `odom_geometry()` plus the model-error terms of `OdometrySensor` — the
#                             odometry integrator finally believes its own wheel constants, so the
#                             commonest real odometry error becomes expressible and demonstrable
#                             (config/demo_odom_error.json) — the GPS delay ring buffer, and the
#                             `lidar.max_walls` cap. Most of the growth is what each term does to a
#                             drive; without that the code is a list of multipliers.
#   engine.py  340 -> 395     `_make_odometer`/`_make_gps` (believed geometry, `kf.gps_delay` turned
#                             into ticks), `dropped` + `_warn_drop` on the 0.5 s accumulator,
#                             `sub_step` for a fixed-step loop, `_push(..., stamp=)` so a delayed fix
#                             keeps the stamp it was measured with.
#   node.py    550 -> 615     `--speed N` / `--fixed-step` and the `run_loop` pacing that goes with
#                             them (`MAX_LOOP_DT`, with the remainder counted and warned about once),
#                             `pacing()` (fixed-step switches the sleeps off for the node too) and
#                             `wants_gui()` — `cfg["gui"]` was a key nobody read, so a config file
#                             could not close the window.
#   CORE_TOTAL 4500 -> 4750  sensors +70, engine +55, node +65, types +20; grade.py, physics.py,
#                            render.py and robot_io.py did not grow — the grader already ticked on
#                            simulation time and the physics step size stayed where it was.
# Addendum, sensors that say how good the measurement is (numbers measured, `git diff --stat`):
#   sensors.py 400 -> 560    the second half of what a sensor delivers is its own state: GPS quality
#                            and satellite count (`sky()`, +35 lines incl. what each zone costs),
#                            dropout and a latency queue (+30), LIDAR reflectivity, which needs the
#                            face a ray entered and so turns the slab test's answer into a pair
#                            (+35), and the IMU's temperature with the bias that follows it (+30).
#                            `Noise.chance/late` (+12) are the two helpers that keep all four off
#                            by default *without drawing a random number*, which is what keeps the
#                            graded streams byte-identical (measured, tests/test_sensor_reality.py).
#   overlays.py 150 -> 190   `sensor_readout()`: the gps/lidar/imu part of the readout line, with
#                            quality, sats, °C and the count of lost messages. Drawing that belongs
#                            to the overlay kit, not to the frame builder — and it made render.py
#                            shorter (469 -> 466), because the two segments it replaces lived there.
#   types.py   365 -> 395    the four new message fields with their meaning (the difference between
#                            "no fix" and "no radio" is a docstring, not a code path) and one comment
#                            line per new config key.
#   engine.py  395 -> 415    `gps_health()` for the window and the log, the stamp-only odom jitter,
#                            the robot name passed to the receiver so losses are counted per robot.
#   logbook.py 100 -> 110    five columns: q_gps, sats_gps, lost_gps, temp_imu, noecho_scan. The
#                            CSV is where a student proves what a sensor did, so a field that is not
#                            in it might as well not have happened.
#   CORE_TOTAL 4750 -> 5000  sensors +151, overlays +37, types +27, engine +17, logbook +4, render
#                            -3. grade.py, physics.py, node.py and robot_io.py did not grow at all:
#                            nothing about a graded measurement or about the CLI changed.
#
# Addendum, the second drive train `--variant steering` (numbers measured, before -> after):
#   steering.py new, 290   a whole drive train next to the mecanum one instead of 290 lines inside
#                          physics.py: the bicycle geometry with the config's degrees turned into
#                          radians, the rate-limited rack, the Ackermann pair, the four rolling wheel
#                          speeds, and the integrator that replays the angle the rack was *told* to
#                          take. Whoever reads the mecanum equations of CONTRACT §5 should not have
#                          to skip a car to find them.
#   student/steering_example.py new, 214   one lap of a rounded rectangle, then one parking spot by
#                          LIDAR — in the readable-in-five-minutes size of controller_template.py
#                          (123), not in solution.py's (304): it is an example, not a reference.
#   physics.py 139 -> 180  the hooks a subclass needs (geom, wheel_headings, reset_motion,
#                          set_twist, odo_feed) and one sentence each on what they are for. Every one
#                          of them is an alias or a no-op for a mecanum robot; that is the claim
#                          tests/test_steering_w4.py checks by driving both cars in one engine.
#   types.py   392 -> 420  the `steering` config block with one line of meaning per number (what a
#                          metre of wheel base does, what 32 degrees of rack does), the `steering-big`
#                          variant table and `odom.steer_max_scale`.
#   engine.py  414 -> 435   one branch each in spawn() and _make_odometer(), cmd_vel handed to
#                          chassis.set_twist() instead of solved into four wheel speeds at the call
#                          site, and the two additions to /sim/robots and /sim/config (steer_deg, the
#                          steering block) that let a node ask which car it is driving.
#   overlays.py 187 -> 213  steer_readout(): the two rack angles and the radius they make — text, not
#                          drawing, same rule as sensor_readout(). It is also why render.py grew 6
#                          lines instead of 20: the view now turns each wheel by
#                          chassis.wheel_headings and appends the segment, both in one line each.
#   robot_io.py 244 -> 259  the drive(rob) branch of serve() for a node that waits for no task, with
#                          the cmd_vel pass-through switched off after it (CONTRACT §6.8).
#   node.py    612 -> 613   --variant was parsed, printed in the help and never passed to spawn().
#   CORE_TOTAL 5000 -> 5400  steering.py's 290 plus the seams: physics +41, types +28, engine +21,
#                          overlays +26, robot_io +15, render +6, node +1. sensors.py, grade.py,
#                          tasks.py, logbook.py and both reference solutions did not change at all —
#                          which is the same fact the unchanged 100/100 and 90/90 of check.sh show
#                          from the other side.
# Addendum, the empty hall `open` and the radiation sources of `/poi` (numbers measured):
#   pois.py new, 170 (measured 164)   a second field sensor next to the four position ones: the
#                          fall-off, the validation that refuses a source in a wall instead of
#                          dropping it (a scenario that cannot be solved is worse than a crash), and
#                          the Poisson counter that makes a weak reading a rough one. Not inside
#                          sensors.py: that file is 556 lines about measuring a *position*, and a
#                          source that is measured through a wall has nothing in common with that.
#   types.py 420 -> 460  the Poi message with the reason `distance` is not in it by default, one
#                          config comment per new key (why `pois` is empty and what `poi.counts` is
#                          worth), the /poi topic and its ROS mapping.
#   engine.py 435 -> 485  building the detector from the world and the config (None when nothing is
#                          planted — the reason the graded streams did not move), one rate-limited
#                          publish block, and `poi_sources()` for the window with the sentence saying
#                          why there is no topic for it.
#   overlays.py 213 -> 290 the source symbol, the field rings of `debug_truth` and the intensity in
#                          the readout line. Most of it is why a source is drawn as a source and not
#                          as an obstacle — the disagreement with the LIDAR is the teaching point.
#   render.py 472 -> 480  three lines: the `p` key, its boolean, the call. New drawing goes into
#                          overlays.py, that is what that file is for.
#   tools/worldpic.py 240 -> 245  five panels instead of four (three columns) and the caption of a
#                          hall without a goal, which quotes the widest free spot the checker measured
#                          instead of a passage it cannot have.
#   menu.py and ros_bridge.py keep their budgets (90, 490): the menu row and the /poi mapping cost
#                          +1 and +10 lines, both still inside the number that was already there.
#   CORE_TOTAL 5400 -> 5750  measured 5718 (5385 before): pois.py 164 new, overlays +71, engine +46,
#                          types +38, render +3, ros_bridge +10, menu +1. sensors.py, physics.py,
#                          grade.py, node.py, logbook.py, tasks.py, both reference solutions and
#                          config/tasks.json did not grow by one line — no graded number, no sensor of
#                          experiment 1 and no wheel equation is involved, and the 100/100, the 90/90
#                          and the 30/30 of check.sh are the same numbers they were before.
# Addendum, the radio link `wifi.enabled` (numbers measured, `git diff --stat`):
#   wifi.py new, 365 (measured 363)  one access point, the link budget, the delivery rule and the
#                          onboard rule. Not in sensors.py: those 560 lines measure the floor with
#                          rays and fixes, this measures the wire to one address and its output is a
#                          decision about a command frame. Not in engine.py either: engine says what
#                          a robot does with a command, wifi says whether one arrives at all — that
#                          seam is `_deliver()`, and it is one `if` in one file because the two
#                          halves are in two files. Wall crossings reuse `sensors._ray_rect`, so a
#                          LIDAR beam and a radio wave cannot disagree about which walls exist.
#   engine.py 485 -> 620 (measured 618, +137)  the seam itself (`_deliver`/`_apply_cmd`/`_wire`),
#                          `_step_link` (timer, autonomy flip, rate-limited /link), `_hold` for both
#                          onboard spellings, `link_health()` for the window, the radio history
#                          dropped on reset/despawn/set_sensor_profile, `effective_ap` on
#                          /sim/config. Most of it is the sentence saying that with the option off
#                          this is the old assignment and nothing else — which tests/test_wifi_w6.py
#                          checks by comparing two recorded CSVs byte for byte.
#   types.py 460 -> 520 (measured 514, +56)  the Link message (with why `ap` is in a message about
#                          quality), the /link topic and its ROS mapping, the fifteen wifi keys with
#                          one line of meaning per number — what a path-loss exponent of 2.4 is,
#                          what one crossing at 12 dB costs — because a knob without that is a dial
#                          with no label.
#   overlays.py 290 -> 440 (measured 432, +148)  the network layer: access point symbol, the line to
#                          the robot drawn on the very segment the budget is computed on, the quality
#                          bar, `link_text()` with the five numbers, `autonomy_mark()` (outside the
#                          layer, because a layer can be hidden and the failsafe cannot). Same
#                          argument as the two readout additions before it: overlays.py is the kit,
#                          render.py builds a frame, and render.py grew by 6 lines instead of 150 for
#                          that reason.
#   render.py 480 -> 485 (+6)  the `n` key, its boolean, the two calls, the readout segment. Same
#                          three-line rule as `p` and `s`.
#   ros_bridge.py 490 -> 500 (+14)  /link out and in, JSON on a String like /poi: a custom interface
#                          would put a colcon build in front of a student who only wants to read a
#                          quality.
#   robot_io.py 259 -> 275 (+14)  `link()` for the student side, plus the topic line in the table: a
#                          feature a controller cannot observe from inside is not an experiment.
#   student/link_autonomy_example.py new, 155 (measured 151)  the drive that walks out of coverage
#                          and says where the link died, in the readable size of steering_example
#                          (214) rather than solution.py (304): it is an example, not a reference.
#   launch/wifi.launch.py new, 130 (measured 129)  the same fifteen settings as `--set` calls, one
#                          table, both processes. Declares every argument it forwards (W1's lesson).
#   CORE_TOTAL 5750 -> 6500  measured 6458 (5718 before W6): wifi.py 363 new, overlays +148, engine
#                          +137, types +56, ros_bridge +14, robot_io +14, render +6, node +1 (the two
#                          printed topic lists), menu +1 (the row). sensors.py, physics.py, grade.py,
#                          tasks.py, logbook.py, steering.py, pois.py and both reference solutions did
#                          not grow by one line — no sensor of the floor, no grading threshold and no
#                          wheel equation is involved, and the 100/100, the 90/90 and the 30/30 of
#                          check.sh are the same numbers they were before. With `wifi.enabled` false
#                          — the shipped default — not even a random number is drawn for the radio,
#                          which is how that stayed true.
# Addendum, the fix for `./lab run --controller` never having driven a robot (found by W6, measured):
#   node.py 615 -> 640 (measured 634, +20)  `subscribe_all()`, the one call every command path needs:
#                          `cmd_run()` started the student node and ran the loop but never subscribed
#                          the topics, so every frame the node published sat on the bus while the
#                          robot stood at its spawn pose for the whole run and /gps fixed away
#                          merrily beside it. `cmd_sim()` and `cmd_grade()` had the call;
#                          `cmd_controller`, `cmd_teleop` and `cmd_client` have no engine to wire.
#                          Plus the teleop keys recording that they were the last publisher — they
#                          publish on /cmd_vel like any node does, only while a key is held.
#   engine.py 620 -> 630 (+11)  `note_keys()`: who was last, a fact the window shows and no part of
#                          the drive depends on.
#   overlays.py 440 -> 460 (+23)  `command_readout()`: `cmd topic 0.04 s` / `cmd keys 0.02 s` /
#                          `cmd none`. A node and the keyboard share one topic, so the honest answer
#                          to "what is driving this robot" is a timestamp — and it belongs on screen.
#   render.py +1, types.py +1   the segment in the readout line; the stamp next to t_vel.
#   CORE_TOTAL 6500 -> 6550  measured 6514. The regression test is
#                          test_a_node_under_lab_run_reaches_the_robot, which enters through
#                          cmd_run() itself: every harness that called subscribe() on its own — as
#                          two tests in this repository do — is exactly the test that could not
#                          catch this, which is how one missing line survived four command paths.
# The names of the fields students read (report, CSV) cost nothing here: they are strings.
#
# Addendum, W7: papercuts, reproducible pictures, measured-vs-required grading, `/sensor/info`
# (numbers measured with this table, before -> after):
#   node.py 640 -> 755 (measured 743)  `--frame-max`/`--screenshot` and `save_picture()`, which is what
#                          makes a documentation figure regenerable instead of re-screenshotable; the
#                          `time_limit()` line that prints which end-of-run rule is in force; the dead
#                          `./lab -h` branch (COMMANDS is now name -> (fn, help), so the help lists all
#                          eleven commands); and the setup that `cmd_run`/`cmd_sim`/`cmd_grade` used to
#                          each write themselves — `spawn_player`, `student_nodes`, `timed_run` — which
#                          is where the missing `subscribe_all()` of W6 had hidden. Most of the growth
#                          is the sentence saying what each option decides, not new behaviour.
#   grade.py 620 -> 675 (measured 669)  MISSION_CRITERIA + `_apply()`: the eight hand-written limit `if`
#                          blocks of one mission became one table, and the same rows are what the report
#                          prints, so the limit a student reads is the limit that was applied (a 0/90
#                          run that names neither number is what the K3 and T4 questions came from).
#                          Also the phase reason that was a bare string inside `"; ".join(...)`.
#   types.py 520 -> 565 (measured 560)  `SensorInfo`, the `/sensor/info` entry of MSG_SPECS, and the
#                          two docstrings that had drifted from the code: `Gps.quality` claimed a
#                          receiver could answer with quality 0 (no code path ever did — quality 0 is a
#                          property of a *place*, `GpsSensor.sky()`), and `Scan.range_max` now says why
#                          it is spelled like `LaserScan.range_max` and not like `max_range`.
#   engine.py 630 -> 650 (measured 644)  one `_build_sensors()` for startup and for a profile switch
#                          (the two copies were how a switch kept an old instrument), `reset()` through
#                          `reset_robot()`, and `sensor_info()` for the message above.
#   overlays.py 460 -> 490 (measured 481)  `sigma_legend()`: the σ ellipse in metres and the same
#                          half-axis in pixels at the current zoom, in the readout line rather than in
#                          the estimate layer, because `k` hides a drawing and not a number.
#   ros_bridge.py 500 -> 515, robot_io.py 275 -> 290, logbook.py 110 -> 118, tasks.py 210 -> 215
#                          the JSON mapping of `/sensor/info`, the `poi()` accessor and its topic row,
#                          the `intensity_poi`/`name_poi` columns (a counter you can only watch during
#                          the run cannot be plotted afterwards), and one legacy task key less.
#   student/kf_template.py 200 -> 235 (measured 231)  the docstring rewritten to say what the template
#                          does and does not do: it grades as an empty estimate (rmse 6.84, "no
#                          standard deviations in kf/pose"), which is what its old header denied.
#   launch/kf.launch.py 185 -> 195, launch/lab.launch.py stays at 60   the 47 arguments kf.launch.py
#                          used to forward undeclared, each with its `deprecated, use 'X'` line, and
#                          lab.launch.py's arguments as one `BASICS` table so that all nine carry help
#                          inside the 60-line limit `tests/test_package_f.py` holds it to.
#   launch/lab.launch.py 60 -> 100 (measured 124)   the same test caps that file on *code* (85, of
#                          which 77 are used) precisely because prose is not what a launch file must
#                          not grow; this metric counts what it measures, and the file now explains
#                          the four rules it exists for: the typed arguments, the graded run, the
#                          hall that comes from the task, and where a relative `controller:=` points.
#   tools/kfplot.py 250 -> 310 (measured 305)  `sensor_state()`: q_gps, lost_gps, temp_imu and
#                          intensity_poi as four sparklines, each on its own scale (a shared axis would
#                          show the temperature and hide the quality).
#   student/poi_seek_example.py new, 180 (measured 176)  the climb to the radiation source; ~45 of its
#                          lines are the four reasons the search is shaped the way it is, measured
#                          against the three versions that did not work.
#   tools/launchargs.py new, 195 (measured 192)  the guard for the above: every argument a launch file
#                          reads has to be declared and has to have one line of help, checked from the
#                          AST of all five files (112 arguments, 5 files) because a table that only
#                          exists in a printed handout is how `--show-args` becomes a lie.
#   CORE_TOTAL 6550 -> 6850  measured 6807: node +103, grade +49, types +40, overlays +21, engine +14,
#                          ros_bridge +10, robot_io +11, logbook +5, tasks +1. physics.py, sensors.py,
#                          worlds.py, wifi.py, pois.py, steering.py, cam.py, menu.py, tf_bcast.py,
#                          render.py (+1, inside its 495) and both reference solutions did not grow by
#                          one line — no wheel equation, no sensor model and no graded threshold of
#                          either experiment changed, and the 100/100 twice, the 90/90 and the 30/30 of
#                          check.sh are the same numbers they were before this package.
BUDGET = {
    "mecanum_lab/types.py": 565, "mecanum_lab/stub.py": 115,
    "mecanum_lab/engine.py": 650, "mecanum_lab/worlds.py": 135,
    "mecanum_lab/physics.py": 180, "mecanum_lab/sensors.py": 560,
    "mecanum_lab/steering.py": 290, "mecanum_lab/pois.py": 170,
    "mecanum_lab/wifi.py": 365,
    "mecanum_lab/overlays.py": 490,
    "mecanum_lab/render.py": 485, "mecanum_lab/cam.py": 115, "mecanum_lab/menu.py": 90,
    "mecanum_lab/ros_bridge.py": 515, "mecanum_lab/tf_bcast.py": 135,
    "mecanum_lab/node.py": 755, "mecanum_lab/robot_io.py": 290,
    "mecanum_lab/tasks.py": 215, "mecanum_lab/grade.py": 675,
    "mecanum_lab/logbook.py": 118,
    "student/controller_template.py": 125, "student/solution.py": 310,
    "student/kf_template.py": 235, "student/kf_solution.py": 310,
    "student/steering_example.py": 220,
    "student/link_autonomy_example.py": 155,
    "student/poi_seek_example.py": 180,
    "lab": 65, "launch/sim.launch.py": 60, "launch/student.launch.py": 50,
    "launch/lab.launch.py": 100, "launch/kf.launch.py": 195,
    "launch/wifi.launch.py": 130,
    "tools/kfplot.py": 310, "tools/fastgrade.py": 145, "tools/worldpic.py": 245,
    "tools/launchargs.py": 195,
}
SIM_CORE = [k for k in BUDGET if k.startswith("mecanum_lab/")]
CORE_TOTAL = 6850


def loc(path):
    full = os.path.join(ROOT, path)
    if not os.path.exists(full):
        return None
    with open(full, encoding="utf-8", errors="replace") as fh:
        lines = fh.read().splitlines()
    code = [l for l in lines if l.strip() and not l.strip().startswith("#")]
    return len(lines), len(code)


def main():
    strict = "--strict" in sys.argv
    over, core = [], 0
    print(f"{'File':34} {'Lines':>7} {'Code':>6} {'Budget':>7}  Status")
    for path, budget in BUDGET.items():
        n = loc(path)
        if n is None:
            print(f"{path:34} {'—':>7} {'—':>6} {budget:>7}  missing")
            continue
        total, code = n
        if path in SIM_CORE:
            core += total
        flag = "ok" if total <= budget else f"+{total - budget} ({100 * total / budget:.0f} %)"
        if total > budget:
            over.append((path, total, budget))
        print(f"{path:34} {total:>7} {code:>6} {budget:>7}  {flag}")
    rest = sum(loc(p)[0] for p in BUDGET if p not in SIM_CORE and loc(p))
    print(f"\nSimulator core (mecanum_lab/): {core} lines (budget {CORE_TOTAL})")
    print(f"Rest (CLI, launch, student code): {rest} lines")
    print(f"Total: {core + rest} lines")
    if over:
        print("\nover budget: " + ", ".join(f"{p} ({t}/{b})" for p, t, b in over))
    return 1 if (strict and (over or core > CORE_TOTAL)) else 0


if __name__ == "__main__":
    sys.exit(main())
