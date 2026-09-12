#!/usr/bin/env python3
"""LOC guard: compares the module sizes with the budget in docs/CONTRACT.md.

For the supervisor only (not part of the student lab):
    python3 tools/loc.py            # table + deviations
    python3 tools/loc.py --strict   # exit 1 when a budget is exceeded
"""
import os
import subprocess
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
#                           (_zeitbezug), a task with its own GPS outage (set_task),
#                           config_json/robots_info for topics and the measurement log.
#   node.py    465 -> 520   arena from the tasks (cfg_get_welt), erster_auftrag for
#                           /sim/task, the robot list published continuously (simlauf),
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
BUDGET = {
    "mecanum_lab/types.py": 345, "mecanum_lab/stub.py": 115,
    "mecanum_lab/engine.py": 340, "mecanum_lab/worlds.py": 135,
    "mecanum_lab/physics.py": 140, "mecanum_lab/sensors.py": 295,
    "mecanum_lab/render.py": 455, "mecanum_lab/cam.py": 115, "mecanum_lab/menu.py": 90,
    "mecanum_lab/ros_bridge.py": 490, "mecanum_lab/tf_bcast.py": 135,
    "mecanum_lab/node.py": 540, "mecanum_lab/robot_io.py": 255,
    "mecanum_lab/tasks.py": 170, "mecanum_lab/grade.py": 620,
    "mecanum_lab/logbook.py": 100,
    "student/controller_template.py": 125, "student/solution.py": 310,
    "student/kf_template.py": 200, "student/kf_solution.py": 310,
    "lab": 65, "launch/sim.launch.py": 60, "launch/student.launch.py": 50,
    "launch/lab.launch.py": 60, "launch/kf.launch.py": 160,
    "tools/kfplot.py": 250, "tools/fastgrade.py": 140,
}
SIM_CORE = [k for k in BUDGET if k.startswith("mecanum_lab/")]
CORE_TOTAL = 4300


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
