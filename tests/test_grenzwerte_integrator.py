"""Limits that a finished solution can miss are not requirements — this pins the measured ones.

Every number here comes from repeated `./lab grade` runs of the reference solution on this
machine (ROS-stub). The point of the test is the relation "threshold vs measured spread":
if someone tightens a limit below what a correct solution reproducibly achieves, grading becomes
a lottery, and the lab cannot have that.

Since the pace of a run became a switch (`--speed N`, `--fixed-step`), the spread is recorded **per
pace** — three runs each, seed 1, tables in CONTRACT §9.1 and CONTRACT-KF §5.1. Two rules that came
out of measuring it are pinned here too: no limit had to be retuned for any pace that experiment 2
may be graded at, and experiment 2 is *not* gradable at `--fixed-step` (see the last test).
"""
from mecanum_lab import tasks

TASKS = tasks.load_tasks()

# Worst value measured per task/metric/pace. `real time before the switch` is kept on purpose: a
# limit has to stay above the worst value ever seen, not only above the latest three runs.
SPREAD = {
    ("quadrat", "closure"): {"real time": 0.186, "speed 4": 0.191, "fixed-step": 0.175},
    ("korridor", "target_error"): {"real time": 0.185, "speed 4": 0.177, "fixed-step": 0.181},
    ("gps_anfahrt", "target_error"): {"real time before the switch": 0.33, "real time": 0.147,
                                      "speed 4": 0.250, "fixed-step": 0.260},
    ("kf_gps", "rmse"): {"real time": 0.068, "speed 4": 0.145},
    ("kf_fusion", "rmse"): {"real time": 0.506, "speed 4": 0.372},
    ("kf_kovarianz", "rmse"): {"real time": 0.114, "speed 4": 0.117},
    ("kf_kovarianz", "nees"): {"real time": 0.74, "speed 4": 0.46},
    ("kf_dynamik", "rmse"): {"real time": 0.091, "speed 4": 0.225},
}
LIMIT_KEY = {"closure": "closure_max", "target_error": "target_max", "rmse": "rmse_max"}
# How much room a limit needs above the worst measurement to survive another host.
MARGIN = {"closure": 0.05, "target_error": 0.1, "rmse": 0.1}


def threshold(task: str, key: str):
    return [a for a in TASKS["tasks"] if a["id"] == task][0][key]


def test_t4_target_threshold_covers_the_measured_scatter():
    """GPS approach: 0.07 / 0.17 / 0.18 / 0.25 / 0.33 m over paced runs; with the pace as a switch
    0.094…0.147 m at `--speed 1`, 0.158…0.250 m at 4×, 0.087…0.260 m at `--fixed-step`.

    The drive plan ends while the robot is still closing on the spot, so the stopping point
    depends on when the last command is cut off — the spread is timing, not GPS noise (default
    σ_xy for this task is 0.06 m). 0.30 m was inside that spread; the limit is 0.45 m now and
    must stay above the worst measured value plus its margin.
    """
    schlechteste_gemessene = max(SPREAD[("gps_anfahrt", "target_error")].values())
    assert schlechteste_gemessene >= 0.33        # the old paced spread is not quietly forgotten
    assert threshold("gps_anfahrt", "target_max") >= schlechteste_gemessene + 0.1


def test_the_limits_of_both_experiments_stay_above_the_spread_of_every_pace():
    """One assertion per measured task/metric: the worst value ever measured, plus its margin."""
    for (task, metric), per_pace in SPREAD.items():
        worst = max(per_pace.values())
        if metric == "nees":
            lo, hi = threshold(task, "nees")
            assert lo <= min(per_pace.values()) and worst <= hi, (task, per_pace)
            continue
        limit = threshold(task, LIMIT_KEY[metric])
        assert limit >= worst + MARGIN[metric], (task, metric, per_pace)


def test_experiment_2_is_not_gradable_at_fixed_step_and_the_limits_stay_that_way():
    """`rate_hz` counts kf/pose messages per **sim** second, and a node only publishes as fast as it
    loops: 2.6…4.0 Hz at `--fixed-step` (~36× realtime here), 9.1…9.6 at `--speed 6`, 7.0 at 8×.

    Those numbers are below `rate_min` — which is the point: at that pace the check would measure
    the node's CPU share instead of its filter. Lowering `rate_min` to make a fixed-step run pass
    would remove the only requirement that a node reports continuously, so the pace is what gets
    restricted instead. If this test ever goes red, the limit was loosened, not the filter.
    """
    assert threshold("kf_gps", "rate_min") > 4.0
    assert threshold("kf_dynamik", "rate_min") > 4.0


def test_t3_target_threshold_may_stay_sharp():
    """The corridor task measures 0.175…0.185 m at every pace (LIDAR is smooth), so 0.3 m is fair."""
    assert threshold("korridor", "target_max") == 0.3


def test_t4_is_not_passed_by_blind_printing():
    """Loosening the goal radius must not make the task free: contacts and 'not beforehand' stay."""
    task = [a for a in TASKS["tasks"] if a["id"] == "gps_anfahrt"][0]
    assert task["contacts_max"] == 0
    assert task["target"] == "spawn" and task["target_index"] == -1
