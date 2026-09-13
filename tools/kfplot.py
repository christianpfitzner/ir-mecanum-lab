#!/usr/bin/env python3
"""Evaluate the measurement log from `./lab ... --log FILE.csv` — numbers and diagram.

Without matplotlib you still get the numbers for the lab report and an ASCII plot of the
error time series; with matplotlib (optional, not installed) you can also write a PNG.
Columns and their meaning are listed in `mecanum_lab/logbook.py` (semicolons, decimal point).

    python3 tools/kfplot.py messung.csv                       # numbers + ASCII plot
    python3 tools/kfplot.py messung.csv --sensor odom         # compare a raw sensor
    python3 tools/kfplot.py messung.csv --interval 1         # 1 s averages in the plot
    python3 tools/kfplot.py messung.csv --list                # show the columns
    python3 tools/kfplot.py messung.csv -o bild.png           # PNG, if matplotlib exists
"""
import argparse
import csv
import math
import sys

ROHSENSOR = {"gps": ("x_gps", "y_gps"), "odom": ("x_odom", "y_odom")}


def num(value):
    """Read one CSV cell as a number — empty cells and 'nan' give None, never a crash."""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def lese(path):
    try:
        with open(path, newline="", encoding="utf-8", errors="replace") as fh:
            rows = list(csv.DictReader(fh, delimiter=";"))
    except OSError as exc:
        sys.exit(f"file '{path}' not readable: {exc}")
    if not rows or "t" not in (rows[0] or {}):
        sys.exit(f"'{path}' is not a measurement log — expected a semicolon CSV with "
                 f"header 't;robot;x_truth;...' (see ./lab --log FILE.csv).")
    return rows


def by_robot(rows):
    groups = {}
    for r in rows:
        groups.setdefault(r.get("robot") or "?", []).append(r)
    for lines in groups.values():
        lines.sort(key=lambda r: num(r.get("t")) or 0.0)
    return groups


def accuracy(rows, sensor):
    """Error over time: (t, error) per row, NEES samples, and the GPS fix stamps."""
    px, py = ROHSENSOR[sensor]
    kf, raw, nees, fixes = [], [], [], []
    for r in rows:
        t, xw, yw = num(r.get("t")), num(r.get("x_truth")), num(r.get("y_truth"))
        if t is None or xw is None or yw is None:
            continue                                        # without truth there is nothing to measure
        xa, ya = num(r.get(px)), num(r.get(py))
        if xa is not None:
            raw.append((t, math.hypot(xa - xw, ya - yw)))
        xk, yk = num(r.get("x_kf")), num(r.get("y_kf"))
        if xk is not None:
            kf.append((t, math.hypot(xk - xw, yk - yw)))
            s2 = [num(r.get(s)) for s in ("sx_kf", "sy_kf")]
            if all(s and s > 1e-9 for s in s2):
                nees.append((((xk - xw) ** 2) / s2[0] ** 2 + ((yk - yw) ** 2) / s2[1] ** 2) / 2)
        alt = num(r.get("t_age_gps"))
        if alt is not None:
            fixes.append(round(t - alt, 2))                 # fix stamp recomputed from its age
    return kf, raw, nees, fixes


def fix_gaps(fixes):
    """Gap between two different GPS fixes in s — from the stamps, not the message rate.

    The file holds no fix stamp, only its age in `t_age_gps`; converting it back leaves
    rounding wrinkles in the decimals that look like new fixes. So smooth over 2 cm of
    time and drop everything below 20 ms as jitter.
    """
    previous, gaps = None, []
    for t in sorted(set(fixes)):
        if previous is not None and t - previous > 0.02:
            gaps.append(t - previous)
        previous = t
    return gaps


def center(points, interval):
    """Mean per interval — averaging away the noise makes a time series far easier to read."""
    if not points or not interval or interval <= 0:
        return points
    karten = {}
    for t, e in points:
        karten.setdefault(int(t / interval), []).append((t, e))
    out = []
    for key in sorted(karten):
        group = karten[key]
        out.append((sum(t for t, _ in group) / len(group),
                    sum(e for _, e in group) / len(group)))
    return out


def rmse(points):
    return math.sqrt(sum(e * e for _, e in points) / len(points)) if points else None


def ascii_chart(curves, width=76, height=13):
    """Error time series as text: curves = [(name, char, points), ...] — width <= 100."""
    curves = [(n, z, p) for n, z, p in curves if p]
    if not curves:
        return "  (no error data)"
    tmin = min(t for _, _, p in curves for t, _ in p)
    tmax = max(t for _, _, p in curves for t, _ in p)
    ymax = max(max(e for _, e in p) for _, _, p in curves) or 1.0
    raster = [[" "] * width for _ in range(height)]
    for _, zeichen, points in curves:
        for t, e in points:
            col = int(round((t - tmin) / (tmax - tmin) * (width - 1))) if tmax > tmin else 0
            row_rect = int(round((ymax - e) / ymax * (height - 1)))
            raster[max(0, min(height - 1, row_rect))][col] = zeichen
    legende = "   ".join(f"{z} = {n}" for n, z, _ in curves)
    ausgabe = [f"Error against the truth [m], scale 0 … {ymax:.2f} m   ({legende})"]
    for i, row_rect in enumerate(raster):
        stab = ymax * (height - 1 - i) / (height - 1)
        ausgabe.append(f"{stab:6.2f} |" + "".join(row_rect))
    ausgabe.append("       +" + "-" * width)
    started, end = f"{tmin:g}s", f"{tmax:g}s"
    ausgabe.append("       " + started + " " * max(1, width + 1 - len(started) - len(end)) + end)
    return "\n".join(ausgabe)


# One glyph per eighth of a series' own range. A block per sample is how two minutes of a run fit on
# one line, and these four columns are worth looking at as a shape rather than quoting as numbers:
# where the sky went empty, when the chip had warmed enough to move its bias, when the counter began
# to hear the source.
BARS = "▁▂▃▄▅▆▇█"


def sparkline(values, width=44):
    """Values over time as one line of blocks, on their own scale: `(drawing, low, high)`.

    Thinned to `width` samples by taking every k-th one rather than by averaging: a quality that was
    at 0 for two seconds has vanished by averaging, and those two seconds are the point.
    """
    if not values:
        return "", None, None
    if len(values) > width:
        values = values[::max(1, round(len(values) / width))]
    low, high = min(values), max(values)
    if high - low < 1e-12:
        return BARS[0] * len(values), low, high
    return "".join(BARS[min(7, int((v - low) / (high - low) * 7.999))] for v in values), low, high


def sensor_state(rows):
    """What the instruments did during the run — the four columns that explain all the others.

    Each line on its own scale, because they share no unit: the quality is 0-2, the lost count starts
    at 0 and only grows, the chip temperature sits near 24 °C and moves by tenths. On a shared axis
    the temperature would be the only visible line, which is the opposite of what these columns are
    there for. Where the model behind each one lives: `q_gps`, `lost_gps` and the LIDAR gaps in
    CONTRACT §6.4, `temp_imu` in CONTRACT-KF §3, `intensity_poi` in CONTRACT §6.13.
    """
    series = [("q_gps", "fix quality 2 good · 1 degraded · 0 nothing usable"),
              ("lost_gps", "fixes the transport dropped since the run started"),
              ("temp_imu", "IMU chip temperature in °C — its bias walks with it"),
              ("intensity_poi", "counts/s of the loudest radiation source")]
    lines = []
    for column, meaning in series:
        values = [v for v in (num(r.get(column)) for r in rows) if v is not None]
        if not values:
            lines.append(f"  {column:13} —    nothing logged ({meaning})")
            continue
        drawing, low, high = sparkline(values)
        span = f"{low:g}" if low == high else f"{low:g} … {high:g}"
        lines.append(f"  {column:13} {drawing:<44} {span:>13}   {meaning}")
    seen = sorted({(r.get("name_poi") or "").strip() for r in rows} - {""})
    if seen:
        # Named, not seen: the counter always says which contribution is loudest, and past the
        # source's `range` that contribution is 0 — so this row is about the world, not about the run.
        lines.append(f"  {'':13}{'':44} {'':>13}   named by the counter as loudest: "
                     + ", ".join(seen))
    return "\n".join(lines)


def png(path, rows, curves, target):
    """Three panels: path, error over time, message rates. Needs matplotlib (optional)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is missing — plots stay ASCII (pip install --user matplotlib).")
        return
    fig, field = plt.subplots(1, 3, figsize=(17, 5))
    bahn = [("Truth", "x_truth", "y_truth", "k-"), ("GPS", "x_gps", "y_gps", ".g"),
            ("Odometry", "x_odom", "y_odom", ".b"), ("KF", "x_kf", "y_kf", "r-")]
    for name, xs, ys, stil in bahn:
        x = [num(r.get(xs)) for r in rows if num(r.get(xs)) is not None]
        y = [num(r.get(ys)) for r in rows if num(r.get(ys)) is not None]
        if x and y:
            field[0].plot(x, y, stil, label=name, linewidth=1)
    field[0].set_title(f"Path — {path}")
    field[0].set_xlabel("x [m]"); field[0].set_ylabel("y [m]")
    field[0].legend(); field[0].grid(True, alpha=.3); field[0].set_aspect("equal")
    for name, _, points in curves:
        field[1].plot([t for t, _ in points], [e for _, e in points], label=name)
    field[1].set_title("Error against the truth"); field[1].set_xlabel("t [s]")
    field[1].set_ylabel("Error [m]"); field[1].legend(); field[1].grid(True, alpha=.3)
    seconds = [int(num(r.get("t")) or 0) for r in rows]
    gps = [s for s, r in zip(seconds, rows) if num(r.get("t_age_gps")) is not None]
    kfz = [(int(num(r.get("t")) or 0), num(r.get("n_kf"))) for r in rows
           if num(r.get("n_kf")) is not None]
    kf_rate = [max(0.0, kfz[i][1] - kfz[i - 1][1]) for i in range(1, len(kfz))
               if kfz[i][0] != kfz[i - 1][0]]
    field[2].hist([gps, kf_rate], bins=range(0, max(seconds, default=0) + 2),
                 label=["GPS fixes/s", "kf/pose/s"], color=["#4faf6f", "#d1705a"])
    field[2].set_title("Message rates"); field[2].set_xlabel("second"); field[2].legend()
    fig.tight_layout()
    fig.savefig(target, dpi=110)
    print(f"PNG written: {target}")


def main_plot():
    e = argparse.ArgumentParser(description="Evaluate a measurement log (./lab --log)")
    e.add_argument("log", help="CSV file written by ./lab ... --log FILE.csv")
    e.add_argument("-o", "--output", metavar="FILE.png", help="write a PNG (matplotlib)")
    e.add_argument("--sensor", choices=sorted(ROHSENSOR), default="gps",
                   help="raw sensor for the comparison and the plot (default: gps)")
    e.add_argument("--interval", type=float, default=0.0, metavar="S",
                   help="average the error time series over S seconds in the plot")
    e.add_argument("--robot", default="", help="only this robot (default: the one with kf data)")
    e.add_argument("--width", type=int, default=76, help="width of the ASCII plot (<= 92)")
    e.add_argument("--list", action="store_true", help="show the columns of the log")
    a = e.parse_args()
    rows = lese(a.log)
    if a.list:
        print(f"{len(rows)} lines, columns: " + ", ".join((rows[0] or {}).keys()))
        for col, value in (rows[len(rows) // 2] or {}).items():
            print(f"  {col:12} {value!r}")
        return 0
    groups = by_robot(rows)
    if a.robot and a.robot not in groups:
        sys.exit(f"robot '{a.robot}' is not in the log — it contains: "
                 + ", ".join(sorted(groups)))
    name = a.robot or ("" if len(groups) == 1 else
                       max(groups, key=lambda k: sum(1 for r in groups[k] if num(r.get("x_kf")))))
    for robot in [name] if name else sorted(groups):
        bericht(groups[robot], a, robot, len(groups))
    return 0


def bericht(rows, a, robot, n_robots):
    """Numbers and ASCII plot for one robot — this is what goes into the lab report."""
    kf, raw, nees, fixes = accuracy(rows, a.sensor)
    t = sorted(x for x in (num(r.get("t")) for r in rows) if x is not None)
    if not t:
        print(f"\n=== {a.log} · robot '{robot}' ===\nno timestamps in the file")
        return
    extra = "" if n_robots == 1 else f" (1 of {n_robots}, --robot selects)"
    print(f"\n=== {a.log} · robot '{robot}'{extra} ===")
    print(f"Lines: {len(rows)}   time: {t[0]:.1f} … {t[-1]:.1f} s   raw sensor: {a.sensor}")
    r_kf, r_raw = rmse(kf), rmse(raw)
    if not raw:
        print(f"No {a.sensor} value against the truth — are columns x_{a.sensor}/y_{a.sensor} empty?")
    else:
        print(f"RMSE truth↔{a.sensor:<4}: {r_raw:6.3f} m   Max: {max(x[1] for x in raw):6.3f} m")
    if kf:
        print(f"RMSE truth↔kf  : {r_kf:6.3f} m   Max: {max(x[1] for x in kf):6.3f} m")
        if r_raw and r_kf:
            print(f"Improvement    : {r_raw / r_kf:5.2f}   (rmse_{a.sensor} / rmse_kf)")
        if nees:
            print(f"NEES mean      : {sum(nees) / len(nees):5.2f}   (1 = honest standard deviation)")
        else:
            print("NEES mean      :   —   (sx_kf/sy_kf missing — pass sigma to send_kf)")
        n = [num(r.get("n_kf")) for r in rows if num(r.get("n_kf")) is not None]
        duration = kf[-1][0] - kf[0][0]
        if duration > 0:
            messages = int(max(n) - min(n)) if len(n) > 1 else len(kf)
            print(f"kf/pose rate   : {messages / duration:5.2f} Hz  ({messages} messages)")
    else:
        print("Not a single kf/pose value in the log: is your node running, and does it "
              "call rob.send_kf()? This file holds the raw sensor data only.")
    gaps = fix_gaps(fixes)
    if gaps:
        mittel = sum(gaps) / len(gaps)
        mitte = sorted(gaps)[len(gaps) // 2]
        print(f"GPS fix gap    : {mittel:.2f} s mean, {mitte:.2f} s median, "
              f"largest {max(gaps):.2f} s  ({len(set(fixes))} fixes)")
    else:
        print(f"GPS fix gap    : —   ({len(set(fixes))} fixes)")
    print("\nInstruments over time — each line on its own scale:")
    print(sensor_state(rows))
    curves = [(f"{a.sensor}−truth", "o", center(raw, a.interval)),
              ("kf−truth", "*", center(kf, a.interval))]
    print()
    print(ascii_chart(curves, width=max(20, min(92, a.width))))
    if a.output:
        png(a.log, rows, curves, a.output)


if __name__ == "__main__":
    sys.exit(main_plot())
