#!/usr/bin/env python3
"""Reference solution, Experiment 2 — one Kalman filter, four sensor profiles, no numpy.

The grader drives, you estimate: this node sends no wheels and no cmd_vel (pass-through
mode, CONTRACT-KF §1). It reads GPS, odometry and IMU and reports its estimate together
with its uncertainty on /<robot>/kf/pose.

State in the WORLD frame (in the body frame the drift you want to shed comes along):

    x = (x, y, vx, vy)                          position and velocity [m, m/s]

    prediction   x' = F·x,  P' = F·P·Fᵀ + Q(dt)         F = CV model, Q from dt
    heading      θ' = θ + ω·dt,  ω from the IMU gyro (bias averaged at standstill first)
    update 1     position update:    GPS (x, y)      R = sigma_xy²      → H = [I 0]
    update 2     motion update:      odometry (vx, vy), rotated into the world by θ
                                     → H = [0 I], R = sigma_v²
    report       sx = √P_xx, sy = √P_yy, sth = √var_θ   — 1σ, not variance (K3!)

Update 2 is the reason this works without GPS as well: the odometry holds the state
variables vx, vy to a few mm/s on every tick, so the prediction pushes the position on with
the *measured* body velocity. In K2's GPS outage exactly this update is what remains — the
filter then keeps computing cleanly instead of breaking out straight ahead. The IMU
acceleration is deliberately NOT integrated: its bias of 0.05 m/s² double-integrates to
2.5 m after 10 s (K2, hints).

Three details that make the difference in practice:

  * Time is simulation time. The filter ticks on the stamps of the measurements
    (odom.t, imu.t, fix.t), never on the wall clock — tools/fastgrade.py runs 25× faster.
  * A fix is already a few milliseconds old when it arrives. It is applied at its own
    measurement time (rewind the parked steps, then replay them), not at "now"; at
    0.85 m/s, 100 ms of lag would otherwise be 8.5 cm of following error.

  * The uncertainty you claim has to match the error: every 20 fixes the filter compares the
    scatter of its innovation with the announced S and corrects Q in small steps (K3, the
    only way to NEES ≈ 1 without guessing). Two settings also keep K2 from failing: Q_GAP
    during the GPS outage (odometry drift is not white noise) and dropping fixes from before
    the mission starts — the bus still offers the ones from the previous task.

Recheck it (both without ROS, without a window, the same grading):

    python3 tools/fastgrade.py --task kf_alle --controller student/kf_solution.py --speed 25
    ./lab grade --task kf_alle --controller student/kf_solution.py
"""
import math
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import wrap_angle

# ------------------------------------------------------------------- parameters
Q_ACC = 6.0          # m²/s³  process noise: everything the CV model cannot do
Q_GAP = 3.0          # factor while no GPS is correcting (outage longer than 2 s)
SIGMA_V = 0.08        # m/s    scatter of the measured wheel speed (update 2)
SIGMA_TH = 0.03       # rad/√s gyro heading uncertainty — only used for the reported sth
BIAS_SAMPLES = 80      # IMU samples at standstill, after that the gyro bias is averaged
STANDSTILL = 0.02     # m/s: below this the robot counts as standing (calibration window)
P0_POS = 0.50         # m      initial 1σ position — deliberately large, the GPS leads first
P0_VEL = 0.50         # m/s    initial 1σ velocity
P0_TH = 0.05          # rad    initial 1σ heading
REPORT_DT = 0.02       # s = 50 Hz: kf/pose report rate (threshold: at least 10 Hz)
BUFFER = 120          # steps that can be rewound for late fixes
STALE_AFTER = 2.0         # s: older fixes no longer belong to this task
SLEEP = 2.0          # s: a longer step means the node slept — not a prediction


# ------------------------------------------------------- matrix helpers by hand
# Handwork on purpose: four by four, lists of rows, no library. Anyone who sees the
# arithmetic in matrix form recognizes it from the lecture — and sees at once that a
# Kalman filter is nothing more than a few matrix multiplications.

def mul(A, B):
    """A·B for small matrices (lists of rows) — three loops, nothing else is needed."""
    C = [[0.0] * len(B[0]) for _ in range(len(A))]
    for i, zeile in enumerate(A):
        for k, a in enumerate(zeile):
            if a:
                for j, b in enumerate(B[k]):
                    C[i][j] += a * b
    return C


def mv(A, v):
    """A·x — matrix times a column vector."""
    return [sum(a * b for a, b in zip(zeile, v)) for zeile in A]


def transpose(A):
    """Aᵀ — the columns become rows."""
    return [list(spalte) for spalte in zip(*A)]


def madd(A, B):
    return [[a + b for a, b in zip(x, y)] for x, y in zip(A, B)]


def msub(A, B):
    return [[a - b for a, b in zip(x, y)] for x, y in zip(A, B)]


def inv2(M):
    """Inverse of a 2×2 — the formula from the lecture. Degenerate: identity, not a crash."""
    d = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(d) < 1e-12:
        return [[1.0, 0.0], [0.0, 1.0]]
    return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]]


def cv_matrix(dt):
    """F of the CV model: the position grows by v·dt, the velocity stays as it was."""
    return [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def q_matrix(q, dt):
    """Q from the time step dt: white acceleration noise with density q [m²/s³].

    Per axis the exactly integrated form q·[[dt³/3, dt²/2], [dt²/2, dt]] (the dt⁴/4 variant
    in the task hints is the approximation of it); the two axes are independent, so the
    2×2 form sits twice diagonal-crossed in the 4×4. It is positive semidefinite:
    determinant q²·dt⁴/12 > 0 and trace > 0.
    """
    a, b, c = q * dt ** 3 / 3.0, q * dt ** 2 / 2.0, q * dt
    return [[a, 0.0, b, 0.0], [0.0, a, 0.0, b],
            [b, 0.0, c, 0.0], [0.0, b, 0.0, c]]


class KF:
    """Kalman filter for (x, y, vx, vy) in the world frame — 4×4, unrolled by hand."""

    def __init__(self, x, y, vx, vy, theta, t):
        self.x = [x, y, vx, vy]
        d = [P0_POS ** 2, P0_POS ** 2, P0_VEL ** 2, P0_VEL ** 2]
        self.P = [[d[i] if i == j else 0.0 for j in range(4)] for i in range(4)]
        self.t, self.theta, self.var_th = float(t), theta, P0_TH ** 2
        self.start = float(t)          # mission starts here: earlier measurements are not it
        self.omega, self.n, self.innov = 0.0, 0, 0.0
        self.last_fix = float(t)              # when a GPS fix last corrected
        self.q_faktor, self.yy, self.ss, self.count = 1.0, 0.0, 0.0, 0   # consistency rule (K3)
        self.steps = []                     # (t_end, dt, ω, vx, vy, R_v, state before)

    # ------------------------------------------------------------------ prediction
    def prediction(self, dt, omega):
        """One step over dt seconds: heading from the gyro, then F·x and F·P·Fᵀ + Q."""
        if dt <= 0.0:
            return
        self.omega = omega
        self.theta = wrap_angle(self.theta + omega * dt)
        self.var_th += (SIGMA_TH * dt) ** 2          # without a measurement you know less
        #  If correction stays away for a long time (GPS outage), the odometry keeps drifting
        #  — and its drift is precisely *not* white noise (prelab question 3). The residual
        #  noise may grow faster than the CV model claims; otherwise P becomes too small and
        #  the own claim dishonest (the NEES climbs above 3).
        Q = q_matrix(Q_ACC * self.q_faktor * (Q_GAP if self.t - self.last_fix > 2.0 else 1.0), dt)
        F = cv_matrix(dt)
        self.x = mv(F, self.x)
        self.P = madd(mul(mul(F, self.P), transpose(F)), Q)
        self.t += dt

    def update(self, slots, measurement, R, gain=False):
        """One update on the states in `slots` (diagonal measurement, R is their variance).

        H has ones exactly on those slots, so S = P[subset] + R and K = P·Hᵀ·S⁻¹. Then as in
        the textbook: x += K·y and P -= K·S·Kᵀ (Joseph form, kept symmetric). Used twice:
        (0, 1) for the GPS, (2, 3) for the odometry.
        """
        S = [[self.P[i][j] + (R if i == j else 0.0) for j in slots] for i in slots]
        Si = inv2(S)                                  # the only inverse we need
        K = [[sum(self.P[i][slots[k]] * Si[k][j] for k in range(2)) for j in range(2)]
             for i in range(4)]
        y = [measurement[m] - self.x[slots[m]] for m in range(2)]
        if gain:
            self.consistency(y, S[0][0] + S[1][1])
        self.x = [self.x[i] + K[i][0] * y[0] + K[i][1] * y[1] for i in range(4)]
        self.P = msub(self.P, mul(mul(K, S), transpose(K)))
        self.n += 1
        return y

    def consistency(self, y, s):
        """K3 in three lines: does the innovation scatter match what the filter announced?

        If the innovation is larger than `S`, the model is wrong — Q was too small. If it is
        smaller, Q was too large and the claimed covariance is inflated. Correction happens
        every 20 fixes by a fixed factor: fast enough to learn, sluggish enough not to chase
        every outlier.
        """
        self.yy += y[0] ** 2 + y[1] ** 2
        self.ss, self.count = self.ss + s, self.count + 1
        if self.count < 20:
            return
        ratio = self.yy / max(self.ss, 1e-9)
        if ratio > 1.4:
            self.q_faktor = min(self.q_faktor * 1.4, 12.0)
        elif ratio < 0.7:
            self.q_faktor = max(self.q_faktor * 0.75, 0.05)
        self.yy = self.ss = 0.0
        self.count = 0

    def step(self, dt, omega, vx_world, vy_world, R_v):
        """Prediction + motion update as one unit — it is replayed when a fix arrives late."""
        vor = (list(self.x), [z[:] for z in self.P], self.theta, self.var_th)
        self.prediction(dt, omega)
        self.update((2, 3), [vx_world, vy_world], R_v)
        self.steps.append((self.t, dt, omega, vx_world, vy_world, R_v, vor))
        del self.steps[:-BUFFER]

    def smoothing(self, t):
        """Reset the state to the last step before `t`; returns the steps after it."""
        back = []
        while self.steps and self.steps[-1][0] > t:
            ende, dt, omega, vx_world, vy_world, R_v, vor = self.steps.pop()
            self.x, self.P, self.theta, self.var_th = vor[0], vor[1], vor[2], vor[3]
            self.t, self.omega = ende - dt, omega
            back.append((dt, omega, vx_world, vy_world, R_v))
        return back

    def turn_update(self, th_gps, R_th):
        """Scalar update of the heading from the GPS angle: holds θ long-term without forcing it."""
        k = self.var_th / (self.var_th + R_th)
        self.theta = wrap_angle(self.theta + k * wrap_angle(th_gps - self.theta))
        self.var_th *= 1.0 - k

    def sigmas(self):
        """1σ from P — the number K3 grades, and the only reason P exists at all."""
        return (math.sqrt(max(self.P[0][0], 0.0)), math.sqrt(max(self.P[1][1], 0.0)),
                math.sqrt(max(self.var_th, 0.0)))


# --------------------------------------------------------------------- glue code

def stamp(*measure) -> float:
    """Newest stamp among the measurements given — 0.0 for those that are not there yet."""
    return max([m.t for m in measure if m is not None] + [0.0])


def mission(rob, task):
    """Estimate for as long as this task runs. Nothing is driven (CONTRACT-KF §8.1).

    The loop ticks on message stamps: it recomputes when a measurement arrives that is
    stamped later than the state. `rob.spin()` keeps the buses alive; the task change ends
    it — the runner then reports "done" for the grader.
    """
    f, sigma_xy, sigma_th = None, 0.5, 0.2
    bias_sum, bias_n, bias = 0.0, 0, 0.0
    last_fix, letzter_meldung = 0.0, -1e9
    R_v = SIGMA_V ** 2
    baseline = stamp(rob.odom(), rob.imu(), rob.gps())       # what the bus already knew
    while rob.running() and rob.task() == task:
        rob.spin(0.005)
        o, i, gps = rob.odom(), rob.imu(), rob.gps()
        if o is None:
            continue
        if f is None and stamp(o, i, gps) <= baseline:
            continue        # still the last messages of the previous task, not this drive

        # 1) Average the gyro bias at standstill — a short calibration, then it is frozen.
        #    The rest of the bias story (random walk) sits in Q; retuning it would be cheating.
        if bias_n < BIAS_SAMPLES and i is not None:
            if math.hypot(o.vx, o.vy) < STANDSTILL and abs(i.gz) < 0.2:
                bias_sum, bias_n = bias_sum + i.gz, bias_n + 1
                bias = bias_sum / bias_n

        if f is None:                                  # start from the odometry: pose is good
            f = KF(o.x, o.y, o.vx, o.vy, o.theta, o.t)
            sigma_xy = float(rob.config("gps.sigma_xy", 0.5) or 0.5)
            sigma_th = float(rob.config("gps.sigma_theta", 0.2) or 0.2)

        # 2) Predict up to the newest message stamp, with yaw rate and world velocity
        gierrate = (i.gz - bias) if i is not None else o.omega
        t_mess = max(o.t, i.t if i is not None else 0.0)
        if t_mess - f.t > SLEEP:                # five seconds of CV model are not a prediction,
            f = KF(o.x, o.y, o.vx, o.vy, o.theta, o.t)      # they are a new start
        if t_mess > f.t:
            c, s = math.cos(f.theta), math.sin(f.theta)
            f.step(t_mess - f.t, gierrate, c * o.vx - s * o.vy, s * o.vx + c * o.vy, R_v)

        # 3) Position update at the fix's measurement time — not at "now"
        #    a fix from the previous task (the bus remembers the last message!) would be a lie.
        if gps is not None and gps.t >= f.start and gps.t > last_fix and gps.t >= f.t - STALE_AFTER:
            last_fix = f.last_fix = gps.t
            nachfahren = f.smoothing(gps.t)                # back to fix.t ...
            if gps.t > f.t:
                f.prediction(gps.t - f.t, f.omega)     # ... close the gap if there is one
            y = f.update((0, 1), [gps.x, gps.y], sigma_xy ** 2, gain=True)
            f.innov = math.hypot(*y)
            f.turn_update(gps.theta, sigma_th ** 2)
            for dt, omega, vx_w, vy_w, rv in reversed(nachfahren):
                f.step(dt, omega, vx_w, vy_w, rv)    # ... and replay the time up to now

        # 4) Report: estimate plus 1σ from P, on the report rate over simulation time
        if f.t - letzter_meldung >= REPORT_DT:
            letzter_meldung = f.t
            sx, sy, sth = f.sigmas()
            rob.send_kf(f.x[0], f.x[1], f.theta, sx, sy, sth,
                        info={"t": round(f.t, 2), "q_acc": Q_ACC, "sigma_xy": sigma_xy,
                              "bias_gz": round(bias, 5), "innov": round(f.innov, 3),
                              "updates": f.n})


if __name__ == "__main__":
    # No inverse_kinematics, no send_wheels, no publish_cmd_vel: the grader drives.
    robot_io.serve(sys.modules[__name__])
