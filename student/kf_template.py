#!/usr/bin/env python3
"""Experiment 2 — state estimation with a Kalman filter. This is your submission file.

The grader drives your robot, you steer nothing: no inverse_kinematics, no send_wheels,
no publish_cmd_vel (CONTRACT-KF §8.1). You read GPS, odometry and IMU and report your
estimate on /<robot>/kf/pose — with 1σ, because K3 grades exactly that number.
Convention (§2): x, y in world meters, x forward, y LEFT, theta counter-clockwise. Odometry
gives the body velocity (vx forward, vy left), the IMU the yaw rate gz; az carries +g and
ax/ay are biased — never integrate an IMU open-loop (K2, hints). Everything in the WORLD
frame, or the drift you are trying to get rid of comes along:

    x = (x, y, vx, vy)   prediction x' = F·x, P' = F·P·Fᵀ + Q(dt)
                         position update    GPS       H = [I 0], R = sigma_xy²
                         motion update      odometry  H = [0 I], R = sigma_v²
                         report: sx = √P_xx, sy = √P_yy    (1σ, not variance!)

Start it (the sim drives itself, --truth shows the exact pose as a ghost outline):
    ./lab run --world arena --task kf_gps --robot alice --controller student/kf_template.py --truth
Check it the way grading will: ./lab grade --task kf_gps --controller student/kf_template.py
Reference solution: student/kf_solution.py — compute first, then compare. The seven TODOs are
the whole computation; with them the node already runs and keeps reporting its start pose.
"""
import math
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import wrap_angle

Q_ACC = 12.0          # m²/s³  process noise: everything the CV model cannot do
SIGMA_V = 0.08        # m/s    scatter of the measured wheel speed (motion update)
SIGMA_TH = 0.03       # rad/√s gyro heading uncertainty — only used for the reported sth
BIAS_PROBEN, STILLSTAND = 80, 0.02     # IMU averaging at standstill: samples, still limit
P0_POS, P0_VEL, P0_TH = 0.50, 0.50, 0.05   # initial 1σ: position, velocity, heading
MELDE_DT = 0.02       # s = 50 Hz: kf/pose report rate (K4 requires at least 10 Hz)
SCHLAF = 2.0          # s: a longer step means the node slept — not a prediction

# ---------------------------------------------------------- matrix helpers by hand
# Glue code, already written: a Kalman filter is a pile of matrix multiplications.

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
    """A + B and A − B element by element (row after row)."""
    return [[a + b for a, b in zip(x, y)] for x, y in zip(A, B)]

def msub(A, B):
    return [[a - b for a, b in zip(x, y)] for x, y in zip(A, B)]

def inv2(M):
    """Inverse of a 2×2 — the lecture formula. Degenerate: identity instead of a crash."""
    d = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(d) < 1e-12:
        return [[1.0, 0.0], [0.0, 1.0]]
    return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]]

def cv_matrix(dt):
    """TODO 1: build F of the CV model over dt seconds.

    Expectation: the position grows by v·dt, the velocity stays — identity matrix with dt
    on the two coupling slots. Check: mv(cv_matrix(0.5), [0,0,2,4]) == [1,2,2,4].
    """
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],      # TODO 1: replace with F
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]

def q_matrix(q, dt):
    """TODO 2: build Q from the time step dt (white acceleration noise, density q m²/s³).

    Expectation: per axis the 2×2 form q·[[dt³/3, dt²/2],[dt²/2, dt]], both axes independent,
    so it sits twice diagonal-crossed in the 4×4. Checks: positive semidefinite, all zero for
    dt = 0. Large Q = "more model error admitted" = the filter follows the GPS faster: K1 gets
    worse, K3's NEES drops — that is the trade-off, not black magic.
    """
    return [[0.0] * 4 for _ in range(4)]                      # TODO 2: replace with Q


class KF:
    """Kalman filter for (x, y, vx, vy) in the world frame — 4×4, unrolled by hand."""

    def __init__(self, x, y, vx, vy, theta, t):
        self.x = [x, y, vx, vy]
        d = [P0_POS ** 2, P0_POS ** 2, P0_VEL ** 2, P0_VEL ** 2]
        self.P = [[d[i] if i == j else 0.0 for j in range(4)] for i in range(4)]
        self.t, self.theta, self.var_th = float(t), theta, P0_TH ** 2
        self.start = float(t)          # mission starts here: earlier measurements are not it
        self.omega, self.n, self.innov = 0.0, 0, 0.0

    def prediction(self, dt, omega):
        """One step over dt: heading from the gyro, then x' = F·x and P' = F·P·Fᵀ + Q."""
        if dt <= 0.0:
            return
        self.omega = omega
        self.theta = wrap_angle(self.theta + omega * dt)
        self.var_th += (SIGMA_TH * dt) ** 2          # without a measurement you know less
        F, Q = cv_matrix(dt), q_matrix(Q_ACC, dt)
        self.x = mv(F, self.x)
        self.P = madd(mul(mul(F, self.P), transpose(F)), Q)
        self.t += dt

    def update(self, stellen, messwert, R):
        """Measurement update on the states in `stellen`: (0,1) = GPS position, (2,3) = velocity.

        H has ones exactly on those slots, so S = P[subset] + R is a 2×2 (the only inverse we
        need) and K = P·Hᵀ·S⁻¹ is a 4×2.
        """
        S = [[self.P[i][j] + (R if i == j else 0.0) for j in stellen] for i in stellen]
        Si = inv2(S)
        # TODO 3: build K (4×2):  K[i][j] = sum(P[i][stellen[k]] * Si[k][j] for k in (0,1))
        K = [[0.0, 0.0] for _ in range(4)]                     # TODO 3: replace with K
        y = [messwert[m] - self.x[stellen[m]] for m in range(2)]  # innovation = measured − predicted
        # TODO 4: correct the state — for every i: self.x[i] += K[i][0]*y[0] + K[i][1]*y[1]
        # TODO 5: correct the covariance — self.P = msub(self.P, mul(mul(K, S), transpose(K)))
        #   (keeps P symmetric; leaving P alone means estimating at the initial 1σ forever)
        self.n += 1
        return y

    def schritt(self, dt, omega, vx_welt, vy_welt, R_v):
        """Prediction plus motion update: the odometry pins vx, vy to a few cm/s."""
        self.prediction(dt, omega)
        self.update((2, 3), [vx_welt, vy_welt], R_v)

    def sigmas(self):
        """TODO 6: the reported 1σ — sx = √P[0][0], sy = √P[1][1], sth = √var_θ.

        Expectation: sx falls from P0_POS to a few centimeters in the first seconds and grows
        again during the GPS outage. Without these numbers K3 cannot be graded.
        """
        return 0.0, 0.0, 0.0                                       # TODO 6: replace with √P


# --------------------------------------------------------------------- glue code

def stempel(*messen) -> float:
    """Newest stamp among the measurements given — 0.0 for those that are not there yet."""
    return max([m.t for m in messen if m is not None] + [0.0])


def mission(rob, task):
    """Estimate for as long as this task runs — the runner reports "done" afterwards.

    The loop ticks on message stamps: it computes only when a measurement is stamped later
    than the filter's own state. Never on the wall clock — tools/fastgrade.py runs 25× faster,
    and a wall-clock-driven filter would run in the wrong direction.
    """
    f, sigma_xy, bias, letzter_fix, letzter_meldung = None, 0.5, 0.0, 0.0, -1e9
    bias_summe, bias_n, R_v = 0.0, 0, SIGMA_V ** 2
    grundlinie = stempel(rob.odom(), rob.imu(), rob.gps())       # what the bus already knew
    while rob.running() and rob.task() == task:
        rob.spin(0.005)
        o, i, gps = rob.odom(), rob.imu(), rob.gps()
        if o is None:
            continue
        if f is None and stempel(o, i, gps) <= grundlinie:
            continue        # still the last messages of the previous task, not this drive
        # TODO 7: average the gyro bias at standstill — while hypot(o.vx, o.vy) < STILLSTAND and
        #   bias_n < BIAS_PROBEN: bias_summe += i.gz, bias_n += 1, bias = bias_summe / bias_n.
        #   Without averaging a 5 mrad/s bias drags the heading 3 degrees off in 10 s.
        if f is None:                                   # start from the odometry: pose is good
            f = KF(o.x, o.y, o.vx, o.vy, o.theta, o.t)
            sigma_xy = float(rob.config("gps.sigma_xy", 0.5) or 0.5)   # guessing is needless
        gierrate = (i.gz - bias) if i is not None else o.omega
        t_mess = max(o.t, i.t if i is not None else 0.0)
        if t_mess - f.t > SCHLAF:                # five seconds of CV model are not a prediction,
            f = KF(o.x, o.y, o.vx, o.vy, o.theta, o.t)      # they are a new start
        if t_mess > f.t:                 # rotate the body velocity into the world by theta
            c, s = math.cos(f.theta), math.sin(f.theta)
            f.schritt(t_mess - f.t, gierrate, c * o.vx - s * o.vy, s * o.vx + c * o.vy, R_v)
        # Position update: every fix exactly once. And only fixes from this task — the bus
        # remembers the last message, so a fix from the previous task (before the respawn)
        # would be a lie for this mission. Checking against f.start is enough.
        if gps is not None and gps.t >= f.start and gps.t > letzter_fix:
            letzter_fix = gps.t
            f.prediction(max(gps.t - f.t, 0.0), f.omega)  # extrapolate to the fix's time
            f.innov = math.hypot(*f.update((0, 1), [gps.x, gps.y], sigma_xy ** 2))
        if f.t - letzter_meldung >= MELDE_DT:            # estimate with its 1σ onto kf/pose
            letzter_meldung = f.t
            sx, sy, sth = f.sigmas()
            rob.send_kf(f.x[0], f.x[1], f.theta, sx, sy, sth,
                        info={"t": round(f.t, 2), "q_acc": Q_ACC, "innov": round(f.innov, 3)})


if __name__ == "__main__":
    # The runner owns the bus and the lifecycle and calls mission() once per task exactly.
    robot_io.serve(sys.modules[__name__])
