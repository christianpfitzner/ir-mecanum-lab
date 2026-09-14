# Second drive train: steering (Ackermann)

`--variant steering` is another kinematic world, not a worse mecanum robot: one driven axle, one steered
axle, **one** degree of freedom — metres per second along the car, and a turn rate that follows from the
steering angle. `--variant` stays empty by default, so every graded task and both reference solutions
drive what they drove before. The car is three numbers:

| number in `DEFAULT_CONFIG["steering"]` | default | what it decides |
|---|---|---|
| `wheel_base` (L) | 1.00 m | the lever between the axles: `omega = v·tan(delta)/L`, `R = L/tan(delta)` |
| `steer_max_deg` | 32° | the physical end stop of the rack, which moves at `steer_rate_deg_s` = 60°/s |
| `v_max` | 0.8 m/s | drive speed; `track` 0.62 m is what makes the two front angles differ |

The first two give the number that shapes every path: `R_min = L / tan(delta_max)` = **1.60 m**. A tighter
corner is not a driving mistake, it is a car that does not exist. Measured as the chord of a half turn —
not as `v/omega`, which compares the model with itself: demanded 14°/22°/30° come out as
4.011/2.475/1.732 m against `L/tan(delta)`, ratio 1.0000 each, and a command of 45° still drives 1.600 m
(`tests/test_steering_w4.py`).

The two front wheels never steer equally; the car turns about one point on the rear axle line, so
`atan(L/(R − W/2))` inside and `atan(L/(R + W/2))` outside. All four wheel speeds stay published under the
old labels `VL/VR/HL/HR` (the rear pair rolls, its mean *is* `v`); the angles are the extra `steer_deg`
field of `/sim/robots`.

```bash
./lab run --world production --robot car --variant steering \
          --controller student/steering_example.py --headless --seconds 60 --truth
```

The example drives one lap of a rounded rectangle — every corner `R_min + 0.15 m`, because the rack needs
half a second from straight ahead to 30° — then parks in front of a shelf by LIDAR. On that command:
20.3 m driven, **0 wall contacts**, mission `done` after 45.2 s of the 60 s budget, gap to the shelf
0.959 m. `vy` appears once, in the comment saying why it never helps: **a steering car cannot strafe.**
The simulator answers `steering robot cannot strafe, vy=0.25 dropped` once per robot and keeps driving on
`vx` and `omega`. Four wheel speeds cannot steer it either: their mean becomes `v` and the rack stays
straight — the legible answer to a student who sends lab 1's inverse kinematics to this car.

The odometry is the second lesson. The mecanum integrator averages four wheel speeds; this one integrates
one steering angle it only *believes*, because the car has no steering encoder. `odom.steer_max_scale` is
that belief (default 1.0 = the true end stop). 20 s at full lock, other sensor noise off:

| `odom.steer_max_scale` | yaw error after 20 s | place error |
|---|---|---|
| 1.0 | +0.0° | 0.00 m |
| 1.1 — rack believed at 35.2° | **+44.9°** | 1.12 m |
| 0.9 | −42.0° | 1.34 m |

`./lab run --variant steering --set odom.steer_max_scale=1.1 --truth --seconds 20` shows it: the odometry
ghost (`o`) leaves a circle of its own under the car. The error is systematic — every corner adds the same
wrong degree — so it never averages out.

Against an obstacle this car behaves like the mecanum one, and that is what makes the demo transfer: the
body stops, the drive wheels keep the speed the motor demands, and the counters integrate what the wheels
did. 30 s of 0.6 m/s into a wall (`steering.slip` = 1.0, the default): **4.38 m driven, 17.91 m counted**,
`contacts 1`, the wheels still at 12.0 rad/s. With `steering.slip=0` the wheels stand still and the
odometry stays honest to 1 cm. Model, wiring and what is published where: `docs/CONTRACT.md` §5.1 and
§6.12.
