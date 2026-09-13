# wifi — every command arrives by radio, and the radio has a range

With `wifi.enabled` true, `/cmd_vel` and `/wheels` travel through one access point per hall, and the two
things that kill a real link kill it here: distance, and walls in the straight line to that access point.
`wifi.enabled` is false in `mecanum_lab/types.py`, and that is a guarantee rather than a hint: with the
option off no radio is built, no random number is drawn for it and no `/link` is published, so a graded
run's command stream is byte for byte what it was before a radio existed.

Drive it yourself and look at layer `n`: the access point as a box with two arcs, the line from it to
your robot, and the quality bar over its head.

```bash
ros2 launch mecanum_lab demo_wifi.launch.py
```

The shipped example drives instead — away from the access point until the link gives out, 60 s of it:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_wifi.json \
    controller:=student/link_autonomy_example.py seconds:=60
```

The radio has a launcher of its own, in which every knob of the budget above is an argument — here a
thicker wall, an access point in the middle of the hall and the rule that keeps driving:

```bash
ros2 launch mecanum_lab wifi.launch.py wall_db:=20 ap:=[10,6] autonomy:=dead_reckoning
```

Without ROS 2: `./lab sim --config config/demo_wifi.json`, the same arguments on `./lab run`, and
`--set wifi.autonomy=dead_reckoning` for the third.

## What it changes

`world: production` and the whole `wifi` block:

| key | value | what it is |
|---|---|---|
| `enabled` | `true` | the one switch; everything below is read only when it is on |
| `ap` | `null` | so the hall decides: `wifi.ap_by_world.production = [1.0, 2.0]`, above the loading area at the west wall. Which layer won is on `/sim/config` under `wifi.effective_ap` |
| `tx_dbm`, `d0`, `n` | `−40.0`, `1.0`, `2.4` | level at one metre, reference distance, path-loss exponent |
| `wall_db` | `12.0` | what one wall crossing costs — the middle of what is measured at 2.4 GHz through loaded shelving (a wooden table alone is 3–6 dB, a steel rack full of tools 15–20 dB) |
| `floor_dbm`, `good_dbm` | `−85.0`, `−50.0` | the two levels `quality` is scaled between |
| `shadow_db`, `shadow_period` | `3.0`, `2.0` | the slow fade: bounded, continuous, and not a random draw per message |
| `latency_ms` | `20.0` | the wire on a good link |
| `link_up_q`, `link_timeout` | `0.15`, `1.5` | the level the failsafe counts against, and how long it has to stay under it — in **simulation** seconds |
| `autonomy` | `stop` | what the robot does when the link is down: `stop` holds its place (a command watchdog), `dead_reckoning` keeps executing the last command that really arrived |
| `rate` | `5.0` | the `/<robot>/link` rate |

## What you see

- `n` — the access point as a box with two arcs, one line from it to every robot, and a bar over each
  robot: quality, the dBm behind it, how many walls the line crosses, the frames that never arrived and
  what the wire costs in ms. The line is drawn along the very straight line the budget is computed on, so
  the moment the bar drops as the line touches a rack is the same fact as "the LIDAR reports that
  rectangle" — the radio counts crossings with the LIDAR's own ray test, which is why the two cannot
  disagree about which walls exist.
- the readout line: `wifi q 0.42 -72.3 dBm 1 wall lost 4/120 43 ms`.
- `c` if you want the whole hall rather than one robot: every cell painted by the quality the budget
  answers there, green at the antenna and dark red behind the racks. Off by default — it is the model
  drawn over the floor, and the sampled number is `wifi.budget()` without the slow fade
  ([docs/wifi.md](../wifi.md)).
- when the link goes down: the robot's outline turns amber and `/sim/robots` says `mode: autonomy`.

Frame 30 of this config is `docs/img/readout-radio.png`, rebuilt by the command in
[docs/demos.md](../demos.md).

## What it measured

With the shipped numbers and the fade at zero, in `production` (AP at 1.0, 2.0):

- free floor 2 m from the AP: **−47.2 dBm, q 1.00**;
- middle of the south aisle, 9 m and nothing in between: **−62.9 dBm, q 0.63**, 14 % of the frames lost,
  35 ms instead of 20;
- far side of two racks at 15 m: **−92.2 dBm, q 0.00**, no link at all;
- holding `Up` in the south aisle: q 1.00 at the spawn, 0.63 at x = 10 m, 0.43 at 18 m — at q 0.43 one
  frame in three never arrives and every one that does is 43 ms late. The robot still drives;
- turning north in the east aisle at x = 19 m: between y = 3.69 and y = 4.58 m the line to the AP starts
  crossing the racks at (13.5, 3.5)–(17, 5.5) and (13.5, 8)–(17, 10), and q falls 0.42 → 0.08 → 0.00 in
  two steps of `wall_db` each. One rack, not two, is already below the level a robot can be driven on.

Quality is worth `p_drop = (1 − q)²` and the wire costs `latency_ms · (1 + 2(1 − q))`; the full nine-cell
table of distance against wall crossings, and the share of this hall's free floor that sits below the
drivable level, are in [docs/wifi.md](../wifi.md).

## The exercise

Four steps, ten minutes: drive the south aisle and watch quality fall with distance alone; drive north in the
east aisle and watch it fall with walls instead; stand in the shadow for 1.5 s and read the
`link to 'muster' down at … dBm` line in the log — from that moment nothing you type arrives, the keys
keep publishing, the counters keep climbing, and the robot does not move. Then repeat with
`wifi.autonomy=dead_reckoning`: the robot keeps executing the last command that really arrived, which
means it drives on into the shelf in front of it.

That last step is not about the radio. Which of the two rules a robot in your hall should ship is a
question for the practice team, and this demo is where the two can be driven against each other.

What the model does **not** do is part of the exercise too: one AP with one omnidirectional antenna, no
second AP and no roaming, no shared airtime, no multipath (attenuation only — multi-path lives in
`gps.zones`), and no retransmission, so a bad link here is lossy and slow but never out of order and
never duplicated. Model, delivery rules and the failsafe: `docs/CONTRACT.md` §6.14.

See also: [docs/wifi.md](../wifi.md) (the model and the nine-cell table), [docs/demos.md](../demos.md)
(all six demos), [demos/sensor_reality.md](sensor_reality.md) (the sensors on the other side of the same
wire).
