# wifi — every command arrives by radio, and the radio has a range

With `wifi.enabled` true, `/cmd_vel` and `/wheels` go through one access point per hall. Two things kill
the link: distance, and walls in the straight line to the antenna. Off is the default in
`mecanum_lab/types.py`, and off means no radio is built, no random number is drawn and no `/link` is
published — a graded command stream is byte for byte what it was before a radio existed.

## Start

Drive it, and look at layer `n`:

```bash
ros2 launch mecanum_lab demo_wifi.launch.py
```

Or let the shipped example drive: away from the access point until the link gives out, 60 s of it.

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_wifi.json \
    controller:=student/link_autonomy_example.py seconds:=60
```

`wifi.launch.py` is a launcher of its own, where every knob below is an argument:

```bash
ros2 launch mecanum_lab wifi.launch.py wall_db:=20 ap:=[10,6] autonomy:=dead_reckoning
```

Without ROS 2: `./lab sim --config config/demo_wifi.json`. The link is also a topic:
`ros2 topic echo /alice/link`.

## What it changes

`world: production` plus the `wifi` block. What the keys mean is written once, in
[docs/wifi.md](../wifi.md) and `docs/CONTRACT.md` §6.14; the shipped values:

| key | value | note |
|---|---|---|
| `enabled` | `true` | the one switch |
| `ap` | `null` | the hall decides: `wifi.ap_by_world.production = [1.0, 2.0]`, above the loading area at the west wall |
| `tx_dbm`, `d0`, `n` | `−40.0`, `1.0`, `2.4` | level at one metre, reference distance, path-loss exponent |
| `wall_db` | `12.0` | one wall crossing. A wooden table alone measures 3–6 dB at 2.4 GHz, a steel rack full of tools 15–20 dB |
| `floor_dbm`, `good_dbm` | `−85.0`, `−50.0` | the two levels `quality` is scaled between |
| `shadow_db`, `shadow_period` | `3.0`, `2.0` | the slow fade: bounded, continuous, not a draw per message |
| `latency_ms`, `rate` | `20.0`, `5.0` | the wire on a good link, and the `/<robot>/link` rate |
| `link_up_q`, `link_timeout` | `0.15`, `1.5` | the failsafe, and the timeout counts **simulation** seconds |
| `autonomy` | `stop` | `stop` holds the robot on a command watchdog, `dead_reckoning` keeps executing the last command that arrived |

## In the window

* `n` — the access point, a line from it to every robot, and a bar over each robot: quality, dBm, walls
  crossed, frames lost, wire time. The line is the same straight line the budget is computed on, and the
  radio counts crossings with the LIDAR's own ray test, so bar and scan cannot disagree about the walls.
* the readout line: `wifi q 0.42 -72.3 dBm 1 wall lost 4/120 43 ms`.
* `c` — the whole hall painted by the quality at each cell: green at the antenna, dark red behind the racks.
  Off by default, because it is the model drawn over the floor.
* when the link goes down the outline turns amber and `/sim/robots` says `mode: autonomy`.

Frame 30 of this config is `docs/img/readout-radio.png`, rebuilt by the command in
[docs/demos.md](../demos.md).

## Measured

With the shipped numbers and the fade at zero, in `production`:

| spot | level | quality | frames lost | wire |
|---|---|---|---|---|
| free floor, 2 m from the AP | −47.2 dBm | 1.00 | none | 20 ms |
| south aisle at x ≈ 10 m, 9 m and nothing in between | −62.9 dBm | 0.63 | 14 % | 35 ms |
| south aisle at x = 18 m | — | 0.43 | one in three | 43 ms |
| far side of two racks at 15 m | −92.2 dBm | 0.00 | no link | — |
| east aisle at x = 19 m, between y = 3.69 and y = 4.58 m, where the line starts crossing the racks at (13.5, 3.5)–(17, 5.5) and (13.5, 8)–(17, 10) | two steps of `wall_db` | 0.42 → 0.08 → 0.00 | — | — |

One rack is already below the level a robot can be driven on. At q 0.43 the robot still drives, so a link
problem is not visible from the drive alone.

Quality is worth `p_drop = (1 − q)²`, the wire costs `latency_ms · (1 + 2(1 − q))`. The nine-cell table of
distance against wall crossings, and the share of this hall's floor below the drivable level, are in
[docs/wifi.md](../wifi.md).

## The exercise

Drive the south aisle and quality falls with distance alone; drive north in the east aisle and it falls
with walls instead. Stand in the shadow for 1.5 s and read `link to 'muster' down at … dBm` in the log:
from that moment nothing you type arrives, the keys keep publishing, the counters keep climbing, the robot
does not move. Then set `wifi.autonomy=dead_reckoning` and drive the same line: the robot keeps executing
the last command that really arrived and drives into the shelf in front of it. Which of those two rules a
hall should ship is a question for the practice team, and this is where they can be driven against each
other.

The model leaves out as much: one access point, one omnidirectional antenna, no roaming, no shared airtime,
no multipath, no retransmission. A bad link here is lossy and slow, never out of order and never duplicated.
Model, delivery rules, failsafe: `docs/CONTRACT.md` §6.14.

Also: [docs/wifi.md](../wifi.md), [docs/demos.md](../demos.md),
[demos/sensor_reality.md](sensor_reality.md).
