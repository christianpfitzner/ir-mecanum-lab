# One access point per hall: the radio link `/<robot>/link`

With `wifi.enabled` true, `/cmd_vel` and `/wheels` arrive by radio through one access point per hall.
What kills a real link kills it here: **distance and walls in the straight line to the access point**.
The model is one line of arithmetic (`mecanum_lab/wifi.py`, stdlib `math`, no WiFi stack):

```
rssi = tx_dbm − 10 · n · log10(d / d0) − (walls crossed) · wall_db + shadow      [dBm]
q    = clamp((rssi − floor_dbm) / (good_dbm − floor_dbm), 0, 1)
```

Wall crossings reuse the LIDAR's own ray test. Measured with the shipped numbers (`tx_dbm −40`,
`n 2.4`, `wall_db 12`, floor −85 dBm, good −50 dBm) in `production`, whose access point hangs at
(1.0, 2.0):

| distance | free floor | 1 wall | 2 walls |
|---|---|---|---|
| 2 m | −47.2 dBm · q 1.00 | −59.2 · q 0.74 | −71.2 · q 0.39 |
| 8 m | −61.7 · q 0.67 | −73.7 · q 0.32 | −85.7 · q 0.00 |
| 15 m | −68.2 · q 0.48 | −80.2 · q 0.14 | −92.2 · q 0.00 |

Six of these nine cells are open floor in `production` and measurable by driving there; `tests/test_wifi_w6.py`
recomputes all nine from the shipped config. Quality is worth `p_drop = (1 − q)²`: q 0.90 loses a frame
in 100, q 0.50 one in four, q 0.32 one in 2.2. The wire costs `latency_ms · (1 + 2(1 − q))` — 20 ms on a
good link, 47 ms at q 0.32. **Walls, not metres, kill a link in a furnished hall**: one rack between you
and the AP at 8 m is worse than open floor at 15 m, and 23 % of this hall's free floor is below the level
a robot can be driven on.

Below `wifi.link_up_q` (0.15) a countdown runs; `wifi.link_timeout` (1.5 s of **simulation** time) under
it and the link is down — nothing external arrives, the outline turns amber, `/sim/robots` says
`mode: autonomy`. `wifi.autonomy` picks the behaviour: `stop` holds the place it was left at,
`dead_reckoning` keeps executing the last command that really arrived, wall in the way or not.

`/<robot>/link` comes out at `wifi.rate` (5 Hz) with `t, quality, rssi_dbm, ap, up, dropped,
latency_ms` — `ap` on purpose, so a controller can drive back into coverage before the failsafe fires.
Layer `n` draws the access point, the line to each robot on the very segment the budget is computed on,
and a quality bar over each; the readout prints `wifi q 0.42 -72.3 dBm 1 wall lost 4/120 43 ms`. The
access point belongs to the hall (`wifi.ap_by_world`), `--set wifi.ap=[10,6]`
moves it for one run, and `wifi.effective_ap` on `/sim/config` names whichever layer won.

The example node leaves the covered corner on its own once the RSSI falls under its limit:

```bash
ros2 launch mecanum_lab demo_wifi.launch.py
```

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_wifi.json \
        controller:=student/link_autonomy_example.py seconds:=60
```

Every key of `config/demo_wifi.json` is a launcher argument for one run, no editing (`--show-args` lists
them; without ROS 2: `./lab sim --config config/demo_wifi.json --set wifi.wall_db=20`):

```bash
ros2 launch mecanum_lab wifi.launch.py wall_db:=20 ap:=[10,6]
```

Off means off: with `wifi.enabled` false — the default in `mecanum_lab/types.py` — no radio is built, no
random number is drawn for it and no `/link` is published, so a graded run's command stream and CSV are
byte for byte what they were before a radio existed (`tests/test_wifi_w6.py` compares two recorded
files). The exercise and the scope the model refuses — no second access point, no roaming, no shared
airtime, no multipath, no retransmission — stand in `config/demo_wifi.json`; the rules are in
`docs/CONTRACT.md` §6.14.

## The whole hall at once: layer `c`

One robot's bar answers "does *this* one hear me". Layer `c` (`m` in the window, or `--layers coverage`)
answers "would a robot over *there* still hear me": every cell is sampled with the same `budget()` and
painted green through amber to dark red.

```bash
./lab sim --world production --config config/demo_wifi.json --layers coverage
```

Measured with the shipped config (AP at 1, 2 in a 20 × 12 m hall): at the antenna `1.00`, at 8 m on the
open floor `0.67`, at 8 m with one rack in the line `0.32`, `0.00` behind two racks — the term a distance
picture hides, `k · wall_db`. Off by default, and sampled with `shadow_db = 0`: a cached map must not
pretend to follow a corridor as it fills. The live number is the bar and the `/link` line.
