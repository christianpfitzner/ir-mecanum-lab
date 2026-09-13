# One access point per hall: the radio link `/<robot>/link`

Every signal level and quality number below is recomputed from the shipped config by
`tests/test_wifi_w6.py`, cell by cell — so a table on this page cannot go stale while the code stays
honest, and the six spots that the furniture does offer are the ones a reader can drive to.

Every command your robot receives arrives by radio in this simulator too — with `wifi.enabled` true,
`/cmd_vel` and `/wheels` go through one access point per hall, and the two things that kill a real
link kill it here: **distance and walls in the straight line to that access point**. The model is one
line of arithmetic (`mecanum_lab/wifi.py`, stdlib `math`, no WiFi stack and no wish to have one):

```
rssi = tx_dbm − 10 · n · log10(d / d0) − (walls crossed) · wall_db + shadow      [dBm]
q    = clamp((rssi − floor_dbm) / (good_dbm − floor_dbm), 0, 1)
```

Wall crossings use the LIDAR's own ray test, so a beam and a radio wave cannot disagree about which
rectangles exist. Measured with the shipped numbers (`tx_dbm −40`, `n 2.4`, `wall_db 12`, floor
−85 dBm, good −50 dBm) at spots in `production`, whose access point hangs at (1.0, 2.0):

| distance | free floor | 1 wall | 2 walls |
|---|---|---|---|
| 2 m | −47.2 dBm · q 1.00 | −59.2 · q 0.74 | −71.2 · q 0.39 |
| 8 m | −61.7 · q 0.67 | −73.7 · q 0.32 | −85.7 · q 0.00 |
| 15 m | −68.2 · q 0.48 | −80.2 · q 0.14 | −92.2 · q 0.00 |

Six of these nine cells are open floor in `production` and measurable by driving there; the six spots
and the three that the furniture does not offer are all asserted in `tests/test_wifi_w6.py`. What
quality is worth: `p_drop = (1 − q)²` — q 0.90 loses a frame in 100, q 0.50 one in four, q 0.32 one
in 2.2 — and the wire costs `latency_ms · (1 + 2(1 − q))`, so 20 ms on a good link and 47 ms at
q 0.32. **Walls, not metres, are what kills a link in a furnished hall**: one rack between you and
the AP at 8 m is worse than open floor at 15 m, and 23 % of this hall's free floor is below the level
a robot can be driven on.

Below `wifi.link_up_q` (0.15) a countdown runs; if the level stays under it for `wifi.link_timeout`
(1.5 s of **simulation** time), the link is down: nothing external arrives any more, the robot's
outline turns amber, `/sim/robots` says `mode: autonomy`, and `wifi.autonomy` decides what it does
then — `stop` holds the place it was left at (what a command watchdog is), `dead_reckoning` keeps
executing the last command that really arrived (what "the robot walks itself home" means, wall in the
way or not). Which of the two a practice robot should ship is a question for the practice team, and
the run below lets you drive both.

`/<robot>/link` comes out at `wifi.rate` (5 Hz) with `t, quality, rssi_dbm, ap, up, dropped,
latency_ms` — `ap` included on purpose, so a controller can drive back into coverage *before* the
failsafe fires. Layer `n` draws the access point, the line to each robot on the very segment the
budget is computed on and a quality bar over each; the readout line prints
`wifi q 0.42 -72.3 dBm 1 wall lost 4/120 43 ms`. The access point belongs to the hall
(`wifi.ap_by_world`), and `--set wifi.ap=[10,6]` moves it for one run — `wifi.effective_ap` on
`/sim/config` names whichever layer won, so nobody has to guess.

```bash
./lab sim --world production --config config/demo_wifi.json          # drive it, look at layer n
./lab run --world production --config config/demo_wifi.json --robot muster \
        --controller student/link_autonomy_example.py --seconds 60    # it drives into the shadow
./lab sim --world production --config config/demo_wifi.json --set wifi.autonomy=dead_reckoning
ros2 launch launch/wifi.launch.py wall_db:=20 ap:=[10,6]             # the same knobs as --set
```

`wifi.enabled` is false in `mecanum_lab/types.py`, and that is not a hint but a guarantee: with the
option off no radio is built, no random number is drawn for it and no `/link` is published, so the
command stream and the recorded CSV of a graded run are byte for byte what they were before a radio
existed (`tests/test_wifi_w6.py` compares two recorded files). The exercise, the four numbers to
check with a tape measure and what this model deliberately does **not** do — no second access point,
no roaming, no shared airtime, no multipath and no retransmission — is spelled out line by line in
`config/demo_wifi.json`. Model and delivery rules: `docs/CONTRACT.md` §6.14.

## The whole hall at once: layer `c`

One robot's bar answers "does *this* one hear me". Layer `c` (`m` in the window, or
`--layers coverage`) answers "would a robot over *there* still hear me": every cell of the world is
sampled with the same `budget()` and painted green through amber to dark red.

```bash
./lab sim --world production --config config/demo_wifi.json --layers coverage
```

Measured with the shipped config (AP at 1, 2 in a 20 × 12 m hall): at the antenna `1.00`, at 8 m on the
open floor `0.67`, at 8 m with one rack in the line `0.32`, and `0.00` in the corners behind two racks —
so the map shows the term that a distance picture would hide, `k · wall_db`.

It is off in every default view because it is a picture of a *model* lying over the floor, it is drawn
under the racks that cast the shadows, and it is sampled with `shadow_db = 0`: the slow fade moves a few
dB as a corridor fills, and a map that is cached for the run must not pretend to be current. The live
number, fade included, is the bar and the `/link` line.
