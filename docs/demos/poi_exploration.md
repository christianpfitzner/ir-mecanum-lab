# poi_exploration — one number, and a source the robot cannot see

A Point of Interest is a thing no other sensor reports. `pois` plants a radiation source in the world,
`pois.py` gives it a field, the robot gets one counter reading per `poi.rate`. The hall is `open`: the first
exercise is about the reading, not about the map.

## Start

```bash
ros2 launch mecanum_lab demo_poi_exploration.launch.py
```

The same source with tables in the way:

```bash
ros2 launch mecanum_lab demo_poi_exploration.launch.py world:=production
```

A controller instead of a hand, 100 s, the series written out:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_poi_exploration.json \
    controller:=student/poi_seek_example.py seconds:=100 log:=poi.csv
```

Without ROS 2: `./lab sim --config config/demo_poi_exploration.json`, with `--world production`,
`--controller … --seconds 100 --log poi.csv` behind. From the robot's side the reading is one topic:
`ros2 topic echo /alice/poi`.

## What it changes

| key | value | note |
|---|---|---|
| `world` | `open` | nothing between the robot and the source but floor |
| `pois` | `src1`, radiation, at `(12.0, 6.0)`, `activity: 1.0`, `range: 4.0` | a source outside the walls is refused at load time, not quietly dropped |
| `poi.rate` | `5.0` | readings per second on `/<robot>/poi` |
| `poi.d0` | `1.0` | the distance at which the counter reads half the activity |
| `poi.counts` | `400.0` | counts per unit intensity per reading: the Poisson noise, and the σ of one reading |

In the defaults `pois` is empty — no detector, no message, no random number in a graded run. Model and
validation: `docs/CONTRACT.md` §6.13, [docs/poi.md](../poi.md).

## In the window

* `p` — the source as a symbol, not as an obstacle, because it is not one. With `--truth` the layer also
  draws the rings where the reading is half and a tenth of the activity.
* the readout line: `poi src1 0.803` — the same fact as the topic, without a second terminal.

## Measured

The field is `intensity = activity / (1 + (d / d0)²)` up to `range`, 0 beyond it, so the shipped source
gives half the activity at 1.00 m, a tenth at 3.00 m and silence past 4.00 m. 4000 readings per spot:

| distance | intensity | σ of one reading | relative |
|---|---|---|---|
| 0.5 m | 0.800 | 0.045 | 5.6 % |
| 2 m | 0.200 | 0.022 | 11 % |
| 4 m (= `range`) | 0.059 | 0.012 | 21 % |
| 4.5 m | 0.000 | 0.000 | — silence |

The absolute σ falls with distance, the relative one grows as `1/sqrt(intensity)`: far from its source a
counter is a rough counter, and a gradient controller has to be written against that.

Straight east at 0.5 m/s, any seed: 0.000 until x = 8.1 m — the edge of the 4 m range — first reading
0.06–0.07, a peak near x = 12 m at about 1.0, 0.000 again from x = 15.9 m. Without counting noise
(`poi.counts = 0`) the reading crosses 0.50 twice, 1.90 m apart: that is `2·d0`, so the series alone says
how far `d0` is and where it was passed. With noise the second crossing lands at x = 12.9 … 13.1, depending
on `--seed`.

**The field goes through the furniture** — a gamma source does not care about a shelf. At (10.0, 3.0) in
`production`, 3.6 m from the source behind the corner of a table, the LIDAR pointed at the source reports
the table at 0.60 m while the counter reports 0.072 ± 0.014.

## The exercise

Drive east from the first spawn and log the series; repeat from the second, which faces north. Two
closest-approach lines cross at the source. Then hand it to `student/poi_seek_example.py`, which climbs
while the reading rises and drives back to where it was loudest once the field is silent; `intensity_poi` in
`poi.csv` is what `tools/kfplot.py` plots afterwards.

The rule to take away: **do not drive by the peak value** — it says nothing about distance unless the
activity is known. Drive by whether the last reading was bigger than the one before it.

Also: [docs/poi.md](../poi.md) (the model on its own page), [docs/demos.md](../demos.md),
[demos/open_odrift.md](open_odrift.md) (the same hall with a wheel error instead of a source).
