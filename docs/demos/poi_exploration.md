# poi_exploration — one number, and a source the robot cannot see

A Point of Interest is a thing to find that no other sensor reports: `pois` plants a radiation source in
the world, `pois.py` gives it a field, and the robot gets one counter reading per `poi.rate`. The hall is
`open` — 30 × 20 m, border wall, nothing else — so the first exercise is about the reading and not about
the map.

```bash
./lab sim --world open --config config/demo_poi_exploration.json        # drive it yourself
./lab sim --world production --config config/demo_poi_exploration.json  # the same source, with tables
./lab run --world open --config config/demo_poi_exploration.json \
          --controller student/poi_seek_example.py --seconds 100 --log poi.csv   # a controller does it
ros2 launch mecanum_lab demo_poi_exploration.launch.py                   # same through ROS, plus RViz
```

From the other side the reading is one topic: `ros2 topic echo /alice/poi`.

## What it changes

| key | value | what it is |
|---|---|---|
| `world` | `open` | nothing between the robot and the source but floor |
| `pois` | one entry: `src1`, `kind: radiation`, at `(12.0, 6.0)`, `activity: 1.0`, `range: 4.0` | the source. A source placed outside the walls is refused at load time, not quietly dropped |
| `poi.rate` | `5.0` | readings per second on `/<robot>/poi` |
| `poi.d0` | `1.0` | the distance at which the counter reads half the activity, so it sets how fast the field falls |
| `poi.counts` | `400.0` | counts per unit intensity per reading — the Poisson noise, and with it the σ of a single reading |

In the defaults `pois` is empty: no detector is built, no message is published, no random number is drawn
in any graded run.

## What you see

- `p` — the source as a symbol rather than as an obstacle, because it is not one. With `--truth` the layer
  also draws the rings where the reading is half and a tenth of the activity.
- the readout line: `poi src1 0.803` — the same fact as the topic, without a second terminal.
- `t` and the trail: the line you drove is the x-axis of the series you are about to read.

## What it measured

The field is `intensity = activity / (1 + (d / d0)²)` up to the source's `range`, 0 beyond it. For the
shipped source that is 1.00 m to half the activity, 3.00 m to a tenth, silence past 4.00 m. Measured with
the shipped counting noise, 4000 readings per spot:

| distance | intensity | σ of one reading | relative |
|---|---|---|---|
| 0.5 m | 0.800 | 0.045 | 5.6 % |
| 2 m | 0.200 | 0.022 | 11 % |
| 4 m (= `range`) | 0.059 | 0.012 | 21 % |
| 4.5 m | 0.000 | 0.000 | — silence |

The absolute σ falls with distance while the relative one grows as `1/sqrt(intensity)`: a counter far
from its source is a rough counter. That is what a gradient controller has to be written against.

Driving straight east from the spawn at 0.5 m/s, any seed: the reading is 0.000 until x = 8.1 m (the edge
of the 4 m range — the first reading is 0.06–0.07), peaks near x = 12 m at about 1.0, and is 0.000 again
from x = 15.9 m. In the noise-free field (`poi.counts = 0`) the reading crosses 0.50 twice, 1.90 m apart:
that is `2·d0` for this line, so the series alone says how far `d0` is and therefore where it was passed.
With the counting noise on, that second crossing lands at x = 12.9 … 13.1 depending on `--seed`, which is
the next lesson.

**The field goes through the furniture.** Nothing asks what stands between the two, because a gamma
source does not care about a shelf: standing at (10.0, 3.0) in `production`, 3.6 m from the source behind
the corner of a table, the LIDAR pointed at the source reports the table at 0.60 m while the counter
reports 0.072 ± 0.014. Two sensors, two worlds, one robot, neither of them wrong.

## The exercise

Drive east from the first spawn and log the series; repeat from the second spawn, which faces north; two
closest-approach lines cross at the source. Then hand it to a controller — `student/poi_seek_example.py`
climbs while the reading rises, arcs when it stops rising, and drives back to where it was loudest when
the field goes silent, and `intensity_poi` in `poi.csv` is what `tools/kfplot.py` plots
afterwards. The rule to take away: **do not drive by the peak value** — it says nothing about distance
unless the activity is known; drive by whether the last reading was bigger than the one before it.

Model, noise, and the validation that refuses a source outside the walls: `docs/CONTRACT.md` §6.13.

See also: [docs/poi.md](../poi.md) (the model on its own page), [docs/demos.md](../demos.md) (all six
demos, and the worked seek example), [demos/open_odrift.md](open_odrift.md) (the same hall with a wheel
error instead of a source).
