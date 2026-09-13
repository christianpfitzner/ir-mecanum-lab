# A radiation source: `/<robot>/poi`

One number per reading, an inverse-square field that goes through the furniture, and a distance the
message does not give you. The shipped scenario is `config/demo_poi_exploration.json`.

`pois` plants a Point of Interest in a world and `mecanum_lab/pois.py` gives it a field:
`intensity = activity / (1 + (d/d0)²)` up to the source's `range`, 0 beyond it, with counting noise
on top (`poi.counts` per unit, so the sigma of one reading is `sqrt(counts)/counts` — the reading is
rough where the field is weak). The robot gets `/<robot>/poi` at `poi.rate` (5 Hz) with a stamp and
that one number, and the readout line prints `poi src1 0.803` so the sensor is observable without a
second terminal. Measured for the shipped source, 4000 readings per spot:

| distance | intensity | σ of one reading | relative |
|---|---|---|---|
| 0.5 m | 0.800 | 0.045 | 5.6 % |
| 2 m | 0.200 | 0.022 | 11 % |
| 4 m (= `range`) | 0.059 | 0.012 | 21 % |
| 4.5 m | 0.000 | 0.000 | — silence |

**The field goes through the furniture.** Nothing in the model asks what stands between the robot and
the source, because a gamma source does not care about a shelf: in `production` the LIDAR pointed at
the source reports the table in front of it at 0.60 m while the counter, 3.6 m away, still reports
0.072. Two sensors that disagree because one of them needs a straight line and the other does not —
and neither of them is wrong. That is why the layer `p` draws the source as a symbol rather than as
an obstacle, and with `debug_truth` the rings where the reading is half and a tenth of the activity.

Which source is loudest, and how far away it is, are **not** in the message — a wide-band counter
cannot tell either, and turning an intensity series into a distance is the exercise. Both stay
simulation truth that the tutor can see: the window asks `SimEngine.loudest_poi()` for the name, the
CSV writes the same answer, and `debug_truth` draws the rings. There is no switch that would put the
metres on the bus; there used to be one, and it was the exercise with an off button. `pois` is empty
in the defaults, so no detector is built, no message is published and no random number is drawn in any
graded run.

```bash
./lab sim --world open --config config/demo_poi_exploration.json         # drive past it, watch `p`
./lab sim --world production --config config/demo_poi_exploration.json   # same source, tables between
./lab sim --world open --config config/demo_poi_exploration.json --truth # + the field rings
```

Model, validation (a source outside the walls is refused, not dropped) and the noise model:
`docs/CONTRACT.md` §6.13.

Model, validation (a source outside the walls is refused, not dropped) and the noise model: `docs/CONTRACT.md` §6.13.
