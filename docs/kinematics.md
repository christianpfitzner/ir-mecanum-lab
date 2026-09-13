# Experiment 1: what `./lab run` does with your node

Two commands start it:

```bash
./lab run --robot alice --controller student/controller_template.py   # window, drive yourself
./lab grade --task v1 --controller student/solution.py               # grade all of v1 at once
```

`./lab run` is the one-process form of the lab: the simulator, your node as a thread and the keyboard,
all three publishing the same `/<robot>/cmd_vel`. A node publishes every tick and the keys publish only
while one is held, so **your node drives and the keys interrupt it**. The readout line says which of the
two was last heard from — `cmd topic 0.04 s`, `cmd keys 0.02 s`, and `cmd none` once `cmd_timeout` has
passed without a frame — because "the keyboard is broken" is nearly always a node that publishes every
tick, and the way to tell the two apart in one glance is to write down who drove.

## On `kinematik` the robot stands still, and that is correct

For task `""` or `kinematik` the runner behind `serve()` never calls `mission()`. It reads every
`cmd_vel` that arrives, pushes it through your `inverse_kinematics()` and publishes the four wheel
speeds — because T1 is graded by the grader sending commands blind and measuring what the wheels do. So
hold an arrow key to watch the conversion: a key *is* a `cmd_vel`, and it is your IK that turns it into
four numbers.

Missions start at T2. `--task quadrat` runs `mission()` once and the robot drives the square by itself;
`serve()` is then the emergency exit — it still answers every `cmd_vel`, so you can take over while the
mission is running.

| what you see | what it means |
|---|---|
| robot stands still on `kinematik` | nothing has sent a `cmd_vel` yet — press a key or send one |
| readout says `cmd keys 0.02 s` | the keyboard drove the last tick |
| readout says `cmd topic 0.04 s` | your node drove the last tick; the keys still work while held |
| readout says `cmd none` | nothing arrived within `cmd_timeout` — the robot is stopped, by design |
| `steering robot cannot strafe, vy=0.25 dropped` | that chassis has two axes, not three (docs/steering.md) |

The four graded tasks of the experiment, their budgets and what each one is examined with:
`./lab docs` prints them, `config/tasks.json` holds them, and the handout in `docs/praktikum/` explains
what is handed in for each. Interface — topics, units, signs, the wheel geometry: `docs/CONTRACT.md`
§5 and §6.
