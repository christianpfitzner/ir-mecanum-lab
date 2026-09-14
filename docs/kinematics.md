# Experiment 1: how your node gets into the loop

Your node and the simulator, one command:

```bash
ros2 launch mecanum_lab lab.launch.py controller:=student/controller_template.py
```

Graded, the four tasks of the experiment at once:

```bash
./lab grade --task alle --controller student/solution.py
```

`./lab run` is the same lab in one process, without ROS 2:
`./lab run --robot alice --controller student/controller_template.py`. Simulator, your node as a thread
and the keyboard publish the same `/<robot>/cmd_vel`. A node publishes every tick, the keys only while
held, so **your node drives and the keys interrupt it** — and the readout says who drove last:
`cmd topic 0.04 s`, `cmd keys 0.02 s`, `cmd none` once `cmd_timeout` passes without a frame. "The
keyboard is broken" is nearly always a node that publishes every tick.

## On `kinematik` the robot stands still, and that is correct

For task `""` or `kinematik` the runner behind `serve()` never calls `mission()`. It reads every `cmd_vel`
that arrives, pushes it through your `inverse_kinematics()` and publishes the four wheel speeds — T1 is
graded by a grader that sends commands blind and measures what the wheels do. So hold an arrow key and
watch the conversion: a key *is* a `cmd_vel`, and your IK turns it into four numbers.

Missions start at T2. `--task quadrat` runs `mission()` once and the robot drives the square alone;
`serve()` stays the emergency exit, answering every `cmd_vel`, so you can take over mid-mission.

| what you see | what it means |
|---|---|
| robot stands still on `kinematik` | nothing has sent a `cmd_vel` yet — press a key or send one |
| readout says `cmd keys 0.02 s` | the keyboard drove the last tick |
| readout says `cmd topic 0.04 s` | your node drove the last tick; the keys still work while held |
| readout says `cmd none` | nothing arrived within `cmd_timeout` — the robot is stopped, by design |
| `steering robot cannot strafe, vy=0.25 dropped` | that chassis has two axes, not three (docs/steering.md) |

`./lab docs` prints the four graded tasks and their budgets, `config/tasks.json` holds them, the handout
in `docs/praktikum/` says what is handed in. Interface — topics, units, signs, wheel geometry:
`docs/CONTRACT.md` §5 and §6.
