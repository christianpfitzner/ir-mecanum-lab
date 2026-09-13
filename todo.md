



UI: 
- the odom ghost trail should have a dashed line or a slightly different color
- add a right click menue to teleport a robot to a certain position (leave the same orientation)
- with pressing "shift" i want to double the velocity for the robot in the ui



README: 
- student exercises should not be in the main readme -> they will move in another repo later
- I am missing an overview of the worlds as in previous versions





BUG: 

I got the following message after the simulation broke: 

[rviz2-2] [INFO] [1789329862.271810034] [rviz]: Message Filter dropping message: frame 'muster/odom' at time 0.040 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
[mecanum_sim-1] Traceback (most recent call last):
[mecanum_sim-1]   File "<frozen runpy>", line 198, in _run_module_as_main
[mecanum_sim-1]   File "<frozen runpy>", line 88, in _run_code
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/node.py", line 852, in <module>
[mecanum_sim-1]     raise SystemExit(main())
[mecanum_sim-1]                      ^^^^^^
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/node.py", line 848, in main
[mecanum_sim-1]     return COMMANDS[command][0](args)
[mecanum_sim-1]            ^^^^^^^^^^^^^^^^^^^^^^^^^^
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/node.py", line 610, in cmd_sim
[mecanum_sim-1]     code = timed_run(args, eng, bus, graders, teleop=not args.no_teleop, tap=tap,
[mecanum_sim-1]            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/node.py", line 505, in timed_run
[mecanum_sim-1]     run_loop(eng, bus, rend, graders, args.seconds, teleop=teleop, tap=tap,
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/node.py", line 145, in run_loop
[mecanum_sim-1]     rend.draw()
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/render.py", line 376, in draw
[mecanum_sim-1]     self._hud()
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/render.py", line 702, in _hud
[mecanum_sim-1]     *overlays.poi_readout(self, r),
[mecanum_sim-1]      ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
[mecanum_sim-1]   File "/home/chris/ros2_ws/install/mecanum_lab/lib/python3.12/site-packages/mecanum_lab/overlays.py", line 277, in poi_readout
[mecanum_sim-1]     source = (getattr(rend.engine, "loudest_poi", lambda _n: None)(robot.name)
[mecanum_sim-1]                                                                    ^^^^^^^^^^
[mecanum_sim-1] AttributeError: 'Robot' object has no attribute 'name'
[ERROR] [mecanum_sim-1]: process has died [pid 22065, exit code 1, cmd '/usr/bin/python3 -m mecanum_lab.node sim --robots muster --world production --config /home/chris/ros2_ws/install/mecanum_lab/share/mecanum_lab/config/demo_poi_exploration.json'].


