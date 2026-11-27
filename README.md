# robot-navigation (variable-acoustics-room)

Scripts for robot motion control, ROS 2 communication, and navigation.

## Scripts

- `vicon_yaw_calibration.py` drives a short straight line and saves `vicon_yaw_offset.json`.<br>
  Run it once a day before using Vicon.
- `demo/grid.py` runs on a grid using the Vicon pose. It won't start without a yaw calibration from the same day.<br>
  `--help` lists the flags to skip that.
- `odom_check.py` compares odometry with Vicon while you move the robot by hand
- `vicon_to_odom.py` republishes the Vicon pose as `nav_msgs/Odometry`. 
  It also needs a yaw calibration from the same day.
- `demo/grid_loop.sh` loops `demo/grid.py` for demos.
- `emergencystop.py` sends zero velocity and turns the motors off.
- `vicon/connection_check.py` checks the Vicon connection.

## Structure

```
demo/grid.py ──► motion_node.py ──┬─► motion_commands.py   Wait, Turn, MoveTo,
                                  │                        PD TrackingCorrection
vicon_to_odom.py ─────────────────┤
vicon_yaw_calibration.py ─────────┴─► nav_utils.py        angle maths, yaw-offset file,
                                                          TwistSender (accel limit), shutdown helpers
```

`motion_node.py` holds `MotionNode` (Vicon pose with the yaw offset and a filtered velocity) and `run_motion()`.
`demo/grid.py` contains the grid planner (`build_grid_plan`).

To add a new motion, add a `Command` subclass in `motion_commands.py` with a `step(node, now)` method
and queue it in the plan builder (`build_grid_plan()` in `demo/grid.py`).

## Robot-side code

Copy everything (the scripts import each other).

`vicon_yaw_offset.json` is written next to the scripts by `vicon_yaw_calibration.py` and read from there by `demo/grid.py` and `vicon_to_odom.py`.