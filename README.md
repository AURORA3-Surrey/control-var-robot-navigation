# robot-navigation (variable-acoustics-room)

Scripts for robot motion control, ROS 2 communication, and navigation.

## Scripts

- `vicon_yaw_calibration.py` drives a short line forward and back, saves `vicon_yaw_offset.json`.<br>
  Run it once a day before using Vicon.
- `demo/grid.py` runs on a grid using the Vicon pose. It won't start without a yaw calibration from the same day.<br>
  `--help` lists the flags to skip that.
- `odom_check.py` compares odometry with Vicon while you move the robot by hand
- `vicon_to_odom.py` republishes the Vicon pose as `nav_msgs/Odometry`. 
  It also needs a yaw calibration from the same day.
- `demo/grid_loop.sh` loops `demo/grid.py` for demos.
- `vicon_goto.py` moves the robot to points given in Vicon coordinates (`--point x y [yaw_deg]`) after checking them against `room.json`.<br>
  Needs a yaw calibration from the same day.
- `room.json` has room limits in Vicon coordinates and the speeds for sessions.<br>
  A keep-out box is `[x0, y0, x1, y1]`, two opposite corners in metres. To measure one, push the robot against the
  obstacle, use `ros2 topic echo --once --field pose.position /vicon/Dingo/Dingo` and add 0.259 m (side) 0.343 m (front).
- `check_session.py` checks a session file (csv of points, `label,x,y,yaw_deg,wait_s`) against `room.json`.
  Format described at beginning of `session.py`.
- `run_session.py` runs a session file and writes a results csv with the Vicon pose reached at each point.<br>
- `gatekeeper.py` lets people on the local network send sessions to the robot over HTTP.<br>
  Starts `run_session.py`, results and log in `aurora3_sessions`.
- `aurora3_client.py` Python client for the gatekeeper. `examples/` has a session template.
- `room_gui.py` top-down room view, click to send the robot to a specific point.<br>
  Runs `vicon_goto.py` on the robot over ssh (key: `ssh-copy-id robot@10.78.115.148`), with the robot's
  `room.json` and yaw calibration. Copy `vicon_yaw_offset.json` from the robot for the drawn heading.
- `environment.yml` conda environment to run the GUI and ROS scripts off the robot.
- `emergencystop.py` sends zero velocity and turns the motors off.
- `vicon/connection_check.py` checks the Vicon connection.

## Run preparations and conventions

- Vicon PC: on eduroam with `10.78.113.197`, Tracker in Live mode with `Dingo` tracked, then
  `vicon/run_vicon_bridge.bat`.<br>
  The bridge loads `/home/aurora3var/vicon_receiver_ws/fastdds_vicon_pc.xml` in WSL. If the address changes,
  change it there and on the robot.
- Robot: `~/.bashrc` sets `ROS_STATIC_PEERS="10.78.113.197"` (Polaris) after the Clearpath line.
- Room frame: origin at the centre, `x` east (4.91 m), `y` north along the long side (7.97 m),
  yaw counter-clockwise from `x`.

## Structure

```
demo/grid.py ──► motion_node.py ──┬─► motion_commands.py   Wait, Turn, MoveTo,
                                  │                        PD TrackingCorrection
vicon_to_odom.py ─────────────────┤
vicon_yaw_calibration.py ─────────┴─► nav_utils.py        angle maths, yaw-offset file,
                                                          PoseFusion, TwistSender (accel limit),
                                                          shutdown helpers
```

`motion_node.py` holds `MotionNode` and `run_motion()`. The node follows the odometry (smooth pose and speed) and `PoseFusion` in
`nav_utils.py` pulls it towards the Vicon pose (with the yaw offset). It aborts if the odometry is stale (0.5 s) or no Vicon sample was accepted for 3 s.
`demo/grid.py` contains the grid planner (`build_grid_plan`).

To add a new motion, add a `Command` subclass in `motion_commands.py` with a `step(node, now)` method
and queue it in the plan builder (`build_grid_plan()` in `demo/grid.py`).

## Robot-side code

Copy everything. `vicon_yaw_offset.json` is written next to the scripts by `vicon_yaw_calibration.py` and read from there by `demo/grid.py` and `vicon_to_odom.py`.