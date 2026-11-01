# linebot — black line follower for ROS 2 Jazzy + Gazebo Harmonic

Differential-drive robot with a downward-looking camera. OpenCV finds the black line,
a PID turns the lateral error into steering, speed is scheduled on how sharp the curve ahead is.

```
 Gazebo camera ──► /camera/image_raw ──► line_detector ──► /line/state ──► line_follower (PID) ──► /cmd_vel ──► DiffDrive
                                              │                                   │
                                     /line/debug_image                        /pid/debug
```

## Packages (`src/`)

| package | type | what's in it |
|---|---|---|
| `linebot_interfaces` | ament_cmake | `LineState.msg` (detected, error, near_error, far_error, slices_detected) |
| `linebot_description` | ament_cmake | `linebot.urdf.xacro` (reference bot) + `linebot_gz.xacro` (DiffDrive, JointStatePublisher, camera sensor as a reusable macro) |
| `linebot_control` | ament_python | `vision.py` (OpenCV tracker), `pid.py`, `control_law.py`, `line_detector` + `line_follower` nodes, params, launch, unit tests |
| `linebot_gazebo` | ament_cmake | 6 ready-made track worlds, track generator, ros_gz_bridge config, `sim.launch.py`, `bringup.launch.py` |

`vision.py`, `pid.py` and `control_law.py` have no ROS imports, so they are unit-tested and
reused by `tools/offline_sim.py`.

## Install & build

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-cv-bridge python3-opencv \
                 ros-jazzy-rqt-image-view ros-jazzy-teleop-twist-keyboard ros-jazzy-plotjuggler-ros
cd ~/linebot_ws            # this folder
rosdep install --from-paths src -yi
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch linebot_gazebo bringup.launch.py track:=oval        # sim + detector + PID
# tracks: straight circle oval s_curve square_loop figure_eight

ros2 launch linebot_gazebo bringup.launch.py follow:=false      # sim only, drive by hand
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run rqt_image_view rqt_image_view /line/debug_image        # what the detector sees
ros2 run plotjuggler plotjuggler                                # plot /pid/debug
ros2 service call /line_follower/enable std_srvs/srv/SetBool "{data: false}"   # pause / resume
```

Headless / no display: `gui:=false headless:=true`.

## Using YOUR URDF

Either replace `linebot_description/urdf/linebot.urdf.xacro`, or keep yours and add at the end:

```xml
<xacro:include filename="$(find linebot_description)/urdf/linebot_gz.xacro"/>
<xacro:linebot_gz left_joint="<your left joint>" right_joint="<your right joint>"
                  wheel_separation="<m>" wheel_radius="<m>" camera_link="<your camera link>"/>
```

Requirements: root link called `base_link` (the DiffDrive plugin publishes `odom -> base_link`),
wheel joints are `continuous` and **positive rotation = forward**, camera link has x forward / z up.
The camera must look at the floor ahead of the robot with the line visible in the lower part of the
image; if your view differs, change `roi_top`, `roi_bottom` and the `slice_weights` in
`linebot_control/config/follower_params.yaml`. Rename the model/joint names in
`config/bridge.yaml` only if you change the topic names in the xacro.

## Using YOUR test paths

* **Gazebo world (.sdf):** `ros2 launch linebot_gazebo bringup.launch.py world:=/abs/path/w.sdf x:=0 y:=0 yaw:=0`
  The world must contain the Physics, UserCommands, SceneBroadcaster and **Sensors (ogre2)**
  system plugins (copy them from any generated world) and be named `line_track`
  (or change `WORLD_NAME` in `sim.launch.py`). Use a light floor and a dark, matte line.
* **Waypoints (CSV, x,y in metres):**
  `python3 src/linebot_gazebo/scripts/generate_track.py --name mytrack --csv path.csv [--closed] [--smooth 2] [--scale 1.0] [--start X Y YAW]`
  then rebuild and `track:=mytrack`.
* **Image (PNG of black line on white):**
  `... generate_track.py --name pngtrack --image track.png --image-width 6 --start 0 0 0`
  (textured floor; see "Not verified" below).

## Tuning

Everything is live: `ros2 param set /line_follower kp 3.0` (doubles need the `.0`),
`ros2 param set /line_detector threshold 100`.

1. **Vision first.** Open `/line/debug_image`. Line pixels should be solid red and nothing else.
   Adjust `threshold` (or `threshold_mode: otsu|adaptive` for uneven lighting). Green dots must sit on the line.
2. **P:** start with `ki=0, kd=0`, low speed. Raise `kp` until it follows but starts to weave, back off ~30 %.
3. **D:** add `kd` (0.1–0.5) to kill the weave. Raise `d_filter` toward 0.9 if the steering looks jittery.
4. **I:** only if it runs with a steady offset (e.g. wheels with different friction): small `ki` (0.01–0.1).
5. **Speed:** raise `max_speed`; `slow_gain` controls how hard it brakes for curves. If it overshoots
   sharp corners, increase `slow_gain` or the weight of the far slices; if it cuts corners, reduce the far weight.

Lost line: coasts `lost_hold_time`, spins toward the side the line was last seen, and
stops after `lost_stop_after` seconds.

`/pid/debug` = `[error, p, i, d, angular_z, linear_x]`.

## Offline check (no ROS / Gazebo needed)

```bash
python3 tools/offline_sim.py --all                 # renders the camera view, runs real vision+PID
python3 tools/offline_sim.py --track s_curve --kp 3 --max-speed 0.3 --save /tmp/run.png
cd src/linebot_control && python3 -m unittest discover -s test -v
```

## Troubleshooting

* **No image / black image:** check `gz topic -l | grep camera`; needs the Sensors system + a working GL.
  In VMs/WSL try `LIBGL_ALWAYS_SOFTWARE=1` or `headless:=true`.
* **Robot doesn't move:** `ros2 topic echo /cmd_vel`, then `gz topic -e -t /cmd_vel` (checks the bridge).
* **cv_bridge crashes / numpy ABI errors:** don't `pip install numpy` (v2) over the apt one; use `python3-numpy` from apt.
* **Robot spins in place at start:** no line in view yet — it is in search mode. Check spawn pose vs the line.
* **Wheels spin the wrong way:** a wheel joint axis points the wrong way (positive must be forward).
