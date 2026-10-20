# ninsar_tf_tools

A single small node, `tf_publisher`: it broadcasts one `parent -> child`
transform at a fixed rate, with every value a plain ROS 2 parameter, so an
extrinsic can be dialled in from `rqt_reconfigure` while RViz - or
`ninsar_lidar_calib`'s `lidar_calib_node` - reacts live.

Deliberately standalone: it depends on nothing but `rclpy`, `tf2_ros`,
`geometry_msgs` and `std_srvs`, and has no build-time tie to the bag-fix or
calibration packages.

## Run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ninsar_tf_tools
source install/setup.bash

ros2 launch ninsar_tf_tools tf_publisher.launch.py            # + rqt_reconfigure
ros2 launch ninsar_tf_tools tf_publisher.launch.py rqt:=false use_sim_time:=true
ros2 run ninsar_tf_tools tf_publisher --ros-args \
    -p parent_frame:=robot_base_link -p child_frame:=robot_lidar_link -p z:=0.52
```

Launch it several times with different `node_name:=` values to publish several
frames.

## Parameters

| name | type | default | |
|---|---|---|---|
| `parent_frame` / `child_frame` | string | `base_link` / `sensor_link` | live |
| `x` / `y` / `z` | double | 0.0 | metres |
| `roll_deg` / `pitch_deg` / `yaw_deg` | double | 0.0 | degrees, fixed-axis ZYX |
| `publish_rate` | double | 30.0 | Hz; the timer is rebuilt when it changes |
| `static_tf` | bool | false | latch on `/tf_static`; **read-only**, set it at launch |
| `qx` / `qy` / `qz` / `qw` | double | - | read-back of the current rotation |

Angles are entered in **degrees** because that is what a human tunes. The
quaternion is mirrored back into `qx..qw` (visible in rqt) and logged on every
accepted change, because that is what the configs downstream want.

`static_tf` latches the pose on `/tf_static` and re-latches it on every change,
so late joiners always get the current value. (It publishes the topic directly
rather than through `tf2_ros.StaticTransformBroadcaster`, which only ever
records the *first* transform it is given for a child frame and would keep
re-broadcasting the original pose.)

Changes are validated on a copy before they are applied, so a rejected set never
half-applies - `parent_frame == child_frame`, a non-positive `publish_rate` or a
NaN is refused with a reason and the previous transform keeps being published.

## Getting the value back out

```bash
ros2 service call /tf_publisher/print_transform std_srvs/srv/Trigger
```

prints (and returns) the current pose in all three forms: the human summary, a
ready-to-paste `ros2 run tf2_ros static_transform_publisher ...` command line,
and a `bagfix` `tf_static` entry with `rpy` **in radians**, which drops straight
into the `edit:` block of `ninsar_bag_fix/config/pom_4x4.yaml`.

## Over bag playback

`pom_4x4.yaml` currently *removes* `robot_base_link -> robot_lidar_link` without
putting anything back (its `edit:` block is commented out), so a fixed bag has
`robot_lidar_link` as a detached root - that missing edge is what this node
supplies. Two things have to line up or the tree still looks broken:

```bash
ros2 bag play data/bags/robot/fixed/<session>_fixed/bag --clock       # note --clock
ros2 launch ninsar_tf_tools tf_publisher.launch.py static_tf:=true use_sim_time:=true
```

- **`--clock` + `use_sim_time:=true`**: without them the node stamps transforms
  with today's wall clock while the bag's data is stamped with its recording
  date, and every lookup fails on extrapolation.
- **`static_tf:=true`**: a latched `/tf_static` entry has no timestamp to
  disagree about, so it is the robust choice over playback. The default `/tf`
  mode is for live robots.

Verified with the pom_4x4 bag: `map -> robot_lidar_link_lidar` resolves end to
end with the node running, and does not without it.

## With the calibration node

Same setup - the frame `lidar_calib_node` looks up is then the one you are
dragging in rqt.

## Test

```bash
colcon test --packages-select ninsar_tf_tools && colcon test-result --verbose
```
