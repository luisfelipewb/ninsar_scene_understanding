# Robot bag tools

Three packages for working with bags recorded on the NINSAR robots. They're built with the rest
of the workspace ([setup/ros.md](../setup/ros.md)).

| package | what it does |
|---|---|
| [ninsar_bag_fix](../../ros/src/ninsar_bag_fix/) | Config-driven repair of recorded bags — frame ids, `CameraInfo`, static transforms, topic drop/rename, ROS 1 → ROS 2 conversion. |
| [ninsar_lidar_calib](../../ros/src/ninsar_lidar_calib/) | Calibrates the `base_link → lidar` extrinsic from a recording of a fixed corner target, using PCL for detection and scipy for the fit. |
| [ninsar_tf_tools](../../ros/src/ninsar_tf_tools/) | A live-tunable TF publisher: one `parent → child` transform driven by ROS 2 parameters, so an extrinsic can be dialled in from `rqt_reconfigure`. |

They meet at the end: both the calibration and the TF publisher print a
`tf_static` rule that goes straight into a `bagfix` config, so the corrected
extrinsic can be written back into the recorded bags.

From `ros/`, after `source install/setup.bash`:

```bash
# repair a recording
bagfix run src/ninsar_bag_fix/config/pom_4x4.yaml ../data/bags/robot/raw/2026-07-09-16-39-34

# calibrate the lidar from a corner-target recording
ros2 run ninsar_lidar_calib detect_from_bag --bag ../data/bags/robot/raw/2026-07-09-16-39-34 \
    --ros-args --params-file src/ninsar_lidar_calib/config/lidar_calib.yaml
ros2 run ninsar_lidar_calib solve detections.jsonl --base-height 0.38

# or nudge the extrinsic by hand and watch RViz follow
ros2 launch ninsar_tf_tools tf_publisher.launch.py
```

Recordings are laid out as `<session name>/bag/bag_N.mcap`; both tools accept
the session folder and find the `bag/` inside it. Repaired bags land in
`data/bags/robot/fixed/<session name>_fixed/bag/`. ROS 1 recordings (Trektor) also need the
`rosbags-convert` CLI: `pipx install rosbags`.

## Tests

```bash
colcon test --packages-select ninsar_bag_fix ninsar_lidar_calib ninsar_tf_tools
colcon test-result --verbose
```
