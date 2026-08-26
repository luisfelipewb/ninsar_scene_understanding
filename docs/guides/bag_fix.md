# ninsar-bag-fix

Colcon workspace for working with bags recorded on the NINSAR robots.

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

| package | what it does |
|---|---|
| [ninsar_bag_fix](src/ninsar_bag_fix/) | Config-driven repair of recorded bags — frame ids, `CameraInfo`, static transforms, topic drop/rename, ROS 1 → ROS 2 conversion. |
| [ninsar_lidar_calib](src/ninsar_lidar_calib/) | Calibrates the `base_link → lidar` extrinsic from a recording of a fixed corner target, using PCL for detection and scipy for the fit. |

The two meet at the end: the calibration prints a `tf_static` rule that goes
straight into a `bagfix` config, so the corrected extrinsic can be written back
into the recorded bags.

```bash
# repair a recording
bagfix run src/ninsar_bag_fix/config/pom_4x4.yaml /data/2026-07-09-16-39-34

# calibrate the lidar from a corner-target recording
ros2 run ninsar_lidar_calib detect_from_bag --bag /data/2026-07-09-16-39-34 \
    --ros-args --params-file src/ninsar_lidar_calib/config/lidar_calib.yaml
ros2 run ninsar_lidar_calib solve detections.jsonl --base-height 0.38
```

Recordings are laid out as `<session name>/bag/bag_N.mcap`; both tools accept
the session folder and find the `bag/` inside it. Repaired bags land in
`output/<session name>_fixed/bag/`.

## Tests

```bash
colcon test && colcon test-result --all
```

## Layout

```
src/ninsar_bag_fix/       ament_python  - the bagfix CLI, configs, tests
src/ninsar_lidar_calib/   ament_cmake   - C++ detectors, Python solver, launch, rviz
legacy/                   the original one-off scripts bagfix replaced
output/                   repaired bags (gitignored)
```
