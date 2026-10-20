# ROS workspace setup

`ros/` is the only colcon workspace in the repo. Build, install and log folders appear there
and nowhere else. Every dependency comes from apt (through rosdep) or from source built by
colcon; there's no pip and no virtualenv.

```
ros/
├── colcon_defaults.yaml   symlink-install, Release (read when colcon runs from ros/)
├── colcon.meta            per-package CMake args (spark_dsg's Python bindings)
├── upstream.repos         upstream sources, pinned to exact SHAs
├── patches/               the few changes upstream code needs, applied by apply.sh
└── src/
    ├── ninsar_*/          our packages
    └── upstream/          vcs import target (git-ignored)
```

## Build

```bash
source /opt/ros/jazzy/setup.bash             # first: rosdep needs ROS_DISTRO to resolve ROS keys
cd ros
vcs import src/upstream < upstream.repos     # clone upstream at the pinned SHAs
./patches/apply.sh                           # safe to re-run
rosdep install --from-paths src --ignore-src -y -r
colcon build
source install/setup.bash
```

Always run colcon from `ros/`, which is where `colcon_defaults.yaml` and `colcon.meta` are
read. `--symlink-install` (on by default) symlinks launch files, config and Python sources into
`install/`, so editing them takes effect without a rebuild. C++ changes need
`colcon build --packages-select <package>`.

After moving or deleting files in a package, a symlink-install can keep stale links; clean that
package with `rm -rf build/<package> install/<package>` and build it again.

## Test

```bash
colcon test --packages-select ninsar_perception ninsar_mapping ninsar_farm ninsar_bag_fix ninsar_lidar_calib ninsar_tf_tools
colcon test-result --verbose
```

## Packages

### Ours

| package | role |
|---|---|
| `ninsar_perception` | perception front end: `semantic_cloud_adapter` (depth + labels → `cloud_in`), `object_detector` (`cloud_in` → per-frame 3D objects) |
| `ninsar_mapping` | Voxeland server config, `voxel_query` (box/class queries on the map), `tf_to_pose_cov` |
| `ninsar_bringup` | system launch files, rviz, per-scenario configs (`config/isaac/`) |
| `ninsar_farm` | farm GeoJSON library and editor, farm scene graph builders |
| `ninsar_bag_fix` | repair of recorded robot bags ([guide](../guides/bag_fix.md)) |
| `ninsar_lidar_calib` | `base_link → lidar` extrinsic calibration |
| `ninsar_tf_tools` | live-tunable TF publisher |

### Upstream (`ros/upstream.repos`)

| package | role |
|---|---|
| `voxeland` (MAPIRlab) | semantic voxel map server, plus `voxeland_msgs` |
| `segmentation_msgs` (MAPIRlab) | `SemanticPointCloud`, input of the Voxeland server |
| `hydra`, `hydra_ros` (MIT-SPARK) | scene graph construction; `hydra_visualizer` shows scene graphs in RViz |
| `spark_dsg` (MIT-SPARK) | scene graph data structure and I/O, C++ and Python |
| `semantic_inference` (MIT-SPARK) | image segmentation models and ROS nodes (Hydra dependency) |
| `kimera_pgmo`, `kimera_rpgo`, `teaser_plusplus`, `pose_graph_tools`, `config_utilities`, `spatial_hash`, `ianvs` | Hydra dependencies |

Voxeland's `voxeland_robot_perception` submodule isn't checked out: `vcs import` skips
submodules, and our `semantic_cloud_adapter` replaces it.

## Upstream code and patches

Upstream checkouts stay at their pinned SHAs. Changes we need go into our packages (config,
launch, adapter nodes). When that's impossible, a patch goes into `ros/patches/`; its
[README](../../ros/patches/README.md) lists each patch, why it exists, and how to add one or bump
a pin. Today there are two:

- `voxeland_modehas.patch`: stops the Voxeland debug GUI flooding the log.
- `spark_dsg_python_install.patch`: installs spark_dsg's Python module where colcon puts it on
  `PYTHONPATH`, so `ninsar_farm` needs no pip.

## Optional tools outside apt

- `rosbags-convert`, only to read ROS 1 bags (Trektor) with `ninsar_bag_fix`:
  `pipx install rosbags`. pipx keeps it in its own environment, away from the ROS Python.
