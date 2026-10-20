# Isaac Sim setup

`sim/isaac/` runs with Isaac Sim 5.1's bundled Python, installed at `~/isaacsim`
(`~/isaacsim/python.sh`). Isaac Sim's Python and ROS 2 Jazzy's (the system one) can't import each
other's packages, so nothing here is installed into either: the scripts run straight from the
repo. Isaac's Python already ships what they need (numpy, PyYAML, scipy, pxr).

## Prerequisites

- Isaac Sim 5.1 in `~/isaacsim`.
- ROS 2 Jazzy, sourced before `python.sh`: Isaac's ROS 2 bridge needs ROS on `LD_LIBRARY_PATH`.
- The assets. `sim/isaac/assets` is a tracked link to `data/assets`, and `data/assets` is a link
  you create once per machine to wherever the Isaac assets are:

  ```bash
  ln -s /path/to/isaac/assets data/assets      # from the repo root
  ```

  A link, not a copy: the USD scenes reference textures and models by relative paths, and they
  keep resolving as long as the files stay where they are. See [data/README.md](../../data/README.md)
  for where to get the assets.

## Check

```bash
source /opt/ros/jazzy/setup.bash
S=~/workspaces/ninsar-scene-understanding/sim/isaac
~/isaacsim/python.sh $S/run_scene.py $S/config/montoldre_v4_small.yaml --headless --build-only
```

`--build-only` assembles the stage, checks it and exits without publishing. The first start takes
about a minute while Isaac Sim boots.

## Which interpreter

| what | runs with |
|---|---|
| `run_scene.py`, the pxr tools in `sim/isaac/tools/` | `~/isaacsim/python.sh` |
| `sim/isaac/tools/bag_to_waypoints.py` | system `python3`, ROS sourced (`rosbag2_py`, `tf2_ros`) |
| `sim/isaac/tools/tour_from_points.py` | either (numpy + PyYAML) |
| `seg_label_colorizer` (label image viewer) | ROS: `ros2 run ninsar_perception seg_label_colorizer` |

Details of each tool: [guides/isaac_sim.md](../guides/isaac_sim.md).
