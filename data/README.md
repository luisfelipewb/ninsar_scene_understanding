# data/

Local data for the whole repo: bags, Isaac Sim assets, farm descriptions, scene graphs and map
exports. Git ignores everything here except this README, so each machine fills it in. Tools in
`ros/` and `sim/` default to these folders. Keep `data/` itself a real folder so this README
stays tracked; the folders inside can be links to a bigger disk.

`assets/` should be a link to wherever the Isaac assets are, not a copy: the USD scenes reference
models and textures by relative paths, which keep resolving only while the files stay in place.
`sim/isaac/assets` is a tracked link to this folder.

```bash
ln -s /path/to/isaac/assets data/assets      # once per machine, from the repo root
```

```
data/
├── bags/
│   ├── isaac/       ROS 2 bags recorded from Isaac Sim
│   └── robot/
│       ├── raw/     as recorded on the robots (ROS 1 or ROS 2)
│       └── fixed/   repaired by ninsar_bag_fix
├── assets/          Isaac Sim USD scenes, rigs, heightmaps, cabbage lists
├── farms/           farm GeoJSON (farm editor)
├── dsg/             scene graph JSON
└── maps/            Voxeland map exports (PLY/PCD)
```

| folder | written by | read by |
|---|---|---|
| `bags/isaac/` | `ros2 bag record` while Isaac Sim plays a scene ([how](../docs/guides/isaac_sim.md#record-a-bag-for-the-ros-side)) | `ros2 launch ninsar_bringup isaac_voxeland.launch.xml` + `ros2 bag play` |
| `bags/robot/raw/` | the robots | `bagfix run` (ninsar_bag_fix) |
| `bags/robot/fixed/` | `bagfix run` (`output.root` in its configs) | the launch files, `ninsar_lidar_calib`, `bag_to_waypoints` |
| `assets/` (a link) | downloaded, or authored in Isaac Sim | `sim/isaac` through `sim/isaac/assets` (scenes, rigs, heightmaps, asset scan) |
| `farms/` | `ros2 run ninsar_farm farm_editor` | `geojson_to_dsg` |
| `dsg/` | `geojson_to_dsg`, `make_farm_dsg`, `make_isaac_farm_dsg` | `hydra_visualizer` (the launch files' `scene_graph` arg), `farm_dsg verify` |
| `maps/` | Voxeland's `save_map` service, which writes into the launch's working directory: start the launch from `data/maps/` | `pcl_viewer` (apt `pcl-tools`), offline analysis |

## Getting the data

| what | where |
|---|---|
| Isaac tour bag (`bags/isaac/short_tour/`) | TODO: add the download link |
| Robot recordings (`bags/robot/raw/`) | TODO: add the download link |
| Isaac assets (`assets/`) | TODO: add the download link (~55 GB) |
