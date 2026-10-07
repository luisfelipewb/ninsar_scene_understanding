# ninsar_mapping

Launch files, configuration and helper nodes for NINSAR semantic mapping on top of
Voxeland and Hydra. Upstream packages in `src/` stay untouched where possible; our
setup lives here.

| path | |
|---|---|
| `launch/isaac_voxeland.launch.xml` | Voxeland on the Isaac Sim tour bag (below); `.yaml` mirror |
| `ninsar_mapping/` | runtime nodes (`ros2 run ninsar_mapping ...`): `semantic_cloud_adapter`, `voxel_query`, `tf_to_pose_cov` |
| `scripts/` | offline tools, run with the venv's python: `make_farm_dsg.py`, `make_isaac_farm_dsg.py`, `view_map.py`, `farm_geojson.py` (farm GeoJSON validate/tree), `geojson_to_dsg.py` (farm GeoJSON → scene graph), `farm_dsg.py` (scene graph spec helpers, `verify`) |
| `config/` | node parameters (`isaac_voxeland.yaml`, `voxeland_server.yaml`), scene graph viz configs, `isaac_assets.json` (input of `make_isaac_farm_dsg.py`), `farm_schema.yaml` (farm GeoJSON catalog) |
| `rviz/` | rviz config for the Voxeland launch |
| `docs/` | [HYDRA_GUIDE.md](docs/HYDRA_GUIDE.md), [VOXELAND_GUIDE.md](docs/VOXELAND_GUIDE.md), [CLOUD_IN_SPEC.md](docs/CLOUD_IN_SPEC.md), [FARM_DSG_SPEC.md](docs/FARM_DSG_SPEC.md), [FARM_GEOJSON_SPEC.md](docs/FARM_GEOJSON_SPEC.md), [FARM_EDITOR.md](docs/FARM_EDITOR.md), [diagrams/ninsar_overview.drawio](docs/diagrams/ninsar_overview.drawio) (overview diagrams, draw.io) |
| `patches/` | the only change still needed in upstream code ([README](patches/README.md)) |
| `test/` | unit tests (`colcon test --packages-select ninsar_mapping`) |

Generated files (DSG JSON, map exports) go to `~/data/ninsar/`, never into git.

# Running Voxeland on the Isaac Sim "tour" bag

Semantic 3D mapping from a pre-recorded Isaac Sim bag: RGB-D and per-pixel semantic labels go
in, and a class-annotated voxel map comes out. No object detector is involved: the labels come
straight from the bag's `semantic_segmentation` topic.

```
bag ── rgb, depth, camera_info ──► depth_image_proc/point_cloud_xyzrgb_node ── points (organized)
bag ── semantic_segmentation, semantic_labels ──┐                                │
TF  ── world → output_frame ────────────────────┴──► semantic_cloud_adapter ◄────┘
                                                        │ cloud_in (SemanticPointCloud)
                                                        ▼
                                                  voxeland_server
```

Upstream's `voxeland_robot_perception` node is not used. The adapter produces the server's
input directly, following [docs/CLOUD_IN_SPEC.md](docs/CLOUD_IN_SPEC.md).

## Prerequisites

- ROS 2 **Jazzy** with `depth_image_proc` (`ros-jazzy-depth-image-proc`)
- The bag: `~/workspaces/ninsar-isaac-sim/data/tour/` (mcap)
- The scene graph: `~/data/ninsar/isaac_farm_dsg.json` (`scripts/make_isaac_farm_dsg.py`)
- `src/voxeland` with [`patches/voxeland_modehas.patch`](patches/README.md) applied (recommended; without it the server log is flooded)
- `.venv` at the workspace root, needed only for the offline scripts (`open3d`). See below.

### The venv

`scripts/view_map.py` uses `open3d`. It isn't packaged for Ubuntu 24.04, and PEP 668
blocks installing it into the system Python. The mapping launch itself no longer needs the
venv: upstream's perception node, which imported `open3d` and `scikit-learn`, isn't used.

`--system-site-packages` is what keeps apt's `cv2`/`numpy`/`scipy` visible. `rclpy` is
unaffected, since it arrives via `PYTHONPATH` from the ROS setup files.

To recreate it (from the workspace root):

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install open3d scikit-learn
touch .venv/COLCON_IGNORE          # keep colcon out of it
```

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

`--symlink-install` matters: launch files, config and Python sources are symlinked into
`install/`, so editing them takes effect without rebuilding. Only C++ changes
(the `voxeland` package) need `colcon build --packages-select voxeland`.

## Run

Two terminals, **in this order**.

```bash
# 1 — depth_image_proc + semantic_cloud_adapter + Voxeland server + scene graph markers + rviz2
cd ~/data/ninsar                     # any directory; save_map writes the PLY here
source /opt/ros/jazzy/setup.bash
source ~/workspaces/ninsar-semantic-segmentation/install/setup.bash
ros2 launch ninsar_mapping isaac_voxeland.launch.xml

# 2 — the bag, started last
cd ~/workspaces/ninsar-isaac-sim/data
ros2 bag play tour/
```

| launch arg | default | |
|---|---|---|
| `camera` | `/sensor_rig/front_camera` | namespace of `rgb`, `depth`, `camera_info`, `semantic_segmentation`, `semantic_labels` |
| `output_frame` | `zed_x_front` | frame the points are sent in, and whose pose is sent; `""` keeps the optical frame |
| `drop_far_points` | `false` | `true` drops points beyond `max_range` instead of using them for free-space carving |
| `rviz` | `true` | |
| `show_scene_graph` | `true` | static scene graph on `/hydra_dsg_visualizer/graph`, frame `world`, shown in rviz as "Scene Graph" |
| `scene_graph` | `~/data/ninsar/isaac_farm_dsg.json` | from `scripts/make_isaac_farm_dsg.py` |
| `scene_graph_viz_config` | `config/isaac_farm_viz.yaml` (installed share) | hides the FARM root layer |

A missing scene graph file stops only the visualizer node, not the mapping.

## Save and visualise the map

```bash
ros2 service call /voxeland_server/save_map std_srvs/srv/Empty {}
~/workspaces/ninsar-semantic-segmentation/.venv/bin/python \
  ~/workspaces/ninsar-semantic-segmentation/src/ninsar_mapping/scripts/view_map.py voxeland_pointcloud.ply --voxels
```

The PLY holds occupied voxel centres in the `world` frame: `x y z r g b` plus a per-voxel
`uncertainty_categories` (Dirichlet entropy). Colour is the argmax class from the palette in
`voxeland_map/src/semantics.cpp`. The first category (`ground`) is grey, and voxels whose
argmax is `unknown` keep their RGB. The class id itself is *not* stored: recover it from the
palette index, or query the live map instead:

```bash
ros2 service call /voxeland/get_class_distributions \
  voxeland_msgs/srv/GetClassDistributions "{query_points: [{x: -25.9, y: -117.5, z: 5.26}]}"
```

For a box and/or classes, use `voxel_query` (started by the launch file). It returns every
matching voxel with its full class distribution on `/voxel_query/result`, and RViz shows it as
"Query Result". See [VOXELAND_GUIDE.md §5.6](docs/VOXELAND_GUIDE.md#56-querying-the-map-voxel_query).

```bash
ros2 param set /voxel_query query.classes "['grapevine']"
ros2 param set /voxel_query query.dimensions "[0.0, 0.0, 0.0]"   # whole map
ros2 service call /voxel_query/query std_srvs/srv/Trigger
```

## Gotchas

- **Start order.** `/tf_static` is published once at t=0. Start the launch before
  the bag, or the camera extrinsics never resolve.
- **No `--start-offset`.** The Jazzy player skips those t=0 `/tf_static`
  messages, with the same result.
- **`save_map` writes to the cwd of terminal 1** and silently overwrites
  `voxeland_pointcloud.ply`. Rename the file if you want to keep it.
- **Categories are fixed per run.** The server reads the category list from the first
  cloud only. Changing `categories` in `config/isaac_voxeland.yaml` needs a server restart.
- **Label ids are mapped by name.** The adapter reads id → class from the `semantic_labels`
  JSON of the same stamp. The tour bag's ids are stable (0 BACKGROUND, 1 UNLABELLED,
  2 obstacle, 3 ground, 4 fence, 5 building, 6 tree, 7 grapevine, 8 lemon, 9 cabbage), and
  that table is also the `label_names` fallback.
- **Ray origin.** The server casts rays from the `output_frame` origin. `zed_x_front` is
  the ZED body frame, not the left optical centre: the left camera sits at
  (0.015, 0.060, 0.015) m in it, so rays start about 6 cm off. That's negligible at 0.10 m
  voxels. Set `output_frame:=front_left_camera_optical_link` for the exact origin.
- **One TF listener per node.** The adapter's `TransformListener` has its own internal node
  (`node=None`, `spin_thread=True`). Passing the adapter node itself puts it in two executors,
  and the cloud callbacks then starve TF: lookups fall about 1.5 s behind and every frame is skipped.

## Key parameters

`config/isaac_voxeland.yaml` (adapter) and `config/voxeland_server.yaml` (server):

| Parameter | Current | Notes |
|---|---|---|
| `categories` | ground, tree, grapevine, lemon, cabbage, fence, building, obstacle | `unknown` is appended last; the order defines the class indices and colours |
| `min_range` / `max_range` | 0.1 / 10 m | adapter; far points are kept for carving unless `drop_far_points` |
| `pixel_stride` | 1 | subsample the image grid to speed things up |
| `confidence_topic` | `""` | per-pixel confidence (32FC1) for model output; see the spec |
| `resolution` | 0.10 m | server |
| `sensor_model.max_range` | 10 m | server: hits beyond this become free-space rays |
| `semantics_as_instances` | `false` | dense semantics, no instance fusion |

## Upstream code

`src/voxeland` (MAPIRlab/Voxeland) and its submodule `voxeland_robot_perception` stay at
their base commits. The only remaining change is the one-line `modeHas()` fix in
[`patches/`](patches/README.md), which also explains why it can't live here. The earlier
in-place edits (a `label_image` mode in the perception node, launch edits, a debug GUI fix)
are archived in `patches/archive/` and replaced by this package.
