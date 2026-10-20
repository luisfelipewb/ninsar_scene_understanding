# Running Voxeland on the Isaac Sim "tour" bag

Semantic 3D mapping from a pre-recorded Isaac Sim bag: RGB-D and per-pixel semantic labels go
in, and a class-annotated voxel map comes out. No segmentation model is involved: the labels come
straight from the bag's `semantic_segmentation` topic.

```
bag ── rgb, depth, camera_info ──► depth_image_proc/point_cloud_xyzrgb_node ── points (organized)
bag ── semantic_segmentation, semantic_labels ──┐                                │
TF  ── world → output_frame ────────────────────┴──► semantic_cloud_adapter ◄────┘
                                                        │ cloud_in (SemanticPointCloud)
                                                        ├──────────► object_detector
                                                        ▼              per-frame objects: detections, markers
                                                  voxeland_server
```

Both `semantic_cloud_adapter` and `object_detector` are in `ninsar_perception`; the launch file
and the Isaac-specific parameters are in `ninsar_bringup`. Upstream's `voxeland_robot_perception`
node is not used. The adapter produces the server's input directly, following
[CLOUD_IN_SPEC.md](../specs/CLOUD_IN_SPEC.md).

## Prerequisites

- The workspace built as in [setup/ros.md](../setup/ros.md). That includes the
  [voxeland patch](../../ros/patches/README.md); without it the server log is flooded.
- A bag in `data/bags/isaac/` (mcap), e.g. `short_tour/` (people and landmarks) or
  `tour/`. See [data/README.md](../../data/README.md) for where to get it.
- The scene graph `data/dsg/isaac_farm_dsg.json`: `ros2 run ninsar_farm make_isaac_farm_dsg`.

## Run

Two terminals, **in this order**, both from the repo root.

```bash
# 1 — depth_image_proc + semantic_cloud_adapter + Voxeland server + scene graph markers + rviz2
source ros/install/setup.bash
mkdir -p data/maps && cd data/maps   # save_map writes the PLY into the launch's working directory
ros2 launch ninsar_bringup isaac_voxeland.launch.xml

# 2 — the bag, started last
source ros/install/setup.bash
ros2 bag play data/bags/isaac/short_tour/
```

| launch arg | default | |
|---|---|---|
| `camera` | `/sensor_rig/front_camera` | namespace of `rgb`, `depth`, `camera_info`, `semantic_segmentation`, `semantic_labels` |
| `output_frame` | `zed_x_front` | frame the points are sent in, and whose pose is sent; `""` keeps the optical frame |
| `drop_far_points` | `false` | `true` drops points beyond `max_range` instead of using them for free-space carving |
| `rviz` | `true` | |
| `show_scene_graph` | `true` | static scene graph on `/hydra_dsg_visualizer/graph`, frame `world`, shown in rviz as "Scene Graph" |
| `data_dir` | the repo's `data/` | found from the installed `ninsar_bringup` share directory |
| `scene_graph` | `data/dsg/isaac_farm_dsg.json` | from `ros2 run ninsar_farm make_isaac_farm_dsg` |
| `scene_graph_viz_config` | `ninsar_bringup` `config/isaac_farm_viz.yaml` | hides the FARM root layer |
| `detector` | `true` | per-frame objects from `cloud_in` (`object_detector`), shown in rviz as "Detections" |

A missing scene graph file stops only the visualizer node, not the mapping. The `.yaml` launch
file mirrors the `.xml` one and also starts `voxel_query`.

## Save and look at the map

```bash
ros2 service call /voxeland_server/save_map std_srvs/srv/Empty {}
pcl_viewer data/maps/voxeland_pointcloud.ply      # apt pcl-tools
```

The PLY holds occupied voxel centres in the `world` frame: `x y z r g b` plus a per-voxel
`uncertainty_categories` (Dirichlet entropy). Colour is the argmax class from the palette in
Voxeland's `voxeland_map/src/semantics.cpp`. The first category (`ground`) is grey, and voxels
whose argmax is `unknown` keep their RGB. The class id itself is *not* stored: recover it from the
palette index, or query the live map instead:

```bash
ros2 service call /voxeland/get_class_distributions \
  voxeland_msgs/srv/GetClassDistributions "{query_points: [{x: -25.9, y: -117.5, z: 5.26}]}"
```

For a box and/or classes, use `voxel_query` (started by the `.yaml` launch file). It returns
every matching voxel with its full class distribution on `/voxel_query/result`, and RViz shows it
as "Query Result". See [VOXELAND_GUIDE.md §5.6](VOXELAND_GUIDE.md#56-querying-the-map-voxel_query).

```bash
ros2 param set /voxel_query query.classes "['grapevine']"
ros2 param set /voxel_query query.dimensions "[0.0, 0.0, 0.0]"   # whole map
ros2 service call /voxel_query/query std_srvs/srv/Trigger
```

## Per-frame objects

`object_detector` (C++/PCL) clusters each `cloud_in` frame per class, skipping ground, and
publishes the objects in the camera frame. `/object_detector/detections`
(`vision_msgs/Detection3DArray`) is the landmark list: class, score, centroid with covariance,
and a gravity-aligned yaw box. RViz shows `/object_detector/markers` as "Detections". Parameters
are in `ninsar_bringup` `config/isaac/object_detector.yaml`. See
[OBJECT_DETECTOR.md](../specs/OBJECT_DETECTOR.md).

```bash
ros2 topic echo --once /object_detector/detections
```

## Gotchas

- **Start order.** `/tf_static` is published once at t=0. Start the launch before
  the bag, or the camera extrinsics never resolve.
- **No `--start-offset`.** The Jazzy player skips those t=0 `/tf_static`
  messages, with the same result.
- **`save_map` writes to the launch's working directory** and silently overwrites
  `voxeland_pointcloud.ply`. Rename the file if you want to keep it.
- **Categories are fixed per run.** The server reads the category list from the first
  cloud only. Changing `categories` in `config/isaac/semantic_cloud_adapter.yaml` needs a server
  restart.
- **Label ids are mapped by name.** The adapter reads id → class from the `semantic_labels`
  JSON of the same stamp. Ids are stable within a bag but differ between bags: `tour` has
  2 obstacle, 3 ground, …, 9 cabbage; `short_tour` has 2 obstacle, 3 cabbage, 4 fence, 5 ground,
  6–10 and 16 the landmarks, 15 person. The `label_names` fallback is the `short_tour` table.
- **A class not in `categories` becomes `unknown`.** Its voxels then show the camera's last
  RGB instead of a class colour, so a new class in the scene needs adding to `categories`.
- **Ray origin.** The server casts rays from the `output_frame` origin. `zed_x_front` is
  the ZED body frame, not the left optical centre: the left camera sits at
  (0.015, 0.060, 0.015) m in it, so rays start about 6 cm off. That's negligible at 0.10 m
  voxels. Set `output_frame:=front_left_camera_optical_link` for the exact origin.
- **One TF listener per node.** The adapter's `TransformListener` has its own internal node
  (`node=None`, `spin_thread=True`). Passing the adapter node itself puts it in two executors,
  and the cloud callbacks then starve TF: lookups fall about 1.5 s behind and every frame is skipped.

## Key parameters

`ninsar_bringup` `config/isaac/semantic_cloud_adapter.yaml` (adapter) and `ninsar_mapping`
`config/voxeland_server.yaml` (server):

| Parameter | Current | Notes |
|---|---|---|
| `categories` | ground, tree, grapevine, lemon, cabbage, fence, building, obstacle, person, landmark_d4, _d6, _d8, _d10, _d12, _d20 | `unknown` is appended last; the order defines the class indices and colours |
| `min_range` / `max_range` | 0.1 / 10 m | adapter; far points are kept for carving unless `drop_far_points` |
| `pixel_stride` | 1 | subsample the image grid to speed things up |
| `confidence_topic` | `""` | per-pixel confidence (32FC1) for model output; see the spec |
| `resolution` | 0.10 m | server |
| `sensor_model.max_range` | 10 m | server: hits beyond this become free-space rays |
| `semantics_as_instances` | `false` | dense semantics, no instance fusion |
