# Voxeland guide

How the Voxeland code in this workspace works: what each package does, how data
flows from the bag to the voxel map, and how the server uses Bonxai underneath.

For build and run steps, see [the package README](../README.md). This guide covers the
code. `src/semantic_octomap` is a separate mapper and is out of scope here.

---

## 1. The big picture

Voxeland is a **semantic voxel mapper**. It does two jobs:

1. **Geometry:** a probabilistic occupancy grid, like OctoMap. Rays are cast
   from the sensor, so cells along each ray become free and the endpoint cells
   become occupied.
2. **Semantics:** every voxel keeps a **Dirichlet distribution** over a fixed
   list of classes. Each observation of a voxel adds evidence (the "alpha"
   concentration parameters). The argmax gives the voxel's class, and the
   Dirichlet's expected entropy gives its uncertainty.

Our pipeline has three nodes. Only the last one is Voxeland:

```
 bag / sensors                  depth_image_proc            ninsar_mapping                 voxeland (C++)
 ─────────────                  (C++)                       semantic_cloud_adapter         ("server")
 RGB image      ─┐
 depth image    ─┼─ approx. ──► back-project depth ──┐
 camera_info    ─┘   sync       organized XYZRGB     │ approx. sync
 label image    ─────────────────────────────────────┼─► attach per-pixel        transform points to world
 labels JSON    ─────────────────────────────────────┤   instance_id, build  ──► ray-cast into Bonxai grid
 TF             ─────────────────────────────────────┘   instances, frame, pose  fuse class evidence per voxel
                                                         publish SemanticPointCloud  publish / save / query map
                                                              on /cloud_in
```

The only link to the server is the `segmentation_msgs/SemanticPointCloud` message on
`/cloud_in`. Its exact contract is in [CLOUD_IN_SPEC.md](CLOUD_IN_SPEC.md). Upstream ships
its own frontend for that message (`voxeland_robot_perception`, §2.4). We don't run it:
it is built around a Detectron2 instance-segmentation service and the COCO classes.

---

## 2. What's where

### 2.1 Workspace root (`~/workspaces/ninsar-semantic-segmentation`)

| Path | What it is |
|---|---|
| `src/voxeland/` | Clone of `MAPIRlab/Voxeland` (branch `humble`; only `ninsar_mapping/patches/voxeland_modehas.patch` applied). Holds the packages below. |
| `src/segmentation_msgs/` | Message package shared by Voxeland and the perception node (`SemanticPointCloud`, `InstanceSemanticMap`, …). |
| `src/ninsar_mapping/` | Our package: launch, config, nodes, `scripts/view_map.py` (Open3D viewer for the exported `voxeland_pointcloud.ply`), these docs. Its README covers building and running on the Isaac Sim "tour" bag. |
| `.venv/` | Python venv (with system site-packages) that adds `open3d` and `scikit-learn`, for the offline scripts. It has `COLCON_IGNORE`. |
| `build/`, `install/`, `log/` | colcon output. |
| `~/data/ninsar/` | Generated files, outside the workspace: map exports from earlier runs (`voxeland_pointcloud.ply`, `semantic_voxeland_pointcloud.ply`, ~170 MB ASCII PLY each), scene graph JSON. |

### 2.2 `src/voxeland/` (the Voxeland repo root)

| Path | Package | Built? | Role |
|---|---|---|---|
| `voxeland/` | `voxeland` | yes | The C++ **server node**, plus the `voxeland_map` library and the vendored Bonxai core. |
| `voxeland_msgs/` | `voxeland_msgs` | yes | `ClassDistribution.msg`, `GetClassDistributions.srv`, `UpdateMapResults.srv`. |
| `voxeland_robot_perception/` | `voxeland_robot_perception` | yes | **git submodule.** Upstream Python frontend that turns images into point clouds. Not used (§2.4). |
| `voxeland_disambiguation/` | `voxeland_disambiguation` | **no** (`COLCON_IGNORE`) | Offline post-processing for *instance* maps: it asks a vision-language model about uncertain objects. We don't use instances, so it is ignored. |
| `.gitmodules` | | | Lists the perception submodule and an `fmt` submodule (unused: CMake finds the system `fmt`). |

### 2.3 Inside the `voxeland` package

```
voxeland/
├── CMakeLists.txt          builds bonxai_core → voxeland_map → voxeland_server (component + executable)
├── bonxai_core/include/bonxai/
│   ├── bonxai.hpp          sparse voxel grid (vendored Bonxai, header-only)
│   └── serialization.hpp   grid (de)serialization; not used by Voxeland
├── voxeland_map/           library: occupancy + per-voxel data on top of Bonxai
│   ├── include/voxeland_map/
│   │   ├── probabilistic_map.hpp            log-odds, ProbabilisticCell<DataT>, ray iterator
│   │   ├── probabilistic_map_templated.hpp  ProbabilisticMapT<DataT>: insertPointCloud, getOccupiedVoxels
│   │   ├── data_modes.hpp                   DataMode bit-flags
│   │   ├── cell_types.hpp                   DataMode → cell type dispatch (AUTO_TEMPLATE macros)
│   │   ├── cell_types/                      Empty, Color, Semantics, RGBSemantics, *Instances
│   │   ├── semantics.hpp                    SemanticMap singleton (categories, local/global object maps)
│   │   ├── dirichlet.hpp                    expected Shannon entropy of a Dirichlet
│   │   └── pcl_utils.hpp                    custom PCL point types PointXYZ[RGB]Semantics
│   └── src/semantics.cpp                    SemanticMap methods + colour palette
└── voxeland/               the ROS node
    ├── include/voxeland_server.hpp
    ├── include/semantics_ros_wrapper.hpp    Detection2D[] → SemanticObject[]
    ├── src/voxeland_server.cpp
    ├── src/debug_gui.cpp                    ImGui window (#included into voxeland_server.cpp)
    ├── launch/voxeland_server.launch.xml
    └── rviz/voxeland.rviz
```

`ENABLE_DEBUG_GUI` is `ON` in `CMakeLists.txt`, so the server opens an ImGui
window and links `ament_imgui`. CMake fetches `ament_imgui` from git if it is
not in the workspace.

### 2.4 Inside `voxeland_robot_perception` (upstream frontend, not used)

```
voxeland_robot_perception/
├── voxeland_robot_perception/robot_perception_node.py   the node (class MinimalMapper)
├── modules/
│   ├── camera.py               intrinsics, extrinsics, depth → xyz back-projection
│   ├── data_standarization.py  per-dataset decoding of rgb/depth/semantics msgs
│   ├── semantics.py            Semantics: per-pixel id image + list of Detection2D
│   ├── pointclouds.py          builds the SemanticPointCloud msg; DBSCAN "background removal"
│   └── transformations.py      pose/quaternion ↔ 4×4 SE(3) helpers
├── launch/semantic_mapping.launch.xml   upstream example launch (Detectron2 service)
├── launch/semantic_mapping.launch.py    upstream Python variant
└── rviz/rgbd_nn_map.rviz
```

Things to know if you ever use it:

- Its categories are hardcoded to the 80 COCO classes. The server maps a class name it
  doesn't know to index 0 ("person"), so other label sets need a code change.
- Its `topic` mode is broken: it subscribes to an `Image` but reads `.instances`.
- It packs points as `[forward, left, up]`, so `camera_frame_id` must be an x-forward
  frame at the optical centre.
- It imports `open3d` and `scikit-learn`, which is why the workspace `.venv` exists.

Earlier we ran it with a local `label_image` mode. That edit is archived in
`patches/archive/voxeland_robot_perception.patch`.

---

## 3. Input data (what goes in)

[isaac_voxeland.launch.xml](../launch/isaac_voxeland.launch.xml) wires up these inputs
(camera namespace `/sensor_rig/front_camera`):

| Topic | Type | Used by | For |
|---|---|---|---|
| `rgb` | `sensor_msgs/Image` rgb8 | depth_image_proc | Point colour. |
| `depth` | `sensor_msgs/Image` 32FC1 (metres) | depth_image_proc | Point geometry. |
| `camera_info` | `sensor_msgs/CameraInfo` | depth_image_proc | Pinhole intrinsics. The images are rectified (zero distortion). |
| `semantic_segmentation` | `sensor_msgs/Image` **32SC1** | adapter | Per-pixel label id. |
| `semantic_labels` | `std_msgs/String` (JSON) | adapter | id → class table, matched to the label image by `time_stamp`. |
| TF `world → … → zed_x_front → front_left_camera_optical_link` | TF | adapter | Pose of `output_frame` at each cloud stamp. The `output_frame ← optical` transform is looked up once. |

All images are 1280×720 in `front_left_camera_optical_link`. The bag's message counts
differ slightly per topic (3673–3679), so both stages sync approximately.

---

## 4. From images to `/cloud_in`

### 4.1 depth_image_proc (`point_cloud_xyzrgb_node`)

Standard ROS back-projection: for pixel (u, v) with depth d in the optical frame,
`x = (u − cx)/fx · d`, `y = (v − cy)/fy · d`, `z = d`. The output on `points` is
**organized** (720×1280, NaN where depth is missing or infinite), stamped with the depth
header. Being organized is what lets the adapter attach labels by pixel index.

### 4.2 semantic_cloud_adapter

File: [semantic_cloud_adapter.py](../ninsar_mapping/semantic_cloud_adapter.py), parameters in
[config/isaac_voxeland.yaml](../config/isaac_voxeland.yaml).

1. **Sync** `points` with `semantic_segmentation` (small queues: while a frame is being
   processed, newer frames are dropped instead of queued).
2. **Labels → local instance ids.** The JSON table maps each label id to a class name,
   and `categories` maps the name to an index:
   - **Ground truth (default):** instance `0` is `"unknown"` with score `0.05`. Instance
     `k+1` is `categories[k]` with score `1.0`. Pixels whose class isn't in
     `categories` (BACKGROUND, UNLABELLED, …) get `0`.
   - **Confidence bins** (`confidence_topic` set): one instance per (category,
     confidence bin). See the spec.
3. **Filter:** drop NaN/inf points and points nearer than `min_range`. Finite far
   points are kept, because the server uses them to carve free space; set
   `drop_far_points` to drop points beyond `max_range` instead.
4. **Frame:** move the points into `output_frame` (default `zed_x_front`, FLU) and send
   TF `world ← output_frame` at the cloud stamp as `pose`. The server applies that pose.
5. **Publish** `SemanticPointCloud` with fields `x y z rgb instance_id`, the fixed
   instance table, and `categories` with `"unknown"` appended last.

### 4.3 What reaches the server

See [CLOUD_IN_SPEC.md](CLOUD_IN_SPEC.md) for every field: how the server uses it, the
constraints that fail silently, and how model uncertainty can be carried. Two things matter
most. The **`rgb` and `instance_id` fields decide the server mode** (§5.2), and the server
only reads them, and `categories`, from the **first** message it receives.

---

## 5. The server (`voxeland_server_node`)

Files: [voxeland_server.hpp](../../voxeland/voxeland/voxeland/include/voxeland_server.hpp),
[voxeland_server.cpp](../../voxeland/voxeland/voxeland/src/voxeland_server.cpp)

It is an `rclcpp` component (`voxeland_server::VoxelandServer`) that is also
built as the executable `voxeland_server_node`. Our launch file names it
`voxeland_server`, so `~/…` services live under `/voxeland_server/…`.

### 5.1 Interfaces

| Kind | Name | Type | Notes |
|---|---|---|---|
| sub | `cloud_in` | `SemanticPointCloud` | Depth 1, so it drops old clouds when busy. |
| pub | `bonxai_point_cloud_centers` | `PointCloud2` (XYZRGB, or XYZRGB+instance_id) | All occupied voxel centres, in `frame_id`, **republished after every insert**. |
| pub | `semantic_map_instances` | `InstanceSemanticMap` | Instance modes only. |
| pub | `/voxeland/IDs` | `MarkerArray` | Instance modes only (text labels). |
| srv | `/voxeland_server/save_map` | `std_srvs/Empty` | Writes `voxeland_pointcloud.ply` (plus JSON files in instance mode) to the **cwd**. |
| srv | `/voxeland/get_class_distributions` | `voxeland_msgs/GetClassDistributions` | Created when the **first cloud** arrives. Query points → per-class probabilities. |
| srv | `/voxeland_server/update_map_results` | `voxeland_msgs/UpdateMapResults` | Loads refined instance alphas from JSON (used by disambiguation). |
| srv | `/voxeland_server/reset` | `std_srvs/Empty` | **Don't use.** It recreates the map with the wrong template type (`ProbabilisticMapT<ProbabilisticCell<DataT>>`), and the next `With<DataT>()` returns null. Restart the node instead. |
| GUI | `/voxeland/debug`, `/voxeland/UIQueryPoint`, sub `/clicked_point` | | ImGui debug window. The RViz "Publish Point" tool picks the voxel to inspect. |

Parameters (declared in `initializeBonxaiObject()` on the first cloud):

| Param | Default | Ours | Meaning |
|---|---|---|---|
| `frame_id` | `map` | `world` | Frame of the published map. |
| `resolution` | 0.1 | 0.05 | Voxel edge length (m). |
| `sensor_model.max_range` | 100 | 20 | Rays longer than this are cut short and their end is marked free, not occupied. |
| `sensor_model.hit` / `.miss` | 0.7 / 0.4 | – | Occupancy update per hit or miss (same as OctoMap). |
| `sensor_model.min` / `.max` | 0.12 / 0.97 | – | Log-odds clamping. |
| `occupancy_min_z` / `_max_z` | ±100 | – | Height filter, applied only when publishing. |
| `semantics_as_instances` | false | false | Turns on instance fusion (see §5.7). |
| `latch` | true | – | Publish even with no subscribers. The QoS is *not* transient-local. |

The server declares `base_frame_id` and a TF listener but never uses them. It
takes the sensor pose **only** from `cloud->pose`. The launch file sets
`use_height_map`, but the server never declares or reads it.

### 5.2 DataMode: one cell type per map, fixed at the first message

[data_modes.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/data_modes.hpp) defines bit-flags:

```
Empty = 0, RGB = 1<<1, Semantics = 1<<2, SemanticsInstances = Semantics | 1<<3
```

On the first cloud, `insertCloudCallback` looks at the PointCloud2 field names:

- `rgb` present → `|= RGB`
- `instance_id` present → `|= Semantics`
- `semantics_as_instances && Semantics` → `|= SemanticsInstances`

With `pointcloud_type=XYZRGBSemantics` and `semantics_as_instances=false` we get
**`RGBSemantics`**.

The mode chooses the voxel payload type (`DataT`):

| Mode | `DataT` (cell payload) | PCL point type |
|---|---|---|
| `Empty` | `voxeland::Empty` | `PointXYZ` |
| `RGB` | `voxeland::Color` | `PointXYZRGB` |
| `Semantics` | `voxeland::Semantics` | `PointXYZSemantics` |
| **`RGBSemantics`** ← ours | **`voxeland::RGBSemantics`** | `PointXYZRGBSemantics` |
| `SemanticsInstances` | `voxeland::SemanticsInstances` | `PointXYZSemantics` |
| `RGBSemanticsInstances` | `voxeland::RGBSemanticsInstances` | `PointXYZRGBSemantics` |

`DataMode` is a runtime value, but cell types are template parameters. The
`AUTO_TEMPLATE(mode, expr)` macros in
[cell_types.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/cell_types.hpp)
bridge the two. Each expands to a `switch` that calls a templated lambda with
the right `DataT`, which is why the server code is full of `AUTO_TEMPLATE(currentMode, …<DataT>…)`.

The same first message also sets the **category list** (`SemanticMap::initialize`).
Neither the mode nor the category list can change without restarting the
server.

### 5.3 What happens for each cloud (our `RGBSemantics` mode)

`insertCloudCallback` → `insertPointCloudSemantics<RGBSemantics>`:

```
1. pcl::fromROSMsg(cloud->cloud, pc)                         // PointXYZRGBSemantics, camera frame
2. transformPointCloudToGlobal(pc, cloud->pose)              // pc now in world; returns sensor origin
3. semantics_ros_wrapper.addLocalSemanticMap(instances, pc)
      Detection2D[i] → SemanticObject i with
      alphaParamsCategories[categoryIndex(class_id)] += score
      → stored as SemanticMap::lastLocalSemanticMap           // "evidence per instance_id" for this frame
4. bonxai_->With<RGBSemantics>()->insertPointCloud(pc.points, origin, max_range)
      for each point: hit (or miss, if beyond max_range) → ray-cast free space   (§6)
      for each hit:   cell.data.update(point)                                    (§5.4)
5. publishAll<RGBSemantics>(stamp)
      walk the whole grid, take occupied cells, colour = cell.data.toColor(),
      publish as PointCloud2 on bonxai_point_cloud_centers
```

**Step 5 walks the entire map on every frame.** Its cost grows with map size,
so it gets slower as the run goes on, independent of the cloud size.

### 5.4 Semantic fusion in a voxel

[cell_types/Semantics.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/cell_types/Semantics.hpp),
[RGBSemantics.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/cell_types/RGBSemantics.hpp)

```cpp
struct Semantics    { std::vector<double> alphasDirichlet; ... };   // one alpha per category
struct RGBSemantics : Semantics { Color rgb; ... };                 // + last seen colour
```

`update(point)` runs for **every point** that lands in the voxel, not once per
voxel per frame:

```cpp
alphasDirichlet += SemanticMap::lastLocalSemanticMap[point.instance_id].alphaParamsCategories;
rgb = point.rgb;                        // last write wins, no averaging
```

With our label-image detections, that means:

- A pixel of class *c* adds **+1.0** to `alpha[c]`.
- A pixel of an unmapped label (index 0, "unknown") adds **+0.05** to `alpha[unknown]`,
  which is the last slot.

Evidence is therefore weighted by **pixel count**. Voxels close to the camera,
which receive many pixels, gather evidence much faster than distant ones. The
alphas are never decayed, and **free-space rays do not touch them**. When a
voxel is later carved free, its semantic history stays, but the voxel no longer
appears in the output because output only includes occupied cells.

**Reading a voxel:**

- **Class** = argmax(alphas).
- **`GetClassProbabilities()`** = alphas with each entry floored at 1, then
  normalized.
- **Uncertainty** = `expected_shannon_entropy(alphas)` = ψ(A) − (1/A)·Σ αᵢ·ψ(αᵢ),
  where A = Σ αᵢ over the non-zero αᵢ and ψ is the digamma function
  ([dirichlet.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/dirichlet.hpp)).
  A low value means confident.

**Colour** (`toColor()`, used both for the topic and for the PLY):

| argmax index | Colour |
|---|---|
| 0 | grey `#bcbcbc`, because `indexToHexColor(0)` is hard-wired to grey |
| 1…N−2 | `color_palette[i % 22]` in [semantics.cpp](../../voxeland/voxeland/voxeland_map/src/semantics.cpp) |
| N−1 (`unknown`) | the voxel's own RGB (`RGBSemantics`) or grey (`Semantics`) |

With our `categories` (`ground, tree, grapevine, lemon, cabbage, fence, building, obstacle, person,
landmark_d4, landmark_d6, landmark_d8, landmark_d10, landmark_d12, landmark_d20`, then `unknown`):
ground `#bcbcbc` · tree `#9DBBE3` · grapevine `#BFE3DF` · lemon `#B59CD9` ·
cabbage `#FFF5CC` · fence `#FFD9BD` · building `#EE9D94` · obstacle `#F7ADCF` ·
person `#e6194B` · landmark_d4 `#3cb44b` · landmark_d6 `#ffe119` · landmark_d8 `#4363d8` ·
landmark_d10 `#f58231` · landmark_d12 `#911eb4` · landmark_d20 `#42d4f4` · unknown → camera RGB.
A class missing from `categories` falls into `unknown`, so it shows in camera RGB.

### 5.5 Outputs

**Live topic** `bonxai_point_cloud_centers`: one point per occupied voxel
centre, coloured as above. No class id is included.

**`save_map`** → `voxeland_pointcloud.ply` (ASCII, cwd of the server process):

```
property float x / y / z          voxel centre, world frame
property uchar red / green / blue class colour (or RGB for unknown)
property float uncertainty_categories
```

The class id is not written. You can recover it from the colour, or query the
live map.

**`/voxeland/get_class_distributions`**: for each query point, the server finds
the cell and combines occupancy with class probability:

```
p_occ = prob(cell.probability_log)            (0.5 if the cell was never touched)
p(class_i)      = p_occ · p(class_i | occ)                for i < N−1
p(background)   = (1 − p_occ) + p_occ · p(bg | occ)       last entry
```

A free voxel therefore reports mostly "unknown/background".

In **instance mode** only, `save_map` also writes `voxeland_instanceMap.json`
and `voxeland_instanceMap_appearances.json` (per-object bboxes, alphas, and the
image timestamps where each object was seen). `voxeland_disambiguation`
consumes these files.

### 5.6 Querying the map (`voxel_query`)

`ninsar_mapping/voxel_query.py` answers "which voxels are in this box, and of
which classes" and returns each voxel's **full class distribution**. It doesn't
modify Voxeland: it only uses the two outputs above. Bonxai has no box or label
queries (only point lookup and a full scan, §6.1), and the published cloud carries
no distributions, so each query has two steps:

1. **Snapshot.** The node subscribes to `bonxai_point_cloud_centers` until one
   message arrives, then unsubscribes. With `latch: false` the server builds that
   cloud only while someone subscribes. The voxel colour gives the top class:
   class 0 is `0xbcbcbc`, class i is `color_palette[i]`, and any other colour is
   the camera RGB of an "unknown" voxel (§5.4). The node selects voxels by box
   and top class with numpy. If no cloud arrives within `snapshot_timeout_s`
   (because mapping stopped), it reuses the last snapshot and says so.
2. **Fetch.** It calls `/voxeland/get_class_distributions` for the selected
   voxel centres only, in chunks of `fetch_chunk`, so the server can insert clouds
   between calls. Then it filters by `min_prob`.

The query is set as parameters (`config/voxel_query.yaml`). The service
(`std_srvs/Trigger`) runs it and returns a summary:

```bash
ros2 param set /voxel_query query.classes "['grapevine']"
ros2 param set /voxel_query query.center "[10.0, 5.0, 1.0]"
ros2 param set /voxel_query query.dimensions "[4.0, 1.0, 2.0]"   # all zero = whole map
ros2 param set /voxel_query query.yaw 0.5                          # about z, rad
ros2 service call /voxel_query/query std_srvs/srv/Trigger
```

The box uses the format of spark_dsg's `BoundingBox`: centre, full side lengths,
and a yaw about z. The RAABB boxes from `scripts/make_isaac_farm_dsg.py` can be
copied as they are.

| Kind | Name | Notes |
|---|---|---|
| srv | `/voxel_query/query` | `std_srvs/Trigger`. Runs the query set in `query.*`. |
| pub | `/voxel_query/result` | `PointCloud2`, transient local: `x y z rgb` + one `p_<class>` float per class. `rgb` is the top-class colour. In RViz, set the display's Color Transformer to Intensity and its channel to a `p_*` field. |
| pub | `/voxel_query/query_box` | `Marker` (cube), transient local. Deleted for whole-map queries. |

Limits:

- **Selection uses the top class.** A voxel where class X is likely but not the
  top class is found only by a box query with no class filter. Then use
  `min_prob` or the `p_X` field.
- **The probabilities have occupancy folded in** (see the formula above), so
  they are not p(class | occupied).
- **Colour collisions:** a camera colour equal to a class colour reads as that
  class at selection. It is rare, and the fetched distribution is still exact.
- **`query.max_voxels` caps the fetch.** The summary says when the cap was hit.
- **A fresh snapshot needs the server to be inserting clouds.** After a run,
  the PLY from `save_map` has the top class (from the colour) but no distributions.

### 5.7 Instance mode (upstream feature, not used by us)

With `semantics_as_instances=true`, each voxel stores **votes for global object
ids** (`SemanticsInstances`), not class alphas. The class evidence lives on the
object in `SemanticMap::globalSemanticMap`. On each frame:

1. It collects the voxels and 3D bbox of each local detection.
2. It matches each local detection to global objects by bbox overlap plus a
   voxel IoU/IoS above 0.3, computed on a 2× coarser grid. A match fuses the
   alphas, bbox and appearances into the global object. Otherwise a new object
   is created.
3. It maps local ids to global ids so that `update()` votes for the right
   global object.
4. Every 20 frames, `refineGlobalSemanticMap` merges overlapping objects
   (`pointsTo` redirection) and discards objects seen 5 times or fewer.

This design suits detector output, where each mask is one object. For semantic
classes like `ground` it makes little sense, so we keep it off.

---

## 6. How Voxeland uses Bonxai

Bonxai ([bonxai.hpp](../../voxeland/voxeland/bonxai_core/include/bonxai/bonxai.hpp))
is a header-only sparse voxel grid, similar to a simplified OpenVDB. Voxeland
vendors it and puts its own probabilistic layer on top. Bonxai itself stores
cells and knows nothing about occupancy or semantics.

### 6.1 Bonxai's data structure

```
VoxelGrid<CellT>(resolution, inner_bits = 2, leaf_bits = 3)

root_map : unordered_map<CoordT, InnerGrid>        hash map, unbounded, sparse
  InnerGrid : 4×4×4 array of shared_ptr<LeafGrid> + 64-bit active mask
    LeafGrid : 8×8×8 array of CellT              + 512-bit active mask
```

- A **leaf** covers 8³ voxels, which is 0.4 m per side at 5 cm.
- A **root block** covers 32³ voxels, which is **1.6 m** per side at 5 cm.
- Memory is only allocated where blocks are touched. The world does not need
  bounds.
- `CoordT` is an integer voxel index. `posToCoord` floors `pos / resolution`,
  and `coordToPos` returns the voxel **centre** (`(i + 0.5)·res`).
- An **`Accessor`** caches the last inner and leaf pointer. Neighbouring
  lookups, as in ray casting, skip the hash lookup.
- `forEachCell(visitor)` visits every **active** cell. `getOccupiedVoxels`
  uses it.

### 6.2 Voxeland's layer: `ProbabilisticMapT<DataT>`

[probabilistic_map.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/probabilistic_map.hpp),
[probabilistic_map_templated.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/probabilistic_map_templated.hpp)

The server holds a `std::unique_ptr<Bonxai::ProbabilisticMap>`, a
non-templated base class. The actual object is a `ProbabilisticMapT<DataT>`
that wraps a `VoxelGrid<ProbabilisticCell<DataT>>`.
`bonxai_->With<DataT>()` is a `dynamic_cast` back to the typed map.

Each Bonxai cell is:

```cpp
template <typename DataT>
struct ProbabilisticCell {
    int32_t update_id       : 4;   // "already touched in this scan?" (cycles 1..3)
    int32_t probability_log : 28;  // occupancy log-odds, fixed-point ×1e6
    DataT   data;                  // Empty / Color / RGBSemantics / ...
};
```

`logodds(p) = int(1e6 · ln(p/(1−p)))`. With the defaults, a hit adds +0.85, a
miss adds −0.41, and the value is clamped to [−1.99, +3.48]. A cell counts as
occupied when its log-odds is > 0, which is p > 0.5.

### 6.3 `insertPointCloud(points, origin, max_range)`

```
for each point p (already in world frame):
    if |p − origin| ≥ max_range:
        addMissPoint(origin + dir · max_range)   # endpoint → free, remembered in _miss_coords
    else:
        addHitPoint(p, point)                    # endpoint → occupied, remembered in _hit_coords
            cell = accessor.value(coord, create=true)
            cell.data.update(point)              # ← semantics/colour fusion (every point)
            if cell.update_id != this scan:      # ← occupancy update (once per voxel per scan)
                cell.log += hit, clamp; mark
updateFreeCells(origin):
    for each remembered endpoint:
        3D Bresenham from origin voxel to endpoint voxel (endpoint excluded)
        each traversed cell not yet touched this scan: log += miss, clamp; mark
    advance update_id (1 → 2 → 3 → 1)
```

The key details:

- **Occupancy** is updated at most once per voxel per scan, thanks to
  `update_id`. **Semantics** are updated once per *point*.
- A voxel hit in this scan is marked first, so rays passing through it in the
  same scan do not clear it.
- Ray casting is the dominant cost. It grows with `max_range / resolution` and
  with the number of distinct endpoint voxels. That is why `max_range` 20 m
  costs about 4× what 8 m does (see the package README).

### 6.4 `SemanticMap`: global state beside the grid

[semantics.hpp](../../voxeland/voxeland/voxeland_map/include/voxeland_map/semantics.hpp) is a **process-wide singleton**.
Cell `update()` functions have no other way to reach per-frame context, so they
go through it. It holds:

- `default_categories` and `categoryIndexMap`: the class list from the first
  message. The last entry is the background/unknown class.
- `lastLocalSemanticMap`: this frame's `SemanticObject` per `instance_id`,
  which `Semantics::update()` reads.
- `globalSemanticMap` and the local→global id table: used in instance mode.
- `BonxaiQuery<DataT>`: a static pointer to the typed map plus an accessor,
  used by instance-mode geometry queries.

---

## 7. Where to change things

| You want to… | Look at |
|---|---|
| Change classes or their order | `categories` in `config/isaac_voxeland.yaml`. The order sets the colours (§5.4). Keep it fixed during a run. |
| Map new Isaac label ids | Nothing, as long as the `semantic_labels` JSON names them. `label_names` is only the fallback table. |
| Use another camera or dataset | The `camera` launch arg, the remaps of `point_cloud_xyzrgb_node`, and the adapter topics. Any organized XYZRGB cloud + label image works. |
| Change the frame points are sent in | The `output_frame` launch arg. |
| Feed model output with confidence | `confidence_topic` / `confidence_bins` (see [CLOUD_IN_SPEC.md](CLOUD_IN_SPEC.md)). |
| Tune the occupancy model | The `sensor_model.*` params in `config/voxeland_server.yaml`. Note that `onParameter` is a no-op, so changes need a restart. |
| Change how evidence is weighted | `Semantics::UpdateProbabilities` (per point) and the scores in the adapter's instance tables (1.0 / 0.05). |
| Change the export format | The `toPLY` / `getHeaderPLY` functions of the cell type, and `VoxelandServer::mapToPLY`. |
| Change the colour scheme | `SemanticMap::color_palette` / `indexToHexColor`, and `toColor()` of the cell type. |


## 8. Known quirks (from reading the code)

- **Mode and categories are latched from the first `/cloud_in` message.** If
  the first message is wrong (for example, published before parameters
  settled), restart the server.
- `/voxeland_server/reset` is broken (§5.1).
- `modeHas()` tests "any bit", so every semantic mode also counts as instance mode.
  In our mode this only causes log spam and empty instance JSON files on `save_map`.
  It is fixed by `patches/voxeland_modehas.patch` (see `patches/README.md`).
- `publishAll` re-sends the whole map on every frame (§5.3).
- `rgb` per voxel is the **last** colour written, not an average.
- Upstream code carries only the `modeHas()` patch. Everything else lives in
  `ninsar_mapping`.
