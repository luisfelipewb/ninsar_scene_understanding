# Object detector (`ninsar_perception/object_detector`)

A per-frame perception front end. It turns each semantic camera cloud (`cloud_in`) into a list of
3D objects: class, centroid, spread and a gravity-aligned box. Ground is skipped. The points of
each other class are clustered, and every cluster becomes one detection. The detections are
**measurements in the sensor frame**, meant for RViz and for a SLAM or tracking back end.

```
cloud_in (SemanticPointCloud, zed_x_front + pose in world)
  │  decode instances[] → class + score per point          (docs/specs/CLOUD_IN_SPEC.md)
  │  drop ground / unknown / non-finite / beyond max_range
  │  rotate by the pose's orientation → level frame (z = world up, origin = sensor)
  ▼
per class: VoxelGrid → KdTree EuclideanClusterExtraction → minAreaRect yaw box + centroid/covariance
  ▼
back to the sensor frame → ~/detections   vision_msgs/Detection3DArray
                           ~/markers      visualization_msgs/MarkerArray
                           ~/clusters     sensor_msgs/PointCloud2 (debug)
```

The node is C++ on PCL 1.14. There are no maintained PCL Python bindings for Python 3.12 / Jazzy,
which is why it lives in its own `ament_cmake` package, `ros/src/ninsar_perception`. Its pure code
(`clustering.{hpp,cpp}`) has gtests. It needs no TF: it levels the points with the orientation
carried in `cloud_in.pose`.

## Running

`isaac_voxeland.launch.{xml,yaml}` starts it by default (`detector:=false` to skip it), with
`ninsar_bringup`'s `config/isaac/object_detector.yaml`. RViz shows the boxes as "Detections". "Clusters" is
disabled by default; enable it to see the clustered points coloured by cluster.

Standalone:

```bash
ros2 run ninsar_perception object_detector --ros-args \
    --params-file $(ros2 pkg prefix --share ninsar_bringup)/config/isaac/object_detector.yaml
```

## Output

Everything is published with the **cloud's header**: frame `zed_x_front` (the adapter's
`output_frame`) and the cloud stamp. A detection is what the sensor saw at that instant. It is not
a world estimate, so a SLAM back end gets observations relative to the robot without the current
pose estimate baked in. RViz places them with TF.

### `~/detections`: `vision_msgs/Detection3DArray` (the landmark list)

`Detection3DArray` is the standard ROS message for 3D detections and needs no custom types.
It carries a per-detection class, score, pose **with covariance**, and a box. Each `Detection3D`:

| Field | Content |
|---|---|
| `header` | cloud header |
| `id` | `""`: no tracking yet; data association is the tracker's / SLAM's job |
| `results[0].hypothesis.class_id` | class name from `cloud_in.categories` (`tree`, `grapevine`, …) |
| `results[0].hypothesis.score` | mean class score of the cluster's points. 1.0 with ground-truth labels; the mean bin confidence with the adapter's `confidence_topic` encoding |
| `results[0].pose.pose.position` | **cluster centroid** (mean of the downsampled points): the landmark point |
| `results[0].pose.pose.orientation` | box orientation |
| `results[0].pose.covariance` | 6×6 row-major. The xyz block is the **spread** of the cluster's points (population covariance), rotated into the sensor frame. It describes the object's extent, not sensor noise; a back end should combine it with its own measurement model. The rotation block is 0. |
| `bbox.center` | box centre and orientation: yaw about **world up**, expressed in the sensor frame |
| `bbox.size` | full side lengths; `size.x ≥ size.y`, so the box x axis is the long horizontal side |

Box convention: the same as spark_dsg / the farm DSG RAABB (centre, dimensions, yaw about z). To
get the world box, apply the sensor pose: `world_T_sensor · bbox.center`. The box's z axis is then
world z, and its yaw is `atan2(R[1,0], R[0,0])` of the result.

To display `~/detections` directly instead of the markers: `sudo apt install ros-jazzy-vision-msgs-rviz-plugins`.

### `~/markers`: `visualization_msgs/MarkerArray`

Each frame starts with a `DELETEALL`. Then, per detection:
- a wireframe box (`LINE_LIST`, namespace = class)
- a label `class (n)` (`<class>/label`), where `n` is the number of points after downsampling

Colours are the **Voxeland map colours** of the class: palette index = position in `categories`,
as in `voxel_store.class_colors()`.

### `~/clusters`: `sensor_msgs/PointCloud2` (debug)

Fields `x, y, z, rgb, cluster_id`, in the sensor frame. `rgb` is the class colour.
`cluster_id` is the index of the detection in the same frame's `~/detections`. The cloud is built
only while something subscribes. Use it to tune `cluster_tolerance`.

## Parameters

| Parameter | Default | |
|---|---|---|
| `input_topic` | `cloud_in` | |
| `max_range` | 10.0 m | Sensor frame. `cloud_in` keeps farther points for free-space carving; they're dropped here. |
| `min_score` | 0.0 | Drop points whose class score is lower. |
| `ignore_classes` | `[ground, unknown]` | |
| `classes` | `[""]` = all | Classes to detect. Only these get `per_class.*` parameters. |
| `gravity_align` | true | Boxes upright in world. With `false`, the sensor's own z is "up". |
| `publish_clusters` | true | |
| `marker_line_width` | 0.03 m | |
| `voxel_size` | 0.05 m | VoxelGrid leaf; ≤ 0 disables downsampling. |
| `cluster_tolerance` | 0.2 m | Keep it ≥ 2 × `voxel_size`, or neighbouring voxels split apart. |
| `min_cluster_size` / `max_cluster_size` | 20 / 100000 | Point counts after downsampling. Clusters above the max are **dropped**, not split. |
| `max_dimension` | 0 (off) | Drop boxes with a longer side (m). |
| `per_class.<class>.{voxel_size, cluster_tolerance, min_cluster_size, max_cluster_size, max_dimension}` | the defaults | Only for names in `classes`. Write floats with a decimal point (`0.3`, not `0`), or the parameter type is rejected. |

If `voxel_size` is so small for the cloud's extent that VoxelGrid's int32 voxel index would
overflow, the class is clustered without downsampling and the node warns once. PCL itself would
warn on every frame and silently skip the downsampling.

Every 10 s the node logs frames, published, ms per frame (mean, max) and detections per class.

## On the tour bag (2026-10-15)

These come from the full bag played at 3×, with `ninsar_bringup`'s `config/isaac/object_detector.yaml` (676 frames,
all processed). Each frame took 22–81 ms on average and at most 138 ms, against a frame period
of about 175 ms.

| class | detections / frame | median box (m) | Isaac asset (m) |
|---|---|---|---|
| lemon (whole plant) | 0.42 | 1.39 × 1.11 × 1.84 | ~1.5 × 1.7 × 2.1 |
| cabbage | 0.65 | 0.47 × 0.38 × 0.26 | ~0.5 × 0.6 × 0.3 |
| obstacle | 0.03 | 2.73 × 1.55 × 2.13 | 2.9 × 2.3 × 4.2 |
| building | 0.14 | 0.25 × 0.11 × 2.01 (wall slices at ~8 m) | tens of m |

`lemon` and `grapevine` label whole plants, not fruit. Tree and grapevine never come within
10 m of the tour route, and fence barely does, so their parameters are untested. In the first
60 s, only lemon plants are in range.

## Limitations

- **Touching objects of the same class merge.** Neighbouring vines in a row, overlapping tree
  canopies, and lemons on one branch become one cluster. Semantic labels can't separate
  instances. A smaller `cluster_tolerance` helps only where there is a gap. The real fix is
  instance segmentation from Isaac (an instance-id image), clustering per instance id.
- **Single view.** The box and centroid cover the visible surface only, so the centroid is
  biased toward the sensor, and depth along the view ray is underestimated for solid objects.
- **No tracking.** Detections have no persistent ids across frames.
- **Large structures** (`fence`, `building`) give long boxes whose extent depends on the field of
  view. Use `max_dimension` or drop them from `classes` if they aren't useful as landmarks.
