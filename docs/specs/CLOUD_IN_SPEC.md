# Voxeland `cloud_in` input specification

What `voxeland_server` actually reads from its input, and how
[`semantic_cloud_adapter`](../ninsar_mapping/semantic_cloud_adapter.py) produces it.
Traced through upstream `voxeland_server.cpp::insertCloudCallback`, `semantics_ros_wrapper.hpp`,
`voxeland_map/semantics.{hpp,cpp}`, `cell_types/{Semantics,RGBSemantics}.hpp` and
`probabilistic_map_templated.hpp` (MAPIRlab/Voxeland `389e3ec`).

Message: `segmentation_msgs/SemanticPointCloud` on `cloud_in`.

## Fields

| Field | How the server uses it | Constraint (breaking it is a silent bug) | Adapter |
|---|---|---|---|
| `cloud` field names | **First message only:** `rgb` sets the RGB bit and `instance_id` sets the Semantics bit. Together they fix the voxel cell type for the whole run. | The same field set in every message. | Always `x,y,z,rgb,instance_id` (mode `RGBSemantics`). |
| `cloud.x,y,z` (float32) | Moved into the map with `pose`. A point closer than `sensor_model.max_range` is a **hit** (occupancy + semantics). A farther point becomes a **miss ray** that only clears free space and adds no semantics. | **No NaN or inf.** NaN becomes a hit at an invalid voxel, and inf gives inf/inf = NaN on the miss ray. Finite far points are useful for free-space clearing. | Drops non-finite points and points nearer than `min_range`. Keeps far points unless `drop_far_points`. |
| `cloud.rgb` (float32, PCL-packed `0x00RRGGBB`) | Stored per voxel, last write wins. Shown only when the voxel's most likely class is "unknown". | Registered as `float` in `voxeland_map/pcl_utils.hpp`. | Copied from depth_image_proc. |
| `cloud.instance_id` (int32) | Each hit does `voxel.alphas += instances[instance_id].alphas`. | `0 <= id < len(instances)`. An id outside that range reads past the end of the server's table (undefined behaviour). | Checked per frame. |
| `instances[]` (`vision_msgs/Detection2D`) | A **local** table, rebuilt for every message. Entry `i` goes to index `atoi(instances[i].id)`. Its alpha vector is the sum of `results[k].score`, each added at `categoryIndexMap[results[k].class_id]`. | `id`s exactly `"0".."N-1"`. Every `class_id` must be in `categories`. An unknown name is silently looked up as index 0, so it adds evidence to the **first** category. | Fixed table, checked at startup (`check_table`). |
| `instances[].bbox`, `.header` | Only stored as object appearances, which instance mode and the JSON export use. | Ignored in dense mode. | Empty bbox, cloud header. |
| `categories` (string[]) | **First message only:** builds `default_categories` and `categoryIndexMap`. The **last** entry is the background, "unknown". | Fixed, complete and ordered from the first message on, with "unknown" last. | `categories` param + `"unknown"`. |
| `pose.pose` | Transform from the points' frame to the world frame. Its translation is the origin of every ray. | Must describe the frame of the xyz, at the cloud stamp. | TF `world_frame <- output_frame` at the cloud stamp. |
| `pose.covariance` | Unused. | — | Zeros. |
| `header.stamp` | Stamp of the published map and markers. | — | Cloud stamp. |
| `header.frame_id` | Unused: points are placed with `pose`. | — | `output_frame`, for debugging. |

## Semantics and uncertainty

Each voxel holds Dirichlet concentration parameters `alphas[C]` over `categories`. Every
hit adds the alpha vector of the point's local instance. The uncertainty is available in two places:

- `save_map` writes a per-voxel `uncertainty_categories` value (the expected Shannon entropy of the Dirichlet) to the PLY.
- `voxeland/get_class_distributions` returns the normalized probabilities.

Evidence grows with the number of hits, so dense, near-range surfaces weigh more.

A model's uncertainty can only enter through `instances[].results`: one class distribution
**per local instance**, not per pixel. A `Detection2D` may carry a full distribution, with
several `(class_id, score)` results.

| Encoding | Local instance table | When |
|---|---|---|
| Ground truth (default) | Instance 0 = `("unknown", 0.05)`. Instance `k+1` = `(categories[k], 1.0)`. | Isaac labels: one-hot. Every hit counts +1 for its class, so the entropy only reflects disagreement between views. |
| Confidence bins (`confidence_topic` set) | Instance 0 = `("unknown", 0.05)`. Instance `1 + k*B + b` = `(categories[k], p_b)` and `("unknown", 1 - p_b)`, where `p_b = (b + 0.5) / B`. | A model with a per-pixel top-1 confidence (32FC1). At most C·B + 1 instances. |

Pixels whose label id has no class name, or whose class is not in `categories`
(BACKGROUND, UNLABELLED, unmapped classes), use instance 0.

A full per-pixel softmax doesn't fit this interface without a further encoding, such as
clustering the distributions into local instances. That isn't implemented.

## Label ids

The label id → class name table comes from the `semantic_labels` JSON (Isaac Sim), matched by
its `time_stamp` to the label image. Ids are mapped by **name**, so the mapping holds if the
writer reassigns ids. Before the first JSON arrives, or if no JSON matches a frame's stamp,
the adapter uses the merged table, seeded from the `label_names` param.

`test/test_semantic_cloud_adapter.py` checks the constraints above on the encoding functions.
