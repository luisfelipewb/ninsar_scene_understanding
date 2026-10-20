# ninsar_perception

The NINSAR perception front end, from camera images to per-frame 3D objects.

- `semantic_cloud_adapter` (Python) turns an organized depth cloud, a label image and its labels
  JSON into the semantic cloud `cloud_in` (`segmentation_msgs/SemanticPointCloud`), the input of
  both the Voxeland server and the detector.
- `object_detector` (C++, PCL) turns each `cloud_in` into per-frame 3D objects. Ground is skipped,
  and every other class goes through VoxelGrid and Euclidean clustering, with a gravity-aligned
  yaw box per cluster. The results are published in the sensor frame as
  `vision_msgs/Detection3DArray` (landmark list), a `MarkerArray` (RViz) and a debug cluster cloud.

| path | |
|---|---|
| `ninsar_perception/semantic_cloud_adapter.py` | the adapter node |
| `src/object_detector_node.cpp` | the detector node |
| `include/ninsar_perception/clustering.hpp`, `src/clustering.cpp` | pure code: instance table decoding, class split, clustering, boxes, frames |
| `test/` | gtests and pytest (`colcon test --packages-select ninsar_perception`) |

The `cloud_in` contract: [docs/specs/CLOUD_IN_SPEC.md](../../../docs/specs/CLOUD_IN_SPEC.md). The
detector's output format, parameters and limitations:
[docs/specs/OBJECT_DETECTOR.md](../../../docs/specs/OBJECT_DETECTOR.md). Both nodes are launched
and configured for a scenario from `ninsar_bringup` (`launch/isaac_voxeland.launch.xml`,
`config/isaac/`).
