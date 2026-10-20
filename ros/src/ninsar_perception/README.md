# ninsar_perception

The NINSAR perception front end. `object_detector` (C++, PCL) turns each semantic camera cloud
(`cloud_in`, `segmentation_msgs/SemanticPointCloud`) into per-frame 3D objects. Ground is skipped,
and every other class goes through VoxelGrid and Euclidean clustering, with a gravity-aligned yaw box per cluster. The
results are published in the sensor frame as `vision_msgs/Detection3DArray` (landmark list), a
`MarkerArray` (RViz) and a debug cluster cloud.

| path | |
|---|---|
| `src/object_detector_node.cpp` | the node |
| `include/ninsar_perception/clustering.hpp`, `src/clustering.cpp` | pure code: instance table decoding, class split, clustering, boxes, frames |
| `test/test_clustering.cpp` | gtests (`colcon test --packages-select ninsar_perception`) |

Output format, parameters and limitations: `ninsar_mapping/docs/OBJECT_DETECTOR.md`. The input
contract: `ninsar_mapping/docs/CLOUD_IN_SPEC.md`. It's launched and configured from `ninsar_mapping`
(`launch/isaac_voxeland.launch.xml`, `config/isaac_object_detector.yaml`).
