# ninsar_mapping

Semantic voxel mapping around the Voxeland server: its configuration and the nodes that work with
its map.

| path | |
|---|---|
| `config/voxeland_server.yaml` | Voxeland server parameters (resolution, sensor model, semantics mode) |
| `config/voxel_query.yaml` | `voxel_query` defaults |
| `ninsar_mapping/voxel_query.py` | node: box/class queries on the live map (`ros2 run ninsar_mapping voxel_query`) |
| `ninsar_mapping/voxel_store.py` | pure code behind `voxel_query`: palette, class decoding, box selection |
| `ninsar_mapping/tf_to_pose_cov.py` | node: a TF frame as `PoseWithCovarianceStamped`, for Hydra |
| `test/` | pytest (`colcon test --packages-select ninsar_mapping`) |

How to run it on the Isaac Sim bag: [docs/guides/run_isaac_bag.md](../../../docs/guides/run_isaac_bag.md).
Voxeland in depth: [docs/guides/VOXELAND_GUIDE.md](../../../docs/guides/VOXELAND_GUIDE.md). The
Voxeland server needs one upstream patch, applied by `ros/patches/apply.sh`
([why](../../patches/README.md)).
