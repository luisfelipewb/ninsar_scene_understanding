# ninsar_bringup

System launch files and the configs that are specific to one scenario. Node defaults live in the
node's own package; this package only says which nodes run together and how they're set up for a
given robot or simulator.

| path | |
|---|---|
| `launch/isaac_voxeland.launch.{xml,yaml}` | Voxeland + perception on an Isaac Sim bag |
| `config/isaac/` | Isaac-specific params: `semantic_cloud_adapter.yaml`, `object_detector.yaml` |
| `config/*_viz.yaml` | hydra_visualizer renderer configs for farm scene graphs |
| `rviz/` | rviz configs for the launch files |

```bash
ros2 launch ninsar_bringup isaac_voxeland.launch.xml
```
