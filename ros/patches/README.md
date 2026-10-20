# Patches to upstream clones

Everything Isaac-specific lives in this package: launch, config, rviz, and the
[`semantic_cloud_adapter`](../ninsar_mapping/semantic_cloud_adapter.py) node that feeds the server
([contract](../docs/CLOUD_IN_SPEC.md)). The upstream clones should stay at their base commits,
except for the patches listed here. Each one changes compiled code that no node, parameter or
launch file in this package can reach.

| patch | repo | base commit | why it can't live outside |
|---|---|---|---|
| `voxeland_modehas.patch` | `src/voxeland` (MAPIRlab/Voxeland, `humble`) | `389e3ec27b8b49caf252948939366381cad0b48c` | See below. |

`src/voxeland/voxeland_robot_perception` (MAPIRlab/voxeland_robot_perception @ `626f413`) needs
**no** patch: this package doesn't use its node.

## `voxeland_modehas.patch`

`VoxelandServer::modeHas()` tests "any bit" (`(currentMode & mode) != 0`). Because
`SemanticsInstances = Semantics | 1<<3`, `modeHas(SemanticsInstances)` is true in **every**
semantic mode, including the dense `RGBSemantics` mode we run (`semantics_as_instances: false`).
The bug is on every upstream branch as of 2026-10.

Without the patch the map is still correct. The instance-only code paths reach a fallback that
only logs an error. The side effects are:

- The debug GUI logs `Tried to auto-template a semantics function with non-semantic enum value!`
  every 30 ms (about 33 lines/s) from the first cloud onward, which buries the server output.
- `save_map` logs the same error once and writes empty `voxeland_instanceMap*.json` files.

There's no outside fix. The GUI is always compiled in (`SET(ENABLE_DEBUG_GUI ON)` is a plain
CMake variable, so `-D` can't override it). Setting the `VXL` logger to `fatal` would also hide
the real messages. This would make a good upstream PR.

## Apply on a fresh clone

```bash
cd src/voxeland && git checkout 389e3ec && git apply ../ninsar_mapping/patches/voxeland_modehas.patch
colcon build --packages-select voxeland
```