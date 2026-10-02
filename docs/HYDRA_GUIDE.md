# Hydra guide

Hydra builds a **3D scene graph** (DSG) online from RGB-D + semantics + odometry.
This page covers how the pipeline is put together, which package does what, and
how to render a hand-built scene graph in RViz without running the pipeline.

## What a scene graph is

A `spark_dsg::DynamicSceneGraph` is a layered graph. Every node has a layer, a
`NodeSymbol` id (prefix char + index, e.g. `O(3)`), and a typed attribute struct.
Edges connect nodes inside a layer or between layers (parent → child).

| Layer id | Name      | Typical symbol | Attributes               | Meaning                                |
|----------|-----------|----------------|--------------------------|----------------------------------------|
| 1        | SEGMENTS  |                | `ObjectNodeAttributes`   | pre-object mesh segments               |
| 2        | OBJECTS   | `O`            | `ObjectNodeAttributes`   | objects: label, bounding box           |
| 2 / `a`  | AGENTS    | `a`            | `AgentNodeAttributes`    | robot poses (partition `a` of layer 2) |
| 3        | PLACES    | `p`            | `PlaceNodeAttributes`    | free-space graph (GVD nodes)           |
| 4        | ROOMS     | `R`            | `RoomNodeAttributes`     | rooms clustered from places            |
| 5        | BUILDINGS |                | `SemanticNodeAttributes` | buildings                              |

The graph may also hold a mesh. It carries free-form JSON `metadata`, and the
visualizer reads `metadata["labelspaces"]["_l{layer}p{partition}"]` (a list of
`[label_id, "name"]` pairs) to show class names instead of ids.

On disk a graph is either binary (default) or JSON (choose a `.json` extension).
Both go through `DynamicSceneGraph::save()/load()`.

## How the pipeline runs

`hydra_ros/app/hydra_node.cpp` → `HydraRosPipeline`
([hydra_ros_pipeline.cpp](../../hydra_ros/hydra_ros/src/hydra_ros_pipeline.cpp)) wires
up these modules. Each one runs in its own thread and they talk through queues:

```
ROS topics (color, depth, labels) + TF
        │   RosInputModule (hydra_ros/src/input)
        ▼
active_window   hydra/src/active_window, reconstruction
        │   local TSDF + semantic voxels around the robot, marching-cubes mesh
        ▼
frontend        hydra/src/frontend, places
        │   mesh → kimera_pgmo deformation graph
        │   GVD → places graph (layer 3)
        │   mesh segmentation by label → objects (layer 2)
        │   agent poses, place ↔ object connections
        ▼
backend         hydra/src/backend, rooms
        │   pose-graph + mesh deformation optimization (kimera_pgmo / kimera_rpgo)
        │   node merging, room detection from the places graph (layer 4)
        ▼
sinks           ROS DSG publisher (/hydra/backend/dsg), ZMQ, logger
        ▼
hydra_visualizer → RViz MarkerArray
```

An optional loop-closure module (`hydra/src/loop_closure`, `enable_lcd:=true`)
matches scene-graph descriptors and sends loop closures to the backend.

Everything is configured through `config_utilities`: YAML files and inline YAML are
merged on the command line (see `--config-utilities-file` in
[hydra.launch.yaml](../../hydra_ros/hydra_ros/launch/hydra.launch.yaml)), and
factory `type:` keys choose implementations at runtime.

## Packages in `src/`

| Package | Role |
|---------|------|
| `spark_dsg` | Scene-graph data structure, (de)serialization, labelspaces, ZMQ transport, Python bindings (`import spark_dsg`, `spark-dsg` CLI) |
| `hydra` | ROS-free core pipeline: reconstruction, places/GVD, objects, rooms, backend, loop closure |
| `hydra_ros/hydra_ros` | ROS 2 wrapper: input receivers, publishers/visualizers for internal state, launch files, dataset configs (`config/datasets/*.yaml`) |
| `hydra_ros/hydra_visualizer` | Turns a DSG into RViz markers; standalone node with streaming and static launch files |
| `hydra_ros/hydra_msgs` | `DsgUpdate` msg (serialized DSG over ROS), `GetDsg` srv |
| `kimera_pgmo` | Mesh compression + deformation graph; also the RViz mesh display plugin (`kimera_pgmo_rviz`) |
| `kimera_rpgo` | Robust pose-graph optimization (GNC/PCM) used by the backend |
| `pose_graph_tools` | Pose-graph messages and utilities |
| `config_utilities` | YAML config/factory system, plus `config_utilities_ros` GUI for live reconfiguration |
| `ianvs` | ROS 2 node-handle helpers used throughout |
| `spatial_hash` | Voxel block hashing used by the volumetric maps |
| `teaser_plusplus` | Registration for loop closure |
| `semantic_inference` | Optional closed/open-set segmentation networks feeding the label image |

## How hydra_visualizer works

`hydra_visualizer_node`
([visualizer_node.cpp](../../hydra_ros/hydra_visualizer/src/visualizer_node.cpp)):

1. **Graph source** (`graph: {type: ...}`), a `GraphWrapper`:
   - `GraphFromRos`: subscribes to `DsgUpdate` (live Hydra; `streaming_visualizer.launch.yaml`)
   - `GraphFromZmq`: receives the graph over ZMQ (`streaming_visualizer.launch.yaml use_zmq:=true`)
   - `GraphFromFile`: loads a saved file (`static_visualizer.launch.yaml`)
2. **SceneGraphRenderer** draws each layer using
   [visualizer_config.yaml](../../hydra_ros/hydra_visualizer/config/visualizer_config.yaml):
   z-offset per layer (`layer_z_step × z_offset_scale`), node color adapters
   (`LabelColorAdapter`, `ParentColorAdapter`, `IdColorAdapter`, ...), text adapters
   (`IdTextAdapter` by default, `LabelTextAdapter`), bounding boxes, and intra- and
   interlayer edges. Everything goes out as one `MarkerArray` on `~/graph`.
3. **Plugins** ([visualizer_plugins.yaml](../../hydra_ros/hydra_visualizer/config/visualizer_plugins.yaml))
   publish extras on their own topics: `~/mesh` (MeshPlugin), `~/places_basis_points`,
   `~/agent_poses`, `~/static_objects`/`~/dynamic_objects` (Khronos).
4. Renderer settings are exposed to the config GUI (`start_config_gui:=true`) and can
   be tweaked live.

## Visualize a hand-built scene graph (no data, no pipeline)

Needs the spark_dsg Python bindings in `.venv` (`.venv/bin/pip install ./src/spark_dsg`).

```bash
cd ~/workspaces/ninsar-semantic-segmentation
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 1. build a toy graph: 2 rooms, 18 places, 7 labeled objects with bounding boxes
.venv/bin/python tools/make_toy_dsg.py -o toy_dsg.json

# 2. render it
ros2 launch hydra_visualizer static_visualizer.launch.yaml scene_graph:=$PWD/toy_dsg.json
#    add start_config_gui:=false to skip the reconfigure GUI

# 3. after regenerating or hand-editing the JSON, reload without restarting
ros2 service call /hydra_dsg_visualizer/graph/reload std_srvs/srv/Empty
#    or switch to a different file
ros2 topic pub --once /hydra_dsg_visualizer/graph/load std_msgs/msg/String "{data: $PWD/other.json}"
```

`scene_graph:=` must be an absolute path. To make your own scene, edit `SCENE` and
the label dicts in [make_toy_dsg.py](tools/make_toy_dsg.py), or build the graph from
any source (e.g. a model's detections) using the same calls:

```python
import numpy as np, spark_dsg as dsg
G = dsg.DynamicSceneGraph()
a = dsg.ObjectNodeAttributes()
a.position = np.array([1.0, 2.0, 0.5])
a.semantic_label = 0
a.bounding_box = dsg.BoundingBox(np.float32([0.5, 0.5, 1.0]),   # dimensions
                                 np.float32([1.0, 2.0, 0.5]))   # center
G.add_node(2, dsg.NodeSymbol("O", 0), a)       # layer 2 = OBJECTS
G.insert_edge(dsg.NodeSymbol("p", 0), dsg.NodeSymbol("O", 0))   # parent → child (both must exist)
G.metadata.add({"labelspaces": {"_l2p0": [[0, "chair"]]}})
G.save("my_graph.json", include_mesh=False)
```

Notes:
- Nodes are drawn at `z + layer offset`, so rooms float above places, and places
  above objects. Set `renderer: {collapse_layers: true}` to flatten them.
- Room text shows ids (`R(0)`) because layer 4 uses the default `IdTextAdapter`. Add
  `adapter: {type: LabelTextAdapter}` under `layers: 4: text:` to show room names.
- For a quick look outside ROS: `.venv/bin/spark-dsg visualize toy_dsg.json`
  opens a browser viewer at http://localhost:8080 (`--port` to change it). It needs
  the `viz` extra; see [Installing](#installing) below.
- Graphs saved by a real Hydra run open the same way.

## spark_dsg Python API

Everything below was tried against the spark_dsg 1.1.3 built from `src/spark_dsg`.
The Python bindings wrap the same C++ types that Hydra and the visualizer use, so
anything you build in Python loads unchanged in `hydra_visualizer`.

### Installing

```bash
.venv/bin/pip install './src/spark_dsg[viz]'
```

- `./src/spark_dsg` is a local project directory, not a PyPI name. pip reads its
  `pyproject.toml`, and the build backend (`scikit-build-core`) runs CMake with
  `SPARK_DSG_BUILD_PYTHON=ON` to compile the pybind11 module `_dsg_bindings`. The result
  is installed as a wheel with the C++ linked in statically, independent of the colcon
  build.
- `[viz]` selects an *optional dependency group* declared in `pyproject.toml`
  (`[project.optional-dependencies] viz = ["trimesh", "viser"]`). Without it you get
  only the core dependencies (`click numpy shapely zmq`). The quotes stop zsh from
  treating `[...]` as a glob.
- If spark_dsg is already installed, `.venv/bin/pip install trimesh viser` does the
  same thing without rebuilding. Both are already in this `.venv`.
- Optional extras that are not in any group: `networkx` (for `spark_dsg.networkx`)
  and `torch` + `torch_geometric` (for `G.to_torch()`). Without networkx the conversion
  only warns and returns `None`.

### Data model

```
DynamicSceneGraph
├── metadata                  free-form JSON (labelspaces live here)
├── mesh                      optional spark_dsg.Mesh
├── layers                    one per LayerKey(layer=int, partition=0)
│   └── nodes                 NodeSymbol → SceneGraphNode(attributes)
│   └── edges                 intra-layer edges
├── layer_partitions          extra sub-layers: LayerKey(layer, partition≠0)
└── interlayer edges          between any two different LayerKeys
```

- **`LayerKey(layer, partition)`**: `layer` is any integer id and `partition`
  defaults to 0. Layers may also have **names** (`"PLACES" → 3`). A name is only an
  alias for a key.
- **Partitions** are parallel sub-layers at the same level. Hydra uses them for
  per-robot agent poses (`LayerKey(2, 'a')`) and 2D places (`LayerKey(3, 1)`).
  A partition id can be an int or a single character.
- **`NodeSymbol(char, index)`** packs a character and an index into one 64-bit id
  (`NodeSymbol('p', 12).value`, `.str() == 'p12'`). The id must be **unique across the
  whole graph**: adding `P0` to a second layer returns `False`. The character is only a
  naming convention and is not tied to a layer.
- **Attributes** set a node's type. They form a C++ class hierarchy:

| Class | Adds (on top of its parent) | Used for |
|-------|-----------------------------|----------|
| `NodeAttributes` | `position`, `is_active`, `is_predicted`, `last_update_time_ns`, `metadata` | anything |
| `SemanticNodeAttributes` | `name`, `color`, `semantic_label`, `semantic_feature`, `bounding_box` | generic labeled node |
| `ObjectNodeAttributes` | `world_R_object`, `registered`, `mesh_connections` | objects |
| `RoomNodeAttributes` | `semantic_class_probabilities` | rooms, regions |
| `PlaceNodeAttributes` | `distance`, `num_basis_points`, frontier flags, mesh connections | 3D free-space places |
| `Place2dNodeAttributes` | `boundary`, ellipse, mesh connections | 2D surface places |
| `TraversabilityNodeAttributes` | `boundary`, `distance`, observation times | traversability |
| `AgentNodeAttributes` | `timestamp`, `world_R_body`, `external_key`, DBoW | robot poses |
| `KhronosObjectAttributes` | trajectories, per-object mesh | Khronos dynamic objects |

- **`EdgeAttributes`**: `weight`, `weighted`, `metadata`.

### Building a graph with your own layers

`DynamicSceneGraph()` starts with Hydra's layers (2, 3, 4, 5 and their names).
`DynamicSceneGraph(empty=True)` starts with none, so you choose the whole hierarchy:

```python
import numpy as np
import spark_dsg as dsg

G = dsg.DynamicSceneGraph(empty=True)
G.add_layer(10, name="FARM")
G.add_layer(7, name="PARCELS")
G.add_layer(1, name="PLANTS")
G.add_layer(1, "a", name="ROBOT_A")      # partition 'a' of layer 1

def semantic(pos, label=dsg.SemanticNodeAttributes.NO_SEMANTIC_LABEL, name=""):
    a = dsg.SemanticNodeAttributes()
    a.position = np.array(pos, dtype=float)
    a.semantic_label = label
    a.name = name
    return a

G.add_node("FARM", dsg.NodeSymbol("F", 0), semantic([0, 0, 0], name="farm_A"))
G.add_node(7, dsg.NodeSymbol("r", 0), semantic([0, 0, 0], name="parcel_1"))
G.add_node(1, dsg.NodeSymbol("P", 0), semantic([0.5, 0, 0], label=0))
G.add_node(1, dsg.NodeSymbol("a", 0), dsg.AgentNodeAttributes(), "a")  # into the partition

G.insert_edge(dsg.NodeSymbol("F", 0), dsg.NodeSymbol("r", 0))   # FARM → PARCELS
G.insert_edge(dsg.NodeSymbol("r", 0), dsg.NodeSymbol("P", 0))   # PARCELS → PLANTS

G.set_labelspace(dsg.Labelspace({0: "lettuce", 1: "weed"}), 1)  # layer 1, partition 0
G.metadata.add({"site": "farm_A"})
G.save("farm.json", include_mesh=False)
```

### Rules

These were checked against the bindings, and each call returns `True` or `False`:

| What you do | What happens |
|-------------|--------------|
| `add_node` into a layer that doesn't exist | layer is created on the fly (`True`) |
| same `NodeSymbol` in two layers | second `add_node` returns `False` |
| edge between two layers | allowed; the node in the **higher layer id is the parent**, whatever order you pass the arguments in |
| edge that skips layers (10 → 1) | allowed |
| edge inside one layer | allowed; the nodes become `siblings()` |
| edge between a partition and its base layer (`LayerKey(1,'a')` ↔ `LayerKey(1,0)`) | allowed; treated as siblings |
| a second parent for a node | allowed; `parents()` lists both, but `has_parent()` returns `False` and `get_parent()` returns `None` |
| `insert_edge(..., enforce_single_parent=True)` | replaces the node's existing parent |
| self-loop, or edge to a missing node | `False` |
| `remove_node` | also removes its edges |
| Python subclass of an attribute class | stored as the **base class**; extra Python fields are silently dropped |

The model is a **layered, undirected graph with a parent/child hierarchy derived from
layer order**. You choose how many layers there are, their ids and names, and which
layers connect. The fixed parts are: one global id space for nodes, parent = higher
layer, and attribute types limited to the C++ classes above.

### Attaching your own data

You can't add new attribute classes from Python; that needs a C++ class registered with
the serializer. Use these instead, all of which survive both `.json` and binary
save/load:

- **Per-node or per-edge JSON**: `attrs.metadata.set({...})` or `.add({...})`, read back
  with `.metadata.get()`. Good for species, health scores, tags, timestamps, IDs in
  other systems.
- **Vectors**: `attrs.semantic_feature = np.float32 column vector (N×1)`, for example
  CLIP or other embeddings.
- **Class names per layer**: `G.set_labelspace(dsg.Labelspace({id: name}), layer, partition)`.
  `G.get_labelspace(1).get_node_category(node)` returns the name. The visualizer's
  `LabelTextAdapter` reads the same data.
- **Graph-wide JSON**: `G.metadata.add({...})`.
- **Geometry**: `bounding_box` (`BoundingBox(dims, center)`, with AABB/OBB types and
  `contains`, `intersects`, `compute_iou`); `G.mesh` for a triangle mesh with per-vertex
  colors and labels.

### Reading and traversing

```python
G = dsg.DynamicSceneGraph.load("farm.json")
for layer in G.layers:                        # base layers (partition 0)
    print(layer.key, layer.num_nodes(), layer.num_edges())
for part in G.layer_partitions:               # partitions (not included in G.layers)
    print(part.key, part.num_nodes())         # key prints as 1[97] for partition 'a'
G.layer_names                                 # {"FARM": LayerKey(10), ...}

node = G.get_node(dsg.NodeSymbol("r", 0))     # find_node() returns None instead of raising
node.layer, node.attributes.position, node.attributes.name
node.children(), node.parents(), node.siblings()  # sets of raw int ids
node.connections()                                # list of all neighbours
[dsg.NodeSymbol(c).str() for c in node.children()]  # ids → ["P0"]

for n in G.nodes: ...                         # every node, partitions included
for e in G.edges: ...                         # e.source, e.target, e.info.weight
G.interlayer_edges                            # only edges between different LayerKeys
G.unpartitioned_nodes / G.unpartitioned_edges

G.remove_node(sym); G.remove_edge(a, b); G.remove_layer(7)
G.clone(); G.transform(T_4x4)                 # deep copy / rigid transform of everything
```

Helpers:
- `dsg.add_bounding_boxes_to_layer(G, "PARCELS", "PLANTS")` fills each parent's AABB
  from its descendants' positions. It takes layer **names** (the graph must have
  them), and a parent with only one child gets an `invalid` box because the box has
  zero size.
- `spark_dsg.networkx.graph_to_networkx(G)` (needs networkx).
- `G.to_torch(...)` converts to PyTorch Geometric data for GNNs (needs torch).
- `G.to_binary()` / `DynamicSceneGraph.from_binary(bytes)` for sending graphs over the
  network. `spark_dsg.zmq` has the same ZMQ sender and receiver Hydra uses.

### Rendering custom layers in RViz

`hydra_visualizer` draws a layer **only if the renderer config lists it**. A layer
that isn't listed gets `visualize: false`, so it stays hidden. The stock
[visualizer_config.yaml](../../hydra_ros/hydra_visualizer/config/visualizer_config.yaml)
only covers layers 2–4 and partitions of layers 2–3, so a new layer id (or Hydra's
own layer 5) needs its own config file:

```yaml
# farm_viz.yaml
renderer:
  layer_z_step: 2.0          # height between layer "floors"
  collapse_layers: false
  layers:
    1:  {visualize: true, z_offset_scale: 1.0,
         nodes: {scale: 0.2, color: {type: LabelColorAdapter}},
         text: {draw: true, adapter: {type: LabelTextAdapter}},
         bounding_boxes: {draw: true}}
    7:  {visualize: true, z_offset_scale: 2.0, nodes: {scale: 0.3, color: {type: IdColorAdapter}}}
    10: {visualize: true, z_offset_scale: 3.0, nodes: {scale: 0.5}, text: {draw: true}}
  partitions:                # one setting per layer id, shared by all its partitions
    1:  {visualize: true, z_offset_scale: 1.0, nodes: {scale: 0.3}}
```

```bash
ros2 launch hydra_visualizer static_visualizer.launch.yaml \
  scene_graph:=$PWD/farm.json visualizer_config_path:=$PWD/farm_viz.yaml
```

The file replaces the default config, so copy over any stock layers you still want.
Interlayer edges are drawn for any pair of visible layers, including edges that skip
layers. The full set of per-layer options (edges, text, bounding boxes, boundaries,
color adapters) is in
[layer_info.h](../../hydra_ros/hydra_visualizer/include/hydra_visualizer/layer_info.h).
You can tune them live with the config GUI.

## Example: a farm scene graph

[make_farm_dsg.py](../scripts/make_farm_dsg.py) builds an agriculture graph that uses the
connection model: hierarchy edges, edges that skip layers, spatial edges, and a
navigation partition. [farm_viz.yaml](../config/farm_viz.yaml) is the matching renderer
config.

```bash
.venv/bin/python src/ninsar_mapping/scripts/make_farm_dsg.py   # -> ~/data/ninsar/farm_dsg.json, then queries the saved file
ros2 launch hydra_visualizer static_visualizer.launch.yaml \
  scene_graph:=$HOME/data/ninsar/farm_dsg.json \
  visualizer_config_path:=$(ros2 pkg prefix --share ninsar_mapping)/config/farm_viz.yaml
```

| Layer | Content | Parent |
|-------|---------|--------|
| 4 `FARM` | `F0` | none (root) |
| 3 `PARCELS` | parcels A–D, box = boundary incl. headlands | farm |
| 2 `ROWS` | one crop type per row (parcel D mixes cabbage and vine rows) | parcel |
| 1 `DETECTIONS` | crops (cabbage, vine, lemon), buildings, obstacles | crop → row; building → **farm** (skips 2 layers); obstacle → parcel it lies in, else farm |
| 1 `'n'` `NAVIGATION` | lane, headland and road waypoints | none; spatial links only |

Every edge has `EdgeAttributes.metadata = {"relation": ...}`, and spatial edges have
`weight` = planar distance:

| Relation | Between | Meaning |
|----------|---------|---------|
| `contains` | layer N → lower layer | membership (the hierarchy) |
| `next_in_row` | crop ↔ crop | order along a row |
| `near` | obstacle ↔ detection | axis-aligned boxes closer than 2 m |
| `adjacent_row` / `adjacent_parcel` | inside ROWS / PARCELS | neighbouring rows; parcels whose boundaries are closer than 6 m |
| `lane` / `headland` / `road` / `entrance` | inside NAVIGATION | drivable graph; a lane segment an obstacle sits on gets `{"blocked": true, "blocked_by": [...]}` |
| `access` | DETECTIONS ↔ NAVIGATION | crop ↔ nearest waypoint in each lane beside its row; building ↔ nearest road waypoint |

The navigation waypoints sit in a **partition** of layer 1, not in a layer of their
own. Edges between a partition and its base layer are sibling links, so connecting a
waypoint to a crop doesn't make either one the other's parent. A separate layer id
would force every waypoint–crop edge into a parent/child link.

The script's report shows queries you can run on the graph: walking parents up to the
farm, counting crops per parcel through the hierarchy, finding "near" neighbours, and a
shortest route (Dijkstra over `NAV_RELATIONS` edges) from the barn to the crops beside
each obstacle, with and without the blocked lane segments.
