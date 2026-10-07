# Farm scene graph specification

Draft v0, 2026-10-07. Structure of the farm scene graph (`spark_dsg::DynamicSceneGraph`)
used by `scripts/make_farm_dsg.py`, `scripts/make_isaac_farm_dsg.py` and the planned
GeoJSON importer.

## 1. Scope

- One graph describes one farm.
- The graph is built from prior knowledge (GeoJSON) and updated by robots: they confirm,
  correct, add and mark missing.
- Consumers: planners (graph queries, routing) and LLM queries (serialized graph).
- Task-agnostic. Task state (health, growth stage, counts, last treatment) is stored as
  node attributes, not as nodes.
- Out of scope: plant parts (fruit, cane, leaf), tasks and events as nodes, observation
  history.

## 2. Structure overview

The graph holds four sub-structures:

| Sub-structure | Location                    | Edges                                   | Answers                       |
|---------------|-----------------------------|-----------------------------------------|-------------------------------|
| Tree          | partition 0 of layers 1–4   | `contains`                              | where a thing is (one answer) |
| Overlays      | layer 4, partition `o`      | `member_of`                             | which cross-cutting groups apply |
| Navigation    | layer 1, partition `n`      | `lane`, `headland`, `road`, `entrance`, `access` | how to get there      |
| Agents        | layer 1, partition `a`      | `at`                                    | what is moving, and where     |

| Layer key | Name       | Prefix                | Attributes class         | Content                                  | Tree parent      |
|-----------|------------|-----------------------|--------------------------|------------------------------------------|------------------|
| 4         | FARM       | `F`                   | `SemanticNodeAttributes` | root; exactly one node                   | none             |
| 4 / `o`   | OVERLAYS   | `Z`                   | `SemanticNodeAttributes` | cross-cutting groups                     | none (not in tree) |
| 3         | AREAS      | `A`                   | `SemanticNodeAttributes` | non-overlapping subdivisions of the farm | farm             |
| 2         | ROWS       | `r`                   | `SemanticNodeAttributes` | lines of plants of one crop              | area (`parcel`)  |
| 1         | OBJECTS    | per category (§5.1)   | `ObjectNodeAttributes`   | individual physical things               | row, area or farm |
| 1 / `n`   | NAVIGATION | `w`                   | `SemanticNodeAttributes` | waypoints                                | none (not in tree) |
| 1 / `a`   | AGENTS     | `a`                   | `AgentNodeAttributes`    | robots, vehicles, people, animals        | none (not in tree) |

### spark_dsg behaviour this design relies on

- spark_dsg decides parent/child **from layer ids only**. An edge between different layer
  ids makes the higher-layer node a parent. An edge between nodes with the same layer id
  (any partitions) makes them siblings. The edge meaning is stored only in
  `EdgeAttributes.metadata["relation"]`.
- Consequence 1: every `member_of` edge makes the overlay a spark_dsg parent of its
  member. A node in an overlay therefore has more than one spark_dsg parent, and
  `has_parent()` returns `false` and `get_parent()` returns `None` for it. **Never use
  `get_parent()`. Use the `contains` helper in §8.**
- Consequence 2: edges between layer 1, 1/`n` and 1/`a` (`access`, `at`) are sibling
  edges and do not affect the tree.

## 3. Tree

Rules:

- **T1** Every partition-0 node except the farm has exactly one `contains` parent.
- **T2** The parent is in a higher layer. A `contains` edge may skip layers
  (farm → building).
- **T3** The child's footprint lies inside the parent's footprint, within a tolerance.
- **T4** The parent is the lowest tree node whose footprint contains the child: the row
  for a node in a row strip, otherwise the area, otherwise the farm.
- **T5** Nodes outside partition 0 (waypoints, agents, overlays) have no `contains`
  edges.
- **T6** Areas do not overlap each other.

## 4. Overlays

An **overlay** is a node that names a set of tree nodes chosen by a criterion other than
spatial containment. Examples: which valve waters them, what a soil survey found, where
robots may not drive. A group gets an overlay when it has its
own identity and attributes (a name, a valve, a rule). A property without its own
identity is a node attribute instead (`soil_type: clay`).

Rules:

- **O1** Overlay nodes live in layer 4 / `o`. They are not part of the tree. Layer 4 is
  above every member layer, so each `member_of` edge is an interlayer edge with the
  overlay on top.
- **O2** Membership is a `member_of` edge between the member and the overlay. Allowed
  members are partition-0 nodes of layers 1–3 and waypoints (1/`n`). The farm, other
  overlays and agents cannot be members.
- **O3** Membership is inherited downwards: a member's tree descendants are members too.
  Link the highest tree node that lies entirely inside the overlay: a whole parcel links
  the parcel, and part of a parcel links the rows or objects inside it.
- **O4** Each overlay kind is either exclusive or not. For an exclusive kind, a node
  together with its ancestors belongs to at most one overlay of that kind (or of that
  `scheme`, where a scheme is given). For a non-exclusive kind, any number is allowed.
- **O5** Overlays do not nest and are not linked to each other.
- **O6** The overlay's polygon is stored in `geometry` and used for display and for
  recomputing membership on import. Queries use the `member_of` edges, not the geometry.

| Kind                | Exclusive         | Typical members          | Attributes                                         |
|---------------------|-------------------|--------------------------|----------------------------------------------------|
| `irrigation_sector` | yes               | areas, rows              | `valve` (object uid), `method` (drip, sprinkler)   |
| `management_zone`   | yes, per `scheme` | areas, rows              | `scheme` (e.g. `vigor_2026`), `value`              |
| `restricted_area`   | no                | areas, rows, objects, waypoints | `restriction` (`no_entry`, `caution`), `reason` (`protected`, `slope`, `wet`, `soft_ground`, `private`, …) |

Waypoints are not in the tree, so they do not inherit membership (O3). Every waypoint
inside a `restricted_area` polygon must therefore be linked to it directly. Routing
excludes waypoints in a `no_entry` area, and planners may add a cost for waypoints in a
`caution` area.

## 5. Categories

### 5.1 OBJECTS (layer 1)

`semantic_label` gives the class through the layer's labelspace. Label ids are defined
in config, not in this spec.

| Category      | Prefix | Classes                                                          | Tree parent         | Notes                                    |
|---------------|--------|------------------------------------------------------------------|---------------------|------------------------------------------|
| crop          | `c`    | crop species (`cabbage`, `grapevine`, `lemon`, …)                | row                 | `variety`                                |
| weed          | `k`    | `weed`                                                           | row or area         | normally `source: perceived`             |
| vegetation    | `v`    | `tree`, `shrub`                                                  | area or farm        | non-crop individual plants               |
| row_support   | `i`    | `post`, `end_anchor`, `trellis_wire`, `drip_line`                | row                 | wire and drip line use LineString geometry |
| equipment     | `e`    | `soil_sensor`, `insect_trap`, `weather_station`, `valve`, `sprinkler` | row, area or farm | `device_id` for linking time series   |
| structure     | `B`    | `building`, `shed`, `silo`, `tank`, `fence`, `gate`, `wall`      | area or farm        | fence and wall use LineString geometry   |
| service_point | `s`    | `dock`, `refill_point`, `bin_drop`, `parking`                    | area or farm        | must have an `access` edge               |
| movable       | `m`    | `bin`, `crate`, `trailer`, `hose`                                | row, area or farm   | position may be stale                    |
| obstacle      | `o`    | `rock`, `pole`, `debris`, `unknown`                              | row, area or farm   | anything to avoid with no other class    |

Recommended crop attributes: `health` (0–1), `growth_stage_bbch`, `fruit_count`,
`row_index` (position along the row, from 0).

### 5.2 ROWS (layer 2)

One crop per row. The labelspace holds the crop classes of layer 1.

Attributes: `crop`, `variety`, `n_plants`, `length_m`, `plant_spacing_m`,
`heading_deg`, `row_index` (position within the parcel, from 0). Geometry: LineString
(the row axis).

### 5.3 AREAS (layer 3)

| `kind`       | Contains                          | Attributes                                                                        |
|--------------|-----------------------------------|-----------------------------------------------------------------------------------|
| `parcel`     | rows, objects                     | `crops`, `n_rows`, `row_spacing_m`, `heading_deg`, `headland_m`, `area_m2`         |
| `yard`       | structures, service points, movables | `area_m2`                                                                      |
| `water_body` | (none)                            | `water_type` (`pond`, `stream`, `ditch`)                                           |
| `vegetation` | vegetation objects                | `vegetation_type` (`hedgerow`, `woodland`, `grass_strip`)                          |

Geometry: Polygon.

### 5.4 FARM (layer 4)

One node. Attributes: `name`, `area_m2`. Geometry: Polygon (farm boundary).

### 5.5 NAVIGATION (layer 1 / `n`)

| `kind`     | Meaning                                       |
|------------|-----------------------------------------------|
| `lane`     | between two rows, or outside the outer row    |
| `headland` | lane end, at the parcel boundary              |
| `road`     | farm road                                     |
| `yard`     | open ground in a yard                         |

Lane waypoints carry `parcel` (area uid) and `lane_index` (the lane between rows *i*−1
and *i* has index *i*).

### 5.6 AGENTS (layer 1 / `a`)

There is one node per agent. It holds the latest observed pose
(`position`, `world_R_body`, `timestamp`). Pose history is not stored in the graph.

Metadata: `uid`, `kind` (`robot`, `vehicle`, `person`, `animal`), `last_seen`.
`AgentNodeAttributes` has no `name`, bounding box or `semantic_label` field, so all of
these go in metadata.

## 6. Node attributes

spark_dsg fields: `name`, `position`, `bounding_box` (oriented, yaw in radians),
`semantic_label` (layers 1 and 2), `color` (display only).

Common metadata on every node:

| Key             | Type       | Values / meaning                                                       |
|-----------------|------------|------------------------------------------------------------------------|
| `uid`           | string     | stable id. Prior nodes use the GeoJSON feature id; new nodes get a generated id |
| `kind`          | string     | class name (layers 1, 2) or kind (other layers), for readability       |
| `geometry`      | object     | GeoJSON geometry in the local frame (§7); optional for point objects    |
| `source`        | enum       | `prior` (from GeoJSON) or `perceived` (added by a robot)              |
| `status`        | enum       | `expected`, `confirmed` or `missing`                                  |
| `modified`      | bool       | observed geometry or attributes differ from the prior                  |
| `confidence`    | float 0–1  | confidence in the current geometry and class                           |
| `last_observed` | ISO 8601   | last time a robot observed the node; absent if never observed          |

Lifecycle:

| Event                                       | `source`    | `status`    | `modified` |
|---------------------------------------------|-------------|-------------|------------|
| imported from GeoJSON                       | `prior`     | `expected`  | false      |
| observed as described                       | `prior`     | `confirmed` | false      |
| observed with different geometry or attributes | `prior`  | `confirmed` | true       |
| looked for and not found                    | `prior`     | `missing`   | unchanged  |
| found but not in the prior                  | `perceived` | `confirmed` | false      |

Robots never delete prior nodes; a node they cannot find is marked `missing`. The
GeoJSON file remains the record of the prior.

## 7. Frames and units

- Frame: local ENU, metres, z up, named `map`. The origin is stored in the graph
  metadata. GeoJSON coordinates (WGS84) are converted on import.
- Angles: radians in `bounding_box`, degrees in `*_deg` metadata.
- Graph metadata: `spec_version`, `site`, `origin` (`lat`, `lon`, `alt`), `frame`,
  `units`, `labelspaces`.

## 8. Edges

All edges carry `metadata["relation"]`. Spatial and navigation edges also carry
`weight` = planar distance between the node positions, in metres. spark_dsg edges are
undirected; "from → to" below gives the meaning.

| Relation         | Group      | From → to                       | Layers             | Rule                                         |
|------------------|------------|---------------------------------|--------------------|----------------------------------------------|
| `contains`       | tree       | parent → child                  | higher → lower, p0 | §3                                           |
| `member_of`      | overlay    | member → overlay                | 1–3, 1/`n` → 4/`o` | §4                                           |
| `next_in_row`    | spatial    | crop → next crop                | 1                  | consecutive plants in the same row           |
| `adjacent_row`   | spatial    | row → next row                  | 2                  | consecutive rows in the same parcel          |
| `adjacent_area`  | spatial    | area ↔ area                     | 3                  | boundary gap below a threshold; `gap_m`      |
| `near`           | spatial    | object ↔ object                 | 1                  | box gap below a threshold                    |
| `access`         | navigation | object ↔ waypoint               | 1 ↔ 1/`n`          | waypoint from which the object is reached    |
| `lane`           | navigation | waypoint ↔ waypoint             | 1/`n`              | consecutive lane waypoints                   |
| `headland`       | navigation | waypoint ↔ waypoint             | 1/`n`              | joins the lane ends on one parcel side       |
| `road`           | navigation | waypoint ↔ waypoint             | 1/`n`              | consecutive road waypoints                   |
| `entrance`       | navigation | waypoint ↔ waypoint             | 1/`n`              | headland → road                              |
| `at`             | agent      | agent ↔ waypoint                | 1/`a` ↔ 1/`n`      | nearest waypoint to the agent's latest pose  |

Navigation edges may carry `blocked: true` and `blocked_by` (a list of object uids).
Routing skips blocked edges and `no_entry` waypoints (§4) by default.

Thresholds (`near`, `adjacent_area`, `T3` tolerance) are builder parameters, stored in
the graph metadata as `thresholds`.

### Query conventions

- `parent(n)`: the `contains` neighbour of `n` in a higher layer.
- `lineage(n)`: `parent` applied repeatedly up to the farm.
- `overlays(n)`: overlays linked by `member_of` to `n` or to any node in `lineage(n)`.
- `members(z)`: nodes linked to `z` by `member_of`, plus their tree descendants.
- LLM serialization: nest nodes by `contains`, list `overlays(n)` by uid in a
  `member_of` field, and list spatial edges per node.

## Appendix A. Isaac scene mapping

The Isaac asset scan acts as the prior (`source: prior`).

| Isaac class                       | Layer   | Category  | Class            |
|-----------------------------------|---------|-----------|------------------|
| `cabbage`, `grapevine`, `lemon`   | OBJECTS | crop      | same name        |
| `building`                        | OBJECTS | structure | `building`       |
| `tree`                            | OBJECTS | vegetation | `tree`          |
| `fence`                           | OBJECTS | structure | `fence`          |
| `obstacle`, unlabelled prop       | OBJECTS | obstacle  | `unknown`        |

Rows and parcels are derived from the plant positions.

## Appendix B. Changes from the current scripts

| Current                      | This spec                                           |
|------------------------------|-----------------------------------------------------|
| layer 1 `DETECTIONS`         | `OBJECTS`                                           |
| layer 3 `PARCELS`            | `AREAS` with `kind: parcel`                         |
| obstacle classes `tree`, `fence` | category `vegetation` / `structure`             |
| `adjacent_parcel`            | `adjacent_area`                                     |
| (none)                       | partitions 4/`o` (overlays) and 1/`a` (agents)      |
| (none)                       | common metadata: `uid`, `source`, `status`, `modified`, `confidence`, `last_observed`, `geometry` |
