# Farm GeoJSON profile

Draft v0, 2026-10-07. This is the file format for the prior description of a farm.
`scripts/farm_editor/` writes it, and the GeoJSON importer reads it to build the graph
described in [FARM_DSG_SPEC.md](FARM_DSG_SPEC.md).

## 1. Scope

- One file describes one farm. Everything in it is prior knowledge (`source: prior`).
- The file stores **geometry and intrinsic attributes only**. Relations (tree parent,
  overlay membership, ordering) and measures (area, length, heading, counts) are derived
  on import (§6). The file has no parent, membership or index fields.
- Not in the file: agents, lifecycle metadata (`source`, `status`, `modified`,
  `confidence`, `last_observed`), navigation waypoints. Roads are the only navigation
  input (§4.2).

## 2. File

An RFC 7946 `FeatureCollection` in UTF-8, with the extension `.geojson`.

Foreign members of the collection:

| Key            | Type   | Required | Meaning                                                              |
|----------------|--------|----------|----------------------------------------------------------------------|
| `farm_geojson` | string | yes      | profile version, `"0.1"`                                             |
| `site`         | string | yes      | site name, copied to graph metadata `site`                           |
| `origin`       | object | yes      | `{lat, lon, alt}` origin of the `map` frame. Set once, never moved   |
| `next_id`      | object | yes      | next free counter per id prefix, e.g. `{"c": 452, "r": 31}` (§4.1)   |
| `imagery`      | list   | no       | background images used while editing; ignored by the importer        |

Some tools drop foreign members (QGIS, for example). Without `origin`, the importer uses
the centroid of the farm polygon and warns. Without `next_id`, it recomputes the counters
from the ids in the file.

## 3. Coordinates and frame

- Positions are WGS84 `[lon, lat]` (RFC 7946 order), written with 8 decimals (about 1 mm).
  A third value is ignored. Heights are not described (§4.3).
- The `map` frame is local ENU in metres, with z up, at `origin`. Conversion goes
  geodetic → ECEF → ENU on the WGS84 ellipsoid. All rules in §5 and everything derived in
  §6 are computed in this frame.
- Polygons have one exterior ring and may have holes (for example, a parcel around a
  pond). MultiPolygon and MultiLineString are not allowed: one feature is one node. Either
  ring orientation is accepted.
- A LineString has a direction. For a row, the first vertex is the row start.
- Angles (`*_deg`) are in degrees, counter-clockwise from east (+x), in [0, 360). This is
  the same convention as the bounding-box yaw, which is stored in radians.

## 4. Features

### 4.1 Common members

| Member                  | Required | Meaning                                                              |
|-------------------------|----------|----------------------------------------------------------------------|
| `id`                    | yes      | node uid: `<prefix><n>`, e.g. `A3`, `c451`. Prefix from §4.2. The id maps to the spark_dsg `NodeSymbol(prefix, n)` |
| `properties.layer`      | yes      | `farm`, `overlay`, `area`, `row`, `object`, `navigation`             |
| `properties.category`   | objects  | object category (DSG §5.1)                                           |
| `properties.kind`       | yes      | farm: `farm`; area/overlay kind; object class; row crop; `road`      |
| `properties.name`       | no       | display name                                                         |
| `properties.description`| no       | free text, copied to node metadata (useful for LLM queries)          |

- Ids are unique in the file. When an id is assigned, `next_id[prefix]` is incremented. A
  deleted id is never reused, so a uid in a graph built from an older version of the file
  never refers to a different thing.
- Property keys not listed in this profile are allowed and copied unchanged into node
  metadata. The derived keys (§6) and the lifecycle keys (§1) are reserved and must not
  appear.

### 4.2 Feature types

Required attributes are in **bold**. Crop classes are open-ended: they are the crop
entries of the labelspace config (for example `cabbage`, `grapevine`, `lemon`).

| `layer`      | `category`      | `kind`                                                       | Prefix | Geometry            | Attributes                                                |
|--------------|-----------------|--------------------------------------------------------------|--------|---------------------|-----------------------------------------------------------|
| `farm`       |                 | `farm`                                                       | `F`    | Polygon             | **`name`**                                                |
| `overlay`    |                 | `irrigation_sector`                                          | `Z`    | Polygon             | **`valve`** (id of a `valve` object), `method` (`drip`, `sprinkler`) |
| `overlay`    |                 | `management_zone`                                            | `Z`    | Polygon             | **`scheme`**, **`value`**                                 |
| `overlay`    |                 | `restricted_area`                                            | `Z`    | Polygon             | **`restriction`** (`no_entry`, `caution`), `reason` (`protected`, `slope`, `wet`, `soft_ground`, `private`, `other`) |
| `area`       |                 | `parcel`                                                     | `A`    | Polygon             | `row_spacing_m`, `headland_m`                             |
| `area`       |                 | `yard`                                                       | `A`    | Polygon             |                                                           |
| `area`       |                 | `water_body`                                                 | `A`    | Polygon             | **`water_type`** (`pond`, `stream`, `ditch`)              |
| `area`       |                 | `vegetation`                                                 | `A`    | Polygon             | **`vegetation_type`** (`hedgerow`, `woodland`, `grass_strip`) |
| `row`        |                 | crop class                                                   | `r`    | LineString          | `variety`, `plant_spacing_m`                              |
| `object`     | `crop`          | crop class                                                   | `c`    | Point               | `variety`                                                 |
| `object`     | `weed`          | `weed`                                                       | `k`    | Point               |                                                           |
| `object`     | `vegetation`    | `tree`, `shrub`                                              | `v`    | Point               |                                                           |
| `object`     | `row_support`   | `post`, `end_anchor`                                         | `i`    | Point               |                                                           |
| `object`     | `row_support`   | `trellis_wire`, `drip_line`                                  | `i`    | LineString          |                                                           |
| `object`     | `equipment`     | `soil_sensor`, `insect_trap`, `weather_station`, `valve`, `sprinkler` | `e` | Point          | `device_id`                                               |
| `object`     | `structure`     | `building`, `shed`, `silo`, `tank`                           | `B`    | Polygon             |                                                           |
| `object`     | `structure`     | `fence`, `wall`                                              | `B`    | LineString          |                                                           |
| `object`     | `structure`     | `gate`                                                       | `B`    | Point               | `width_m`                                                 |
| `object`     | `service_point` | `dock`, `refill_point`, `bin_drop`, `parking`                | `s`    | Point               |                                                           |
| `object`     | `movable`       | `bin`, `crate`, `trailer`, `hose`                            | `m`    | Point               |                                                           |
| `object`     | `obstacle`      | `rock`, `pole`, `debris`, `unknown`                          | `o`    | Point or Polygon    |                                                           |
| `navigation` |                 | `road`                                                       | `R`    | LineString          | `width_m`                                                 |

A road is not a node. The importer samples it into `w` waypoints (§6), so the prefix `R`
has no `NodeSymbol` counterpart.

The editor does not offer weeds by default, because they are normally perceived. The
profile still allows them for known weed patches.

### 4.3 Object size

The file does not describe object dimensions. Nobody is expected to measure every plant
and object.

- Polygon and LineString objects (buildings, fences, debris) get their horizontal extent
  from the geometry.
- Point objects get a per-class default box from the schema catalog.
- The z extent is always the per-class default height.
- Robot perception later corrects boxes in the graph and sets `modified: true` (DSG §6).
  The GeoJSON file is not updated.

## 5. Validation rules

The importer refuses a file with an **error** and imports a file with **warnings**. The
editor checks the same rules with the same code (`scripts/farm_geojson.py`).

Structure:

- **G1** error: exactly one `farm` feature.
- **G2** error: ids are unique, match `<prefix><n>` with the prefix of their type, and
  `n < next_id[prefix]`.
- **G3** error: the `layer` / `category` / `kind` combination is in §4.2, and the geometry
  type is allowed for it.
- **G4** error: the geometry is valid. Polygons are OGC-valid. LineStrings have at least
  two distinct vertices and do not cross themselves.
- **G5** error: required attributes are present, enum values are valid, and lengths are
  positive.
- **G6** error: no reserved keys (§4.1).

Tree (DSG T3, T4, T6), evaluated with tolerance `contain_tol_m`:

- **G7** error: every feature lies inside the farm polygon.
- **G8** error: areas do not overlap each other (by more than `overlap_tol_m`).
- **G9** error: every row lies inside exactly one area, and that area is a `parcel`.
- **G10** error: every `crop` and `row_support` object resolves to a row (§6, row strip),
  and a crop's `kind` equals its row's `kind` (one crop per row).
- **G11** error: the derived parent is allowed by the "Tree parent" column of DSG §5.1.
  For example, a building inside a row strip is an error.
- **G12** error: rows of the same parcel do not cross each other.

Overlays (DSG §4):

- **G13** error: overlays of an exclusive kind do not overlap: `irrigation_sector` always,
  `management_zone` only with the same `scheme`.
- **G14** error: the `valve` of an `irrigation_sector` is the id of an object with
  `kind: valve`.
- **G15** warning: an overlay contains no tree node and no road.

Navigation:

- **G16** warning: a `service_point` is farther than `access_m` from every road. This is
  needed because DSG §5.1 requires an `access` edge for service points.

| Threshold       | Default | Used by       |
|-----------------|---------|---------------|
| `contain_tol_m` | 0.10    | G7, G9–G11, §6 |
| `overlap_tol_m` | 0.05    | G8, G13       |
| `access_m`      | 15.0    | G16           |
| `row_strip_m`   | 0.5     | §6 row strip  |

These are importer parameters and are stored in the graph metadata `thresholds`.

## 6. Derived on import

| Value                    | Rule                                                                                         |
|--------------------------|----------------------------------------------------------------------------------------------|
| row strip                | row axis buffered by half the row spacing: the parcel's `row_spacing_m`, or else the median distance between adjacent row axes, or else `row_strip_m` as the half-width (a parcel with one row). Where strips touch, the nearest axis wins |
| tree parent (`contains`) | T4: the lowest tree node whose footprint contains the whole geometry: row strip, then area, then farm |
| overlay membership       | O3: the highest tree nodes entirely inside the overlay polygon. Waypoints inside a `restricted_area` are linked directly |
| row `row_index`          | rows of a parcel sorted by offset along the parcel normal (heading + 90°), from 0             |
| crop `row_index`         | plants of a row sorted by distance along the axis from its first vertex, from 0              |
| crop `variety`           | the row's `variety` when the plant has none                                                  |
| `heading_deg`            | row: direction from the first to the last vertex. Parcel: mean of its rows' headings, taken mod 180° |
| `length_m`, `n_plants`   | rows                                                                                         |
| `n_rows`, `crops`        | parcels                                                                                      |
| `area_m2`                | farm and areas                                                                               |
| `position`               | Point: the point. LineString: midpoint along the line. Polygon: centroid. z = half the box height |
| `bounding_box`           | Point: class default box, axis-aligned. LineString and Polygon: minimum rotated rectangle of the geometry. z: class default height (§4.3) |
| `geometry` metadata      | the feature geometry converted to `map` ENU                                                  |
| navigation               | road → `road` waypoints every `road_step_m`. Lanes, headlands, entrances and yard waypoints follow DSG §5.5. Waypoint ids are regenerated on every import and are not stable |

## 7. Example

```json
{
  "type": "FeatureCollection",
  "farm_geojson": "0.1",
  "site": "montoldre",
  "origin": {"lat": 46.33742, "lon": 3.43398, "alt": 250.0},
  "next_id": {"F": 1, "A": 1, "r": 1, "c": 2, "Z": 1},
  "features": [
    {"type": "Feature", "id": "F0",
     "properties": {"layer": "farm", "kind": "farm", "name": "Montoldre"},
     "geometry": {"type": "Polygon", "coordinates": [[[3.4330, 46.3370], [3.4350, 46.3370],
                  [3.4350, 46.3380], [3.4330, 46.3380], [3.4330, 46.3370]]]}},
    {"type": "Feature", "id": "A0",
     "properties": {"layer": "area", "kind": "parcel", "name": "North vines",
                    "row_spacing_m": 2.5, "headland_m": 6.0},
     "geometry": {"type": "Polygon", "coordinates": [[[3.4332, 46.3372], [3.4340, 46.3372],
                  [3.4340, 46.3378], [3.4332, 46.3378], [3.4332, 46.3372]]]}},
    {"type": "Feature", "id": "r0",
     "properties": {"layer": "row", "kind": "grapevine", "variety": "gamay",
                    "plant_spacing_m": 1.0},
     "geometry": {"type": "LineString", "coordinates": [[3.43328, 46.33728], [3.43392, 46.33728]]}},
    {"type": "Feature", "id": "c0",
     "properties": {"layer": "object", "category": "crop", "kind": "grapevine"},
     "geometry": {"type": "Point", "coordinates": [3.43328, 46.33728]}},
    {"type": "Feature", "id": "c1",
     "properties": {"layer": "object", "category": "crop", "kind": "grapevine"},
     "geometry": {"type": "Point", "coordinates": [3.43329301, 46.33728]}},
    {"type": "Feature", "id": "Z0",
     "properties": {"layer": "overlay", "kind": "restricted_area",
                    "restriction": "caution", "reason": "slope"},
     "geometry": {"type": "Polygon", "coordinates": [[[3.4332, 46.3372], [3.4334, 46.3372],
                  [3.4334, 46.3374], [3.4332, 46.3374], [3.4332, 46.3372]]]}}
  ]
}
```
