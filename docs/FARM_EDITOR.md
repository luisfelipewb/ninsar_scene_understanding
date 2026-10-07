# Farm editor

Draft v0, 2026-10-07. This is the specification and implementation plan for a local web
tool that writes the farm GeoJSON ([FARM_GEOJSON_SPEC.md](FARM_GEOJSON_SPEC.md)). The DSG
importer reads that file ([FARM_DSG_SPEC.md](FARM_DSG_SPEC.md)).

## 1. Purpose

Generic editors such as geojson.io can draw shapes, but they do not know the farm layers,
classes and attributes. They do not check the spec rules, and they cannot lay out hundreds
of rows and plants precisely. This editor:

- guides the user through the farm in a fixed order of steps (§4), with buttons named in
  farm terms ("Add parcel", "Add building"), not geometry terms;
- builds attribute forms from a schema catalog, so every feature is complete and uses
  valid values;
- generates rows, plants and posts from a few parameters;
- validates with the same Python code the importer uses;
- runs locally and works offline with local imagery.

## 2. Running

```bash
.venv/bin/python src/ninsar_mapping/scripts/farm_editor/serve.py ~/data/ninsar/farms/montoldre.geojson \
    [--imagery ~/data/ninsar/imagery] [--port 8765] [--host 127.0.0.1] [--no-browser]
```

- The server creates the file when it is missing and opens the browser at
  `http://127.0.0.1:8765`.
- It binds to localhost by default. `--host 0.0.0.0` lets a tablet in the field connect.
- Saving writes `<file>.bak` before overwriting. Unsaved changes are written to
  `<file>.autosave` every 30 s.

## 3. Components

```
src/ninsar_mapping/
  config/farm_schema.yaml       catalog: layers, categories, kinds, attributes, colors, presets
  scripts/farm_geojson.py       shared library (editor + importer) and CLI
  scripts/farm_editor/
    serve.py                    Flask app + CLI
    static/index.html, style.css
    static/js/api.js            fetch wrappers for §6
    static/js/map.js            Leaflet map, layer groups, styling, Geoman draw/edit
    static/js/panels.js         properties form, issue list, derived tree
    static/js/workflow.js       step panel and presets
    static/js/app.js            state, undo/redo, autosave, import/export
    static/vendor/              Leaflet 1.9, leaflet-geoman-free, Leaflet.ImageOverlay.Rotated
  test/test_farm_geojson.py
  test/data/mini_farm.geojson
```

- No JS build step: the editor is vanilla ES modules. The vendor files are committed so
  the editor works offline.
- Python dependencies are already in `.venv`: flask, shapely, numpy, PyYAML.

### 3.1 Schema catalog (`config/farm_schema.yaml`)

This file is the single source for the UI and the validator. Adding a class or attribute
means editing this file only. Each entry `(layer, category, kind)` has:

| Field        | Meaning                                                                  |
|--------------|--------------------------------------------------------------------------|
| `prefix`     | id prefix (GeoJSON spec §4.2)                                            |
| `geometry`   | allowed geometry types                                                   |
| `parents`    | allowed tree parents (DSG §5.1)                                          |
| `color`, `label` | map style and UI name                                                |
| `attributes` | list of `{name, type: str/float/int/enum/uid_ref, unit, values, required, default, ref_kind}` |
| `exclusive`  | overlays: `true`, `false` or `{per: scheme}`                             |
| `default_size_m` | objects: `[x, y, z]` default box; z is also the height of footprint objects |

The `crops` list (the crop classes) is also defined here. Rows and crop objects take
their `kind` from it.

### 3.2 Shared library (`scripts/farm_geojson.py`)

| Function                                             | Purpose                                                    |
|------------------------------------------------------|------------------------------------------------------------|
| `load(path)`, `save(path, fc)`                       | normalize, round to 8 decimals, sort by layer, `.bak`       |
| `Frame(origin)`: `to_enu(lon, lat)`, `to_wgs84(x, y)` | WGS84 ↔ ECEF ↔ local ENU, numpy                           |
| `to_shapely(feature, frame)`, `from_shapely(geom, frame)` | geometry in ENU metres                                 |
| `new_id(fc, prefix)`                                 | uses and increments `next_id`                              |
| `derive(fc, schema, thresholds)`                     | GeoJSON spec §6, returns `{id: {parent, overlays, row_index, …}}` |
| `validate(fc, schema, thresholds)`                   | GeoJSON spec §5, returns `[Issue(id, level, rule, message)]` |
| `generate_rows(...)`, `generate_plants(...)`, `generate_posts(...)` | §5, return new Features (no ids yet)         |

CLI: `farm_geojson.py validate FILE` (exit code 1 on errors) and `farm_geojson.py tree FILE`
(prints the derived hierarchy with counts).

## 4. User interface

```
+--------------------------------------------------------------------------------+
| Montoldre  [Open] [Save] [Import] [Export] [Validate]  [Undo] [Redo]   ● saved |
+----------------+------------------------------------------+--------------------+
| 1 Farm      ✓  |                                          | Parcel A3          |
| 2 Restricted 2 |                                          | name   [North    ] |
| 3 Areas     5  |                 MAP                      | row_spacing_m [2.5]|
| 4 Rows    120  |                                          | headland_m    [6.0]|
| 5 Plants 4.2k  |                                          | ── derived ──      |
| 6 Structures   |                                          | parent  F0         |
| 7 Roads        |                                          | area_m2 12 540     |
| 8 Overlays     |                                          | n_rows  40         |
| 9 Check  ⚠ 3   |                                          | overlays Z1, Z4    |
| [step buttons] |  x 12.40  y -3.15   46.33742 N 3.43398 E |                    |
+----------------+------------------------------------------+--------------------+
```

### 4.1 Workflow steps

Each step shows a count and an issue badge, and only the buttons that belong to it. Steps
can be revisited in any order; the numbering is a guide, not a lock.

| Step                     | Buttons (preset → feature type)                                                                 |
|--------------------------|--------------------------------------------------------------------------------------------------|
| 1 Farm boundary          | *Draw farm boundary* → farm (disabled once one exists)                                          |
| 2 Restricted zones       | *Add no-entry zone*, *Add caution zone* → `restricted_area` with `restriction` preset            |
| 3 Areas                  | *Add parcel*, *Add yard*, *Add water body*, *Add vegetation area*; *Cut hole*                    |
| 4 Rows                   | *Generate rows…* (needs a selected parcel), *Draw single row*, *Offset row…*                     |
| 5 Plants & row supports  | *Fill rows with plants…*, *Add plant*, *Add posts…*, *Draw trellis wire*, *Draw drip line*       |
| 6 Structures & equipment | *Draw building*, *Draw shed/silo/tank* (footprint polygons), *Draw fence/wall*, *Add gate*, *Add equipment*, *Add service point*, *Add obstacle*, *Add tree/shrub*, *Add movable* |
| 7 Roads                  | *Draw road*                                                                                     |
| 8 Overlays               | *Add irrigation sector* (then pick a valve on the map), *Add management zone*                    |
| 9 Check & export         | *Validate*, *Show tree*, *Export*                                                               |

A preset fixes `layer`, `category`, `kind` and the geometry type. It starts the matching
Geoman draw mode. When the shape is finished, the new feature gets its id, its defaults
and its style from the catalog, and the properties panel opens. A kind dropdown in the
panel changes the class later, for example from `shed` to `building`.

### 4.2 Map

- Base layers: Esri World Imagery, OpenStreetMap, and local overlays (§7), with an opacity
  slider for overlays.
- One toggleable layer group per layer/category, styled with the catalog color. Points are
  drawn on the canvas renderer so that 10k plants stay responsive. Plants are hidden below
  a zoom threshold.
- Editing (Geoman): move vertices, drag, delete, snap to vertices and edges (so adjacent
  areas share a boundary without overlapping), and cut holes.
- Precision:
  - The cursor readout shows ENU x/y and lat/lon.
  - A measure tool shows distance and heading.
  - For a selected point or vertex, the user can type lat/lon or ENU x/y.
  - *Offset row…* copies a row at a typed perpendicular distance.
- Selection: click to select, shift-click or a box to multi-select. Clicking a feature in
  the issue list or the tree zooms to it.

### 4.3 Properties panel

- The form is generated from the catalog:
  - enum → dropdown
  - number → input with a unit label
  - `uid_ref` → dropdown plus "pick on map", filtered by `ref_kind`
  - custom keys → key/value rows
- Derived values (GeoJSON spec §6) come from the last validate call and are shown
  read-only.
- Multi-select edits the attributes that the selected features share, for example
  `variety` on every row of a parcel.

### 4.4 Validation and tree

- *Validate* posts the collection to the server. Issues appear in a list, sorted by level.
  Offending features get a red outline (errors) or an orange one (warnings).
- Validation runs automatically on save, and optionally live, debounced by 1 s. Saving
  never blocks, but the status shows the error count.
- *Show tree* lists farm → areas → rows → objects with counts, using the derived parents,
  plus the overlays and their members. It is the quickest way to check that every plant
  landed in the right row.

### 4.5 Import, export, history

- *Import* merges a GeoJSON file. Features without `layer` (from geojson.io or QGIS) are
  listed as *unclassified* until the user assigns a preset. Ids that clash with existing
  ones are reassigned.
- *Export* downloads the current collection. *Save* writes the working file through the
  server.
- Undo/redo keeps a snapshot stack of the collection, up to 50 entries. A warning appears
  when the page is closed with unsaved changes.

## 5. Generators

The generators work in ENU on the server. The user sees a preview and commits it.

| Generator | Inputs                                                                                                 | Output                                   |
|-----------|--------------------------------------------------------------------------------------------------------|------------------------------------------|
| rows      | parcel; heading (click a parcel edge, or type `heading_deg`); `row_spacing_m`; `headland_m`; first-row offset from the parcel edge; crop; variety; optional row count | parallel row axes clipped to the parcel shrunk by `headland_m`. Rows shorter than a minimum length are dropped. The rows run in the heading direction. `row_spacing_m` and `headland_m` are written to the parcel |
| plants    | selected rows or a parcel; `plant_spacing_m`; start offset; end margin                                 | one crop Point every `plant_spacing_m` along each axis. `plant_spacing_m` is written to the row |
| posts     | selected rows; `post_spacing_m`; end anchors yes/no; anchor offset beyond the row end                  | `post` Points along the axis and `end_anchor` Points at both ends |

If a parcel already has rows, or a row already has plants, the generator asks whether to
replace them. Replaced ids are deleted, not reused (GeoJSON spec §4.1).

## 6. Server API

| Method and path              | Body → response                                                     |
|------------------------------|---------------------------------------------------------------------|
| `GET /`, `GET /static/*`     | UI                                                                  |
| `GET /api/schema`            | → catalog as JSON                                                   |
| `GET /api/farm`              | → working FeatureCollection                                         |
| `PUT /api/farm`              | FeatureCollection → `{saved: true, issues}`                         |
| `POST /api/autosave`         | FeatureCollection → `{ok}`                                          |
| `POST /api/validate`         | FeatureCollection → `{issues, derived}`                             |
| `POST /api/generate/rows`    | `{fc, parcel_id, params}` → `{features}`                            |
| `POST /api/generate/plants`  | `{fc, row_ids, params}` → `{features}`                              |
| `POST /api/generate/posts`   | `{fc, row_ids, params}` → `{features}`                              |
| `GET /api/imagery`           | → list of overlays found in `--imagery`                             |
| `GET /imagery/<path>`        | image or tile file                                                  |

The client assigns ids when it commits generated features, so preview and cancel leave
`next_id` unchanged.

## 7. Local imagery

The server scans `--imagery DIR` for two forms of imagery.

- **Image with sidecar**: `name.png` or `name.jpg` next to `name.json`:

  ```json
  {"label": "Isaac top-down", "frame": "enu",
   "origin": {"lat": 46.33742, "lon": 3.43398, "alt": 250.0},
   "corners": {"topleft": [-150.0, 120.0], "topright": [150.0, 120.0], "bottomleft": [-150.0, -180.0]}}
  ```

  `frame` is `wgs84` (corners as `[lon, lat]`) or `enu` (corners as `[x, y]` in metres
  around `origin`). An Isaac top-down render uses `enu`. The three corners allow rotation
  (`Leaflet.ImageOverlay.Rotated`).
- **XYZ tile directory**: `name/{z}/{x}/{y}.png` (for example from `gdal2tiles.py`) with
  `name/tiles.json` holding `{"label", "min_zoom", "max_zoom"}`. Use this for large drone
  orthomosaics.

## 8. Implementation plan

| # | Milestone                  | Content                                                                                   | Done when |
|---|----------------------------|-------------------------------------------------------------------------------------------|-----------|
| 1 | Format and library         | `farm_schema.yaml`, `farm_geojson.py` (Frame, load/save, ids, derive, validate, CLI), `mini_farm.geojson`, tests | pytest passes; `validate` and `tree` work on `mini_farm.geojson` |
| 2 | Editor skeleton            | `serve.py`, map with base layers, presets for every type, properties form, save/load/import/export | a farm drawn by hand round-trips through save and reload |
| 3 | Guided workflow, validation UI | step panel, issue list with zoom-to, outlines, derived tree                           | every rule in GeoJSON spec §5 shows up in the UI on a crafted bad file |
| 4 | Generators                 | rows, plants, posts, with preview and replace                                             | a 40-row parcel with plants in under a minute of user time |
| 5 | Precision and imagery      | local overlays, typed coordinates, offset row, measure, snapping, undo/redo, autosave     | the Isaac check below passes |
| 6 | Importer (separate plan)   | `scripts/geojson_to_dsg.py` on top of `farm_geojson.derive()`                             | |

## 9. Verification

- `.venv/bin/python -m pytest src/ninsar_mapping/test/test_farm_geojson.py`:
  - ENU round trip below 1 mm.
  - Each rule G1–G16 fires on a crafted bad feature and stays quiet on `mini_farm.geojson`.
  - Row generator: spacing, headland and count on a rectangular parcel; heading on a
    rotated parcel.
  - Derived parents and `row_index` match T4.
- End to end:
  1. Start the editor on a new file.
  2. Draw the boundary, two parcels, generated rows and plants, a building footprint, a
     no-entry zone, a road, and an irrigation sector linked to a valve.
  3. Validate shows 0 errors. Save, reload, and get the same collection.
  4. Run `farm_geojson.py validate` and `farm_geojson.py tree` on the saved file.
- Precision against Isaac:
  1. Load an Isaac top-down render as an `enu` overlay. Producing the render belongs in
     ninsar-isaac-sim.
  2. Generate rows and plants over the cabbage, grapevine and lemon blocks.
  3. Compare the generated plant positions in ENU with the crop centers in
     `config/isaac_assets.json`. Target: mean error below 0.2 m.
