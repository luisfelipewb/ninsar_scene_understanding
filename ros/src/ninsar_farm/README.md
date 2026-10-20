# ninsar_farm

The farm description toolchain: a farm drawn as GeoJSON (with the web editor), validated, and
turned into a farm scene graph (spark_dsg). Plain Python with no ROS runtime; it's a colcon
package so it builds, installs and tests like the rest of the workspace.

| command (`ros2 run ninsar_farm ...`) | |
|---|---|
| `farm_editor FILE.geojson` | local web UI that edits the farm GeoJSON |
| `farm_geojson validate FILE` / `tree FILE` | check a farm GeoJSON, print its tree |
| `geojson_to_dsg FILE.geojson` | farm GeoJSON → farm scene graph |
| `farm_dsg verify GRAPH.json` / `summary GRAPH.json` | check a farm scene graph against the spec |
| `make_farm_dsg`, `make_isaac_farm_dsg` | older hand-built graphs (synthetic farm, Isaac assets) |

| path | |
|---|---|
| `config/farm_schema.yaml` | the farm catalog shared by the editor, the validator and the importer |
| `config/isaac_assets.json` | Isaac scene asset list, written by the Isaac tool `scan_isaac_assets.py` |
| `ninsar_farm/editor/` | editor server and its static web UI (vendored Leaflet + Geoman) |

spark_dsg's Python bindings come from the colcon build of spark_dsg (see `ros/patches/`).
