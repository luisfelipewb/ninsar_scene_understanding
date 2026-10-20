# Docs

Start with the architecture, build the workspace, then run the Isaac bag. The specs are the
contracts between components; read them when you change one side of an interface.

1. [architecture.md](architecture.md): what the system does, which package does what, diagrams
2. [setup/ros.md](setup/ros.md): build and test the ROS workspace, upstream pins and patches
3. [../data/README.md](../data/README.md): what goes in `data/` and where to get it
4. [guides/run_isaac_bag.md](guides/run_isaac_bag.md): semantic mapping on the Isaac Sim bag

## Guides

| guide | |
|---|---|
| [run_isaac_bag.md](guides/run_isaac_bag.md) | Voxeland + perception on an Isaac Sim bag: run, save, query, gotchas |
| [VOXELAND_GUIDE.md](guides/VOXELAND_GUIDE.md) | Voxeland in depth: packages, input, server, map queries |
| [HYDRA_GUIDE.md](guides/HYDRA_GUIDE.md) | Hydra scene graphs, and rendering a hand-built one in RViz |
| [FARM_EDITOR.md](guides/FARM_EDITOR.md) | the farm GeoJSON editor |
| [bag_fix.md](guides/bag_fix.md) | robot bag tools: repair, lidar calibration, TF tuning |

## Specs

| spec | contract between |
|---|---|
| [CLOUD_IN_SPEC.md](specs/CLOUD_IN_SPEC.md) | `semantic_cloud_adapter` → Voxeland server and `object_detector` |
| [OBJECT_DETECTOR.md](specs/OBJECT_DETECTOR.md) | `object_detector` → its consumers (output format, parameters) |
| [FARM_GEOJSON_SPEC.md](specs/FARM_GEOJSON_SPEC.md) | farm editor → GeoJSON → `geojson_to_dsg` |
| [FARM_DSG_SPEC.md](specs/FARM_DSG_SPEC.md) | farm scene graph builders → planners and queries |

## Diagrams

[diagrams/ninsar_overview.drawio](diagrams/ninsar_overview.drawio) holds all pages (open it with
draw.io or its VS Code extension); [diagrams/png/](diagrams/png/) has an export of each page. After
editing the drawio file, re-export the pages so the PNGs stay in sync.
