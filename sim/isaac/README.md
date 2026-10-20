# ninsar-isaac-sim

Synthetic sensor data from a free-flying sensor rig in Isaac Sim, published to
ROS 2. A rig carrying two ZED X cameras and an Ouster OS1 is flown along a
trajectory through a scene; clock, TF, images and point clouds go out on ROS 2
topics as it moves. Trajectories are hand-written waypoints, a lawnmower path
over a crop parcel, or a real robot's rosbag replayed against the synthetic field.

## Two interpreters

Isaac Sim ships its own Python and ROS 2 Jazzy uses the system one; neither can
import the other's packages. Source ROS 2 in **both** cases — Isaac needs it on
`LD_LIBRARY_PATH` for the ROS 2 bridge.

```bash
source /opt/ros/jazzy/setup.bash
```

| Script | Interpreter | Why |
|---|---|---|
| `run_scene.py` | `~/isaacsim/python.sh` | `isaacsim`, `omni.*`, `pxr` |
| `tools/bake_heightmap.py`, `tools/get_cabbage_positions.py`, `tools/cabbages_to_waypoints.py`, `tools/rows_to_lanes.py`, `tools/circle_facing_center.py`, `tools/scan_isaac_assets.py` | `~/isaacsim/python.sh` | `pxr` |
| `tools/bag_to_waypoints.py`, `tools/seg_label_colorizer.py` | system `python3` | `rosbag2_py`, `tf2_ros`, `rclpy` |
| `tools/tour_from_points.py` | either | numpy + PyYAML |

`rigsim/` (the shared modules) imports under both.

## Run

```bash
cd ~/isaacsim
P=~/workspaces/ninsar-isaac-sim

./python.sh $P/run_scene.py $P/config/empty.yaml                 # rig in an empty scene, flying a 2 x 2 m square
./python.sh $P/run_scene.py $P/config/montoldre.yaml --headless  # the Montoldre field, lawnmower over cabbage parcel 01

# write the assembled stage (path baked in, scrubbable in the GUI) and exit without playing
./python.sh $P/run_scene.py $P/config/montoldre.yaml --headless --build-only \
    --save-stage $P/assets/montoldre_cabbages_rig.usda
```

| flag | |
|---|---|
| `CONFIG` | run config YAML — everything else comes from here |
| `--headless` | no window; ROS 2 output unaffected |
| `--save-stage OUT.usda` | write the assembled stage: scene as a relative sublayer, rig as a payload, TF graph. With a trajectory, base_link also gets one time sample per frame and the path is drawn (hidden) under `<world>/trajectory` |
| `--build-only` | assemble, verify, save if asked, exit — nothing published |
| `--waypoints FILE` | override `trajectory.waypoints`; `none` parks the rig |
| `--print-pose` | log base_link once a second |

Without waypoints the rig sits at `base_link.translate` and the stage plays
until interrupted; with them it flies once and exits (`trajectory.loop` repeats).

What gets built on top of the scene, checked with `ros2 run tf2_tools view_frames`:

```
<world> ── base_link        60 Hz, /tf          authored by run_scene.py
           └── sensor_rig   latched, /tf_static authored by run_scene.py
               ├── os1                          the rig asset's own publishers
               ├── zed_x_bottom ── bottom_left_camera_optical_link
               └── zed_x_front  ── front_left_camera_optical_link
```

ROS frame names are prim names, so `scene.world_prim` is the fixed frame:
`/World` publishes `World`, Montoldre's `/world` publishes `world`. In RViz2 set
Fixed Frame accordingly and turn on **Use simulation time**.

## Config

A run config (`config/*.yaml`) has six sections; `config/empty.yaml` shows every
key with its default. Relative paths are project-relative. Configs are validated
before Isaac Sim boots, so a typo costs a second, not a minute.

| section | holds |
|---|---|
| `scene` | `usd` (null = empty built-in scene), `world_prim`, `fps`, `skybox` HDRI |
| `base_link` | the free-flying frame: `name`, starting `translate`/`rotate` |
| `rig` | `usd`, `name`, `load_as` payload/reference, mounting `translate`/`rotate` |
| `ros` | `graph_prim`, `/tf` and `/tf_static` topic names, `domain_id` |
| `people` | optional walkers: `terrain` heightmap, `loop`, and `walkers`, each a `name`, a library `asset`, `translate`, `yaw` and `commands` |
| `trajectory` | `speed`, `yaw_rate`, `terrain` heightmap, `loop`, `realtime`, `settle`, and last `waypoints` — a file under `config/waypoints/` or an inline list |

A waypoint is `{x, y, z, roll, pitch, yaw}` in metres and degrees, every field
defaulting to 0, or a six-element list. Waypoints may all carry `t` (seconds):
the path then replays on its own clock and `speed`/`yaw_rate` are ignored — that
is what `bag_to_waypoints.py` writes. Angles are roll/pitch/yaw applied as
scipy's extrinsic `"xyz"`, everywhere.

`trajectory.terrain` points at a baked heightmap and turns every `z` into a
clearance above the ground under it, sampled per frame so the path hugs the
relief between waypoints. Use it for paths written in flat `z`; not for
`cabbage_01.yaml`, whose `z` already is the ground height at each cabbage.

**People.** Each walker is a character from the Isaac People library, loaded at
`<world>/Characters/<name>` with semantic class `person` and walked by
`omni.anim.people` through its `commands` (`GoTo`, `Idle`, `LookAround`) while the
timeline plays. They walk at about 1.3 m/s in straight lines from point to point
(no navmesh) and steer around each other. The animation keeps a character at the
height it started at, so on relief set `people.terrain` (usually the same file as
`trajectory.terrain`): each walker is then held at the ground under it on every
frame. The characters stream from the Isaac assets server, so the first run needs
the network. `--build-only`/`--save-stage` show them standing at their start.

## Tools

**Trajectory from a rosbag** — reads only `/tf` and `/tf_static`, resolves one
frame against another, decimates to what reconstructs the path within
`--pos-tol`/`--ang-tol`, and recentres it on its first pose (a bag's `map` is
usually hundreds of metres from any scene origin; `--no-recentre` keeps it).
`--terrain ... --z 1.0` writes terrain-following `z` with a 1 m clearance.

```bash
python3 tools/bag_to_waypoints.py ~/path/to/bag --root map --base robot_base_footprint --out pom4x4_test.yaml
./python.sh run_scene.py config/empty.yaml --waypoints config/waypoints/pom4x4_test.yaml
```

**Lawnmower over the cabbages** — the plants are read straight from the scene
(default `assets/Montoldre_V4.usdc`): every visible prim whose name contains
`--match` (default `cabbage`; `--match lemon` for another crop), optionally
only below `--under`. Rows are found from the data (the direction each plant's
nearest neighbour lies in, then clustering at `--row-gap`), every plant is a
waypoint, turns are an overshoot plus a semicircle.
`tools/get_cabbage_positions.py` lists the matching prims if you want to see
what would be picked up.

```bash
~/isaacsim/python.sh tools/cabbages_to_waypoints.py --out cabbage_v4.yaml
```

**Lanes beside and between the rows of trees** — same plant and row finding,
but the waypoints go on the lanes the rows define rather than on the plants:
outside the first row, between each pair of rows, outside the last, with the
same turns in between. Two rows of lemon trees give three sections: down the
side of row 1, up the middle, down the side of row 2. Outer lanes sit
`--offset` from their row (default half the row spacing); each lane has a
waypoint abreast of every tree it passes.

```bash
~/isaacsim/python.sh tools/rows_to_lanes.py --out lemon_v4.yaml
./python.sh run_scene.py config/montoldre_v4.yaml --waypoints config/waypoints/lemon_v4.yaml
```

**A tour of several paths** — stitch waypoint files and loose `x,y` points, in
driving order, into one file. The files are replayed as they are; the
stretches between stops are cubic curves that leave and arrive along the
heading the rig already has, sampled every `--spacing` metres with yaw along
the tangent, so the heading never jumps and the rig never moves sideways. The
tightest link turn is reported; add a point to open one up. z is left to
`trajectory.terrain`.

```bash
python3 tools/tour_from_points.py config/waypoints/lemon_v4.yaml -40,11 \
    config/waypoints/cabbage_v4.yaml 8,-100 -7,-76 1.5,-11 35,3.5 99,27 --out tour_01.yaml
./python.sh run_scene.py config/montoldre_v4.yaml --waypoints config/waypoints/tour_01.yaml
```

**Circle around a prim** — `--steps` waypoints on a circle of `--radius`
around the prim's world position, `--z-offset` above it, every yaw facing it.
`--prim` is an absolute path or a name unique in the stage; the file is
named after the prim unless `--name` is given. `--close` repeats the first
waypoint at the end so the circle completes.

```bash
~/isaacsim/python.sh tools/circle_facing_center.py assets/Montoldre_V4.usdc --prim Tractor --radius 5 --steps 12
~/isaacsim/python.sh tools/circle_facing_center.py assets/Montoldre_V5.usdc --prim Crane_Lily --radius 2 --close
```

**Terrain heightmap** — bake once per environment. Montoldre's ground is a
clean 2.5D heightfield mesh with no collider, so the bake reads the lattice
straight off the mesh; how, and why bilinear interpolation is not enough, is in
`rigsim/terrain_height.py`.

```bash
~/isaacsim/python.sh tools/bake_heightmap.py --usd assets/Montoldre_V3_grouped_clean.usdc --out assets/montoldre_heightmap.npz
```

**Segmentation viewer relay** — turns Isaac's `32SC1` label images into RGB for
Foxglove, colouring by class name so objects keep their colour across frames
and cameras. One node per camera.

```bash
python3 tools/seg_label_colorizer.py --ros-args -p camera_namespace:=/sensor_rig/front_camera
```

## Layout

```
run_scene.py                 build scene + rig + TF graph (+ walkers) from a config, save it, fly it
config/
  empty.yaml                 annotated reference config: empty scene, square path, one walker
  montoldre.yaml             Montoldre field, cabbage parcel 01
  waypoints/                 square.yaml, pom4x4_test.yaml (rosbag, timed), cabbage_01.yaml
rigsim/                      shared modules, importable under both interpreters
  scene_config.py            run-config schema and validation
  waypoint_config.py         waypoint YAML read/write
  trajectory.py              waypoints -> dense (N, 8) pose array: t, x, y, z, qw, qx, qy, qz
  terrain_height.py          ground height at (x, y) from a baked heightmap
tools/                       standalone CLIs, see above
assets/                      sensor_rig.usd, the Montoldre scenes, montoldre_heightmap.npz
```

## Notes

- `/tf_static` is latched, so a plain echo shows nothing:
  `ros2 topic echo /tf_static --qos-durability transient_local --qos-reliability reliable --once`
- The pose is written per frame, not baked as USD time samples: in Isaac Sim 5.1
  the TF publisher does not follow time-sampled xformOps.
- In a `--save-stage` file the drawn path under `<world>/trajectory` is
  invisible by default so it cannot leak into camera or lidar output; toggle
  its visibility in the GUI to see it while scrubbing.
- `assets/sensor_rig.usd` declares `metersPerUnit = 0.01` but is authored in
  metres. USD does not rescale across a payload, so it composes correctly here,
  but a tool that honours the metadata would scale the rig by 100.
