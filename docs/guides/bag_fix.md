# bagfix

Config-driven repair of rosbags recorded on the NINSAR robots. A YAML file per
robot declares an ordered pipeline of fixes; bags live wherever they were
recorded and the repaired copy lands under `output/` in this workspace.

Replaces the three hardcoded scripts now kept in [legacy/](legacy/).

## Install

```bash
source /opt/ros/jazzy/setup.bash          # rosbag2_py, rclpy, rosidl_runtime_py
pip install -r requirements.txt           # PyYAML; rosbags only for ROS 1 bags
```

## Layout

Recordings look like this — the split files are always `bag_N.mcap` inside a
folder called `bag`, and the folder **above** it is what identifies the session:

```
2026-07-09-16-39-34/        <- session name, used to name the output
└── bag/
    ├── bag_0.mcap
    ├── bag_1.mcap
    └── metadata.yaml
```

Point at the session folder; the `bag/` sub-folder is found automatically (the
`bag/` folder itself, or one of its `.mcap` files, works too). The output
mirrors the input, so the naming is never ambiguous:

```
output/2026-07-09-16-39-34_fixed/bag/bag_0.mcap
```

Splitting is inherited from the source unless `output.max_bagfile_size` says
otherwise, and rosbag2 names split files after the bag directory, so the
`bag_N.mcap` convention carries through.

## Use

```bash
# see what is in a recording
python3 -m bagfix inspect /data/2026-07-09-16-39-34

# see what would change, without writing anything
python3 -m bagfix run configs/pom_4x4.yaml /data/2026-07-09-16-39-34 --dry-run

# write output/2026-07-09-16-39-34_fixed/bag/
python3 -m bagfix run configs/pom_4x4.yaml /data/2026-07-09-16-39-34

# one-off override without editing the config
python3 -m bagfix run configs/pom_4x4.yaml <session> --set stamp_offset.enabled=true

# what each transform accepts
python3 -m bagfix list-transforms
```

`run` exits 0 on success, 2 if some rule changed nothing (usually a topic name
that no longer matches), 1 on error.

Trektor's ROS 1 `.bag` files are converted to ROS 2 mcap first. The conversion
is written in the same layout so the naming scheme is reused, and it is cached
so tuning the config does not reconvert:

```bash
python3 -m bagfix run configs/trektor.yaml /data/test_acqui
#   /data/test_acqui/{bag/,}*.bag  ->  output/test_acqui_ros2/bag/   (cached)
#                                  ->  output/test_acqui_fixed/bag/
```

## Configs

One config per robot ([configs/pom_4x4.yaml](configs/pom_4x4.yaml),
[configs/trektor.yaml](configs/trektor.yaml)). Structure:

```yaml
robot: pom_4x4
output: {root: output, suffix: _fixed, storage: mcap}   # -> <root>/<session>_fixed/bag
convert: {from: ros1, cache: true}    # ROS 1 sources only

pipeline:
  - transform: frame_id               # rule name = the transform name
    topics: ["/visionary_b_two/*/rgb"]
    frame_id: robot_b_two_left_camera_optical_frame

bags:                                 # optional overrides, keyed by session name
  2026-07-08-12-00-05:
    stamp_offset: {enabled: true}
```

- `topics` entries are **globs**, so a changed camera serial still matches.
- Message types are read from the bag itself, never from the config.
- Any rule can be switched off with `enabled: false`.
- Override precedence: pipeline → `bags.<bag name>` → `--set rule.option=value`.
- Two rules of the same transform need distinct `id:` keys; the `id` (or the
  transform name) is what `bags:` and `--set` address.

## Transforms

| name | what it does |
|---|---|
| `frame_id` | Sets `header.frame_id` on the selected topics. |
| `camera_info` | Repairs `CameraInfo`: positive focal lengths, principal point defaulted to the image centre when degenerate, `R` := identity, `P` rebuilt from `K`. Each step is switchable. |
| `tf_static` | `mode: patch` corrects existing parent→child pairs; `mode: append` adds them, synthesizing `/tf_static` with `transient_local` QoS if the topic is absent. Takes `xyz` + `rpy` or `quat`. |
| `topic_filter` | `drop` / `keep` globs and exact `rename` mappings. Purely topic-level, so the payloads are never touched. |
| `stamp_offset` | Legacy lidar fix. `mode: auto` estimates `median(receive − header)` in a first pass; `fixed` uses `offset_ns`; `receive_time` overwrites the stamp. Disabled by default. |

## How it works

One read/write pass over the bag. The pipeline builds a `topic → transforms`
map up front: topics no transform claims are copied **raw** and never
deserialized, which is what keeps images and point clouds cheap. A claimed
message is deserialized once, passed through its transforms in config order, and
serialized once. Source `TopicMetadata` is reused verbatim so ids, QoS (e.g.
`transient_local` on `/tf_static`) and type hashes survive.

Only `stamp_offset` in `auto` mode needs a first pass; without it the run is
single-pass.

## Adding a transform

Drop a module in [bagfix/transforms/](bagfix/transforms/), subclass `Transform`,
decorate with `@register("name")`, implement `configure()` and `apply()`, and
import it from [bagfix/transforms/__init__.py](bagfix/transforms/__init__.py).
Topic-level hooks (`plan_topics`, `route`, `emit`) are there for transforms that
add, drop or rename topics.

## Tests

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest tests/ -v
```

`tests/test_roundtrip.py` builds a synthetic bag, runs the full pipeline and
asserts the fixes landed — including that untouched topics come out
byte-identical. No external data needed.
