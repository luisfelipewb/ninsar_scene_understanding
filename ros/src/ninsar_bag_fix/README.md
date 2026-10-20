# bagfix

Config-driven repair of rosbags recorded on the NINSAR robots. A YAML file per
robot declares an ordered pipeline of fixes; bags live wherever they were
recorded and the repaired copy lands under `output/` in this workspace.

Replaces the three hardcoded scripts now kept in [legacy/](../../legacy/).

## Install

This is a package in the workspace at the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace root> && colcon build --packages-select ninsar_bag_fix
source install/setup.bash                 # puts `bagfix` on the PATH
```

To run straight from the source tree instead, without building:

```bash
export PYTHONPATH=src/ninsar_bag_fix:$PYTHONPATH
```

`rosbags` (pip) is needed only for Trektor's ROS 1 bags.

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
bagfix inspect /data/2026-07-09-16-39-34

# see what would change, without writing anything
bagfix run config/pom_4x4.yaml /data/2026-07-09-16-39-34 --dry-run

# write output/2026-07-09-16-39-34_fixed/bag/
bagfix run config/pom_4x4.yaml /data/2026-07-09-16-39-34

# one-off override without editing the config
bagfix run config/pom_4x4.yaml <session> --set stamp_offset.enabled=true

# what each transform accepts
bagfix list-transforms
```

`run` exits 0 on success, 2 if some rule changed nothing (usually a topic name
that no longer matches), 1 on error.

Trektor's ROS 1 `.bag` files are converted to ROS 2 mcap first. The conversion
is written in the same layout so the naming scheme is reused, and it is cached
so tuning the config does not reconvert:

```bash
bagfix run config/trektor.yaml /data/test_acqui
#   /data/test_acqui/{bag/,}*.bag  ->  output/test_acqui_ros2/bag/   (cached)
#                                  ->  output/test_acqui_fixed/bag/
```

## Configs

One config per robot ([config/pom_4x4.yaml](config/pom_4x4.yaml),
[config/trektor.yaml](config/trektor.yaml)). Structure:

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
| `image_encoding` | Rewrites `Image.encoding` (e.g. `mono16` -> `16UC1`) without touching the pixels. Refuses before writing if the buffer is not laid out the way the target encoding requires. See below. |
| `image_channel_swap` | Exchanges the red and blue channels of an image and relabels it (e.g. `bgr8` -> `rgb8`). Unlike `image_encoding`, this one *does* rewrite the pixels. See below. |
| `tf_static` | Three lists — `remove`, `edit`, `add` — over the static transforms. See below. |
| `topic_filter` | `drop` / `keep` globs and exact `rename` mappings. Purely topic-level, so the payloads are never touched. |
| `stamp_offset` | Legacy lidar fix. `mode: auto` estimates `median(receive − header)` in a first pass; `fixed` uses `offset_ns`; `receive_time` overwrites the stamp. Disabled by default. |

### Editing static transforms

Three lists, applied in the order `remove` → `edit` → `add`, so replacing a
frame is a single rule:

```yaml
- transform: tf_static
  remove:                                   # globs allowed here
    - {parent: base_link, child: old_lidar}
    - {parent: base_link, child: "cam_*"}   # either key may be a glob
    - {child: junk_frame}                   # omit a key to mean "any"
  edit:                                     # must already be in the bag
    - parent: base_link
      child: lidar
      xyz: [0.35, -0.08, 1.42]
      rpy: [0.03, -0.06, 0.11]              # radians; `rpy_deg:` takes degrees,
                                            # or give `quat: [x, y, z, w]`
  add:                                      # must not already be in the bag
    - {parent: base_link, child: gnss, xyz: [0.1, 0.0, 0.9], rpy: [0, 0, 0]}
```

`edit` and `add` take the same entry — a complete pose — and differ only in what
they assert about the bag. **That assertion is checked before anything is
written**: naming a frame that is not there (or one that already is) fails the
run with nothing produced, so a typo in a frame name costs a second rather than
a bag that looks repaired and is not. The check reads only `/tf_static`, so it
is an index lookup, not a pass over the whole bag. `--dry-run` performs it too.

Other behaviour worth knowing:

- **Globs only in `remove`.** In `edit`/`add` a glob would give several frames
  the same pose, so it is refused.
- A `remove` entry with **neither** key is refused rather than silently matching
  everything — write `{parent: "*"}` if you really mean all of them.
- A removal matching nothing is a **warning**, not an error: deleting what is
  already absent is harmless.
- A `/tf_static` message that removals emptied is dropped; `drop_empty: false`
  keeps it.
- If the bag has no `/tf_static` at all, `add` synthesizes the topic with
  `transient_local` QoS.

### Relabelling an image encoding

The Visionary-B records depth as `mono16`, and `depth_image_proc` rejects it:

```
[ERROR] [PointCloudXyzrgbNode]: Depth image has unsupported encoding [mono16]
```

`mono16` and `16UC1` describe the same buffer — one channel, 16-bit unsigned —
so the fix is a relabel, not a conversion:

```yaml
- transform: image_encoding
  topics: ["/visionary_b_two/*/depth"]
  from: mono16      # only rewrite frames currently in this encoding
  to: 16UC1
```

**The label carries a unit.** `mono16` says nothing about what the values mean;
`16UC1` is defined by the depth pipeline to be **millimetres** (`32FC1` is
metres). Relabelling therefore asserts a unit the bytes cannot prove — if the
sensor's raw values are not millimetres, every derived point is scaled by a
constant and nothing about the bag looks wrong. Confirm the unit against
something external before trusting the output.

What the transform *can* check, it checks in `preflight` — before any output is
written — by reading the first frame of each selected topic:

- source and target must be the same bytes/pixel (a relabel cannot reinterpret
  a buffer),
- `step == width * bytes/pixel`,
- `len(data) == height * step`,
- the encoding actually present is the `from` you named. A bag already in the
  target encoding fails rather than being relabelled twice.

`is_bigendian` being set is reported as a warning, since consumers that assume
host byte order will misread the image.

### Swapping the colour channels

The Visionary-B colour stream is recorded as `bgr8`, and the bytes really are
B, G, R — so everything that trusts the header renders red and blue the wrong
way round. `bgr8` and `rgb8` are both three bytes per pixel, so `image_encoding`
would accept the rule and produce a bag that is still wrong, just relabelled.
Reordering the channels is a different transform:

```yaml
- transform: image_channel_swap
  topics: ["/visionary_b_two/*/rgb"]
  from: bgr8
  to: rgb8
```

**Pick the transform against the image, not the header.** The two cases look
identical from the metadata and want opposite fixes:

| the pixels are… | the header says | use |
|---|---|---|
| already R, G, B | `bgr8` | `image_encoding` — relabel, pixels untouched |
| really B, G, R | `bgr8` | `image_channel_swap` — reorder, then relabel |

Choosing wrong gives a bag that looks repaired and is not, so confirm against
something in the scene whose colour you know before trusting the output.

Supported pairs are `bgr8 <-> rgb8` and `bgra8 <-> rgba8`; the swap exchanges
channel 0 and channel 2 and leaves any alpha byte where it is. Packed formats
like `yuv422` and bayer patterns have no channel to exchange and are refused
rather than mangled.

The swap is **its own inverse**, so a second run would silently undo the first.
That is why `from:` is required here (it is optional for `image_encoding`), and
why `preflight` refuses a topic that is already in the target encoding — before
any output is written. It checks the same layout invariants as the relabel:
`step == width * bytes/pixel` and `len(data) == height * step`.

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
colcon test --packages-select ninsar_bag_fix && colcon test-result --all
# or directly:
python3 -m pytest src/ninsar_bag_fix/test -v
```

`test/test_roundtrip.py` builds a synthetic bag, runs the full pipeline and
asserts the fixes landed — including that untouched topics come out
byte-identical. No external data needed.
