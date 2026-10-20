# CLAUDE.md

NINSAR WP4 scene understanding. Read [README.md](README.md) for the layout and
[docs/README.md](docs/README.md) for the reading order; this file only holds the rules that are
easy to break.

## Two Python worlds

- `ros/` is the only colcon workspace: ROS 2 Jazzy, system Python. Every dependency is an apt
  package through rosdep, or upstream source built by colcon. No pip, no venv, no
  `requirements.txt`; a new Python dependency is a rosdep key in `package.xml`.
- `sim/isaac/` runs on Isaac Sim's Python (`~/isaacsim/python.sh`) and is invisible to colcon.
  The one exception is `tools/bag_to_waypoints.py`, which reads bags and so runs under the ROS
  Python; each script's docstring says which interpreter it needs. `ros/` never imports Isaac or
  `rigsim`. The two sides meet only through bags, frames, labels and generated files (the
  contract is in [docs/architecture.md](docs/architecture.md#isaac-sim-and-ros)).

## Build and test

From `ros/`, after `source /opt/ros/jazzy/setup.bash && source install/setup.bash`. Always select
our packages; a bare `colcon build` or `colcon test` also rebuilds and tests all of upstream.

```bash
P="ninsar_perception ninsar_mapping ninsar_bringup ninsar_farm ninsar_bag_fix ninsar_lidar_calib ninsar_tf_tools"
colcon build --packages-select $P
colcon test --packages-select $P && colcon test-result --verbose
```

Python, launch and config edits take effect without a rebuild (symlink-install); C++ edits don't.
More in [docs/setup/ros.md](docs/setup/ros.md).

## Upstream sources

Never edit `ros/src/upstream/`; it's imported by vcs and git-ignored. Fix things in our packages
(config, launch, adapter nodes). Only when that's impossible, add a patch: the rules are in
[ros/patches/README.md](ros/patches/README.md). A pin in `ros/upstream.repos` and its patch's base
SHA change in the same commit.

## data/

Git-ignored except `data/README.md`. Never commit files from it, and never commit absolute paths
to it: code finds it relative to the repo (`ninsar_farm.paths.data_dir()`, the launch files'
`data_dir` arg, bagfix's `output.root` relative to `ros/`). `data/assets` and
`sim/isaac/assets` are links, never copies.

## Keep in sync

- `docs/specs/` are the contracts between components. Changing one side of an interface means
  updating its spec in the same commit.
- `ninsar_bringup/launch/isaac_voxeland.launch.xml` and `.yaml` are the same launch in two
  formats: change both.
- After editing `docs/diagrams/ninsar_overview.drawio`, re-export the changed pages to
  `docs/diagrams/png/`.
- Doc examples run from `ros/` and use the `data/` layout (`../data/bags/robot/raw/<session>`).

## Commits

Imperative subject without a prefix ("Add X", "Split Y into Z"). For more than one change, the body
is a bullet per change that says why, not only what.
