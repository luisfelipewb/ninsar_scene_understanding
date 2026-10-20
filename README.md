# ninsar-scene-understanding

NINSAR WP4: scene understanding for farm robots. Robots and a simulated farm feed a perception
front end and a semantic 3D map, organized as a farm scene graph that planners can query.

| folder | what | runs with |
|---|---|---|
| [`ros/`](ros/) | the colcon workspace: our ROS 2 packages, pinned upstream sources, patches | ROS 2 Jazzy, `colcon` |
| `sim/` | simulators. Isaac Sim moves here next; until then it's in `~/workspaces/ninsar-isaac-sim` | the simulator's own Python |
| [`data/`](data/README.md) | bags, Isaac assets, farms, scene graphs, maps (local, not in git) | |
| [`docs/`](docs/README.md) | architecture, setup, guides, specs | |

## Quick start

```bash
source /opt/ros/jazzy/setup.bash               # first: rosdep needs ROS_DISTRO
cd ros
vcs import src/upstream < upstream.repos && ./patches/apply.sh
rosdep install --from-paths src --ignore-src -y -r
colcon build && source install/setup.bash
```

Then follow [docs/guides/run_isaac_bag.md](docs/guides/run_isaac_bag.md) to map the Isaac Sim
bag. The details of the build are in [docs/setup/ros.md](docs/setup/ros.md).
