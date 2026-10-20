# sim/isaac

Isaac Sim scenes and tools: a free-flying sensor rig (2× ZED X + Ouster OS1) flown through a
farm scene, publishing the same ROS 2 interface a robot has. Everything here runs with Isaac
Sim's own Python, `~/isaacsim/python.sh`, not with ROS's; it's outside `ros/`, so colcon never
sees it.

```bash
source /opt/ros/jazzy/setup.bash        # the ROS 2 bridge needs it, even under python.sh
cd ~/isaacsim
S=~/workspaces/ninsar-scene-understanding/sim/isaac
./python.sh $S/run_scene.py $S/config/montoldre_v4_small.yaml --headless
```

| path | |
|---|---|
| `run_scene.py` | build scene + rig + TF graph (+ walkers) from a run config, save it, fly it |
| `config/` | run configs; `config/waypoints/` trajectories |
| `rigsim/` | shared modules (config schema, waypoints, trajectory, terrain), importable under both interpreters |
| `tools/` | trajectory and scene tools (pxr), plus `bag_to_waypoints.py` (system `python3`) |
| `assets` | tracked link to `../../data/assets`: the scenes, rigs and heightmaps |

Setup: [docs/setup/isaac.md](../../docs/setup/isaac.md). Configs, tools and how to record a bag:
[docs/guides/isaac_sim.md](../../docs/guides/isaac_sim.md).
