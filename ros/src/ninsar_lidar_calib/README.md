# ninsar_lidar_calib

Calibrates the `base_link → lidar` extrinsic from a recording of the robot
driving around a **fixed corner target** — three mutually perpendicular faces of
a cube — while RTK GPS provides `map → robot_base_link`.

Two facts make it solvable:

1. The **ground plane** is horizontal in the world, which fixes roll and pitch.
2. The **corner does not move**, so with the RTK pose known, any apparent motion
   of the detected corner is extrinsic error. Minimizing it recovers the rest.

The RTK pose is treated as ground truth and is never optimized.

Detection is C++ (PCL `SACSegmentation`, `CropBox`, `VoxelGrid`); the fit is
Python (`scipy.optimize.least_squares`).

## Workflow

### 1. Tune the guesses live

```bash
ros2 launch ninsar_lidar_calib calibrate.launch.py
ros2 bag play /data/2026-07-09-16-39-34/bag --clock
```

Every guess is a live ROS parameter — change it and the next scan uses it:

```bash
ros2 param set /lidar_calib_node corner.roi_center [2.5, 0.0, 0.4]
ros2 param set /lidar_calib_node corner.roi_size   [1.5, 1.5, 1.2]
ros2 param set /lidar_calib_node ground.eps_angle_deg 20.0
```

RViz shows the ROI as a yellow wireframe box (in `robot_base_link`, so it rides
with the robot), the detected ground, the three faces, the corner and its axes.
The terminal reports what is being accepted and, when it is not, **why** — too
few inliers, faces not perpendicular, extents outside `size_guess`. That is what
tells you which parameter to move. Bad values are rejected with a reason rather
than accepted and failing later.

Save the tuned values when the detections look right:

```bash
ros2 param dump /lidar_calib_node > my_calib.yaml
# or, from the node itself:
ros2 service call /lidar_calib_node/save_config std_srvs/srv/Trigger
```

### 2. Run the offline pass

Deterministic, and it does not need `ros2 bag play` — it reads the bag directly
and replays TF into a `BufferCore`, so pose lookups are exact:

```bash
ros2 run ninsar_lidar_calib detect_from_bag \
    --bag /data/2026-07-09-16-39-34 \
    --ros-args --params-file my_calib.yaml
```

Accepts the session folder or the `bag/` folder inside it, like `bagfix`.
Writes `detections.jsonl`, one record per accepted scan.

### 3. Solve

```bash
ros2 run ninsar_lidar_calib solve detections.jsonl \
    --xyz 0.2 0.0 1.3 --rpy 0 0 0 \
    --base-height 0.38 \
    --base-frame robot_base_link --lidar-frame lidar
```

Prints the calibrated extrinsic, the corner spread before and after, and a
ready-to-paste `tf_static` rule for a `bagfix` config, so a recorded bag can be
repaired with the result.

Exits 2 when some parameter was not observable — see below.

## Two things that will bite you

**Pass `--base-height`** (base_link above the ground, from the URDF). Without
it, raising the lidar, the ground and the target *together* changes nothing that
is measured: a plane's normal is unchanged by translation, and a ground-driving
robot only rotates about world z. The height is then genuinely unrecoverable and
the solver says so instead of inventing a number.

**Vary the recording.** Circling the target at constant range while pointing at
it puts the target in the *same* `base_link` position on every scan, which
constrains the extrinsic no better than one observation does. Vary the distance
and approach the target off-centre. Both degeneracies are covered by tests, and
the solver reports them from the Jacobian's null space rather than from residual
size — a degenerate recording still fits perfectly, it is ambiguous, not noisy.

## Parameters

See [config/lidar_calib.yaml](config/lidar_calib.yaml). Notable ones:

| parameter | meaning |
|---|---|
| `corner.roi_center` / `roi_size` | the ROI box in `robot_base_link`; the main thing you tune |
| `corner.exclude_ground` | drop ground points before segmenting. An upright cube's vertical faces *are* perpendicular to the ground, so without this a ground patch is accepted as a third face and the corner lands on the wrong edge |
| `corner.size_guess` / `size_margin` | optional; `0` disables the check. Only rejects bad fits, never used in the maths |
| `ground.eps_angle_deg` | how far from the guessed normal RANSAC may wander. This is what stops it latching onto a wall |
| `initial_extrinsic.source` | `tf_static` reads the current (wrong) extrinsic from the bag |

## Design notes

- **One detector library, two frontends** (`calib_node`, `detect_from_bag`), so
  what you tuned live is exactly what the offline pass computes.
- **Detections are JSONL**, not a custom message — the solver reads them
  directly and a run's detections outlive the bag and the node.
- **Face labelling** is canonicalized in the *world* frame against the first
  accepted detection, choosing the best of the 24 proper rotations of the axis
  triad. RANSAC returns faces in arbitrary order with arbitrary normal signs; a
  permutation or flip would corrupt the orientation residual silently.
- The solver carries the target's world pose as unknowns alongside the extrinsic
  rather than differencing against a running mean — better conditioned.

## Tests

```bash
colcon test --packages-select ninsar_lidar_calib && colcon test-result --all
```

C++ tests run the detectors on synthetic ground/corner clouds with a known
answer; Python tests run the solver on synthetic trajectories, checking both
that a good recording recovers the extrinsic to <1 mm and that the two
degenerate cases above are *reported* rather than answered.
