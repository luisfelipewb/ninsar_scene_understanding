# Patches to upstream sources

The upstream checkouts in `ros/src/upstream/` stay at the SHAs pinned in
[`upstream.repos`](../upstream.repos). Changes we need go into our own packages (config, launch,
adapter nodes) whenever possible. A patch is the last resort: each one here changes something no
node, parameter or launch file of ours can reach.

| patch | upstream repo | base SHA (= pin) | why it can't live outside | upstream PR | remove when |
|---|---|---|---|---|---|
| `voxeland_modehas.patch` | MAPIRlab/Voxeland | `389e3ec27b8b49caf252948939366381cad0b48c` | compiled code, see below | not yet | upstream fixes `modeHas()` |
| `spark_dsg_python_install.patch` | MIT-SPARK/Spark-DSG | `3c40997e4ae739c56ed3cdd6ef5e882d403d8b64` | CMake install rule, see below | not yet | upstream installs the Python module in plain CMake builds |

Files:

- `series`: which patch goes to which checkout, in order.
- `apply.sh`: applies the series. Safe to re-run; it stops if a patch no longer fits its checkout.
- `*.patch`: plain `git diff` output made inside the upstream checkout, e.g.
  `git -C ros/src/upstream/voxeland diff > ros/patches/voxeland_modehas.patch`.

After `apply.sh`, `vcs status` shows the patched checkouts as modified. That's expected.

## `voxeland_modehas.patch`

`VoxelandServer::modeHas()` tests "any bit" (`(currentMode & mode) != 0`). Because
`SemanticsInstances = Semantics | 1<<3`, `modeHas(SemanticsInstances)` is true in **every**
semantic mode, including the dense `RGBSemantics` mode we run (`semantics_as_instances: false`).
The bug is on every upstream branch as of 2026-10.

Without the patch the map is still correct. The instance-only code paths reach a fallback that
only logs an error. The side effects are:

- The debug GUI logs `Tried to auto-template a semantics function with non-semantic enum value!`
  every 30 ms (about 33 lines/s) from the first cloud onward, which buries the server output.
- `save_map` logs the same error once and writes empty `voxeland_instanceMap*.json` files.

There's no outside fix. The GUI is always compiled in (`SET(ENABLE_DEBUG_GUI ON)` is a plain
CMake variable, so `-D` can't override it). Setting the `VXL` logger to `fatal` would also hide
the real messages. This would make a good upstream PR.

## `spark_dsg_python_install.patch`

`ninsar_farm` writes and checks scene graphs with spark_dsg's Python bindings. Upstream's
supported route for those is `pip install`, which would need a venv next to ROS. Its plain CMake
build (what colcon runs) can compile the bindings (`-DSPARK_DSG_BUILD_PYTHON=ON`, set for
spark_dsg only in `ros/colcon.meta`, so other CMake packages don't warn about an unused
variable), but it installs only `_dsg_bindings.so`, into
`<prefix>/spark_dsg/`, which isn't on any Python path, and leaves out the `.py` files.

The patch changes only the non-pip branch of `python/CMakeLists.txt`: the bindings and
`spark_dsg/**/*.py` go to `lib/python3.X/site-packages/spark_dsg`, which colcon adds to
`PYTHONPATH`. pip builds (`SKBUILD`) keep upstream's behaviour, so this could go upstream as is.
The bindings only need apt packages (pybind11-dev, python3-numpy, python3-shapely, python3-click,
python3-zmq).

## Apply, add, bump

```bash
cd ros
vcs import src/upstream < upstream.repos
./patches/apply.sh
```

- **Add a patch.** Edit the checkout, write the diff into `ros/patches/`, add a line to `series`
  and a row to the table above.
- **Bump a pin.** Change the SHA in `upstream.repos`, run `vcs import --force src/upstream < upstream.repos`
  and `./patches/apply.sh`. If a patch fails, redo the change on the new SHA, regenerate the patch
  and update its base SHA here, all in the same commit.
- **Upstream merged it.** Delete the patch, its `series` line and its row.
