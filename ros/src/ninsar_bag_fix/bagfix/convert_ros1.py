"""Convert a ROS 1 .bag into a ROS 2 bag with the `rosbags-convert` CLI.

No ROS 1 installation is needed. The CLI comes from the pure-Python `rosbags` package,
installed on its own with `pipx install rosbags`, so it never touches the ROS Python.
It's only needed for ROS 1 recordings (Trektor); the ROS 2-only workflow (POM_4x4) runs
without it.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

CONVERTED_SUFFIX = "_ros2"
CONVERT_CLI = "rosbags-convert"


class ConversionError(Exception):
    pass


def convert(source: str | Path, dest: str | Path, storage: str = "mcap",
            cache: bool = True) -> Path:
    """Convert `source` (.bag) to a ROS 2 bag at `dest`. Returns `dest`.

    With `cache` an existing `dest` is reused instead of being reconverted.
    """
    source, dest = Path(source), Path(dest)
    if not source.exists():
        raise ConversionError(f"ROS 1 bag not found: {source}")

    if dest.exists():
        if cache:
            print(f"  reusing cached conversion at {dest}")
            return dest
        raise ConversionError(f"{dest} already exists (set convert.cache or remove it)")

    exe = shutil.which(CONVERT_CLI)
    if exe is None:
        raise ConversionError(
            f"ROS 1 bags need the {CONVERT_CLI} CLI: pipx install rosbags"
        )

    dest.parent.mkdir(parents=True, exist_ok=True)
    print(f"  converting ROS 1 bag {source} -> {dest} ({storage})")
    cmd = [exe, "--src", str(source), "--dst", str(dest), "--dst-storage", storage]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise ConversionError(f"{CONVERT_CLI} failed to convert {source}: {detail}")
    return dest


def find_ros1_bag(path: str | Path) -> Path | None:
    """Locate a ROS 1 .bag under `path`, accepting the same layouts as ROS 2 bags.

    Handles a .bag file directly, a session folder holding one, and a session
    folder with the .bag inside a `bag/` sub-directory.
    """
    path = Path(path)
    if path.is_file():
        return path if path.suffix == ".bag" else None
    if not path.is_dir():
        return None

    for directory in (path, path / "bag"):
        if not directory.is_dir():
            continue
        found = sorted(directory.glob("*.bag"))
        if len(found) == 1:
            return found[0]
        if found:
            raise ConversionError(
                f"{directory} holds several ROS 1 bags "
                f"({', '.join(f.name for f in found)}); rosbags converts one at a "
                "time, so point at a single .bag file"
            )
    return None


def converted_path(session: str, root: str | Path) -> Path:
    """Where the converted copy lands: <root>/<session>_ros2/bag."""
    from .bagio import BAG_SUBDIR

    return Path(root) / f"{session}{CONVERTED_SUFFIX}" / BAG_SUBDIR
