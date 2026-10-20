"""Convert a ROS 1 .bag into a ROS 2 bag using the pure-Python `rosbags` package.

No ROS 1 installation is needed. `rosbags` is imported lazily so the ROS 2-only
workflow (POM_4x4) runs without it installed.
"""

from __future__ import annotations

from pathlib import Path

CONVERTED_SUFFIX = "_ros2"


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

    try:
        from rosbags.convert import convert as rosbags_convert
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise ConversionError(
            "the 'rosbags' package is required for ROS 1 bags: pip install rosbags"
        ) from exc

    dest.parent.mkdir(parents=True, exist_ok=True)
    print(f"  converting ROS 1 bag {source} -> {dest} ({storage})")
    try:
        rosbags_convert(**_convert_kwargs(rosbags_convert, source, dest, storage))
    except Exception as exc:  # pragma: no cover - depends on environment
        raise ConversionError(f"rosbags failed to convert {source}: {exc}") from exc
    return dest


def _convert_kwargs(func, source: Path, dest: Path, storage: str) -> dict:
    """Build the kwargs `rosbags.convert.convert` accepts in this version.

    The signature has changed across rosbags releases (`src` vs `srcs`, and
    `dst_storage` only exists in newer ones), so pass only what it declares.
    """
    import inspect

    params = inspect.signature(func).parameters
    kwargs: dict = {"srcs": [source]} if "srcs" in params else {"src": source}
    kwargs["dst"] = dest
    if "dst_storage" in params:
        kwargs["dst_storage"] = storage
    return kwargs


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
