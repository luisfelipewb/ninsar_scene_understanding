"""The repo's data/ folder (bags, farms, scene graphs, maps), next to ros/ and sim/."""

from pathlib import Path


def data_dir() -> Path:
    """<repo>/data, found by walking up from this file.

    Works from the source tree and from ros/install, since both live inside the repo.
    """
    here = Path(__file__).resolve()
    for directory in here.parents:
        if (directory / "ros").is_dir() and (directory / "data").is_dir():
            return directory / "data"
    raise FileNotFoundError(f"no data/ folder next to ros/ above {here}")
