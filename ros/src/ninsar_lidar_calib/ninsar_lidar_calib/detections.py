"""Read the JSONL detection log written by the C++ detectors."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Detection:
    """One scan's contribution to the calibration."""

    stamp_ns: int
    #: RTK pose, world <- base_link, as a 4x4 matrix
    world_from_base: np.ndarray
    #: ground plane in the lidar frame, or None
    ground_normal: np.ndarray | None = None
    ground_offset: float | None = None
    #: corner in the lidar frame, or None
    corner_point: np.ndarray | None = None
    corner_axes: np.ndarray | None = None

    @property
    def has_ground(self) -> bool:
        return self.ground_normal is not None

    @property
    def has_corner(self) -> bool:
        return self.corner_point is not None


class DetectionError(Exception):
    pass


def _isometry(record: dict) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(record["R"], dtype=float).reshape(3, 3)
    matrix[:3, 3] = np.asarray(record["t"], dtype=float)
    return matrix


def load(path: str | Path) -> list[Detection]:
    """Parse a detections file. Malformed lines are an error, not a silent skip."""
    path = Path(path)
    if not path.is_file():
        raise DetectionError(f"detections file not found: {path}")

    detections: list[Detection] = []
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        line = line.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
            detection = Detection(
                stamp_ns=int(record["stamp_ns"]),
                world_from_base=_isometry(record["world_from_base"]),
            )
            if "ground" in record:
                detection.ground_normal = np.asarray(record["ground"]["normal"], float)
                detection.ground_offset = float(record["ground"]["offset"])
            if "corner" in record:
                detection.corner_point = np.asarray(record["corner"]["point"], float)
                detection.corner_axes = np.asarray(
                    record["corner"]["axes"], float
                ).reshape(3, 3)
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            raise DetectionError(f"{path}:{number}: malformed detection ({exc})") from exc
        detections.append(detection)

    if not detections:
        raise DetectionError(f"{path} holds no detections")
    return detections
