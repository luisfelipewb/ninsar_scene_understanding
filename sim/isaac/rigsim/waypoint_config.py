#!/usr/bin/env python3
"""Read and write the waypoint YAML files that drive the sensor rig.

Only PyYAML: no numpy, no ROS, no Isaac Sim. That keeps it importable before
SimulationApp starts -- so a typo in a config fails in a second rather than
after the simulator has booted -- and equally importable from the bag
extractor, which runs under the ROS 2 interpreter instead.

A waypoint is (x, y, z, roll_deg, pitch_deg, yaw_deg). Waypoints may also carry
a `t` in seconds; see load_waypoints() for what that changes.
"""

import os

import yaml

WAYPOINT_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
TIME_FIELD = "t"

# Two waypoints closer than this in time cannot be told apart once written at
# millisecond precision, and would give the interpolator a zero-length segment.
MIN_TIME_SPACING = 1e-3


def load_waypoints(path):
    """Read waypoints, and their timing if the file carries any, from a YAML file.

    The document is either a bare list of waypoints or a mapping with a
    `waypoints` key, leaving room for other settings alongside them. See
    parse_waypoints() for the format of the list and what is returned.
    """
    with open(path) as handle:
        document = yaml.safe_load(handle)
    if isinstance(document, dict):
        document = document.get("waypoints")
    return parse_waypoints(document, path)


def parse_waypoints(document, label):
    """Validate an already-parsed waypoint list, from a file or inline in a config.

    Returns (waypoints, times): a list of (x, y, z, roll_deg, pitch_deg,
    yaw_deg) tuples, and either a matching list of seconds or None. `label`
    names the source in error messages.

    A waypoint is a mapping of any of WAYPOINT_FIELDS -- omitted ones are 0.0,
    so a waypoint only names what it changes -- or a six-element list in that
    order.

    A mapping may also carry `t`, the time in seconds at which the rig should be
    there. Either every waypoint has one or none does. With times the trajectory
    replays on the file's own clock, keeping the pauses and accelerations of
    whatever produced it; without them the pace comes from a speed and a yaw
    rate. Times must be strictly increasing; the first one is the origin, so it
    need not be zero.
    """
    path = label
    if not isinstance(document, list) or not document:
        raise ValueError(f"{path}: expected a non-empty list of waypoints")

    waypoints, times = [], []
    for index, entry in enumerate(document):
        if isinstance(entry, dict):
            unknown = set(entry) - set(WAYPOINT_FIELDS) - {TIME_FIELD}
            if unknown:
                raise ValueError(
                    f"{path}: waypoint {index} has unknown field(s) {sorted(unknown)}; "
                    f"expected any of {list(WAYPOINT_FIELDS)} and {TIME_FIELD!r}"
                )
            row = [entry.get(field, 0.0) for field in WAYPOINT_FIELDS]
            stamp = entry.get(TIME_FIELD)
        elif isinstance(entry, (list, tuple)) and len(entry) == len(WAYPOINT_FIELDS):
            row, stamp = list(entry), None
        else:
            raise ValueError(
                f"{path}: waypoint {index} must be a mapping of {list(WAYPOINT_FIELDS)} "
                f"or a list of six numbers; got {entry!r}"
            )
        try:
            waypoints.append(tuple(float(value) for value in row))
            times.append(None if stamp is None else float(stamp))
        except (TypeError, ValueError):
            raise ValueError(f"{path}: waypoint {index} has a non-numeric value: {entry!r}") from None

    timed = [index for index, stamp in enumerate(times) if stamp is not None]
    if not timed:
        return waypoints, None
    if len(timed) != len(times):
        untimed = [index for index, stamp in enumerate(times) if stamp is None]
        raise ValueError(
            f"{path}: waypoints {untimed[:5]} have no {TIME_FIELD!r} while others do; "
            f"either every waypoint is timed or none is"
        )
    for index in range(1, len(times)):
        if times[index] <= times[index - 1]:
            raise ValueError(
                f"{path}: waypoint {index} has {TIME_FIELD}={times[index]}, which does not "
                f"advance on waypoint {index - 1}'s {times[index - 1]}; times must increase"
            )
    return waypoints, times


def save_waypoints(path, waypoints, times=None, comments=(), source=None):
    """Write a waypoint YAML that load_waypoints() reads back unchanged.

    `comments` are emitted as a leading comment block and `source` as a
    top-level mapping -- provenance for generated files. load_waypoints()
    ignores every top-level key but `waypoints`.
    """
    if times is not None and len(times) != len(waypoints):
        raise ValueError(f"got {len(waypoints)} waypoints but {len(times)} times")

    lines = [f"# {comment}" for comment in comments]
    if lines:
        lines.append("")
    if source:
        lines.append(yaml.safe_dump({"source": source}, sort_keys=False, default_flow_style=False).rstrip())
        lines.append("")
    lines.append("waypoints:")
    for index, waypoint in enumerate(waypoints):
        fields = [] if times is None else [f"{TIME_FIELD}: {times[index]:.3f}"]
        fields += [f"{name}: {value:.3f}" for name, value in zip(WAYPOINT_FIELDS, waypoint)]
        lines.append("  - {" + ", ".join(fields) + "}")

    with open(path, "w") as handle:
        handle.write("\n".join(lines) + "\n")
    return os.path.abspath(path)
