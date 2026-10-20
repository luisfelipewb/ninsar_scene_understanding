#!/usr/bin/env python3
"""Read and validate the run config that run_scene.py builds and flies from.

Only PyYAML, so this imports before SimulationApp does: a missing rig asset or
a malformed prim path is worth hearing about in a second rather than after the
simulator has spent a minute booting.

load_scene() returns a plain dict with every default filled in, every path made
absolute and every prim path resolved, so nothing downstream has to ask whether
a key is present. Relative paths resolve against the project directory (the one
holding run_scene.py), never against the shell's cwd.
"""

import os

import yaml

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# key -> (default, type). A default of REQUIRED means the file has to say. The
# type may be a tuple, in which case any of them is accepted as given.
REQUIRED = object()

SCHEMA = {
    "scene": {
        "usd": (None, str),
        "world_prim": ("/World", str),
        "fps": (60.0, float),
        # HDRI for the dome light: a local path (resolved like scene.usd) or a
        # URI (omniverse://, http(s)://) left as-is. null keeps the flat-color
        # dome the empty scene starts with; the sublayered-scene branch defines
        # a dome light only if this is set, so a scene's own lighting is left
        # alone by default.
        "skybox": (None, str),
        "skybox_intensity": (1000.0, float),
    },
    "base_link": {
        "name": ("base_link", str),
        "translate": ([0.0, 0.0, 0.0], list),
        "rotate": ([0.0, 0.0, 0.0], list),
    },
    "rig": {
        "usd": (REQUIRED, str),
        "name": ("sensor_rig", str),
        "load_as": ("payload", str),
        "translate": ([0.0, 0.0, 0.0], list),
        "rotate": ([0.0, 0.0, 0.0], list),
    },
    "ros": {
        "graph_prim": ("/World/Graphs/ROS_TF", str),
        "dynamic_topic": ("/tf", str),
        "static_topic": ("/tf_static", str),
        "domain_id": (None, int),
    },
    # Animated people, driven by omni.anim.people. Each walker is checked
    # against WALKER_FIELDS; (list,) rather than list because a bare list
    # means a translate/rotate triple to _field.
    "people": {
        # Baked heightmap, as trajectory.terrain. When set, every walker's z is
        # held at a clearance above the ground under it, frame by frame;
        # otherwise walkers stay at their starting height whatever GoTo says.
        "terrain": (None, str),
        "loop": (False, bool),          # walk back to the start and repeat, forever
        "walkers": (None, (list,)),
    },
    # Last, and `waypoints` last within it: an inline waypoint list can run to
    # hundreds of rows, and everything else should stay readable above it.
    "trajectory": {
        "speed": (0.5, float),          # m/s along the path; untimed waypoints only
        "yaw_rate": (30.0, float),      # deg/s for in-place turns; untimed waypoints only
        # Baked heightmap (.npz from tools/bake_heightmap.py). When set, every
        # z in the trajectory becomes a clearance above the ground under it.
        "terrain": (None, str),
        "loop": (False, bool),
        "realtime": (False, bool),      # pace to the wall clock, dropping frames
        "settle": (0.0, float),         # seconds slept after each frame for DDS to drain
        # A waypoint YAML (path) or an inline list in the same format. null
        # parks the rig at base_link.translate and plays until interrupted.
        "waypoints": (None, (str, list)),
    },
}

LOAD_MODES = ("payload", "reference")

WALKER_FIELDS = {
    # Prim name under <world>/Characters, and the name the commands address.
    "name": (REQUIRED, str),
    # A folder of the Isaac People library (e.g. F_Business_02), or a
    # character .usd: a local path resolved like scene.usd, or a URI.
    "asset": (REQUIRED, str),
    "translate": ([0.0, 0.0, 0.0], list),
    "yaw": (0.0, float),                # degrees about +z
    # omni.anim.people commands without the leading name, e.g. "GoTo 4 0 0 _".
    "commands": (None, (list,)),
}

# The omni.anim.people commands a walker may be given: only the ones that need
# nothing but the ground. Sit, Queue and GoToObject need scene objects.
COMMANDS = ("GoTo", "Idle", "LookAround")

# omni.anim.people keeps its shared animation graph here, beside the walkers.
RESERVED_WALKER_NAMES = ("Biped_Setup",)


def load_scene(path):
    """Parse, validate and normalise a scene YAML."""
    with open(path) as handle:
        document = yaml.safe_load(handle)
    if not isinstance(document, dict):
        raise ValueError(f"{path}: expected a mapping of {list(SCHEMA)}, got {type(document).__name__}")

    unknown = set(document) - set(SCHEMA)
    if unknown:
        raise ValueError(f"{path}: unknown section(s) {sorted(unknown)}; expected {list(SCHEMA)}")

    config = {}
    for section, fields in SCHEMA.items():
        given = document.get(section) or {}
        if not isinstance(given, dict):
            raise ValueError(f"{path}: section {section!r} must be a mapping, got {type(given).__name__}")
        unknown = set(given) - set(fields)
        if unknown:
            raise ValueError(f"{path}: unknown key(s) {sorted(unknown)} in {section!r}; expected {list(fields)}")
        config[section] = {
            name: _field(path, section, name, given, default, kind) for name, (default, kind) in fields.items()
        }

    _normalise(path, config)
    return config


def _field(path, section, name, given, default, kind):
    """One key, defaulted and type-checked. None stays None where it is allowed."""
    if name not in given:
        if default is REQUIRED:
            raise ValueError(f"{path}: {section}.{name} is required")
        return default
    value = given[name]
    if value is None and default is None:
        return None
    if kind is float and isinstance(value, int) and not isinstance(value, bool):
        return float(value)
    if kind is list:
        return _triple(path, f"{section}.{name}", value)
    if not isinstance(value, kind) or isinstance(value, bool) is (kind is not bool):
        expected = " or ".join(k.__name__ for k in (kind if isinstance(kind, tuple) else (kind,)))
        raise ValueError(f"{path}: {section}.{name} must be {expected}, got {value!r}")
    return value


def _triple(path, where, value):
    """A three-number list, the shape every translate and rotate takes."""
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{path}: {where} must be a list of three numbers, got {value!r}")
    try:
        return [float(component) for component in value]
    except (TypeError, ValueError):
        raise ValueError(f"{path}: {where} has a non-numeric component: {value!r}") from None


def _normalise(path, config):
    """Resolve paths and derive the prim paths the rest of the code works from."""
    if config["rig"]["load_as"] not in LOAD_MODES:
        raise ValueError(f"{path}: rig.load_as must be one of {list(LOAD_MODES)}, got {config['rig']['load_as']!r}")
    if config["scene"]["fps"] <= 0.0:
        raise ValueError(f"{path}: scene.fps must be positive, got {config['scene']['fps']}")

    trajectory = config["trajectory"]
    for key in ("speed", "yaw_rate"):
        if trajectory[key] <= 0.0:
            raise ValueError(f"{path}: trajectory.{key} must be positive, got {trajectory[key]}")
    if trajectory["settle"] < 0.0:
        raise ValueError(f"{path}: trajectory.settle must not be negative, got {trajectory['settle']}")

    # An inline waypoint list is validated by waypoint_config when it is read;
    # only a path is resolved here.
    file_keys = [("scene", "usd"), ("rig", "usd"), ("people", "terrain"), ("trajectory", "terrain")]
    if isinstance(trajectory["waypoints"], str):
        file_keys.append(("trajectory", "waypoints"))
    for section, key in file_keys:
        asset = config[section][key]
        if asset is not None:
            asset = os.path.abspath(os.path.join(PROJECT_DIR, os.path.expanduser(asset)))
            if not os.path.exists(asset):
                raise ValueError(f"{path}: {section}.{key} does not exist: {asset}")
            config[section][key] = asset

    skybox = config["scene"]["skybox"]
    if skybox is not None and "://" not in skybox:
        # A local file, same rule as scene.usd/rig.usd; a Nucleus or http(s)
        # URI is left untouched for Usd/Kit to resolve.
        resolved = os.path.abspath(os.path.join(PROJECT_DIR, os.path.expanduser(skybox)))
        if not os.path.exists(resolved):
            raise ValueError(f"{path}: scene.skybox does not exist: {resolved}")
        config["scene"]["skybox"] = resolved
    if config["scene"]["skybox_intensity"] < 0.0:
        raise ValueError(f"{path}: scene.skybox_intensity must not be negative, got {config['scene']['skybox_intensity']}")

    world = config["scene"]["world_prim"]
    for where, value in (("scene.world_prim", world), ("ros.graph_prim", config["ros"]["graph_prim"])):
        if not value.startswith("/") or value.rstrip("/") != value or "//" in value:
            raise ValueError(f"{path}: {where} must be an absolute prim path without a trailing slash, got {value!r}")
    for where, name in (("base_link.name", config["base_link"]["name"]), ("rig.name", config["rig"]["name"])):
        if "/" in name or not name:
            raise ValueError(f"{path}: {where} must be a single prim name, got {name!r}")

    # Derived, so that no two places can disagree about where anything lives.
    config["base_link"]["prim"] = f"{world}/{config['base_link']['name']}"
    config["rig"]["prim"] = f"{config['base_link']['prim']}/{config['rig']['name']}"
    config["people"]["prim"] = f"{world}/Characters"
    config["people"]["walkers"] = _walkers(path, config["people"]["walkers"] or [])
    return config


def _walkers(path, walkers):
    """Each walker defaulted and type-checked, its asset resolved, its commands parsed."""
    checked, names = [], set()
    for index, given in enumerate(walkers):
        where = f"people.walkers[{index}]"
        if not isinstance(given, dict):
            raise ValueError(f"{path}: {where} must be a mapping, got {given!r}")
        unknown = set(given) - set(WALKER_FIELDS)
        if unknown:
            raise ValueError(f"{path}: unknown key(s) {sorted(unknown)} in {where}; expected {list(WALKER_FIELDS)}")
        walker = {
            name: _field(path, where, name, given, default, kind) for name, (default, kind) in WALKER_FIELDS.items()
        }

        name = walker["name"]
        # The behaviour script finds its commands by the first word of a line,
        # so the name has to be one word as well as a valid prim name.
        if not name.replace("_", "a").isalnum() or name[0].isdigit() or not name.isascii():
            raise ValueError(f"{path}: {where}.name must be letters, digits and _, not starting with a digit, got {name!r}")
        if name in RESERVED_WALKER_NAMES:
            raise ValueError(f"{path}: {where}.name {name!r} is reserved")
        if name in names:
            raise ValueError(f"{path}: {where}.name {name!r} is used twice")
        names.add(name)

        asset = walker["asset"]
        if asset.endswith((".usd", ".usda", ".usdc")) and "://" not in asset:
            asset = os.path.abspath(os.path.join(PROJECT_DIR, os.path.expanduser(asset)))
            if not os.path.exists(asset):
                raise ValueError(f"{path}: {where}.asset does not exist: {asset}")
            walker["asset"] = asset

        commands = walker["commands"] or []
        walker["commands"] = [_command(path, f"{where}.commands[{i}]", line) for i, line in enumerate(commands)]
        checked.append(walker)
    return checked


def _command(path, where, line):
    """One command line, checked the way omni.anim.people will read it, normalised to single spaces."""
    if not isinstance(line, str):
        raise ValueError(f"{path}: {where} must be a string such as 'GoTo 4 0 0 _', got {line!r}")
    words = line.split()
    if not words or words[0] not in COMMANDS:
        raise ValueError(f"{path}: {where} must start with one of {list(COMMANDS)}, got {line!r}")

    verb, arguments = words[0], words[1:]
    if verb == "GoTo":
        # x y z, any number of times, then a final heading in degrees or _.
        numbers = arguments[:-1] if arguments and arguments[-1] == "_" else arguments
        shape_ok = len(arguments) >= 4 and len(arguments) % 3 == 1
        usage = "GoTo x y z [x y z ...] yaw, with yaw in degrees or _ to keep the heading"
    else:
        numbers = arguments
        shape_ok = len(arguments) == 1
        usage = f"{verb} seconds"
    if not shape_ok:
        raise ValueError(f"{path}: {where} must be '{usage}', got {line!r}")
    try:
        [float(number) for number in numbers]
    except ValueError:
        raise ValueError(f"{path}: {where} has a non-numeric argument: {line!r}") from None
    return " ".join(words)
