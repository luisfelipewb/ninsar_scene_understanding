#!/usr/bin/env python3
"""Build the sensor-rig scene from a run config and fly it, publishing to ROS 2.

One config describes everything; see config/empty.yaml for the annotated
schema. From it this assembles, at runtime, on top of a props-only scene USD
(or an empty built-in scene):

    <world>/base_link                 an empty free-flying frame
    <world>/base_link/sensor_rig      the rig asset, payloaded in
    <world>/Graphs/ROS_TF             an OmniGraph publishing
        world -> base_link   on /tf         every tick, because it moves
        base_link -> rig     on /tf_static  once, because it does not

The rig asset brings its own graphs (clock, cameras, lidar, its internal
/tf_static), so only the two transforms it cannot know are added here. The
scene comes in as a sublayer of a fresh root layer, so its file is never
written to and --save-stage writes the additions only, a few kilobytes.

If the config lists people, each is a character from the Isaac People library
under <world>/Characters, walked by omni.anim.people's behaviour script along
the commands the config gives it. They move only while the timeline plays.

If the config names waypoints, base_link is moved along the trajectory once per
rendered frame while the timeline plays; the rig's publishers tick on
OnPlaybackTick, so clock, TF, images and point clouds all follow. The pose is
written explicitly per frame rather than baked as USD time samples because in
Isaac Sim 5.1 the TF publisher does not follow time-sampled xformOps. Without
waypoints the rig stays parked at base_link.translate and plays until
interrupted.

Run, with your ROS 2 environment sourced:

    source /opt/ros/jazzy/setup.bash
    cd ~/isaacsim
    S=~/workspaces/ninsar-scene-understanding/sim/isaac
    ./python.sh $S/run_scene.py $S/config/montoldre.yaml --headless

Then `ros2 run tf2_tools view_frames`, or RViz2 with Fixed Frame = the world
prim's name and "Use simulation time" on.
"""

import argparse
import os
import sys
import tempfile
import time

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_DIR)

import yaml  # noqa: E402  (only to name its errors when a config will not parse)
from rigsim import scene_config, trajectory, waypoint_config  # noqa: E402  (needs PROJECT_DIR on the path)

# Node names inside the graph this script authors. They are also what shows up
# in the OmniGraph editor, so they are worth keeping readable.
TICK, CONTEXT, SIM_TIME = "OnPlaybackTick", "Context", "ReadSimTime"
DYNAMIC_TF, STATIC_TF = "PublisherTF_base_link", "PublisherTF_rig"

GROUND_HALF_SIZE = 50.0

# Under the Isaac assets root: one folder per character, plus the hidden biped
# that carries the animation graph every character shares.
PEOPLE_LIBRARY = "/Isaac/People/Characters"
PEOPLE_BIPED = "Biped_Setup"
PEOPLE_SCRIPT = "omni/anim/people/scripts/character_behavior.py"
PEOPLE_LABEL = "person"

# omni.anim.people reads these once, when the timeline starts playing.
PEOPLE_SETTINGS = "/exts/omni.anim.people"
PEOPLE_ROOT_SETTING = "/persistent/exts/omni.anim.people/character_prim_path"

# Appearance of the drawn path in a --save-stage export, in metres.
PATH_WIDTH = 0.03
MARKER_LENGTH = 0.4
MARKER_EVERY_SECONDS = 1.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config", help="run config YAML, e.g. config/montoldre.yaml")
    parser.add_argument("--headless", action="store_true", help="no Isaac Sim window; ROS 2 output is unaffected")
    parser.add_argument(
        "--save-stage",
        metavar="OUT.usda",
        help="write the assembled stage; with a trajectory it also carries the path as time "
        "samples on base_link, scrubbable in the GUI (overwrites)",
    )
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="assemble, verify, save if asked, and exit without playing, so nothing is published",
    )
    parser.add_argument(
        "--waypoints",
        metavar="FILE",
        help="override trajectory.waypoints from the config; 'none' parks the rig",
    )
    parser.add_argument("--print-pose", action="store_true", help="log base_link, and any walkers, once per second")
    return parser.parse_args()


def log(message):
    """SimulationApp.close() hard-exits, so buffered stdout would be lost."""
    print(f"[run_scene] {message}", flush=True)


def load_config(args):
    """The run config, with the CLI override applied; exits on any problem."""
    try:
        config = scene_config.load_scene(args.config)
    except (OSError, ValueError, yaml.YAMLError) as error:
        sys.exit(f"[run_scene] {error}")

    if args.waypoints is not None:
        if args.waypoints.lower() == "none":
            config["trajectory"]["waypoints"] = None
        elif not os.path.isfile(args.waypoints):
            sys.exit(f"[run_scene] --waypoints {args.waypoints}: no such file")
        else:
            config["trajectory"]["waypoints"] = os.path.abspath(args.waypoints)
    return config


def build_trajectory(config):
    """The (N, 8) trajectory the config asks for, or None when the rig is parked.

    Done before the simulator boots: a typo in a waypoint file should fail in a
    second, not after Isaac Sim has spent a minute starting.
    """
    settings = config["trajectory"]
    source = settings["waypoints"]
    if source is None:
        return None

    try:
        if isinstance(source, str):
            waypoints, times = waypoint_config.load_waypoints(source)
            where = source
        else:
            where = f"{CONFIG_PATH}: trajectory.waypoints"
            waypoints, times = waypoint_config.parse_waypoints(source, where)
    except (OSError, ValueError, yaml.YAMLError) as error:
        sys.exit(f"[run_scene] {error}")

    fps = config["scene"]["fps"]
    if times is None:
        traj = trajectory.from_waypoints(
            waypoints, fps=fps, speed=settings["speed"], yaw_rate_deg=settings["yaw_rate"], close_loop=False
        )
        pacing = f"paced at {settings['speed']:g} m/s / {settings['yaw_rate']:g} deg/s"
    else:
        # The waypoints carry their own clock, so speed and yaw_rate say nothing.
        traj = trajectory.from_timed_waypoints(waypoints, times, fps=fps)
        pacing = "on their own clock (trajectory.speed and yaw_rate do not apply)"
    log(f"{len(waypoints)} waypoints from {where}, {pacing}")

    if settings["terrain"] is not None:
        traj = raise_onto_terrain(traj, settings["terrain"])
    log(f"trajectory: {trajectory.describe(traj)}")
    return traj


def raise_onto_terrain(traj, heightmap):
    """Lift a trajectory so each frame's z is a clearance above the ground.

    Applied to the sampled trajectory rather than to the waypoints, so the path
    hugs the relief between them instead of cutting a straight line from one
    sampled height to the next. Segment durations were already fixed from the
    horizontal path, so a slope makes the true ground speed slightly higher
    than trajectory.speed -- 0.3% over 90% of Montoldre, whose median slope is
    1.7 deg.
    """
    from rigsim import terrain_height

    terrain = terrain_height.TerrainHeight(heightmap)
    x, y = traj[:, 1], traj[:, 2]

    outside = terrain.outside(x, y)
    if outside.any():
        x0, x1, y0, y1 = terrain.bounds
        log(
            f"WARNING {outside.sum()} of {len(traj)} frames fall outside the terrain "
            f"(x {x0:.1f}..{x1:.1f}, y {y0:.1f}..{y1:.1f}) and were held at its nearest edge"
        )

    ground = terrain(x, y)
    traj[:, 3] += ground
    log(f"terrain under the path: {ground.min():.2f}..{ground.max():.2f}m, from {heightmap}")
    return traj


args = parse_args()
CONFIG_PATH = os.path.abspath(args.config)
CONFIG = load_config(args)
TRAJ = build_trajectory(CONFIG)

from isaacsim import SimulationApp  # noqa: E402  (must precede every other Isaac import)

simulation_app = SimulationApp({"headless": args.headless, "renderer": "RaytracedLighting"})

import carb.settings  # noqa: E402
import numpy as np  # noqa: E402
import omni.client  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleXFormPrim  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

PATH_COLOR = Gf.Vec3f(1.0, 0.78, 0.12)
AXIS_COLORS = (Gf.Vec3f(0.9, 0.15, 0.15), Gf.Vec3f(0.2, 0.85, 0.25), Gf.Vec3f(0.25, 0.45, 1.0))

# Gf types to author with, keyed by the precision the prim's xformOp already uses.
_PRECISION_TYPES = {
    UsdGeom.XformOp.PrecisionHalf: (Gf.Vec3h, Gf.Quath),
    UsdGeom.XformOp.PrecisionFloat: (Gf.Vec3f, Gf.Quatf),
    UsdGeom.XformOp.PrecisionDouble: (Gf.Vec3d, Gf.Quatd),
}


# --------------------------------------------------------------------------
# Scene assembly
# --------------------------------------------------------------------------


def open_scene(scene_usd, world_prim, fps, skybox=None, skybox_intensity=1000.0):
    """Compose the props-only scene, or build an empty one, and return the stage.

    The scene comes in as a sublayer of a new, empty root layer. Every edit made
    afterwards lands in that root layer, which is what keeps the scene file
    clean and lets --save-stage write the additions without the scene.

    Either way the world prim is guaranteed to exist as an Xform and to be the
    stage's default prim, so everything added afterwards has somewhere to go.

    skybox, if given, replaces the dome light's flat color with an HDRI -- on
    the empty scene's own dome light, or a new one layered over whatever
    lighting a sublayered scene brought with it.
    """
    create_new_stage()
    stage = omni.usd.get_context().get_stage()
    if scene_usd is None:
        add_empty_scene(stage)
        log("no scene given: built the empty one (dome light, sun, ground plane)")
    else:
        scene_layer = Sdf.Layer.FindOrOpen(scene_usd)
        if scene_layer is None:
            raise RuntimeError(f"could not open {scene_usd}")
        stage.GetRootLayer().subLayerPaths.append(scene_usd)
        simulation_app.update()  # let the sublayer's references stream in before we look
        log(f"sublayered scene {scene_usd}")
        if scene_layer.defaultPrim:
            log(f"note: the scene's own default prim is /{scene_layer.defaultPrim}; this stage's is {world_prim}")
        scene_up = scene_layer.pseudoRoot.GetInfo(UsdGeom.Tokens.upAxis)
        if scene_up and scene_up != UsdGeom.Tokens.z:
            log(f"warning: scene declares upAxis={scene_up}; the stage is Z-up, so its props may appear rotated")

    # The root layer holds the strongest opinion in the layer stack, so what it
    # says about the up-axis is what the running stage and --save-stage both
    # see. Author it here, for both branches, so a scene exported Y-up cannot
    # flip the stage: the rig, the trajectory and the TF frames are all Z-up.
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    if skybox:
        add_skybox(stage, skybox, skybox_intensity)

    world = stage.GetPrimAtPath(world_prim)
    if not world:
        world = UsdGeom.Xform.Define(stage, world_prim).GetPrim()
        log(f"scene had no {world_prim}; defined it alongside the scene's own prims")
    elif not world.GetTypeName():
        # A typeless `def` -- what a scene gets when someone parents a stray
        # curve under /World in the GUI. It can hold children but not
        # transforms; typing it in the root layer keeps its children and makes
        # it a frame the publisher can follow.
        world = UsdGeom.Xform.Define(stage, world_prim).GetPrim()
        log(f"scene's {world_prim} had no type; made it an Xform, keeping what it holds")
    elif not UsdGeom.Xformable(world):
        raise RuntimeError(f"{world_prim} exists but is a {world.GetTypeName()}, which cannot hold transforms")

    # Nothing but this script authors the root layer, so the default prim is
    # ours to set. The scene's own choice stays in the scene file, untouched.
    stage.SetDefaultPrim(world)

    # Frames are indexed as time * fps, so the stage has to agree with the
    # trajectory about what a time code is worth.
    stage.SetTimeCodesPerSecond(fps)
    return stage


def add_empty_scene(stage):
    """Enough of an environment to see the rig and light what its cameras see."""
    UsdGeom.Xform.Define(stage, "/Environment")

    # Flat-color sky by default; add_skybox() swaps in an HDRI on this same
    # prim afterwards when scene.skybox is configured.
    dome = UsdLux.DomeLight.Define(stage, "/Environment/DomeLight")
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))

    sun = UsdLux.DistantLight.Define(stage, "/Environment/DistantLight")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(0.53)
    set_pose(sun.GetPrim(), [0.0, 0.0, 0.0], [0.0, -45.0, 45.0])

    # A plain quad: this is a backdrop for looking at the rig, not a physics
    # ground, so it carries no collider.
    size = GROUND_HALF_SIZE
    ground = UsdGeom.Mesh.Define(stage, "/Environment/ground")
    corners = [(-size, -size, 0.0), (size, -size, 0.0), (size, size, 0.0), (-size, size, 0.0)]
    ground.CreatePointsAttr([Gf.Vec3f(*corner) for corner in corners])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateExtentAttr([Gf.Vec3f(-size, -size, 0.0), Gf.Vec3f(size, size, 0.0)])
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.30, 0.32)])


def add_skybox(stage, texture, intensity):
    """Point a dome light at an HDRI: a local path or a Nucleus/http(s) URI.

    Defines /Environment/DomeLight, which is a no-op reuse of the empty
    scene's own dome light there, or a new one layered over a sublayered
    scene's own lighting -- the root layer always wins composition, so this
    is the sky every consumer of the saved stage sees regardless of what the
    scene underneath it brought.
    """
    UsdGeom.Xform.Define(stage, "/Environment")
    dome = UsdLux.DomeLight.Define(stage, "/Environment/DomeLight")
    dome.CreateTextureFileAttr(texture)
    dome.CreateIntensityAttr(intensity)
    log(f"skybox {texture} (intensity {intensity:g})")


def set_pose(prim, translate, rotate_deg):
    """Author translate/orient/scale ops, replacing whatever was there.

    Always these three ops in this order and at double precision, because that
    is what SingleXFormPrim.set_world_pose() and bake_time_samples() both
    expect to find when they come to move the frame.

    `rotate_deg` is roll/pitch/yaw applied as scipy's extrinsic "xyz" -- the
    same convention the waypoint configs use, so an angle means one thing
    across the whole project.
    """
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()

    quat_xyzw = Rotation.from_euler("xyz", rotate_deg, degrees=True).as_quat()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translate))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(quat_xyzw[3], Gf.Vec3d(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]))
    )
    xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))
    return prim


def add_base_link(stage, config):
    """The free-flying frame: an empty Xform, deliberately with no geometry."""
    prim = UsdGeom.Xform.Define(stage, config["prim"]).GetPrim()
    set_pose(prim, config["translate"], config["rotate"])
    log(f"base_link {config['prim']} at t={config['translate']} rpy={config['rotate']}")
    return prim


def add_rig(stage, config):
    """Compose the rig asset under base_link at its mounting offset.

    A payload rather than a reference by default: the arc is identical for
    composition, but a payload can be unloaded, which keeps a heavy sensor rig
    from being pulled in by tools that only care about the scene.
    """
    prim = UsdGeom.Xform.Define(stage, config["prim"]).GetPrim()
    if config["load_as"] == "payload":
        prim.GetPayloads().AddPayload(config["usd"])
        stage.Load(prim.GetPath())
    else:
        prim.GetReferences().AddReference(config["usd"])
    set_pose(prim, config["translate"], config["rotate"])

    simulation_app.update()  # let the composed layer stream in before we look
    children = prim.GetChildren()
    if not children:
        raise RuntimeError(
            f"{config['usd']} composed no prims under {config['prim']}; "
            f"does it have a default prim?"
        )
    log(f"rig {config['prim']} <- {config['usd']} (root: {children[0].GetName()})")
    return prim


def add_tf_graph(stage, config, world_prim, base_prim, rig_prim):
    """An OmniGraph publishing the two transforms the rig cannot publish itself.

    Both publishers hang off the same OnPlaybackTick, context and simulation
    clock, so their stamps agree with the rig's own publishers to the tick.
    """
    graph_path = config["graph_prim"]
    keys = og.Controller.Keys

    values = [
        (f"{SIM_TIME}.inputs:resetOnStop", False),
        (f"{DYNAMIC_TF}.inputs:topicName", config["dynamic_topic"]),
        (f"{STATIC_TF}.inputs:topicName", config["static_topic"]),
        # Latched, so a subscriber that starts late still learns the offset.
        (f"{STATIC_TF}.inputs:staticPublisher", True),
    ]
    if config["domain_id"] is not None:
        values += [(f"{CONTEXT}.inputs:domain_id", config["domain_id"]),
                   (f"{CONTEXT}.inputs:useDomainIDEnvVar", False)]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                (TICK, "omni.graph.action.OnPlaybackTick"),
                (CONTEXT, "isaacsim.ros2.bridge.ROS2Context"),
                (SIM_TIME, "isaacsim.core.nodes.IsaacReadSimulationTime"),
                (DYNAMIC_TF, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                (STATIC_TF, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                (f"{TICK}.outputs:tick", f"{DYNAMIC_TF}.inputs:execIn"),
                (f"{TICK}.outputs:tick", f"{STATIC_TF}.inputs:execIn"),
                (f"{CONTEXT}.outputs:context", f"{DYNAMIC_TF}.inputs:context"),
                (f"{CONTEXT}.outputs:context", f"{STATIC_TF}.inputs:context"),
                (f"{SIM_TIME}.outputs:simulationTime", f"{DYNAMIC_TF}.inputs:timeStamp"),
                (f"{SIM_TIME}.outputs:simulationTime", f"{STATIC_TF}.inputs:timeStamp"),
            ],
            keys.SET_VALUES: values,
        },
    )

    # parentPrim/targetPrims are USD relationships, not attributes, so the
    # Controller's SET_VALUES cannot reach them.
    set_targets(stage, f"{graph_path}/{DYNAMIC_TF}", "inputs:parentPrim", [world_prim.GetPath()])
    set_targets(stage, f"{graph_path}/{DYNAMIC_TF}", "inputs:targetPrims", [base_prim.GetPath()])
    set_targets(stage, f"{graph_path}/{STATIC_TF}", "inputs:parentPrim", [base_prim.GetPath()])
    set_targets(stage, f"{graph_path}/{STATIC_TF}", "inputs:targetPrims", [rig_prim.GetPath()])

    log(f"graph {graph_path}")
    log(f"  {config['dynamic_topic']:11s} {world_prim.GetName()} -> {base_prim.GetName()}  (every tick)")
    log(f"  {config['static_topic']:11s} {base_prim.GetName()} -> {rig_prim.GetName()}  (latched)")
    return stage.GetPrimAtPath(graph_path)


def set_targets(stage, node_path, relationship, targets):
    prim = stage.GetPrimAtPath(node_path)
    if not prim:
        raise RuntimeError(f"graph node {node_path} was not created")
    rel = prim.GetRelationship(relationship) or prim.CreateRelationship(relationship)
    rel.SetTargets([Sdf.Path(str(target)) for target in targets])


def add_people(stage, config, terrain):
    """Walkers under <world>/Characters, wired to omni.anim.people.

    Every character gets the animation graph of the hidden biped beside them
    and omni.anim.people's behaviour script, which walks it through the
    commands addressed to its prim name once the timeline plays. Returns
    {name: SkelRoot path}, the prim the script and the graph drive.

    With a terrain, a walker's translate z is a clearance above the ground,
    as for the trajectory; play() keeps it there as they walk.

    Needs omni.anim.people enabled before the stage was opened.
    """
    from isaacsim.core.utils.semantics import add_labels
    from isaacsim.storage.native import get_assets_root_path

    library = None
    root = config["prim"]
    UsdGeom.Xform.Define(stage, root)

    def from_library(path):
        nonlocal library
        library = library or get_assets_root_path() + PEOPLE_LIBRARY
        return f"{library}/{path}"

    biped = stage.DefinePrim(f"{root}/{PEOPLE_BIPED}", "Xform")
    biped.GetReferences().AddReference(from_library(f"{PEOPLE_BIPED}.usd"))
    UsdGeom.Imageable(biped).MakeInvisible()

    walkers = {}
    for walker in config["walkers"]:
        asset = walker["asset"]
        if not asset.endswith((".usd", ".usda", ".usdc")):
            asset = character_usd(from_library(asset))
        prim = UsdGeom.Xform.Define(stage, f"{root}/{walker['name']}").GetPrim()
        # The character goes one level down rather than on this prim: some
        # library roots author float xformOps that set_pose cannot replace.
        # Every root in the library is an identity transform, so nothing moves.
        character = UsdGeom.Xform.Define(stage, f"{root}/{walker['name']}/character").GetPrim()
        character.GetReferences().AddReference(asset)
        x, y, z = walker["translate"]
        if terrain is not None:
            z += float(terrain(x, y))
        set_pose(prim, [x, y, z], [0.0, 0.0, walker["yaw"]])
        # On the walker's own prim, so the label covers body, clothes and hair.
        add_labels(prim, [PEOPLE_LABEL], instance_name="class")
        walkers[walker["name"]] = (prim, asset)

    simulation_app.update()  # let the references stream in before we look
    graph = next((p for p in Usd.PrimRange(biped) if p.GetTypeName() == "AnimationGraph"), None)
    if graph is None:
        raise RuntimeError(f"{biped.GetPath()} composed no AnimationGraph; did {PEOPLE_BIPED}.usd load?")

    script = (
        omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module("omni.anim.people")
        + "/"
        + PEOPLE_SCRIPT
    )
    skeletons = {}
    for name, (prim, asset) in walkers.items():
        skeleton = next((p for p in Usd.PrimRange(prim) if p.GetTypeName() == "SkelRoot"), None)
        if skeleton is None:
            raise RuntimeError(f"{asset} composed no SkelRoot under {prim.GetPath()}; is it a rigged character?")
        skeletons[name] = skeleton.GetPath()

    paths = list(skeletons.values())
    omni.kit.commands.execute("ApplyAnimationGraphAPICommand", paths=paths, animation_graph_path=graph.GetPath())
    omni.kit.commands.execute("ApplyScriptingAPICommand", paths=paths)
    for path in paths:
        stage.GetPrimAtPath(path).GetAttribute("omni:scripting:scripts").Set([script])

    for walker in config["walkers"]:
        name = walker["name"]
        log(f"walker {root}/{name} <- {walkers[name][1]}, {len(walker['commands'])} command(s)")
    return {name: str(path) for name, path in skeletons.items()}


def character_usd(folder):
    """The character in a People library folder: the one .usd it holds, as IRA picks it."""
    result, entries = omni.client.list(folder)
    if result != omni.client.Result.OK:
        result, entries = omni.client.list(os.path.dirname(folder) + "/")
        found = sorted(e.relative_path for e in entries if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
        raise RuntimeError(f"no character folder {folder}; the library has {found}")
    usds = sorted(e.relative_path for e in entries if e.relative_path.endswith(".usd"))
    if not usds:
        raise RuntimeError(f"{folder} holds no .usd")
    return f"{folder}/{usds[0]}"


def verify(stage, config, people):
    """Check the assembled stage says what we think it says, before it runs.

    Cheap, and it turns a silently empty /tf, or a walker standing still, into
    an error at build time.
    """
    problems = []
    world_path = config["scene"]["world_prim"]
    base_path = config["base_link"]["prim"]
    rig_path = config["rig"]["prim"]
    graph_path = config["ros"]["graph_prim"]

    if not stage.GetDefaultPrim():
        problems.append("stage has no default prim")

    for path in (world_path, base_path, rig_path, graph_path):
        if not stage.GetPrimAtPath(path):
            problems.append(f"{path} is missing")

    base = stage.GetPrimAtPath(base_path)
    if base:
        ops = [op.GetOpName() for op in UsdGeom.Xformable(base).GetOrderedXformOps()]
        if "xformOp:translate" not in ops or "xformOp:orient" not in ops:
            problems.append(f"{base_path} has ops {ops}; the trajectory driver needs translate and orient")

    rig = stage.GetPrimAtPath(rig_path)
    if rig and not rig.GetChildren():
        problems.append(f"{rig_path} composed nothing; the rig asset did not load")

    expected = {
        f"{graph_path}/{DYNAMIC_TF}": (world_path, base_path, config["ros"]["dynamic_topic"], False),
        f"{graph_path}/{STATIC_TF}": (base_path, rig_path, config["ros"]["static_topic"], True),
    }
    for node_path, (parent, target, topic, static) in expected.items():
        node = stage.GetPrimAtPath(node_path)
        if not node:
            problems.append(f"{node_path} is missing")
            continue
        problems += check_publisher(node, node_path, parent, target, topic, static)

    for name, path in people.items():
        skeleton = stage.GetPrimAtPath(path)
        if "AnimationGraphAPI" not in skeleton.GetAppliedSchemas():
            problems.append(f"walker {name}: {path} has no animation graph")
        # An asset[] attribute: Sdf.AssetPath elements, whose str() is @-quoted.
        scripts = [script.path for script in skeleton.GetAttribute("omni:scripting:scripts").Get() or []]
        if not any(script.endswith(PEOPLE_SCRIPT) for script in scripts):
            problems.append(f"walker {name}: {path} has scripts {scripts}, not the omni.anim.people behaviour")

    if problems:
        raise RuntimeError("assembled stage is not what was asked for:\n  - " + "\n  - ".join(problems))
    log("verified: frames, rig payload and both publishers are wired as configured"
        + (f"; {len(people)} walker(s) animated and scripted" if people else ""))


def check_publisher(node, node_path, parent, target, topic, static):
    """The four things about a TF publisher that decide what reaches ROS."""
    problems = []
    for relationship, expected in (("inputs:parentPrim", parent), ("inputs:targetPrims", target)):
        rel = node.GetRelationship(relationship)
        found = [t.pathString for t in rel.GetTargets()] if rel else []
        if found != [expected]:
            problems.append(f"{node_path}.{relationship} is {found}, expected ['{expected}']")

    found_topic = node.GetAttribute("inputs:topicName").Get()
    if found_topic != topic:
        problems.append(f"{node_path}.inputs:topicName is {found_topic!r}, expected {topic!r}")

    attr = node.GetAttribute("inputs:staticPublisher")
    if bool(attr.Get()) is not static:
        problems.append(f"{node_path}.inputs:staticPublisher is {attr.Get()}, expected {static}")
    return problems


# --------------------------------------------------------------------------
# Saving
# --------------------------------------------------------------------------


def save_stage(stage, out_path, config, traj):
    """Write the root layer only: the scene stays a sublayer, the rig an arc.

    The sublayer pointer is rewritten relative to the written file when the
    scene sits beneath it, so the pair can move together; a scene elsewhere is
    pointed at by absolute path.

    With a trajectory, the written file also gets one time sample per frame on
    base_link and the path drawn under <world>/trajectory, so it can be
    scrubbed in the GUI. That is authored into the exported layer, never into
    the running stage, so the samples cannot compete with the poses written
    each frame during playback.
    """
    out_path = os.path.abspath(out_path)
    stage.GetRootLayer().Export(out_path)

    written = Sdf.Layer.FindOrOpen(out_path)

    # Kit leaves fingerprints on the root layer while the stage is open: a
    # /Render scope for its viewport, and `over`s on the scene's own OmniGraph
    # nodes where it re-resolved attribute types on load. Neither is part of
    # what was asked for, and the overs are edits to the scene's prims by
    # another name, so only what this script defined stays.
    for spec in list(written.rootPrims):
        if spec.specifier == Sdf.SpecifierOver or spec.name == "Render":
            del written.rootPrims[spec.name]

    rebased = []
    for sublayer in written.subLayerPaths:
        relative = os.path.relpath(sublayer, os.path.dirname(out_path))
        rebased.append(sublayer if relative.startswith("..") else relative)
    written.subLayerPaths = rebased

    if traj is not None:
        fps = config["scene"]["fps"]
        world_prim = config["scene"]["world_prim"]
        base_prim = config["base_link"]["prim"]
        # A sublayer's samples are scaled by the ratio of its timeCodesPerSecond
        # to the root's, so the root has to be on the trajectory's clock. One
        # sample per frame at time code == frame index then lands at t = frame / fps.
        written.timeCodesPerSecond = fps
        written.startTimeCode = 0
        written.endTimeCode = len(traj) - 1

        saved = Usd.Stage.Open(written)
        bake_time_samples(saved, base_prim, traj)
        path_scope = UsdGeom.Scope.Define(saved, f"{world_prim}/trajectory")
        # Hidden by default: the path is ordinary geometry and would otherwise
        # show up in camera and lidar output. Toggle it visible in the GUI.
        UsdGeom.Imageable(path_scope).MakeInvisible()
        draw_path(saved, path_scope.GetPath(), traj)
        draw_orientation_markers(saved, path_scope.GetPath(), traj, stride=max(int(round(fps * MARKER_EVERY_SECONDS)), 1))

    written.Save()
    if traj is not None:
        verify_animated_stage(out_path, config["base_link"]["prim"], f"{config['scene']['world_prim']}/trajectory", traj)
    return out_path


def bake_time_samples(stage, prim_path, traj):
    """One sample per trajectory row on the prim's existing translate/orient ops."""
    prim = stage.GetPrimAtPath(prim_path)
    ops = {op.GetOpType(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    translate_op = ops.get(UsdGeom.XformOp.TypeTranslate)
    orient_op = ops.get(UsdGeom.XformOp.TypeOrient)
    if translate_op is None or orient_op is None:
        raise RuntimeError(f"{prim_path} needs a translate and an orient xformOp to animate")

    vec_type = _PRECISION_TYPES[translate_op.GetPrecision()][0]
    quat_type = _PRECISION_TYPES[orient_op.GetPrecision()][1]
    for frame, row in enumerate(traj):
        time_code = Usd.TimeCode(frame)
        translate_op.Set(vec_type(row[1], row[2], row[3]), time_code)
        orient_op.Set(quat_type(row[4], row[5], row[6], row[7]), time_code)


def draw_path(stage, scope_path, traj):
    """A single polyline through every sampled position, visible at any time code."""
    curves = UsdGeom.BasisCurves.Define(stage, scope_path.AppendChild("path"))
    points = [Gf.Vec3f(*row[1:4]) for row in traj]
    curves.CreateTypeAttr(UsdGeom.Tokens.linear)
    curves.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curves.CreatePointsAttr(points)
    curves.CreateCurveVertexCountsAttr([len(points)])
    curves.CreateWidthsAttr([PATH_WIDTH])
    curves.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curves.CreateDisplayColorAttr([PATH_COLOR])
    set_extent(curves, points)


def draw_orientation_markers(stage, scope_path, traj, stride):
    """Body axes every `stride` frames, so you can see where the rig is pointing."""
    curves = UsdGeom.BasisCurves.Define(stage, scope_path.AppendChild("frames"))
    points, counts, colors = [], [], []
    for row in traj[::stride]:
        origin = Gf.Vec3f(*row[1:4])
        rotation = Gf.Rotation(Gf.Quatd(row[4], row[5], row[6], row[7]))
        for axis, color in zip((Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)), AXIS_COLORS):
            tip = rotation.TransformDir(axis) * MARKER_LENGTH
            points += [origin, origin + Gf.Vec3f(*tip)]
            counts.append(2)
            colors.append(color)

    curves.CreateTypeAttr(UsdGeom.Tokens.linear)
    curves.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curves.CreatePointsAttr(points)
    curves.CreateCurveVertexCountsAttr(counts)
    curves.CreateWidthsAttr([PATH_WIDTH * 0.6])
    curves.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    # One colour per segment, so each axis keeps its own colour.
    UsdGeom.PrimvarsAPI(curves).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.uniform
    ).Set(colors)
    set_extent(curves, points)


def set_extent(boundable, points):
    """Author a bounding box, so the GUI can frame the curves properly."""
    array = np.asarray([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
    # Gf.Vec3f rejects numpy scalars, so unpack to plain floats.
    low = [float(v) for v in array.min(axis=0)]
    high = [float(v) for v in array.max(axis=0)]
    boundable.CreateExtentAttr([Gf.Vec3f(*low), Gf.Vec3f(*high)])


def verify_animated_stage(path, prim_path, scope_path, traj, tolerance=1e-6):
    """Re-open the exported stage and check samples resolve at their own time codes."""
    stage = Usd.Stage.Open(path)
    xformable = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    for frame in (0, len(traj) // 3, len(traj) // 2, len(traj) - 1):
        resolved = xformable.GetLocalTransformation(Usd.TimeCode(frame)).ExtractTranslation()
        expected = traj[frame, 1:4]
        if max(abs(resolved[i] - expected[i]) for i in range(3)) > tolerance:
            raise RuntimeError(
                f"{path}: time sample {frame} resolved to {tuple(resolved)} instead of "
                f"{tuple(expected)}; its time codes are being rescaled"
            )
    scope = UsdGeom.Imageable(stage.GetPrimAtPath(scope_path))
    if scope.ComputeVisibility() != UsdGeom.Tokens.invisible:
        raise RuntimeError(f"{path}: {scope_path} should be invisible so the drawn path stays out of the sensors")


# --------------------------------------------------------------------------
# Playing
# --------------------------------------------------------------------------


def play(config, traj, people, terrain, print_pose):
    """Run the timeline so OnPlaybackTick fires and the publishers publish.

    With a trajectory, base_link is moved to the frame's pose before every
    update; without one it stays parked and the stage plays until the window
    is closed or the process interrupted. Walkers start on their commands with
    the timeline and are not waited for.
    """
    settings = config["trajectory"]
    fps = config["scene"]["fps"]
    flying = traj is not None
    realtime = flying and settings["realtime"]
    loop = flying and settings["loop"]

    carb_settings = carb.settings.get_settings()
    # Advance by a fixed 1/fps per frame rather than by measured wall clock, so
    # sim time, TF stamps and trajectory index stay in lockstep.
    carb_settings.set("/app/player/useFixedTimeStepping", True)
    carb_settings.set("/app/player/CompensatePlayDelayInSecs", 0.0)
    # The sensor publishers count OnPlaybackTicks, not seconds: a camera with
    # frameSkipCount=5 fires every 6th tick. A dropped frame is therefore a
    # dropped publish, which is why frame skipping is off unless asked for.
    carb_settings.set("/app/asyncRendering", False)
    carb_settings.set("/app/asyncRenderingLowLatency", False)
    carb_settings.set("/app/runLoops/main/rateLimitEnabled", realtime)
    carb_settings.set("/app/runLoops/main/rateLimitFrequency", int(fps))

    duration = float(traj[-1, 0]) if flying else 1e6
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_start_time(0.0)
    timeline.set_end_time(duration)
    timeline.set_current_time(0.0)
    timeline.set_looping(loop)
    # True means every update advances time by exactly 1/fps and nothing is
    # skipped, so tick-counted publishers keep their intended rate in sim time.
    timeline.set_play_every_frame(not realtime)
    if people:
        command_file, previous_root = start_people(config["people"], terrain, carb_settings)
        on_ground = keep_on_ground(config["people"], people, terrain)
    timeline.play()
    timeline.commit()

    stage = omni.usd.get_context().get_stage()
    if flying:
        base = SingleXFormPrim(config["base_link"]["prim"])
        log(f"playing {duration:.1f}s at {fps:g} fps" + (" (looping)" if loop else ""))
    else:
        log(f"rig parked; playing at {fps:g} fps until interrupted")
    log(f"RViz2: Fixed Frame = {config['scene']['world_prim'].strip('/').split('/')[-1]}, and turn on 'Use simulation time'")

    last_logged = -1
    while simulation_app.is_running():
        now = timeline.get_current_time()

        if flying:
            frame = min(int(round(now * fps)), len(traj) - 1)
            # Write the pose before stepping, so the OnPlaybackTick that
            # publishes sees it. Stamp and pose may still differ by one frame.
            base.set_world_pose(position=traj[frame, 1:4], orientation=traj[frame, 4:8])
        if people:
            on_ground()

        simulation_app.update()

        if flying and settings["settle"] > 0.0:
            # Rendering and the ROS publishers finish inside update(), but DDS
            # still has to push several MB per camera frame. Pausing here keeps
            # the next frame from queueing on top of a send that is still going.
            time.sleep(settings["settle"])

        if print_pose and int(now) != last_logged:
            last_logged = int(now)
            if flying:
                position, _ = base.get_world_pose()
                log(f"t={now:6.2f}s  frame={frame:5d}  base_link={np.round(position, 3)}")
            for name, path in people.items():
                position = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                log(f"t={now:6.2f}s  {name}={np.round(list(position), 3)}")

        if flying and not loop and not timeline.is_playing():
            break

    timeline.stop()
    if people:
        # A persistent setting outlives the process, so put back what the GUI had.
        carb_settings.set(PEOPLE_ROOT_SETTING, previous_root)
        os.remove(command_file)


def start_people(config, terrain, carb_settings):
    """Hand omni.anim.people its commands; returns the command file and the old root setting.

    The behaviour script reads all of this when the timeline starts, and
    takes a walker's name from the prim directly under the root setting.
    """
    lines = [
        f"{walker['name']} {on_walker_height(command, walker['translate'][2], terrain)}"
        for walker in config["walkers"]
        for command in walker["commands"]
    ]
    with tempfile.NamedTemporaryFile("w", prefix="run_scene_people_", suffix=".txt", delete=False) as handle:
        handle.write("\n".join(lines) + "\n")

    previous_root = carb_settings.get(PEOPLE_ROOT_SETTING)
    carb_settings.set(PEOPLE_ROOT_SETTING, config["prim"])
    carb_settings.set(f"{PEOPLE_SETTINGS}/command_settings/command_file_path", handle.name)
    carb_settings.set(f"{PEOPLE_SETTINGS}/command_settings/number_of_loop", "inf" if config["loop"] else "0")
    # GoTo walks straight from point to point. A navmesh would route around
    # obstacles, but it has to be baked, and Montoldre's ground has no collider
    # to bake one from; walkers still steer around each other without it.
    carb_settings.set(f"{PEOPLE_SETTINGS}/navigation_settings/navmesh_enabled", False)
    log(f"{len(config['walkers'])} walker(s), {len(lines)} command(s) in {handle.name}"
        + (", looping" if config["loop"] else ""))
    return handle.name, previous_root


def on_walker_height(command, clearance, terrain):
    """A command with every GoTo point moved to the height the walker walks at.

    The animation graph steers toward the path in 3D, so points left at the
    config's z, metres below a walker standing on Montoldre, have it creep.
    """
    words = command.split()
    if words[0] != "GoTo":
        return command
    for i in range(1, len(words) - 1, 3):
        x, y = float(words[i]), float(words[i + 1])
        ground = float(terrain(x, y)) if terrain is not None else 0.0
        words[i + 2] = f"{ground + clearance:.3f}"
    return " ".join(words)


def keep_on_ground(config, people, terrain):
    """A per-frame callable holding each walker at its clearance above the terrain.

    The animation graph moves a character in x and y only and keeps the z it
    started with, whatever height GoTo asks for. Without a terrain that is
    flat ground, so there is nothing to do.
    """
    if terrain is None:
        return lambda: None

    import omni.anim.graph.core as ag

    clearance = {walker["name"]: walker["translate"][2] for walker in config["walkers"]}

    def on_ground():
        for name, path in people.items():
            # None until the behaviour script has picked the character up.
            character = ag.get_character(path)
            if character is None:
                continue
            position, orientation = carb.Float3(0.0, 0.0, 0.0), carb.Float4(0.0, 0.0, 0.0, 1.0)
            character.get_world_transform(position, orientation)
            ground = float(terrain(position.x, position.y)) + clearance[name]
            character.set_world_transform(carb.Float3(position.x, position.y, ground), orientation)

    return on_ground


def main():
    enable_extension("isaacsim.ros2.bridge")
    if CONFIG["people"]["walkers"]:
        # Before the stage exists: the animation graph runtime picks up the
        # stages opened after it starts, and silently ignores one already open.
        enable_extension("omni.anim.people")
    simulation_app.update()

    stage = open_scene(
        CONFIG["scene"]["usd"],
        CONFIG["scene"]["world_prim"],
        CONFIG["scene"]["fps"],
        skybox=CONFIG["scene"]["skybox"],
        skybox_intensity=CONFIG["scene"]["skybox_intensity"],
    )
    world = stage.GetPrimAtPath(CONFIG["scene"]["world_prim"])
    base = add_base_link(stage, CONFIG["base_link"])
    rig = add_rig(stage, CONFIG["rig"])
    add_tf_graph(stage, CONFIG["ros"], world, base, rig)
    terrain, people = None, {}
    if CONFIG["people"]["walkers"]:
        if CONFIG["people"]["terrain"] is not None:
            from rigsim import terrain_height

            terrain = terrain_height.TerrainHeight(CONFIG["people"]["terrain"])
        people = add_people(stage, CONFIG["people"], terrain)
    simulation_app.update()

    verify(stage, CONFIG, people)

    if args.save_stage:
        log(f"wrote stage -> {save_stage(stage, args.save_stage, CONFIG, TRAJ)}")
    if args.build_only:
        return

    play(CONFIG, TRAJ, people, terrain, args.print_pose)
    log("done")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        # SimulationApp.close() hard-exits, which would swallow the traceback.
        import traceback

        log("FAILED\n" + traceback.format_exc())
    finally:
        simulation_app.close()
