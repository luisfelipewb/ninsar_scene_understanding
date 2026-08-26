"""Edit the static transforms on /tf_static.

Three lists, applied in this order:

    remove:  delete matching parent->child entries. Either key may be a glob,
             and either may be omitted to mean "any".
    edit:    rewrite entries that are already in the bag.
    add:     insert entries that are not.

`edit` and `add` take the same entry shape - a complete pose, as `xyz` plus
`rpy` (radians), `rpy_deg` (degrees) or `quat` - and differ only in what they
assert about the bag. That
assertion is checked in `preflight`, before any output is written, so naming a
frame that is not there (or one that already is) fails without producing a
half-repaired bag.
"""

from __future__ import annotations

import fnmatch
import math

import rclpy.serialization as ser

from ..bagio import make_topic_metadata, message_class, open_reader
from . import EmitContext, Transform, register

TF_MESSAGE_TYPE = "tf2_msgs/msg/TFMessage"
DEFAULT_TOPIC = "/tf_static"

#: keys that belonged to the old `mode:` + `transforms:` shape
RETIRED_OPTIONS = {
    "mode": "'mode' is gone: use the 'edit' list to change existing transforms "
            "and 'add' to insert new ones",
    "transforms": "'transforms' is gone: rename it to 'edit' (for pairs already "
                  "in the bag) or 'add' (for new ones)",
}


def _strip(frame: str) -> str:
    """TF frame ids are compared without a leading slash (ROS 1 legacy)."""
    return frame.lstrip("/")


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """Convert fixed-axis roll/pitch/yaw to (x, y, z, w) - same convention as tf2."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _removal_spec(spec) -> tuple[str, str]:
    """Normalise one `remove` entry into a (parent glob, child glob) pair."""
    if not isinstance(spec, dict):
        raise ValueError(
            f"each 'remove' entry must be a mapping with 'parent' and/or "
            f"'child', got {spec!r}"
        )
    parent, child = spec.get("parent"), spec.get("child")
    if not parent and not child:
        # A bare {} would match every transform in the bag; make that impossible
        # to write by accident - `remove: [{parent: '*'}]` says it deliberately.
        raise ValueError("a 'remove' entry needs 'parent', 'child', or both")
    return (_strip(str(parent or "*")), _strip(str(child or "*")))


def _matches(removal: tuple[str, str], parent: str, child: str) -> bool:
    return (fnmatch.fnmatchcase(parent, removal[0]) and
            fnmatch.fnmatchcase(child, removal[1]))


def _matches_any(removals, parent: str, child: str) -> bool:
    return any(_matches(removal, parent, child) for removal in removals)


def _spec_pose(spec: dict) -> tuple[tuple[float, float, float], tuple[float, ...]]:
    xyz = tuple(float(v) for v in spec.get("xyz", (0.0, 0.0, 0.0)))
    if len(xyz) != 3:
        raise ValueError(f"'xyz' needs 3 values, got {spec.get('xyz')}")
    if "quat" in spec:
        quat = tuple(float(v) for v in spec["quat"])
        if len(quat) != 4:
            raise ValueError("'quat' needs 4 values (x, y, z, w)")
        norm = math.sqrt(sum(v * v for v in quat))
        if norm == 0.0:
            raise ValueError("'quat' must not be zero")
        quat = tuple(v / norm for v in quat)
    elif "rpy_deg" in spec:
        if "rpy" in spec:
            raise ValueError("give either 'rpy' (radians) or 'rpy_deg', not both")
        rpy_deg = tuple(float(v) for v in spec["rpy_deg"])
        if len(rpy_deg) != 3:
            raise ValueError("'rpy_deg' needs 3 values")
        quat = quaternion_from_rpy(*(math.radians(a) for a in rpy_deg))
    else:
        rpy = tuple(float(v) for v in spec.get("rpy", (0.0, 0.0, 0.0)))
        if len(rpy) != 3:
            raise ValueError("'rpy' needs 3 values")
        quat = quaternion_from_rpy(*rpy)
    return xyz, quat


def _pose_spec(spec, list_name: str) -> tuple[tuple[str, str], tuple]:
    """Normalise one `edit` or `add` entry."""
    if not isinstance(spec, dict):
        raise ValueError(f"each '{list_name}' entry must be a mapping, got {spec!r}")
    parent, child = spec.get("parent"), spec.get("child")
    if not parent or not child:
        raise ValueError(f"each '{list_name}' entry needs 'parent' and 'child': {spec}")
    for key, value in (("parent", parent), ("child", child)):
        # A glob would give several frames the same pose, which is never what
        # anyone means. Globs belong in 'remove', where they delete a family.
        if any(character in str(value) for character in "*?["):
            raise ValueError(
                f"'{list_name}' needs exact frame names, but {key} is {value!r}; "
                "globs are only allowed in 'remove'"
            )
    return (_strip(parent), _strip(child)), (parent, child, *_spec_pose(spec))


@register("tf_static")
class TfStatic(Transform):
    """Options:

    topic:      /tf_static by default
    remove:     list of {parent, child} to delete; globs allowed, either key
                may be omitted to mean "any"
    edit:       list of {parent, child, xyz, rpy | rpy_deg | quat} that must
                already exist; `rpy` is radians, `rpy_deg` degrees
    add:        list of the same shape, that must not already exist
    drop_empty: delete a /tf_static message that removals emptied (default true)
    """

    accepts = (TF_MESSAGE_TYPE,)

    def configure(self) -> None:
        for key, message in RETIRED_OPTIONS.items():
            if key in self.options:
                raise ValueError(message)

        self.topic = self.options.get("topic", DEFAULT_TOPIC)
        self.patterns = self.patterns or [self.topic]
        self.drop_empty = bool(self.options.get("drop_empty", True))

        self.edits = dict(
            _pose_spec(spec, "edit") for spec in self.options.get("edit") or []
        )
        self.additions = dict(
            _pose_spec(spec, "add") for spec in self.options.get("add") or []
        )
        self.removals = [_removal_spec(spec) for spec in self.options.get("remove") or []]

        if not self.edits and not self.additions and not self.removals:
            raise ValueError("tf_static needs a non-empty 'remove', 'edit' or 'add' list")

        for key in self.edits:
            if key in self.additions:
                raise ValueError(
                    f"{key[0]} -> {key[1]} is in both 'edit' and 'add'; it is "
                    "either already in the bag or it is not"
                )
        # Removals run first, so anything also named in edit/add would be
        # deleted and then rewritten - a contradiction worth refusing outright.
        for key, (parent, child, *_) in {**self.edits, **self.additions}.items():
            if _matches_any(self.removals, *key):
                which = "edit" if key in self.edits else "add"
                raise ValueError(
                    f"{parent} -> {child} is in both '{which}' and 'remove'; "
                    "removals are applied first, so the entry would be deleted "
                    "and then written back"
                )

        self.written = False
        self.matched: set[tuple[str, str]] = set()
        self.used_removals: set[tuple[str, str]] = set()
        self.removed = 0

    # ---- preflight --------------------------------------------------------

    def preflight(self, source, types) -> None:
        """Refuse before anything is written if the bag disagrees with the rule.

        Reads only `self.topic`, so this costs an index lookup rather than a
        pass over the whole bag.
        """
        if not self.edits and not self.additions:
            return  # a remove-only rule asserts nothing

        present = self._existing_pairs(source, types)

        missing = [self.edits[key] for key in self.edits if key not in present]
        if missing:
            listed = ", ".join(f"{parent} -> {child}" for parent, child, *_ in missing)
            raise ValueError(
                f"'edit' names {listed}, which {self.topic} does not have. "
                "Move it to 'add' to insert it, or check the frame names with "
                "'bagfix inspect'."
            )

        duplicated = [self.additions[key] for key in self.additions if key in present]
        if duplicated:
            listed = ", ".join(f"{parent} -> {child}" for parent, child, *_ in duplicated)
            raise ValueError(
                f"'add' names {listed}, which {self.topic} already has. "
                "Move it to 'edit' to overwrite it, or to 'remove' to delete it."
            )

    def _existing_pairs(self, source, types) -> set[tuple[str, str]]:
        """Every (parent, child) currently on the topic."""
        if self.topic not in types:
            return set()
        reader = open_reader(source, topics=[self.topic])
        message_type = message_class(types[self.topic])
        pairs = set()
        while reader.has_next():
            _, data, _ = reader.read_next()
            msg = ser.deserialize_message(data, message_type)
            for tf in msg.transforms:
                pairs.add((_strip(tf.header.frame_id), _strip(tf.child_frame_id)))
        return pairs

    # ---- writer-side ------------------------------------------------------

    def plan_topics(self, topics):
        # Only a rule that adds transforms may create the topic; a remove-only
        # rule has nothing to put in it.
        if self.additions and self.topic not in topics:
            topics[self.topic] = make_topic_metadata(
                self.topic, TF_MESSAGE_TYPE, transient_local=True
            )
        return topics

    def emit(self, ctx: EmitContext):
        """Synthesize /tf_static only when the source bag had no such topic."""
        if not self.additions or self.written or self.topic in ctx.existing_topics:
            return ()
        msg = message_class(TF_MESSAGE_TYPE)()
        for parent, child, xyz, quat in self.additions.values():
            msg.transforms.append(self._build(parent, child, xyz, quat, ctx.start_time))
        self.count += len(msg.transforms)
        self.written = True
        self.record_sample(f"(no {self.topic} in source)", self.describe(msg))
        return ((self.topic, msg, ctx.start_time),)

    # ---- message-side -----------------------------------------------------

    def apply(self, topic, msg, stamp):
        before = self.describe(msg)

        if self.removals:
            kept = []
            for tf in msg.transforms:
                key = (_strip(tf.header.frame_id), _strip(tf.child_frame_id))
                hit = next((r for r in self.removals if _matches(r, *key)), None)
                if hit is None:
                    kept.append(tf)
                else:
                    self.used_removals.add(hit)
                    self.removed += 1
                    self.count += 1
            if len(kept) != len(msg.transforms):
                msg.transforms = kept

        for tf in msg.transforms:
            key = (_strip(tf.header.frame_id), _strip(tf.child_frame_id))
            spec = self.edits.get(key)
            if spec is None:
                continue
            _, _, xyz, quat = spec
            self._set_pose(tf, xyz, quat)
            self.matched.add(key)
            self.count += 1

        # Additions go into the first message only; preflight already proved
        # none of them is present, so there is nothing to overwrite.
        if self.additions and not self.written:
            for parent, child, xyz, quat in self.additions.values():
                msg.transforms.append(self._build(parent, child, xyz, quat, stamp))
                self.count += 1
            self.written = True

        self.record_sample(before, self.describe(msg))

        if self.drop_empty and not msg.transforms:
            # Everything in this message was removed; an empty /tf_static is
            # noise, so drop the message rather than publish a hollow one.
            self.note(f"{topic}: message emptied by 'remove', dropped")
            return None
        return msg

    def finish(self) -> None:
        for removal in self.removals:
            if removal not in self.used_removals:
                self.note(f"no transform matched remove {removal[0]} -> {removal[1]}")
        if self.removed:
            self.note(f"removed {self.removed} transform(s)")
        if self.additions and not self.written:
            self.note(
                f"nothing was added: the bag has no {self.topic} message to "
                "append to and none was synthesized"
            )

    # ---- helpers ----------------------------------------------------------

    def _build(self, parent, child, xyz, quat, stamp):
        tf = message_class("geometry_msgs/msg/TransformStamped")()
        tf.header.frame_id = parent
        tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        tf.child_frame_id = child
        self._set_pose(tf, xyz, quat)
        return tf

    @staticmethod
    def _set_pose(tf, xyz, quat):
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quat

    def describe(self, msg) -> str:
        return "; ".join(
            f"{tf.header.frame_id}->{tf.child_frame_id} "
            f"t=({tf.transform.translation.x:.3f}, {tf.transform.translation.y:.3f}, "
            f"{tf.transform.translation.z:.3f})"
            for tf in msg.transforms
        ) or "(empty)"
