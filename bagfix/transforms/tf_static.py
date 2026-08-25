"""Patch or inject static transforms on /tf_static.

mode: patch   rewrite the translation/rotation of matching parent->child pairs
              inside the existing /tf_static messages
mode: append  add the configured transforms to the bag. They are appended to the
              first /tf_static message if the topic exists, otherwise a
              /tf_static message is synthesized (with transient_local QoS) at
              the start of the bag.
"""

from __future__ import annotations

import math

from ..bagio import make_topic_metadata, message_class
from . import EmitContext, Transform, register

TF_MESSAGE_TYPE = "tf2_msgs/msg/TFMessage"
DEFAULT_TOPIC = "/tf_static"


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


def _spec_pose(spec: dict) -> tuple[tuple[float, float, float], tuple[float, ...]]:
    xyz = tuple(float(v) for v in spec.get("xyz", (0.0, 0.0, 0.0)))
    if len(xyz) != 3:
        raise ValueError(f"tf_static: 'xyz' needs 3 values, got {spec.get('xyz')}")
    if "quat" in spec:
        quat = tuple(float(v) for v in spec["quat"])
        if len(quat) != 4:
            raise ValueError("tf_static: 'quat' needs 4 values (x, y, z, w)")
        norm = math.sqrt(sum(v * v for v in quat))
        if norm == 0.0:
            raise ValueError("tf_static: 'quat' must not be zero")
        quat = tuple(v / norm for v in quat)
    else:
        rpy = tuple(float(v) for v in spec.get("rpy", (0.0, 0.0, 0.0)))
        if len(rpy) != 3:
            raise ValueError("tf_static: 'rpy' needs 3 values")
        quat = quaternion_from_rpy(*rpy)
    return xyz, quat


@register("tf_static")
class TfStatic(Transform):
    """Options:

    topic:      /tf_static by default
    mode:       'patch' or 'append'
    transforms: list of {parent, child, xyz: [x,y,z], rpy: [r,p,y] | quat: [x,y,z,w]}
    """

    accepts = (TF_MESSAGE_TYPE,)

    def configure(self) -> None:
        self.topic = self.options.get("topic", DEFAULT_TOPIC)
        self.patterns = self.patterns or [self.topic]
        self.mode = self.options.get("mode", "append")
        if self.mode not in ("patch", "append"):
            raise ValueError(f"tf_static: mode must be 'patch' or 'append', got {self.mode!r}")

        specs = self.options.get("transforms") or []
        if not specs:
            raise ValueError("tf_static transform needs a non-empty 'transforms' list")
        self.specs = {}
        for spec in specs:
            parent, child = spec.get("parent"), spec.get("child")
            if not parent or not child:
                raise ValueError(f"tf_static: each entry needs 'parent' and 'child': {spec}")
            self.specs[(_strip(parent), _strip(child))] = (parent, child, *_spec_pose(spec))

        self.appended = False
        self.matched: set[tuple[str, str]] = set()

    # ---- writer-side ------------------------------------------------------

    def plan_topics(self, topics):
        if self.mode == "append" and self.topic not in topics:
            topics[self.topic] = make_topic_metadata(
                self.topic, TF_MESSAGE_TYPE, transient_local=True
            )
        return topics

    def emit(self, ctx: EmitContext):
        """Synthesize /tf_static only when the source bag had no such topic."""
        if self.mode != "append" or self.appended or self.topic in ctx.existing_topics:
            return ()
        msg = message_class(TF_MESSAGE_TYPE)()
        for parent, child, xyz, quat in self.specs.values():
            msg.transforms.append(self._build(parent, child, xyz, quat, ctx.start_time))
        self.count += len(msg.transforms)
        self.appended = True
        self.record_sample("(no /tf_static in source)", self.describe(msg))
        return ((self.topic, msg, ctx.start_time),)

    # ---- message-side -----------------------------------------------------

    def apply(self, topic, msg, stamp):
        before = self.describe(msg)

        for tf in msg.transforms:
            key = (_strip(tf.header.frame_id), _strip(tf.child_frame_id))
            spec = self.specs.get(key)
            if spec is None:
                continue
            _, _, xyz, quat = spec
            self._set_pose(tf, xyz, quat)
            self.matched.add(key)
            self.count += 1

        if self.mode == "append" and not self.appended:
            for key, (parent, child, xyz, quat) in self.specs.items():
                if key in self.matched:
                    continue  # already present, patched above
                msg.transforms.append(self._build(parent, child, xyz, quat, stamp))
                self.count += 1
            self.appended = True

        self.record_sample(before, self.describe(msg))
        return msg

    def finish(self) -> None:
        if self.mode == "patch":
            for parent, child, *_ in self.specs.values():
                if (_strip(parent), _strip(child)) not in self.matched:
                    self.note(f"no existing transform {parent} -> {child} to patch")

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
