#!/usr/bin/env python3
"""
Relay node: converts Isaac Sim semantic segmentation label images
(sensor_msgs/Image, encoding "32SC1") into an encoding Foxglove can render.

Colors follow the class names published on the camera's `semantic_labels`
topic (std_msgs/String, JSON), e.g.

    {"0":{"class":"BACKGROUND"},"1":{"class":"UNLABELLED"},
     "2":{"class":"cube"},"3":{"class":"cone"},
     "time_stamp":{"sec":483,"nanosec":183358533}}

The numeric IDs in the image are per-frame indices that the Replicator writer
reassigns as objects enter and leave the view, so a color is derived from the
*class name*, not from the ID. That keeps an object's color stable over time
and identical across cameras. `time_stamp` matches the image header stamp, so
each frame is colored with the mapping published for that same frame.

Two output modes:
  mode:=rgb8    -> colorized mask using the class-name palette (recommended)
  mode:=16UC1   -> raw label IDs as 16-bit; use Foxglove's
                   Color mode -> Color map (Turbo) with Value max = num classes

Usage (camera namespace is a parameter; one node per camera):
  ros2 run ninsar_perception seg_label_colorizer --ros-args \
      -p camera_namespace:=/sensor_rig/front_camera

  ros2 run ninsar_perception seg_label_colorizer --ros-args \
      -p camera_namespace:=/sensor_rig/bottom_camera

Set camera_namespace to the empty string to use relative topic names instead,
so the node can be placed with a standard namespace remap. Note the YAML
quoting -- a bare `camera_namespace:=""` fails to parse:
  ... --ros-args -p camera_namespace:="''" -r __ns:=/sensor_rig/front_camera

Depends only on rclpy + numpy (no cv_bridge / OpenCV).
"""

import colorsys
import hashlib
import json
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

PALETTE_SIZE = 256
BLACK = (0, 0, 0)
UNKNOWN_COLOR = (255, 0, 255)  # magenta: an ID the labels message did not describe
STAMP_CACHE_SIZE = 30
# Direct index lookup up to this ID; above it, fall back to searchsorted so
# instance-style IDs cannot allocate a huge table.
MAX_DIRECT_LUT_ID = 65535


def make_palette(n: int = PALETTE_SIZE) -> np.ndarray:
    """Deterministic, well-separated colors, indexed by hashed class name."""
    lut = np.zeros((n, 3), dtype=np.uint8)
    for i in range(n):
        hue = (i * 0.6180339887498949) % 1.0  # golden-ratio hue stepping
        r, g, b = colorsys.hsv_to_rgb(hue, 0.75, 0.95)
        lut[i] = (int(r * 255), int(g * 255), int(b * 255))
    return lut


class _LabelLut:
    """Maps the label IDs of one frame to RGB."""

    def __init__(self, ids: np.ndarray, colors: np.ndarray) -> None:
        self.ids = ids  # sorted int64
        self.colors = colors  # (K, 3) uint8, aligned with ids
        self.direct = None
        if ids.size and ids[0] >= 0 and ids[-1] <= MAX_DIRECT_LUT_ID:
            # Every row starts as UNKNOWN_COLOR, so IDs the labels message did
            # not describe -- inside the range or out of it -- render magenta.
            table = np.full((int(ids[-1]) + 2, 3), UNKNOWN_COLOR, dtype=np.uint8)
            table[ids] = colors
            self.direct = table

    def apply(self, labels: np.ndarray) -> np.ndarray:
        if self.direct is not None:
            sentinel = self.direct.shape[0] - 1
            idx = np.where((labels >= 0) & (labels < sentinel), labels, sentinel)
            return self.direct[idx]

        if self.ids.size == 0:
            return np.broadcast_to(
                np.array(UNKNOWN_COLOR, dtype=np.uint8), labels.shape + (3,)
            ).copy()
        pos = np.clip(np.searchsorted(self.ids, labels), 0, self.ids.size - 1)
        rgb = self.colors[pos]
        rgb[self.ids[pos] != labels] = UNKNOWN_COLOR
        return rgb


class SegLabelColorizer(Node):
    def __init__(self) -> None:
        super().__init__("seg_label_colorizer")

        self.declare_parameter("camera_namespace", "/sensor_rig/front_camera")
        # Empty string on any of these means "derive from camera_namespace".
        self.declare_parameter("input_topic", "")
        self.declare_parameter("labels_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("legend_topic", "")
        self.declare_parameter("mode", "rgb8")  # "rgb8" or "16UC1"
        # Classes rendered black rather than given a palette color.
        self.declare_parameter("background_classes", ["BACKGROUND", "UNLABELLED"])

        ns = self.get_parameter("camera_namespace").value.rstrip("/")
        self.in_topic = self._topic("input_topic", ns, "semantic_segmentation")
        self.labels_topic = self._topic("labels_topic", ns, "semantic_labels")
        self.out_topic = self._topic("output_topic", ns, "semantic_segmentation_colored")
        self.legend_topic = self._topic("legend_topic", ns, "semantic_segmentation_legend")
        self.mode = self.get_parameter("mode").value
        self.background_classes = set(self.get_parameter("background_classes").value)

        self.palette = make_palette()
        self.class_color: dict[str, tuple[int, int, int]] = {}
        self.used_slots: set[int] = set()
        self.lut_by_stamp: OrderedDict[tuple[int, int], _LabelLut] = OrderedDict()
        self.latest_lut: _LabelLut | None = None
        self.warned_encoding = False
        self.warned_no_labels = False

        # BEST_EFFORT subscribers are compatible with either a BEST_EFFORT or a
        # RELIABLE publisher, so this works regardless of the QoS Isaac Sim uses.
        sub_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # Labels are small and drive the colors: keep a deeper queue so the
        # mapping for a frame is not dropped while an image is being handled.
        labels_qos = QoSProfile(
            depth=STAMP_CACHE_SIZE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        pub_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # Latched, so a viewer that connects late still gets the legend.
        legend_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub = self.create_publisher(Image, self.out_topic, pub_qos)
        self.legend_pub = self.create_publisher(String, self.legend_topic, legend_qos)
        self.create_subscription(String, self.labels_topic, self.on_labels, labels_qos)
        self.create_subscription(Image, self.in_topic, self.on_image, sub_qos)

        self.get_logger().info(
            f"{self.in_topic} + {self.labels_topic} -> {self.out_topic} (mode={self.mode})"
        )

    def _topic(self, param: str, ns: str, default_name: str) -> str:
        """Explicit parameter wins; otherwise <camera_namespace>/<default_name>."""
        explicit = self.get_parameter(param).value
        if explicit:
            return explicit
        return f"{ns}/{default_name}" if ns else default_name

    # ------------------------------------------------------------------ colors

    def color_for(self, class_name: str) -> tuple[int, int, int]:
        """Stable color per class name: same class, same color across runs."""
        if class_name in self.class_color:
            return self.class_color[class_name]

        if class_name in self.background_classes:
            color = BLACK
        else:
            digest = hashlib.md5(class_name.encode("utf-8")).digest()
            slot = int.from_bytes(digest[:4], "big") % PALETTE_SIZE
            while slot in self.used_slots:  # linear probe on collision
                slot = (slot + 1) % PALETTE_SIZE
            self.used_slots.add(slot)
            color = tuple(int(c) for c in self.palette[slot])

        self.class_color[class_name] = color
        self.publish_legend()
        self.get_logger().info(f"class '{class_name}' -> rgb{color}")
        return color

    def publish_legend(self) -> None:
        legend = {
            name: "#%02x%02x%02x" % color for name, color in sorted(self.class_color.items())
        }
        self.legend_pub.publish(String(data=json.dumps(legend)))

    # ---------------------------------------------------------------- callbacks

    def on_labels(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Bad JSON on {self.labels_topic}: {exc}")
            return

        stamp = payload.get("time_stamp") or {}
        ids, colors = [], []
        for key, value in payload.items():
            if key == "time_stamp":
                continue
            try:
                label_id = int(key)
            except ValueError:
                continue
            class_name = value.get("class", "") if isinstance(value, dict) else str(value)
            ids.append(label_id)
            colors.append(self.color_for(class_name))

        order = np.argsort(np.array(ids, dtype=np.int64)) if ids else np.array([], dtype=int)
        lut = _LabelLut(
            np.array(ids, dtype=np.int64)[order],
            np.array(colors, dtype=np.uint8).reshape(-1, 3)[order],
        )

        self.latest_lut = lut
        key = (int(stamp.get("sec", 0)), int(stamp.get("nanosec", 0)))
        self.lut_by_stamp[key] = lut
        while len(self.lut_by_stamp) > STAMP_CACHE_SIZE:
            self.lut_by_stamp.popitem(last=False)

    def on_image(self, msg: Image) -> None:
        if msg.encoding not in ("32SC1", "32UC1"):
            if not self.warned_encoding:
                self.get_logger().warn(
                    f"Expected 32SC1 on {self.in_topic}, got '{msg.encoding}'. Ignoring."
                )
                self.warned_encoding = True
            return

        dtype = np.dtype(np.int32).newbyteorder(">" if msg.is_bigendian else "<")
        # Respect `step`: rows may be padded, so slice back to `width`.
        labels = np.frombuffer(msg.data, dtype=dtype)
        if labels.size < msg.height * (msg.step // 4):
            self.get_logger().warn(
                f"Truncated image: {labels.size * 4} bytes for {msg.height}x{msg.step}."
            )
            return
        labels = labels[: msg.height * (msg.step // 4)]
        labels = labels.reshape(msg.height, msg.step // 4)[:, : msg.width]

        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.is_bigendian = 0

        if self.mode == "16UC1":
            values = np.clip(labels, 0, 65535).astype(np.uint16)
            out.encoding = "16UC1"
            out.step = msg.width * 2
            out.data = values.tobytes()
        else:
            # Prefer the mapping published for this exact frame; the labels
            # message carries the same stamp as the image.
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            lut = self.lut_by_stamp.get(key) or self.latest_lut
            if lut is None:
                if not self.warned_no_labels:
                    self.get_logger().warn(
                        f"No labels seen yet on {self.labels_topic}; not colorizing."
                    )
                    self.warned_no_labels = True
                return
            out.encoding = "rgb8"
            out.step = msg.width * 3
            out.data = lut.apply(labels.astype(np.int64)).tobytes()

        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SegLabelColorizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
