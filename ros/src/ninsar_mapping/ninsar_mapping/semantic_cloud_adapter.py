#!/usr/bin/env python3
"""
Feeds the Voxeland server (cloud_in, segmentation_msgs/SemanticPointCloud) from an
organized RGB point cloud plus a per-pixel semantic label image.

Pipeline (see docs/CLOUD_IN_SPEC.md for the exact contract with the server):

  rgb + depth + camera_info -> depth_image_proc/point_cloud_xyzrgb_node -> points (organized)
  points + label image (+ optional confidence image) + labels JSON + TF -> this node -> cloud_in

The organized cloud keeps one point per pixel, so the label of each point is read
from the label image at the same pixel. Label ids are mapped to class names with the
JSON table (Isaac Sim `semantic_labels`, e.g. {"3": {"class": "ground"}, ...,
"time_stamp": {...}}) and then to a fixed, ordered list of categories.

Voxeland fuses per-voxel class evidence from a *local instance* table sent with each
cloud: each point carries `instance_id`, an index into `instances`, and each instance
carries (class, score) pairs. Two encodings of the label image into that table:

  ground truth (default): one instance per category with (class, 1.0); instance 0 is
      ("unknown", 0.05) for pixels whose class is not in `categories`.
  confidence bins (when `confidence_topic` is set): one instance per (category,
      confidence bin) with (class, p) and ("unknown", 1 - p), p = bin centre.

Points are expressed in `output_frame` (e.g. the ZED body frame, FLU) and the message
pose is `output_frame` in `world_frame` at the cloud stamp, both from TF.

Usage (defaults for the Isaac Sim tour bag):
  ros2 run ninsar_mapping semantic_cloud_adapter --ros-args \
      --params-file $(ros2 pkg prefix ninsar_mapping)/share/ninsar_mapping/config/isaac_voxeland.yaml
"""

import json
from collections import OrderedDict

import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovariance
from segmentation_msgs.msg import SemanticPointCloud
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

UNKNOWN = "unknown"
UNKNOWN_SCORE = 0.05  # as voxeland_robot_perception's Semantics.add_unknown_category
STAMP_CACHE_SIZE = 30

# Same layout as voxeland_robot_perception's output (x, y, z, rgb, instance_id).
# rgb is the PCL-packed float that depth_image_proc produces; Voxeland registers
# PointXYZRGBSemantics::rgb as float (voxeland_map/pcl_utils.hpp).
OUTPUT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="instance_id", offset=16, datatype=PointField.INT32, count=1),
]
OUTPUT_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4"), ("instance_id", "<i4")]
)


# --------------------------------------------------------------------------------------
# Encoding (pure functions, no ROS graph needed; covered by test/test_semantic_cloud_adapter.py)
# --------------------------------------------------------------------------------------

def server_categories(categories):
    """Category list sent to the server: the fixed list with "unknown" appended last."""
    return [c for c in categories if c != UNKNOWN] + [UNKNOWN]


def ground_truth_table(categories):
    """Local instance table for one-hot labels: [results of instance 0, 1, ...]."""
    cats = [c for c in categories if c != UNKNOWN]
    return [[(UNKNOWN, UNKNOWN_SCORE)]] + [[(c, 1.0)] for c in cats]


def confidence_table(categories, n_bins):
    """Local instance table for (category, confidence bin) pairs.

    Instance 1 + k * n_bins + b is category k with confidence bin b (centre p_b):
    results (category, p_b) and ("unknown", 1 - p_b).
    """
    cats = [c for c in categories if c != UNKNOWN]
    table = [[(UNKNOWN, UNKNOWN_SCORE)]]
    for c in cats:
        for b in range(n_bins):
            p = (b + 0.5) / n_bins
            table.append([(c, p), (UNKNOWN, 1.0 - p)])
    return table


def check_table(table, categories):
    """Raise ValueError if the table breaks the server contract (docs/CLOUD_IN_SPEC.md)."""
    cats = server_categories(categories)
    if len(set(cats)) != len(cats):
        raise ValueError(f"duplicate categories: {cats}")
    for i, results in enumerate(table):
        for class_id, score in results:
            if class_id not in cats:
                # the server would silently count it as evidence for categories[0]
                raise ValueError(f"instance {i}: class '{class_id}' is not in categories {cats}")
            if score < 0.0:
                raise ValueError(f"instance {i}: negative score {score}")


def category_lut(id_to_name, categories):
    """Lookup table label id -> category index + 1 (0 = not a mapped category)."""
    cats = [c for c in categories if c != UNKNOWN]
    index = {name: k + 1 for k, name in enumerate(cats)}
    max_id = max(id_to_name.keys(), default=-1)
    lut = np.zeros(max_id + 2, dtype=np.int32)  # last entry: ids outside the table
    for label_id, name in id_to_name.items():
        if label_id >= 0:
            lut[label_id] = index.get(name, 0)
    return lut


def label_to_category(labels, id_to_name, categories):
    """Per-pixel category index + 1 (0 = unknown) from dataset label ids."""
    lut = category_lut(id_to_name, categories)
    outside = lut.shape[0] - 1
    idx = np.where((labels >= 0) & (labels < outside), labels, outside)
    return lut[idx]


def encode_ground_truth(labels, id_to_name, categories):
    """instance_id per pixel for ground_truth_table(categories)."""
    return label_to_category(labels, id_to_name, categories).astype(np.int32)


def encode_confidence(labels, confidence, id_to_name, categories, n_bins):
    """instance_id per pixel for confidence_table(categories, n_bins)."""
    cat = label_to_category(labels, id_to_name, categories)
    conf = np.nan_to_num(confidence, nan=0.0, posinf=1.0, neginf=0.0)
    bins = np.clip((conf * n_bins).astype(np.int64), 0, n_bins - 1)
    ids = 1 + (cat - 1) * n_bins + bins
    return np.where(cat > 0, ids, 0).astype(np.int32)


def parse_labels_json(text):
    """Isaac Sim semantic_labels JSON -> ((sec, nanosec) or None, {label id: class name})."""
    data = json.loads(text)
    stamp = None
    ts = data.get("time_stamp")
    if isinstance(ts, dict):
        stamp = (int(ts.get("sec", 0)), int(ts.get("nanosec", 0)))
    table = {}
    for key, value in data.items():
        if key.isdigit():
            table[int(key)] = value["class"] if isinstance(value, dict) else str(value)
    return stamp, table


def select_points(xyz, min_range, max_range, drop_far_points):
    """Mask of points to send: finite, not closer than min_range, optionally not beyond max_range.

    Range is measured in the cloud's own (camera) frame. Finite far points are kept by
    default: the server turns points beyond its sensor_model.max_range into free-space rays.
    """
    finite = np.isfinite(xyz).all(axis=-1)
    rng = np.linalg.norm(np.where(finite[..., None], xyz, 0.0), axis=-1)
    mask = finite & (rng >= min_range)
    if drop_far_points:
        mask &= rng <= max_range
    return mask


def build_instances(table):
    """Detection2D list for a local instance table (id = index, as the server requires)."""
    instances = []
    for i, results in enumerate(table):
        det = Detection2D()
        det.id = str(i)
        for class_id, score in results:
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_id
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
        instances.append(det)
    return instances


def image_to_array(msg, dtype):
    """Single-channel Image -> (H, W) array, honouring step and endianness."""
    dt = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    row = msg.step // dt.itemsize
    return np.frombuffer(msg.data, dtype=dt).reshape(msg.height, row)[:, : msg.width]


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# --------------------------------------------------------------------------------------
# Node
# --------------------------------------------------------------------------------------

class SemanticCloudAdapter(Node):
    def __init__(self):
        super().__init__("semantic_cloud_adapter")

        self.declare_parameter("points_topic", "/sensor_rig/front_camera/points")
        self.declare_parameter("labels_image_topic", "/sensor_rig/front_camera/semantic_segmentation")
        self.declare_parameter("labels_json_topic", "/sensor_rig/front_camera/semantic_labels")
        # Optional per-pixel confidence (32FC1, same size as the label image); "" = ground truth.
        self.declare_parameter("confidence_topic", "")
        self.declare_parameter("confidence_bins", 10)
        self.declare_parameter("output_topic", "cloud_in")
        # Fixed, ordered classes mapped into the map; "unknown" is appended last.
        self.declare_parameter("categories", ["ground"])
        # Fallback label id -> class name table (index = id), used until/unless the JSON arrives.
        self.declare_parameter("label_names", [""])
        self.declare_parameter("world_frame", "world")
        # Frame the points are sent in; "" keeps the cloud's own frame.
        self.declare_parameter("output_frame", "")
        self.declare_parameter("min_range", 0.1)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("drop_far_points", False)
        self.declare_parameter("pixel_stride", 1)
        self.declare_parameter("sync_slop", 0.02)
        self.declare_parameter("tf_timeout", 0.2)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.world_frame = p("world_frame")
        self.output_frame = p("output_frame")
        self.min_range = float(p("min_range"))
        self.max_range = float(p("max_range"))
        self.drop_far_points = bool(p("drop_far_points"))
        self.stride = max(1, int(p("pixel_stride")))
        self.tf_timeout = Duration(seconds=float(p("tf_timeout")))
        self.categories = server_categories(list(p("categories")))
        self.use_confidence = bool(p("confidence_topic"))
        self.n_bins = int(p("confidence_bins"))

        if self.use_confidence:
            self.table = confidence_table(self.categories, self.n_bins)
        else:
            self.table = ground_truth_table(self.categories)
        check_table(self.table, self.categories)
        self.instances = build_instances(self.table)

        self.merged_names = {i: n for i, n in enumerate(p("label_names")) if n}
        self.names_by_stamp = OrderedDict()
        self.static_tf = {}  # cloud frame -> (R, t) into output_frame
        self.warned = set()
        self.stats = dict.fromkeys(["frames", "published", "skip_tf", "skip_size"], 0)
        self.last_tf_error = ""
        self.create_timer(10.0, self.log_stats)

        self.tf_buffer = Buffer()
        # Own listener node + thread (node=None): passing `self` with spin_thread=True would add this
        # node to a second executor, where the heavy cloud callbacks starve the TF updates.
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

        self.pub = self.create_publisher(SemanticPointCloud, p("output_topic"), 1)

        labels_qos = QoSProfile(depth=STAMP_CACHE_SIZE, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String, p("labels_json_topic"), self.labels_cb, labels_qos)

        # Small queues: when processing falls behind, old frames are dropped instead of queued.
        subs = [
            message_filters.Subscriber(self, PointCloud2, p("points_topic"), qos_profile=2),
            message_filters.Subscriber(self, Image, p("labels_image_topic"), qos_profile=2),
        ]
        if self.use_confidence:
            subs.append(message_filters.Subscriber(self, Image, p("confidence_topic"), qos_profile=2))
        self.sync = message_filters.ApproximateTimeSynchronizer(subs, 3, float(p("sync_slop")))
        self.sync.registerCallback(self.frame_cb)

        self.get_logger().info(
            f"categories {self.categories}, {len(self.instances)} local instances "
            f"({'confidence bins' if self.use_confidence else 'ground truth'}), "
            f"output frame '{self.output_frame or '<cloud frame>'}' in '{self.world_frame}'"
        )

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def log_stats(self):
        self.get_logger().info(", ".join(f"{k} {v}" for k, v in self.stats.items()) + " (last 10 s)")
        if self.stats["skip_tf"]:
            self.get_logger().warn(f"last TF error: {self.last_tf_error}")
        self.stats = dict.fromkeys(self.stats, 0)

    def labels_cb(self, msg):
        try:
            stamp, table = parse_labels_json(msg.data)
        except (ValueError, KeyError, TypeError) as e:
            self.warn_once("json", f"Could not parse labels JSON: {e}")
            return
        self.merged_names.update(table)
        if stamp is not None:
            self.names_by_stamp[stamp] = table
            while len(self.names_by_stamp) > STAMP_CACHE_SIZE:
                self.names_by_stamp.popitem(last=False)

    def names_for(self, stamp):
        table = self.names_by_stamp.get((stamp.sec, stamp.nanosec))
        if table is not None:
            return table
        self.warn_once("stamp", "No labels JSON with the label image stamp; using the merged table")
        return self.merged_names

    def output_transform(self, cloud_frame):
        """(R, t) taking points from cloud_frame into output_frame (static, cached)."""
        if not self.output_frame or self.output_frame == cloud_frame:
            return None
        if cloud_frame not in self.static_tf:
            tf = self.tf_buffer.lookup_transform(self.output_frame, cloud_frame, Time(),
                                                 timeout=self.tf_timeout)
            tr = tf.transform.translation
            self.static_tf[cloud_frame] = (quaternion_to_matrix(tf.transform.rotation),
                                           np.array([tr.x, tr.y, tr.z]))
        return self.static_tf[cloud_frame]

    def frame_cb(self, cloud_msg, labels_msg, conf_msg=None):
        self.stats["frames"] += 1
        if cloud_msg.height <= 1:
            self.warn_once("organized", "Input cloud is not organized; cannot attach per-pixel labels")
            return
        if (cloud_msg.height, cloud_msg.width) != (labels_msg.height, labels_msg.width):
            self.warn_once("size", f"Cloud {cloud_msg.width}x{cloud_msg.height} and label image "
                                   f"{labels_msg.width}x{labels_msg.height} differ; skipping frames")
            self.stats["skip_size"] += 1
            return

        cloud_frame = cloud_msg.header.frame_id
        frame = self.output_frame or cloud_frame
        try:
            to_output = self.output_transform(cloud_frame)
            pose_tf = self.tf_buffer.lookup_transform(self.world_frame, frame,
                                                      Time.from_msg(cloud_msg.header.stamp),
                                                      timeout=self.tf_timeout)
        except TransformException as e:
            self.warn_once("tf", f"TF not available, skipping frames: {e}")
            self.stats["skip_tf"] += 1
            self.last_tf_error = str(e)
            return

        s = self.stride
        pts = np.frombuffer(cloud_msg.data, dtype=point_cloud2.dtype_from_fields(
            cloud_msg.fields, cloud_msg.point_step))
        pts = pts.reshape(cloud_msg.height, cloud_msg.width)[::s, ::s]
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1).astype(np.float64)
        labels = image_to_array(labels_msg, "i4")[::s, ::s]

        names = self.names_for(labels_msg.header.stamp)
        if self.use_confidence:
            conf = image_to_array(conf_msg, "f4")[::s, ::s]
            ids = encode_confidence(labels, conf, names, self.categories, self.n_bins)
        else:
            ids = encode_ground_truth(labels, names, self.categories)

        mask = select_points(xyz, self.min_range, self.max_range, self.drop_far_points)
        xyz = xyz[mask]
        if to_output is not None:
            R, t = to_output
            xyz = xyz @ R.T + t

        out = np.empty(xyz.shape[0], dtype=OUTPUT_DTYPE)
        out["x"], out["y"], out["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        out["rgb"] = pts["rgb"][mask] if "rgb" in pts.dtype.names else 0.0
        out["instance_id"] = ids[mask]
        if out.size and out["instance_id"].max() >= len(self.instances):
            self.get_logger().error("instance_id out of range; dropping frame")
            return

        header = Header(stamp=cloud_msg.header.stamp, frame_id=frame)
        msg = SemanticPointCloud()
        msg.header = header
        msg.cloud = point_cloud2.create_cloud(header, OUTPUT_FIELDS, out)
        msg.cloud.is_dense = True
        for det in self.instances:
            det.header = header
        msg.instances = self.instances
        msg.categories = self.categories
        msg.pose = PoseWithCovariance()
        msg.pose.pose.position.x = pose_tf.transform.translation.x
        msg.pose.pose.position.y = pose_tf.transform.translation.y
        msg.pose.pose.position.z = pose_tf.transform.translation.z
        msg.pose.pose.orientation = pose_tf.transform.rotation
        self.pub.publish(msg)
        self.stats["published"] += 1


def main(args=None):
    rclpy.init(args=args)
    node = SemanticCloudAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
