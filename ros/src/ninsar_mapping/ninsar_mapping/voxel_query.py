#!/usr/bin/env python3
"""
Queries the live Voxeland map by box and class, and publishes the matching voxels with their full
class distribution. Voxeland itself is not modified (see docs/VOXELAND_GUIDE.md, "Querying the map").

Two steps per query:

  1. Snapshot: subscribe to bonxai_point_cloud_centers just long enough to take one message
     (with latch: false the server only serializes the map while someone subscribes). Positions
     and top class (decoded from the voxel colour, ninsar_mapping/voxel_store.py) of every
     occupied voxel. Selection by box and top class is numpy on this snapshot.
  2. Fetch: voxeland/get_class_distributions for the selected voxel centres only, in chunks, so
     the server can insert clouds between them. p(class) has occupancy folded in.

The query is set as parameters, then run with the ~/query service (std_srvs/Trigger):

  ros2 param set /voxel_query query.classes "['grapevine']"
  ros2 param set /voxel_query query.center "[10.0, 5.0, 1.0]"
  ros2 param set /voxel_query query.dimensions "[4.0, 1.0, 2.0]"   # all zero = whole map
  ros2 param set /voxel_query query.yaw 0.5                          # rad, about z
  ros2 service call /voxel_query/query std_srvs/srv/Trigger

The result is ~/result (PointCloud2: x, y, z, rgb, p_<class> per class) and the box is
~/query_box (Marker), both transient local for RViz.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from voxeland_msgs.srv import GetClassDistributions

from ninsar_mapping.voxel_store import (
    OrientedBox, build_result_cloud, class_colors, distributions_to_array, keep_by_prob,
    matrix_to_quaternion, store_from_cloud,
)


class VoxelQuery(Node):
    def __init__(self):
        super().__init__("voxel_query")

        self.declare_parameter("cloud_topic", "/bonxai_point_cloud_centers")
        self.declare_parameter("distributions_service", "/voxeland/get_class_distributions")
        self.declare_parameter("snapshot_timeout_s", 5.0)
        self.declare_parameter("service_timeout_s", 10.0)
        self.declare_parameter("fetch_chunk", 5000)
        # The query, read when ~/query is called
        self.declare_parameter("query.center", [0.0, 0.0, 0.0])
        self.declare_parameter("query.dimensions", [0.0, 0.0, 0.0])  # all zero = whole map
        self.declare_parameter("query.yaw", 0.0)
        self.declare_parameter("query.classes", [""])  # top class filter; empty = any class
        self.declare_parameter("query.min_prob", 0.0)  # on max p over query.classes (all if empty)
        self.declare_parameter("query.max_voxels", 200000)

        self.group = ReentrantCallbackGroup()
        self.busy = threading.Lock()
        self.store = None
        self.store_time = None  # time.monotonic() when the snapshot arrived
        self.class_names = None

        self.client = self.create_client(GetClassDistributions, self.param("distributions_service"),
                                         callback_group=self.group)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_pub = self.create_publisher(PointCloud2, "~/result", latched)
        self.box_pub = self.create_publisher(Marker, "~/query_box", latched)
        self.create_service(Trigger, "~/query", self.query_cb, callback_group=self.group)

        self.get_logger().info(f"Ready: map from {self.param('cloud_topic')}, distributions from "
                               f"{self.param('distributions_service')}")

    def param(self, name):
        return self.get_parameter(name).value

    # ----------------------------------------------------------------------------------
    # Server calls (run inside the service callback; the executor serves the futures)
    # ----------------------------------------------------------------------------------

    def wait(self, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            self.client.remove_pending_request(future)
            raise TimeoutError(f"{self.param('distributions_service')} did not answer in {timeout} s")
        return future.result()

    def fetch(self, xyz):
        """(N, C) class probabilities of the voxel centres xyz."""
        timeout = float(self.param("service_timeout_s"))
        chunk = max(1, int(self.param("fetch_chunk")))
        parts = []
        for start in range(0, xyz.shape[0], chunk):
            req = GetClassDistributions.Request()
            req.query_points = [Point(x=float(x), y=float(y), z=float(z))
                                for x, y, z in xyz[start:start + chunk]]
            parts.append(distributions_to_array(self.wait(self.client.call_async(req), timeout).distributions,
                                                self.class_names))
        return np.concatenate(parts) if parts else np.empty((0, len(self.class_names)), np.float32)

    def ensure_class_names(self):
        """Class names in the server's index order, from a one-point probe (cached)."""
        if self.class_names is not None:
            return
        if not self.client.wait_for_service(timeout_sec=float(self.param("service_timeout_s"))):
            raise RuntimeError(f"{self.param('distributions_service')} not available (the server "
                               "creates it when the first cloud arrives)")
        req = GetClassDistributions.Request(query_points=[Point()])
        res = self.wait(self.client.call_async(req), float(self.param("service_timeout_s")))
        self.class_names = [h.class_id for h in res.distributions[0].probabilities]
        self.get_logger().info(f"Classes: {self.class_names}")

    def take_snapshot(self):
        """Fresh snapshot of the map, or the previous one if the server publishes nothing."""
        got = threading.Event()
        box = {}

        def cb(msg):
            if not got.is_set():
                box["msg"] = msg
                got.set()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        sub = self.create_subscription(PointCloud2, self.param("cloud_topic"), cb, qos,
                                       callback_group=self.group)
        try:
            fresh = got.wait(float(self.param("snapshot_timeout_s")))
        finally:
            self.destroy_subscription(sub)
        if fresh:
            self.store = store_from_cloud(box["msg"], len(self.class_names))
            self.store_time = time.monotonic()
        elif self.store is None:
            raise RuntimeError(f"no map on {self.param('cloud_topic')} within "
                               f"{self.param('snapshot_timeout_s')} s (is the server inserting clouds?)")
        return fresh

    # ----------------------------------------------------------------------------------
    # Query
    # ----------------------------------------------------------------------------------

    def read_query(self):
        dims = [float(v) for v in self.param("query.dimensions")]
        box = None
        if any(d > 0.0 for d in dims):
            box = OrientedBox.from_yaw(self.param("query.center"), dims, float(self.param("query.yaw")))
        names = [n for n in self.param("query.classes") if n]
        unknown = [n for n in names if n not in self.class_names]
        if unknown:
            raise ValueError(f"unknown classes {unknown}; the map has {self.class_names}")
        return box, names, [self.class_names.index(n) for n in names]

    def query_cb(self, request, response):
        if not self.busy.acquire(blocking=False):
            response.success, response.message = False, "another query is running"
            return response
        try:
            response.message = self.run_query()
            response.success = True
        except (RuntimeError, TimeoutError, ValueError) as e:
            response.success, response.message = False, str(e)
            self.get_logger().error(response.message)
        finally:
            self.busy.release()
        return response

    def run_query(self):
        t0 = time.monotonic()
        self.ensure_class_names()
        box, names, class_ids = self.read_query()
        fresh = self.take_snapshot()
        store = self.store
        t_snap = time.monotonic()

        idx = store.select(box, class_ids)
        max_voxels = int(self.param("query.max_voxels"))
        capped = idx.size > max_voxels
        idx = idx[:max_voxels]
        probs = self.fetch(store.xyz[idx])
        keep = keep_by_prob(probs, class_ids, float(self.param("query.min_prob")))
        idx, probs = idx[keep], probs[keep]
        t_fetch = time.monotonic()

        colors = np.append(class_colors(len(self.class_names)), np.uint32(0x808080))
        top = probs.argmax(axis=1) if probs.size else np.empty(0, dtype=np.int64)
        self.result_pub.publish(build_result_cloud(store.header, store.xyz[idx], colors[top], probs,
                                                   self.class_names))
        self.publish_box(box, store.header)

        counts = np.bincount(top, minlength=len(self.class_names))
        per_class = ", ".join(f"{n} {c}" for n, c in zip(self.class_names, counts) if c)
        message = (
            f"{idx.size} voxels ({per_class or 'none'}) of {len(store)} in the map"
            f"{f', classes {names}' if names else ''}{'' if box else ', whole map'}; "
            f"{'fresh' if fresh else f'reused {time.monotonic() - self.store_time:.0f} s old'} "
            f"snapshot {t_snap - t0:.2f} s, fetch {t_fetch - t_snap:.2f} s"
            f"{f'; capped at query.max_voxels = {max_voxels}' if capped else ''}"
        )
        self.get_logger().info(message)
        return message

    def publish_box(self, box, header):
        marker = Marker()
        marker.header = header
        marker.ns = "voxel_query"
        if box is None:
            marker.action = Marker.DELETEALL
        else:
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose.position = Point(x=box.center[0], y=box.center[1], z=box.center[2])
            q = marker.pose.orientation
            q.x, q.y, q.z, q.w = (float(v) for v in matrix_to_quaternion(box.rotation))
            marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in box.dimensions)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0, 0.25
        self.box_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VoxelQuery()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
