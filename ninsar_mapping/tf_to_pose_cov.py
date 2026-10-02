#!/usr/bin/env python3
"""
Publishes a geometry_msgs/PoseWithCovarianceStamped computed from the TF
transform world -> base_link (e.g. from a bag being played back).

The node listens to /tf and /tf_static and, at a fixed rate, looks up the
latest available transform `parent_frame -> child_frame`. The pose of
`child_frame` expressed in `parent_frame` is published on `output_topic`
with a fixed diagonal covariance and the transform's own timestamp in the
header. A pose is only published when the transform's stamp advances, so
the output rate never exceeds the TF rate.

Usage (defaults shown):
  python3 tf_to_pose_cov.py --ros-args \
      -p parent_frame:=world \
      -p child_frame:=base_link \
      -p output_topic:=/pose_with_covariance \
      -p rate:=50.0 \
      -p position_variance:=0.01 \
      -p orientation_variance:=0.01

When playing a bag, remember to use sim time if the bag publishes /clock:
  ros2 bag play <bag> --clock
  python3 tf_to_pose_cov.py --ros-args -p use_sim_time:=true
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException


class TfToPoseWithCovariance(Node):
    def __init__(self):
        super().__init__("tf_to_pose_cov")

        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("output_topic", "/pose_with_covariance")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("position_variance", 0.01)
        self.declare_parameter("orientation_variance", 0.01)

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        output_topic = self.get_parameter("output_topic").value
        rate = self.get_parameter("rate").value
        pos_var = self.get_parameter("position_variance").value
        ori_var = self.get_parameter("orientation_variance").value

        # Row-major 6x6 (x, y, z, roll, pitch, yaw); diagonal only.
        self.covariance = [0.0] * 36
        for i in range(3):
            self.covariance[i * 6 + i] = float(pos_var)
            self.covariance[(i + 3) * 6 + (i + 3)] = float(ori_var)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, qos)

        self.last_stamp = None
        self.warned = False
        self.timer = self.create_timer(1.0 / float(rate), self.on_timer)

        self.get_logger().info(
            f"Publishing {self.parent_frame} -> {self.child_frame} on {output_topic} "
            f"at up to {rate:.1f} Hz"
        )

    def on_timer(self):
        try:
            # Time() == latest available transform.
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, Time()
            )
        except TransformException as ex:
            if not self.warned:
                self.get_logger().warn(
                    f"Waiting for transform {self.parent_frame} -> {self.child_frame}: {ex}"
                )
                self.warned = True
            return
        self.warned = False

        stamp = Time.from_msg(tf.header.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return  # no new TF since last publish
        self.last_stamp = stamp

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.parent_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        msg.pose.covariance = self.covariance
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfToPoseWithCovariance()
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
