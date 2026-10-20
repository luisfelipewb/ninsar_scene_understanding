"""Publish one parent->child transform, live-tunable from rqt_reconfigure.

Angles are entered in degrees because that is what a human tunes; the resulting
quaternion is mirrored back into the `qx/qy/qz/qw` parameters and logged, and
`~/print_transform` emits both a `static_transform_publisher` command line and a
bagfix `tf_static` snippet, which is what the downstream configs want.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, replace

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from ninsar_tf_tools.angles import quaternion_from_rpy_deg

# Parameters the node writes to itself; they are outputs, not inputs.
READBACK = ("qx", "qy", "qz", "qw")

HOUSEKEEPING_PERIOD = 0.2  # s; applies rate changes and syncs the read-backs


@dataclass(frozen=True)
class Settings:
    parent_frame: str
    child_frame: str
    x: float
    y: float
    z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    publish_rate: float

    def validate(self) -> str:
        """Return an empty string when usable, otherwise the reason it is not."""
        for name in ("parent_frame", "child_frame"):
            if not getattr(self, name).strip():
                return f"{name} must not be empty"
        if self.parent_frame.strip() == self.child_frame.strip():
            return (
                f"parent_frame and child_frame are both '{self.parent_frame}'; "
                "a frame cannot be its own parent"
            )
        for name in ("x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg"):
            if not math.isfinite(getattr(self, name)):
                return f"{name} must be finite"
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            return f"publish_rate must be > 0, got {self.publish_rate}"
        return ""

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        return quaternion_from_rpy_deg(self.roll_deg, self.pitch_deg, self.yaw_deg)

    @property
    def rpy_rad(self) -> tuple[float, float, float]:
        return tuple(
            math.radians(a) for a in (self.roll_deg, self.pitch_deg, self.yaw_deg)
        )

    def summary(self) -> str:
        quat = self.quaternion
        return (
            f"{self.parent_frame} -> {self.child_frame}  "
            f"xyz=[{self.x:.4f}, {self.y:.4f}, {self.z:.4f}]  "
            f"rpy_deg=[{self.roll_deg:.3f}, {self.pitch_deg:.3f}, {self.yaw_deg:.3f}]  "
            f"quat=[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]"
        )

    def static_transform_publisher_command(self) -> str:
        quat = self.quaternion
        return (
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {self.x:.6f} --y {self.y:.6f} --z {self.z:.6f} "
            f"--qx {quat[0]:.9f} --qy {quat[1]:.9f} "
            f"--qz {quat[2]:.9f} --qw {quat[3]:.9f} "
            f"--frame-id {self.parent_frame} --child-frame-id {self.child_frame}"
        )

    def bagfix_snippet(self) -> str:
        """A drop-in `tf_static` edit entry; bagfix takes rpy in radians."""
        roll, pitch, yaw = self.rpy_rad
        return (
            "  - transform: tf_static\n"
            "    edit:\n"
            f"      - parent: {self.parent_frame}\n"
            f"        child: {self.child_frame}\n"
            f"        xyz: [{self.x:.6f}, {self.y:.6f}, {self.z:.6f}]\n"
            f"        rpy: [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]"
        )


def _describe(description: str, low: float, high: float, step: float = 0.0):
    return ParameterDescriptor(
        description=description,
        floating_point_range=[
            FloatingPointRange(from_value=low, to_value=high, step=step)
        ],
    )


class TfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("tf_publisher")

        self.declare_parameter(
            "parent_frame", "base_link",
            ParameterDescriptor(description="frame the transform is expressed in"),
        )
        self.declare_parameter(
            "child_frame", "sensor_link",
            ParameterDescriptor(description="frame being placed"),
        )
        for axis in ("x", "y", "z"):
            self.declare_parameter(
                axis, 0.0, _describe(f"translation along {axis} [m]", -50.0, 50.0)
            )
        for angle in ("roll", "pitch", "yaw"):
            self.declare_parameter(
                f"{angle}_deg", 0.0,
                _describe(f"{angle} [deg], fixed-axis ZYX", -180.0, 180.0),
            )
        self.declare_parameter(
            "publish_rate", 30.0,
            _describe("broadcast rate [Hz]; ignored when static_tf", 0.1, 200.0),
        )
        self.declare_parameter(
            "static_tf", False,
            ParameterDescriptor(
                description="latch on /tf_static instead of broadcasting on /tf "
                            "(set at launch only)",
                read_only=True,
            ),
        )
        for name in READBACK:
            self.declare_parameter(
                name, 0.0,
                ParameterDescriptor(
                    description=f"read-back: {name} of the current rotation; "
                                "edits are overwritten",
                ),
            )

        self._settings = self._read_settings()
        error = self._settings.validate()
        if error:
            raise ValueError(f"initial parameters are unusable: {error}")

        self._static = self.get_parameter("static_tf").value
        self._syncing = False
        self._pending_static_broadcast = self._static
        self._publish_timer = None
        self._publish_period = 0.0

        if self._static:
            # Not tf2_ros.StaticTransformBroadcaster: it only ever records the
            # first transform per child frame, so an edited pose would be
            # re-broadcast with its original value. A transient-local publisher
            # of our own latches whatever is current instead.
            self._static_publisher = self.create_publisher(
                TFMessage, "/tf_static",
                QoSProfile(
                    depth=1,
                    history=HistoryPolicy.KEEP_LAST,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            self.get_logger().info("static_tf is set; publish_rate is ignored")
        else:
            self._broadcaster = TransformBroadcaster(self)

        self._housekeeping = self.create_timer(HOUSEKEEPING_PERIOD, self._housekeep)
        self.add_on_set_parameters_callback(self._on_parameters)
        self._print_service = self.create_service(
            Trigger, "~/print_transform", self._on_print_transform
        )

        self.get_logger().info(f"publishing {self._settings.summary()}")

    # -- parameters ----------------------------------------------------------

    def _read_settings(self) -> Settings:
        def value(name):
            return self.get_parameter(name).value

        return Settings(
            parent_frame=value("parent_frame"),
            child_frame=value("child_frame"),
            x=value("x"), y=value("y"), z=value("z"),
            roll_deg=value("roll_deg"),
            pitch_deg=value("pitch_deg"),
            yaw_deg=value("yaw_deg"),
            publish_rate=value("publish_rate"),
        )

    def _on_parameters(self, changed: list[Parameter]) -> SetParametersResult:
        if self._syncing:
            return SetParametersResult(successful=True, reason="read-back sync")

        # Validate on a copy so a rejected set never half-applies.
        candidate = self._settings
        tunable = {field.name for field in fields(Settings)}
        for parameter in changed:
            if parameter.name in tunable:
                candidate = replace(candidate, **{parameter.name: parameter.value})

        error = candidate.validate()
        if error:
            self.get_logger().warn(f"rejected parameter change: {error}")
            return SetParametersResult(successful=False, reason=error)

        if candidate != self._settings:
            self._settings = candidate
            self._pending_static_broadcast = self._static
            self.get_logger().info(f"publishing {candidate.summary()}")
        return SetParametersResult(successful=True, reason="ok")

    def _sync_readback(self) -> None:
        """Mirror the current quaternion into the read-back parameters."""
        quat = self._settings.quaternion
        updates = [
            Parameter(name, Parameter.Type.DOUBLE, component)
            for name, component in zip(READBACK, quat)
            if abs(self.get_parameter(name).value - component) > 1e-12
        ]
        if not updates:
            return
        self._syncing = True
        try:
            self.set_parameters(updates)
        finally:
            self._syncing = False

    # -- publishing ----------------------------------------------------------

    def _transform(self) -> TransformStamped:
        settings = self._settings
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = settings.parent_frame
        message.child_frame_id = settings.child_frame
        message.transform.translation.x = settings.x
        message.transform.translation.y = settings.y
        message.transform.translation.z = settings.z
        quat = settings.quaternion
        message.transform.rotation.x = quat[0]
        message.transform.rotation.y = quat[1]
        message.transform.rotation.z = quat[2]
        message.transform.rotation.w = quat[3]
        return message

    def _publish(self) -> None:
        if self._static:
            self._static_publisher.publish(TFMessage(transforms=[self._transform()]))
        else:
            self._broadcaster.sendTransform(self._transform())

    def _housekeep(self) -> None:
        """Owns everything that must not run inside the parameter callback."""
        self._sync_readback()
        if self._static:
            if self._pending_static_broadcast:
                self._pending_static_broadcast = False
                self._publish()
            return

        period = 1.0 / self._settings.publish_rate
        if self._publish_timer is None or abs(period - self._publish_period) > 1e-9:
            if self._publish_timer is not None:
                self.destroy_timer(self._publish_timer)
            self._publish_period = period
            self._publish_timer = self.create_timer(period, self._publish)

    # -- service -------------------------------------------------------------

    def _on_print_transform(self, _request, response) -> Trigger.Response:
        settings = self._settings
        response.success = True
        response.message = (
            f"{settings.summary()}\n\n"
            f"{settings.static_transform_publisher_command()}\n\n"
            f"# bagfix pipeline entry (rpy in radians):\n{settings.bagfix_snippet()}"
        )
        self.get_logger().info(f"\n{response.message}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
