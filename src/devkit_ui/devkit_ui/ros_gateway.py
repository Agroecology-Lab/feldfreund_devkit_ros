"""Thin ROS adapter acting as a temporary bridge during the architectural transition
to independent services.

This class allows various services to consume core ROS functionalities without
creating hard dependencies or tight coupling directly to the NiceGuiNode.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from rclpy.node import Node

if TYPE_CHECKING:
    from rclpy.clock import Clock
    from rclpy.publisher import Publisher
    from rclpy.qos import QoSProfile
    from rclpy.subscription import Subscription


class TransformUnavailable(Exception):
    """A TF lookup failed because the transform is missing, disconnected or out of range."""


class RosGateway:
    """Wrap a ROS node to create publishers/subscriptions without coupling to it."""

    def __init__(self, node: Node) -> None:
        """Store the node used for ROS communication, logging, and time."""
        self._node = node
        self._wall_clock: Clock | None = None
        self._tf_buffer = None

    def create_publisher(self, msg_type, topic: str, qos: int | QoSProfile) -> Publisher:
        """Create a publisher on the wrapped node with the supplied type, topic, and QoS."""
        return self._node.create_publisher(msg_type, topic, qos)

    def create_subscription(self, msg_type, topic: str, callback, qos: int | QoSProfile) -> Subscription:
        """Create a subscription on the wrapped node with the supplied callback and QoS."""
        return self._node.create_subscription(msg_type, topic, callback, qos)

    def create_client(self, srv_type, name: str):
        """Create a service client on the wrapped node."""
        return self._node.create_client(srv_type, name)

    def create_action_client(self, action_type, name: str):
        """Create an action client on the wrapped node."""
        # Imported here so modules that only need topics and services load without rclpy.action.
        from rclpy.action import ActionClient  # pylint: disable=import-outside-toplevel
        return ActionClient(self._node, action_type, name)

    def get_logger(self):
        """Return the wrapped node's logger."""
        return self._node.get_logger()

    def get_clock(self) -> Clock:
        """Return the wrapped node's clock."""
        return self._node.get_clock()

    def now_wall_sec(self) -> float:
        """Return node clock time in seconds, using simulated time when enabled."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def wall_clock(self) -> Clock:
        """Return a system-time clock that keeps running before simulated /clock exists."""
        if self._wall_clock is None:
            # Imported here so modules that only need topics and services load without rclpy.clock.
            from rclpy.clock import Clock, ClockType  # pylint: disable=import-outside-toplevel
            self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        return self._wall_clock

    def wall_time_sec(self) -> float:
        """Return real elapsed time in seconds, independent of simulated time."""
        return self.wall_clock().now().nanoseconds * 1e-9

    def create_timer(self, period_sec: float, callback, clock: Clock | None = None):
        """Create a timer on the wrapped node, optionally driven by a specific clock."""
        if clock is None:
            return self._node.create_timer(period_sec, callback)
        return self._node.create_timer(period_sec, callback, clock=clock)

    def start_tf_listener(self) -> None:
        """Start buffering TF messages now, so lookups have history when first requested."""
        if self._tf_buffer is not None:
            return
        # Imported here so modules that only need topics and services load without tf2_ros.
        # pylint: disable=import-outside-toplevel
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        # pylint: enable=import-outside-toplevel
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer, self._node)

    def lookup_transform(self, target_frame: str, source_frame: str):
        """Return the latest transform between two frames.

        Raises TransformUnavailable when the transform is missing, disconnected or out of range.
        """
        # pylint: disable=import-outside-toplevel
        from rclpy.time import Time
        from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
        # pylint: enable=import-outside-toplevel
        self.start_tf_listener()
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            raise TransformUnavailable(f'{type(exc).__name__}: {exc}') from exc
