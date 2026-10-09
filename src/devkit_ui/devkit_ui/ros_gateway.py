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


class RosGateway:
    """Wrap a ROS node to create publishers/subscriptions without coupling to it."""

    def __init__(self, node: Node) -> None:
        """Store the node used for ROS communication, logging, and time."""
        self._node = node

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
