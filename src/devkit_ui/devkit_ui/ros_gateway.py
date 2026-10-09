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
        self._node = node

    def create_publisher(self, msg_type, topic: str, qos: int | QoSProfile) -> Publisher:
        return self._node.create_publisher(msg_type, topic, qos)

    def create_subscription(self, msg_type, topic: str, callback, qos: int | QoSProfile) -> Subscription:
        return self._node.create_subscription(msg_type, topic, callback, qos)

    def get_logger(self):
        return self._node.get_logger()

    def get_clock(self) -> Clock:
        return self._node.get_clock()

    def now_wall_sec(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9
