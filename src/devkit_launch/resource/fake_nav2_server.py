# Copyright (c) 2026, topological_navigation contributors
# Licensed under the MIT License.
"""
Fake Nav2 action servers for testing topological navigation.

Provides NavigateToPose, NavigateThroughPoses, and FollowWaypoints action
servers that simulate robot movement and visualize a virtual robot in RViz.

The virtual robot:
- Publishes TF (map -> odom -> base_link) so localisation2 works
- Publishes /odometry/global so edge_action_manager2 works
- Shows a coloured cube + direction arrow + trail in RViz
- Moves at a configurable speed toward goal poses

Usage:
    ros2 run topological_nav_simulator fake_nav2_server
    ros2 run topological_nav_simulator fake_nav2_server --ros-args \
        -p robot_speed:=1.0 \
        -p initial_x:=0.0 \
        -p initial_y:=0.0
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav2_msgs.action import (
    FollowWaypoints,
    NavigateThroughPoses,
    NavigateToPose,
)

from topological_nav_simulator.virtual_robot import (
    VirtualRobot,
    _euler_from_quaternion,
    _lerp,
    _quaternion_from_yaw,
    _slerp_yaw,
)


class FakeNav2Server(Node):
    """ROS 2 node providing fake Nav2 action servers with a virtual robot."""

    def __init__(self):
        super().__init__('fake_nav2_server')

        # --- Parameters ---
        self.declare_parameter('robot_speed', 0.8)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self._speed = self.get_parameter('robot_speed').value
        self._ang_speed = self.get_parameter('angular_speed').value
        self._goal_tol = self.get_parameter('goal_tolerance').value
        self._rate = self.get_parameter('update_rate').value
        init_x = self.get_parameter('initial_x').value
        init_y = self.get_parameter('initial_y').value
        init_yaw = self.get_parameter('initial_yaw').value
        map_frame = self.get_parameter('map_frame').value
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

        # --- Virtual robot ---
        self._robot = VirtualRobot(self, map_frame, base_frame, odom_frame)
        initial_pose = Pose()
        initial_pose.position.x = float(init_x)
        initial_pose.position.y = float(init_y)
        initial_pose.orientation = _quaternion_from_yaw(float(init_yaw))
        self._robot.set_pose(initial_pose)

        # Lock for concurrent goal handling
        self._nav_lock = threading.Lock()
        self._current_goal_handle = None

        # Callback group for action servers
        cb_group = ReentrantCallbackGroup()

        # --- Action Servers ---
        self._nav_to_pose_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self._execute_navigate_to_pose,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self._nav_through_poses_server = ActionServer(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
            execute_callback=self._execute_navigate_through_poses,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self._follow_waypoints_server = ActionServer(
            self,
            FollowWaypoints,
            'follow_waypoints',
            execute_callback=self._execute_follow_waypoints,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # Limbic row-follow stub — accepts NavigateToPose goals on
        # /limbic_row_follow so the topo nav row_traversal_bt edge action
        # doesn't time out waiting for a server that doesn't exist in sim.
        self._limbic_row_follow_server = ActionServer(
            self,
            NavigateToPose,
            'limbic_row_follow',
            execute_callback=self._execute_navigate_to_pose,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # --- Initial pose subscriber (RViz 2D Pose Estimate) ---
        self.create_subscription(
            PoseStamped,
            '/initialpose',
            self._initialpose_cb,
            10,
        )

        # --- Periodic TF + marker publishing (even when idle) ---
        timer_period = 1.0 / self._rate
        self._pub_timer = self.create_timer(timer_period, self._publish_tick)

        self.get_logger().info(
            '=== Fake Nav2 Server Started ===\n'
            f'  Speed: {self._speed} m/s  |  Angular: {self._ang_speed} rad/s\n'
            f'  Tolerance: {self._goal_tol} m  |  Rate: {self._rate} Hz\n'
            f'  Initial pose: ({init_x:.2f}, {init_y:.2f}, yaw={init_yaw:.2f})\n'
            f'  Action servers:\n'
            f'    /navigate_to_pose\n'
            f'    /navigate_through_poses\n'
            f'    /follow_waypoints\n'
            f'    /limbic_row_follow (stub)\n'
            f'  Topics published:\n'
            f'    /odometry/global\n'
            f'    /virtual_robot/markers\n'
            f'    /virtual_robot/pose\n'
            f'    TF: {map_frame} -> {odom_frame} -> {base_frame}'
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        """Accept all incoming goals."""
        self.get_logger().info('Goal received')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _initialpose_cb(self, msg):
        """Teleport robot to initial pose from RViz 2D Pose Estimate."""
        self.get_logger().info(
            f'Teleporting robot to ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )
        self._robot.set_pose(msg.pose)
        self._robot.clear_trail()

    def _publish_tick(self):
        """Periodic broadcast of TF and markers (even when idle)."""
        self._robot.publish_all()

    # ── NavigateToPose ───────────────────────────────────────────────────

    def _execute_navigate_to_pose(self, goal_handle):
        """Execute NavigateToPose action — move to a single goal."""
        self.get_logger().info(
            f'NavigateToPose -> '
            f'({goal_handle.request.pose.pose.position.x:.2f}, '
            f'{goal_handle.request.pose.pose.position.y:.2f})'
        )

        target = goal_handle.request.pose.pose
        success = self._move_to_pose(target, goal_handle, 'NavigateToPose')

        result = NavigateToPose.Result()
        if success:
            goal_handle.succeed()
            self.get_logger().info('NavigateToPose succeeded')
        elif goal_handle.is_cancel_requested or goal_handle.status == 5:
            # Already transitioned to CANCELED inside _move_to_pose
            self.get_logger().info('NavigateToPose cancelled')
        else:
            goal_handle.abort()
            self.get_logger().warn('NavigateToPose aborted')
        return result

    # ── NavigateThroughPoses ─────────────────────────────────────────────

    def _execute_navigate_through_poses(self, goal_handle):
        """Execute NavigateThroughPoses — move through a list of poses."""
        poses = goal_handle.request.poses
        self.get_logger().info(
            f'NavigateThroughPoses -> {len(poses)} waypoint(s)'
        )

        success = True
        for i, pose_stamped in enumerate(poses):
            self.get_logger().info(
                f'  Waypoint {i + 1}/{len(poses)}: '
                f'({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f})'
            )
            if not self._move_to_pose(
                pose_stamped.pose, goal_handle,
                f'NavigateThroughPoses [{i + 1}/{len(poses)}]'
            ):
                success = False
                break

        result = NavigateThroughPoses.Result()
        if success:
            goal_handle.succeed()
            self.get_logger().info('NavigateThroughPoses succeeded')
        elif goal_handle.is_cancel_requested or goal_handle.status == 5:
            # Already transitioned to CANCELED inside _move_to_pose
            self.get_logger().info('NavigateThroughPoses cancelled')
        else:
            goal_handle.abort()
            self.get_logger().warn('NavigateThroughPoses aborted')
        return result

    # ── FollowWaypoints ──────────────────────────────────────────────────

    def _execute_follow_waypoints(self, goal_handle):
        """Execute FollowWaypoints — move through waypoints sequentially."""
        poses = goal_handle.request.poses
        self.get_logger().info(
            f'FollowWaypoints -> {len(poses)} waypoint(s)'
        )

        missed = []
        for i, pose_stamped in enumerate(poses):
            self.get_logger().info(
                f'  Waypoint {i + 1}/{len(poses)}: '
                f'({pose_stamped.pose.position.x:.2f}, '
                f'{pose_stamped.pose.position.y:.2f})'
            )
            if not self._move_to_pose(
                pose_stamped.pose, goal_handle,
                f'FollowWaypoints [{i + 1}/{len(poses)}]'
            ):
                missed.append(i)
                break

        result = FollowWaypoints.Result()
        result.missed_waypoints = missed
        if not missed:
            goal_handle.succeed()
            self.get_logger().info('FollowWaypoints succeeded')
        elif goal_handle.is_cancel_requested or goal_handle.status == 5:
            # Already transitioned to CANCELED inside _move_to_pose
            self.get_logger().info('FollowWaypoints cancelled')
        else:
            goal_handle.abort()
            self.get_logger().warn(
                f'FollowWaypoints aborted (missed: {missed})'
            )
        return result

    # ── Core movement simulation ─────────────────────────────────────────

    def _move_to_pose(self, target_pose, goal_handle, action_label):
        """
        Simulate robot movement from current pose to target pose.

        The robot first rotates to face the target, then drives in a
        straight line, and finally rotates to the goal orientation.
        Returns True on success, False if cancelled.
        """
        dt = 1.0 / self._rate

        # Phase 1: Rotate to face the target
        if not self._rotate_toward_target(target_pose, goal_handle, dt):
            return False

        # Phase 2: Drive to the target position
        if not self._drive_to_position(target_pose, goal_handle, dt, action_label):
            return False

        # Phase 3: Rotate to final orientation
        if not self._rotate_to_orientation(target_pose, goal_handle, dt):
            return False

        return True

    def _rotate_toward_target(self, target_pose, goal_handle, dt):
        """Rotate the robot to face the target position."""
        dx = target_pose.position.x - self._robot.pose.position.x
        dy = target_pose.position.y - self._robot.pose.position.y
        dist = math.hypot(dx, dy)

        if dist < self._goal_tol:
            return True  # Already close, skip rotation

        target_yaw = math.atan2(dy, dx)
        current_yaw = _euler_from_quaternion(self._robot.pose.orientation)
        yaw_diff = math.atan2(
            math.sin(target_yaw - current_yaw),
            math.cos(target_yaw - current_yaw),
        )

        if abs(yaw_diff) < 0.05:
            return True  # Close enough

        rotation_time = abs(yaw_diff) / self._ang_speed
        steps = max(int(rotation_time / dt), 1)
        start_yaw = current_yaw

        for i in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            t = float(i) / steps
            yaw = _slerp_yaw(start_yaw, target_yaw, t)
            self._robot.pose.orientation = _quaternion_from_yaw(yaw)
            time.sleep(dt)

        return True

    def _drive_to_position(self, target_pose, goal_handle, dt, action_label):
        """Drive the robot in a straight line toward the target."""
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False

            dx = target_pose.position.x - self._robot.pose.position.x
            dy = target_pose.position.y - self._robot.pose.position.y
            dist = math.hypot(dx, dy)

            if dist < self._goal_tol:
                # Snap to exact position
                self._robot.pose.position.x = target_pose.position.x
                self._robot.pose.position.y = target_pose.position.y
                return True

            # Move toward target
            step = min(self._speed * dt, dist)
            angle = math.atan2(dy, dx)
            self._robot.pose.position.x += step * math.cos(angle)
            self._robot.pose.position.y += step * math.sin(angle)
            self._robot.pose.orientation = _quaternion_from_yaw(angle)

            # Publish feedback
            self._publish_feedback(goal_handle, action_label, dist)

            time.sleep(dt)

    def _rotate_to_orientation(self, target_pose, goal_handle, dt):
        """Rotate the robot to the goal's final orientation."""
        target_yaw = _euler_from_quaternion(target_pose.orientation)
        current_yaw = _euler_from_quaternion(self._robot.pose.orientation)
        yaw_diff = math.atan2(
            math.sin(target_yaw - current_yaw),
            math.cos(target_yaw - current_yaw),
        )

        if abs(yaw_diff) < 0.05:
            self._robot.pose.orientation = target_pose.orientation
            return True

        rotation_time = abs(yaw_diff) / self._ang_speed
        steps = max(int(rotation_time / dt), 1)
        start_yaw = current_yaw

        for i in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            t = float(i) / steps
            yaw = _slerp_yaw(start_yaw, target_yaw, t)
            self._robot.pose.orientation = _quaternion_from_yaw(yaw)
            time.sleep(dt)

        self._robot.pose.orientation = target_pose.orientation
        return True

    def _publish_feedback(self, goal_handle, label, distance_remaining):
        """Publish action feedback with distance remaining."""
        # NavigateToPose and NavigateThroughPoses have different feedback types
        # but both have a current_pose field and distance_remaining
        try:
            # Try NavigateThroughPoses feedback
            feedback = NavigateThroughPoses.Feedback()
            feedback.current_pose = PoseStamped()
            feedback.current_pose.header.frame_id = (
                self.get_parameter('map_frame').value
            )
            feedback.current_pose.header.stamp = (
                self.get_clock().now().to_msg()
            )
            feedback.current_pose.pose = self._robot.pose
            feedback.distance_remaining = float(distance_remaining)
            goal_handle.publish_feedback(feedback)
        except Exception:
            # Fallback: try NavigateToPose feedback
            try:
                feedback = NavigateToPose.Feedback()
                feedback.current_pose = PoseStamped()
                feedback.current_pose.header.frame_id = (
                    self.get_parameter('map_frame').value
                )
                feedback.current_pose.header.stamp = (
                    self.get_clock().now().to_msg()
                )
                feedback.current_pose.pose = self._robot.pose
                feedback.distance_remaining = float(distance_remaining)
                goal_handle.publish_feedback(feedback)
            except Exception:
                pass  # FollowWaypoints has different feedback


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = None
    executor = None

    try:
        node = FakeNav2Server()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if executor is not None and node is not None:
                executor.remove_node(node)
        except Exception:
            pass
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
