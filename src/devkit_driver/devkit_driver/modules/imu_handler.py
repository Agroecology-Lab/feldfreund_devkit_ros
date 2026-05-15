"""ImuHandler — publishes BNO085 data from the Lizard robot brain as sensor_msgs/Imu.

The BNO085 runs in NDOF mode on the ESP32 (via ImuBno085 in Lizard).
Telemetry is streamed at 50 Hz in devkit.liz and arrives here via the
feldfreund_devkit Imu hardware object.

Topic published: /imu/data  (sensor_msgs/Imu, frame_id = "imu_link")

FusionCore can consume this once you uncomment the imu.* params in
fusioncore.yaml.  The devkit_driver_node instantiates this handler
whenever system.feldfreund.imu is present.
"""

import math

from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


# BNO085 noise figures from the Bosch datasheet.
# Gyroscope white noise density: ~0.014 °/s/√Hz → ≈ 2.4e-4 rad/s/√Hz
# At 50 Hz bandwidth: σ ≈ 2.4e-4 * √50 ≈ 1.7e-3 rad/s  → variance ≈ 2.9e-6
_GYRO_VARIANCE = 2.9e-6      # (rad/s)²

# Accelerometer noise density: ~150 µg/√Hz = 1.47e-3 m/s²/√Hz
# At 50 Hz: σ ≈ 1.47e-3 * √50 ≈ 0.010 m/s²  → variance ≈ 1.04e-4
_ACCEL_VARIANCE = 1.04e-4    # (m/s²)²

# Rotation vector (quaternion) accuracy reported by BNO085 is ~2–5° RMS in NDOF.
# Use a conservative 5° = 0.087 rad → variance ≈ 0.0076 rad²
_ORIENT_VARIANCE = 0.0076    # rad²

# Diagonal covariance matrices (row-major, 3×3 stored as 9-element list).
_ORIENT_COV = [_ORIENT_VARIANCE, 0, 0,
               0, _ORIENT_VARIANCE, 0,
               0, 0, _ORIENT_VARIANCE]

_GYRO_COV = [_GYRO_VARIANCE, 0, 0,
             0, _GYRO_VARIANCE, 0,
             0, 0, _GYRO_VARIANCE]

_ACCEL_COV = [_ACCEL_VARIANCE, 0, 0,
              0, _ACCEL_VARIANCE, 0,
              0, 0, _ACCEL_VARIANCE]


class ImuHandler:
    """Subscribe to the feldfreund_devkit Imu object and publish sensor_msgs/Imu."""

    TOPIC = '/imu/data'
    FRAME_ID = 'imu_link'

    def __init__(self, node: Node, imu) -> None:
        self.log = node.get_logger()
        self._node = node
        self._imu = imu

        self._pub = node.create_publisher(Imu, self.TOPIC, 10)

        # The feldfreund_devkit Imu fires ROTATION_CHANGED each time a new
        # quaternion arrives from the robot brain telemetry loop.
        try:
            imu.ROTATION_CHANGED.subscribe(self._on_rotation_changed)
        except AttributeError:
            # Fallback: some library versions use a generic NEW_MEASUREMENT signal.
            try:
                imu.NEW_MEASUREMENT.subscribe(self._on_rotation_changed)
            except AttributeError:
                self.log.warning(
                    'ImuHandler: no ROTATION_CHANGED / NEW_MEASUREMENT signal found on '
                    'the Imu object.  Publishing will not occur.  '
                    'Check the feldfreund_devkit version.'
                )

        self.log.info(f'ImuHandler: publishing BNO085 data on {self.TOPIC}')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _on_rotation_changed(self) -> None:
        """Called by the feldfreund_devkit event loop each time new IMU data arrives."""
        try:
            msg = self._build_message()
        except Exception as exc:  # noqa: BLE001
            self.log.warning(f'ImuHandler: could not build Imu message: {exc}')
            return
        self._pub.publish(msg)

    def _build_message(self) -> Imu:
        imu = self._imu
        msg = Imu()

        # Header
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self.FRAME_ID

        # Orientation quaternion — the library stores the offset-corrected rotation
        # as a pyquaternion.Quaternion in imu.rotation.
        # pyquaternion convention: Quaternion(w, x, y, z)
        rot = imu.rotation  # pyquaternion.Quaternion
        msg.orientation.w = float(rot.w)
        msg.orientation.x = float(rot.x)
        msg.orientation.y = float(rot.y)
        msg.orientation.z = float(rot.z)
        msg.orientation_covariance = _ORIENT_COV

        # Angular velocity (rad/s) from the calibrated gyroscope.
        # The feldfreund_devkit Imu exposes these as imu.angular_velocity
        # (a rosys Velocity3d-like object with x/y/z attributes), or falls
        # back to the raw telemetry fields.
        try:
            av = imu.angular_velocity
            msg.angular_velocity = Vector3(x=float(av.x), y=float(av.y), z=float(av.z))
        except AttributeError:
            # Older / minimal hardware objects may not expose angular_velocity.
            # In that case we leave it zero — it will be flagged by the -1 covariance.
            msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
        else:
            msg.angular_velocity_covariance = _GYRO_COV

        # Linear acceleration (m/s²).
        try:
            la = imu.linear_acceleration
            msg.linear_acceleration = Vector3(x=float(la.x), y=float(la.y), z=float(la.z))
            msg.linear_acceleration_covariance = _ACCEL_COV
        except AttributeError:
            msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

        return msg
