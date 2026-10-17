"""twist_to_position_target must keep the field values pinned in its module docstring."""

import unittest

from devkit_mavlink_bridge.modules.position_target_handler import (
    MAV_FRAME_BODY_NED,
    TYPE_MASK_VEL_YAW_RATE,
    twist_to_position_target,
)


class TwistToPositionTargetTest(unittest.TestCase):
    """Field mapping for SET_POSITION_TARGET_LOCAL_NED."""

    def test_pinned_frame_and_mask(self):
        """Verify body-frame NED and the Vel+Yaw-rate type mask are used."""
        fields = twist_to_position_target(0.5, 0.2, time_boot_ms=1)
        self.assertEqual(fields['coordinate_frame'], MAV_FRAME_BODY_NED)
        self.assertEqual(fields['type_mask'], TYPE_MASK_VEL_YAW_RATE)
        self.assertEqual(TYPE_MASK_VEL_YAW_RATE, 0x5E7)

    def test_velocity_mapping(self):
        """Verify linear.x goes to vx and angular.z to yaw_rate, with everything else zero."""
        fields = twist_to_position_target(0.5, -0.2, time_boot_ms=7)
        self.assertEqual((fields['vx'], fields['vy'], fields['vz'], fields['yaw_rate']), (0.5, 0.0, 0.0, -0.2))
        for name in ('x', 'y', 'z', 'afx', 'afy', 'afz', 'yaw'):
            self.assertEqual(fields[name], 0.0, name)

    def test_zero_twist_is_an_explicit_stop(self):
        """Verify a zero twist produces zero velocity and yaw rate rather than omitting them."""
        fields = twist_to_position_target(0.0, 0.0, time_boot_ms=0)
        self.assertEqual((fields['vx'], fields['yaw_rate']), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
