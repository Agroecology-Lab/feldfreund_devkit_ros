"""Build SET_POSITION_TARGET_LOCAL_NED (#84) messages from a Twist.

Field values are pinned per research/ardurover.md in Sowbot_Data (that
doc is the source of truth -- update there first, then here):

  coordinate_frame = MAV_FRAME_BODY_NED (8)   -- vx is forward, in the
                      rover's own frame, per Rover's own Guided Mode
                      MAVLink docs.
  type_mask        = 1511 (0x5E7)             -- Rover's documented
                      "Vel+Yaw Rate" combination: position ignored,
                      vx/vy used, vz ignored, acceleration ignored,
                      yaw ignored, yaw_rate used. This is the only
                      combination that reaches set_desired_turn_rate_
                      and_speed() -- anything else lands on heading-hold
                      or heading-only control instead of differential
                      drive.
"""

MAV_FRAME_BODY_NED = 8
TYPE_MASK_VEL_YAW_RATE = 1511  # 0x5E7 -- see module docstring.


def twist_to_position_target(linear_x: float, angular_z: float,
                              time_boot_ms: int) -> dict:
    """Map a Twist's linear.x / angular.z onto SET_POSITION_TARGET_LOCAL_NED
    fields, ready to hand to pymavlink's
    mav.set_position_target_local_ned_send(**twist_to_position_target(...)).

    Positional/attitude fields not used by this type_mask (x, y, z, afx,
    afy, afz, yaw) are zeroed -- they're ignored by the flight controller
    per TYPE_MASK_VEL_YAW_RATE, but pymavlink's encoder still expects them.
    """
    return dict(
        time_boot_ms=time_boot_ms,
        target_system=0,
        target_component=0,
        coordinate_frame=MAV_FRAME_BODY_NED,
        type_mask=TYPE_MASK_VEL_YAW_RATE,
        x=0.0, y=0.0, z=0.0,
        vx=linear_x, vy=0.0, vz=0.0,
        afx=0.0, afy=0.0, afz=0.0,
        yaw=0.0, yaw_rate=angular_z,
    )
