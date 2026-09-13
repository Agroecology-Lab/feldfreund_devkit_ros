"""Build GPS_INPUT (#232) messages from FusionCore's fused pose.

NOT WIRED UP YET -- blocked on ardurover.md's "What needs to happen"
item 2: FusionCore's actual output topic name, ROS message type, and
publish rate aren't confirmed. Check the FusionCore node itself before
filling this in; don't guess a topic/type/rate here.

Once that's known, this needs to publish GPS_INPUT well inside
GUID_TIMEOUT (default 3.0s, see ardurover.md) and requires the RTU's
GPS1_TYPE (or GPS2_TYPE) parameter set to 14 (GPS_TYPE_MAV) -- see
ardurover.md item 6, a one-time RTU-side config change, not something
this module can do.
"""


def fused_pose_to_gps_input(*_args, **_kwargs) -> dict:
    """Reject conversion until the FusionCore output interface is known."""
    raise NotImplementedError(
        'FusionCore output topic/type not yet confirmed -- see '
        'ardurover.md item 2 in Sowbot_Data before implementing this.')
