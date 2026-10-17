"""Build GPS_INPUT (#232) messages from FusionCore's fused pose.

NOT WIRED UP YET -- blocked on ardurover.md's "What needs to happen"
item 2: FusionCore's actual output topic name, ROS message type, and
publish rate aren't confirmed. Check the FusionCore node itself before
filling this in; don't guess a topic/type/rate here.

Once that's known, this needs to publish GPS_INPUT well inside
GUID_TIMEOUT (default 3.0s, see ardurover.md) and requires the flight
controller's GPS1_TYPE (or GPS2_TYPE) parameter set to 14 (GPS_TYPE_MAV) --
see ardurover.md item 6, a one-time config change, not something this
module can do.

Requirements to build in from the start (safety_roadmap.md O39):

  FusionCore's pose is already the output of a filter, and ArduPilot's EKF3
  will filter it again. Feeding it a confident-looking fix hides a degraded
  estimate from EKF3 and defeats its variance failsafes. So:

  * fix_type must follow what FusionCore actually knows, never be hard-coded
    to 3D or RTK. Derive it from the fused position covariance and the RTK
    status of the GNSS input; fall back to 0 (no fix) when either is stale.
  * horiz_accuracy, vert_accuracy and speed_accuracy come from FusionCore's
    covariance (square root of the relevant variance), not from constants.
  * hdop/vdop must use receiver-provided DOP values, never FusionCore's
    covariance. When unavailable, mark each field unknown (UINT16_MAX)
    and set its GPS_INPUT_IGNORE_FLAG_HDOP or GPS_INPUT_IGNORE_FLAG_VDOP.
  * ignore_flags must mark fields FusionCore does not provide (for example
    vertical velocity on a force_2d filter) so EKF3 does not trust a zero.
  * stop sending, or send fix_type 0, when FusionCore output is older than
    a fixed limit, so EKF3 and the GPS-glitch failsafes see the loss.
  * test on the bench with the fused covariance inflated and the output
    paused, and confirm the EKF-variance and GPS-loss failsafes still fire
    and bring the rover to a stop.
"""


def fused_pose_to_gps_input(*_args, **_kwargs) -> dict:
    """Reject conversion until the FusionCore output interface is known."""
    raise NotImplementedError(
        'FusionCore output topic/type not yet confirmed -- see '
        'ardurover.md item 2 in Sowbot_Data before implementing this.')
