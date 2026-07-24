from dataclasses import dataclass


@dataclass(frozen=True)
class ActionDef:
    """Definition of a mission action (drive, weed, etc.)."""
    label: str
    icon: str
    tool_topic: str | None = None
    param_schema: tuple = ()  # Optional parameter definitions for the action

ACTIONS: dict[str, ActionDef] = {
    'drive': ActionDef(label='Drive only', icon='🚜', tool_topic=None),
    'weed':  ActionDef(label='Weed',       icon='🌿', tool_topic='/tool/weeder/enable'),
}


def action_ros_msgs(action_key: str,
                    action_params: dict | None,  # pylint: disable=unused-argument
                    enable: bool) -> list[tuple[str, object]]:
    """Map an action + params to the (topic, value) pairs to publish.

    The mission executor calls this once with enable=True before navigating
    a row (engage the implement) and once with enable=False after (disengage),
    publishing each returned pair. A bool value is published as std_msgs/Bool,
    a numeric value as std_msgs/Float64.

    Currently the only ROS seam on an action is tool_topic — a Bool engage/
    disengage line. 'drive' has tool_topic=None and so yields nothing. Per-
    parameter setpoint topics (RPM, depth, blade height) aren't wired yet:
    param_schema carries no topic field, so action_params is accepted for
    forward-compatibility but unused. When a parameter gains its own topic,
    extend the schema and append (param_topic, float(value)) pairs here while
    enable is True.
    """
    adef = ACTIONS.get(action_key)
    if adef is None:
        return []

    msgs: list[tuple[str, object]] = []
    if adef.tool_topic:
        msgs.append((adef.tool_topic, bool(enable)))
    return msgs
